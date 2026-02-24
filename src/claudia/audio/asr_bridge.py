#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR ブリッジ — ASR サーバー結果を消費し ProductionBrain へ橋渡し

result.sock から JSON Lines を読み取り、メッセージタイプに応じて分岐:
  - emergency → 即時 brain 呼出 (キュー迂回) + キュー空化 + 冷却
  - transcript → 信頼度フィルタ + 重複排除 → コマンドキュー
  - heartbeat → watchdog 更新
  - ready → プロトコルバージョン検証 + 準備完了シグナル

コマンドキュー (bounded, maxsize=3) → ワーカーが直列消費 → brain.process_and_execute
"""

import asyncio
import json
import logging
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

from .asr_service.ipc_protocol import (
    ASR_RESULT_SOCKET,
    PROTO_VERSION,
    connect_uds,
    read_json_lines,
    validate_proto_version,
)

logger = logging.getLogger("claudia.asr.bridge")

# 設定定数
QUEUE_MAXSIZE = 3
MIN_CONFIDENCE = 0.35
DEDUP_TTL_S = 10.0
HEARTBEAT_TIMEOUT_S = 15.0
EMERGENCY_COOLDOWN_S = 0.5
RECONNECT_DELAY_S = 2.0

# E2E タイミングログ
_E2E_LOG_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "logs",
)
_E2E_LOG_PATH = os.path.join(_E2E_LOG_DIR, "e2e_timing.jsonl")


class ASRBridge:
    """ASR 結果消費 + コマンドキュー管理

    Args:
        brain: ProductionBrain インスタンス (process_and_execute を呼ぶ)
        socket_path: result socket パス
        on_result: 結果コールバック (デバッグ表示用, Optional)
    """

    def __init__(
        self,
        brain: Any,
        socket_path: str = ASR_RESULT_SOCKET,
        on_result: Any = None,
        on_wake: Any = None,
    ) -> None:
        self._brain = brain
        self._socket_path = socket_path
        self._on_result = on_result

        # 唤醒词ゲート
        from .wake_word import WakeWordGate
        self._wake_gate = WakeWordGate(
            on_wake=self._make_wake_callback(on_wake),
        )

        # 準備完了イベント (ready メッセージ受信で set)
        self.ready_event = asyncio.Event()

        # コマンドキュー (bounded, drop-oldest)
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=QUEUE_MAXSIZE)

        # 重複排除: utterance_id → 処理時刻
        self._processed_ids: Dict[str, float] = {}

        # ハートビート watchdog
        self._last_heartbeat: float = time.monotonic()

        # Emergency 冷却ウィンドウ
        self._cooldown_until: float = 0.0

        # タスク参照
        self._consumer_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._worker_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]
        self._watchdog_task: Optional[asyncio.Task] = None  # type: ignore[type-arg]

        self._running = False

    def _make_wake_callback(self, external_on_wake):
        # type: (Any) -> Any
        """唤醒詞コールバックを生成: 外部コールバック + "wake" イベント通知"""
        def _cb():
            if external_on_wake:
                try:
                    external_on_wake()
                except Exception:
                    pass
            if self._on_result:
                try:
                    self._on_result("wake", "", 0)
                except Exception:
                    pass
        return _cb

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------

    async def start(self) -> None:
        """ブリッジ開始: 結果消費 + コマンドワーカーを起動

        watchdog は ready ハンドシェイク後に開始 (ASR モデル読込中の誤検知防止)。
        """
        self._running = True
        self._last_heartbeat = time.monotonic()

        self._consumer_task = asyncio.ensure_future(self._result_consumer_loop())
        self._worker_task = asyncio.ensure_future(self._command_worker())
        # watchdog は _handle_ready() 内で開始する

        logger.info("ASRBridge 開始")

    async def stop(self) -> None:
        """ブリッジ停止"""
        logger.info("ASRBridge 停止中...")
        self._running = False

        for task in (self._consumer_task, self._worker_task, self._watchdog_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        self._consumer_task = None
        self._worker_task = None
        self._watchdog_task = None

    # ------------------------------------------------------------------
    # 結果消費ループ (自動再接続)
    # ------------------------------------------------------------------

    async def _result_consumer_loop(self) -> None:
        """外層ループ: 接続断 → リトライ"""
        while self._running:
            try:
                reader, writer = await connect_uds(
                    self._socket_path, retries=10, delay=1.0,
                )
                logger.info("result.sock 接続完了")

                try:
                    await self._consume_results(reader)
                finally:
                    try:
                        writer.close()
                    except Exception:
                        pass

            except asyncio.CancelledError:
                break
            except ConnectionError as e:
                if not self._running:
                    break
                logger.warning("result.sock 接続失敗: %s — %.1fs 後リトライ",
                               e, RECONNECT_DELAY_S)
                await asyncio.sleep(RECONNECT_DELAY_S)
            except Exception as e:
                if not self._running:
                    break
                logger.error("結果消費エラー: %s", e, exc_info=True)
                await asyncio.sleep(RECONNECT_DELAY_S)

    async def _consume_results(self, reader: asyncio.StreamReader) -> None:
        """JSON Lines 読取 → メッセージ分岐"""
        async for msg in read_json_lines(reader):
            if not self._running:
                break
            msg_type = msg.get("type", "")
            try:
                if msg_type == "ready":
                    await self._handle_ready(msg)
                elif msg_type == "transcript":
                    await self._handle_transcript(msg)
                elif msg_type == "emergency":
                    await self._handle_emergency(msg)
                elif msg_type == "heartbeat":
                    self._handle_heartbeat(msg)
                elif msg_type == "vad_start":
                    if self._on_result:
                        try:
                            self._on_result("vad_start", "", 0)
                        except Exception:
                            pass
                elif msg_type in ("vad_end", "error", "gate_timeout_audit"):
                    logger.debug("ASR イベント: %s", msg_type)
                else:
                    logger.warning("未知メッセージタイプ: %s", msg_type)
            except Exception as e:
                logger.error("メッセージ処理エラー (%s): %s", msg_type, e)

    # ------------------------------------------------------------------
    # メッセージハンドラ
    # ------------------------------------------------------------------

    async def _handle_ready(self, msg: dict) -> None:
        """ready ハンドシェイク: プロトコルバージョン検証 + watchdog 開始"""
        version = msg.get("proto_version", "")
        model = msg.get("model", "unknown")

        if not validate_proto_version(version):
            logger.error(
                "プロトコルバージョン不一致: remote=%s, local=%s",
                version, PROTO_VERSION,
            )
            return

        logger.info("ASR 準備完了: model=%s, version=%s", model, version)

        # ready 受信後にハートビート watchdog を開始
        # (ASR モデル読込中の誤検知を防ぐ)
        self._last_heartbeat = time.monotonic()
        if self._watchdog_task is None or self._watchdog_task.done():
            self._watchdog_task = asyncio.ensure_future(self._heartbeat_watchdog())

        self.ready_event.set()

    async def _handle_transcript(self, msg: dict) -> None:
        """transcript 処理: emergency 兜底復検 + 信頼度フィルタ + 重複排除 + キュー投入"""
        text = msg.get("text", "").strip()
        confidence = msg.get("confidence", 0.0)
        utterance_id = msg.get("utterance_id", "")

        if not text:
            return

        # 兜底: transcript でも emergency キーワードチェック
        try:
            from .asr_service.emergency_keywords import match_emergency
            emg = match_emergency(text)
            if emg is not None:
                logger.warning("transcript 経由 emergency 検出: '%s'", text)
                await self._handle_emergency_action(utterance_id, text)
                return
        except ImportError:
            pass  # emergency_keywords が無い場合はスキップ

        # 信頼度フィルタ
        if confidence < MIN_CONFIDENCE:
            logger.info("低信頼度スキップ: '%s' (conf=%.2f < %.2f)", text, confidence, MIN_CONFIDENCE)
            return

        # 唤醒詞ゲート (emergency は上で処理済み — ここには到達しない)
        filtered_text = self._wake_gate.filter(text, confidence)
        if filtered_text is None:
            return
        text = filtered_text

        # 重複排除 (emergency が先に処理済みの場合スキップ)
        if self._is_processed(utterance_id):
            logger.debug("重複排除: utt=%s", utterance_id)
            return

        # キューに投入 (mark_processed はワーカー実行時に行う)
        asr_latency_ms = msg.get("asr_latency_ms", 0)
        await self._enqueue_command(text, utterance_id, asr_latency_ms)

        if self._on_result:
            try:
                self._on_result("transcript", text, confidence)
            except Exception:
                pass

    async def _handle_emergency(self, msg: dict) -> None:
        """emergency 即時処理"""
        keyword = msg.get("keyword", "")
        utterance_id = msg.get("utterance_id", "")

        logger.warning("Emergency 受信: keyword='%s', utt=%s", keyword, utterance_id)

        await self._handle_emergency_action(utterance_id, keyword)

        if self._on_result:
            try:
                self._on_result("emergency", keyword, 1.0)
            except Exception:
                pass

    async def _handle_emergency_action(self, utterance_id: str, keyword: str) -> None:
        """Emergency 共通処理: キュー空化 → 冷却設定 → brain 呼出

        順序が重要: キュー空化と冷却を brain 呼出の前に行うことで、
        brain の await 中にワーカーが古いコマンドを実行するのを防ぐ。
        (emergency は _command_lock を取得しないため、ワーカーと並行実行される)
        """
        # 重複排除チェック
        if self._is_processed(utterance_id):
            logger.debug("emergency 重複排除: utt=%s", utterance_id)
            return

        self._mark_processed(utterance_id)

        # 先にキュー空化 + 冷却設定 (brain await 中のワーカー実行を防止)
        flushed = 0
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
                flushed += 1
            except asyncio.QueueEmpty:
                break
        if flushed:
            logger.info("Emergency キュー空化: %d 件破棄", flushed)

        self._cooldown_until = time.monotonic() + EMERGENCY_COOLDOWN_S

        # 唤醒詞ゲートリセット (emergency 時は監聴窓を閉じる)
        self._wake_gate.reset()

        # brain 呼出 (キュー迂回) + E2E 計測
        logger.warning("Emergency 実行: '%s'", keyword)
        brain_start = time.monotonic()
        try:
            result = await self._brain.process_and_execute("止まれ")
            brain_ms = (time.monotonic() - brain_start) * 1000

            self._write_e2e_log(
                command=keyword,
                route="emergency",
                asr_ms=0,  # emergency はキーワード検出、ASR full推論なし
                queue_ms=0,
                brain_ms=brain_ms,
                e2e_ms=brain_ms,
                api_code=result.api_code if result else None,
                execution_status=result.execution_status or "" if result else "",
                utterance_id=utterance_id,
            )

            logger.info("Emergency 結果: %s (brain=%.0fms)", result, brain_ms)
        except Exception as e:
            brain_ms = (time.monotonic() - brain_start) * 1000
            logger.error("Emergency brain 呼出失敗: %s (brain=%.0fms)", e, brain_ms)

    def _handle_heartbeat(self, msg: dict) -> None:
        """heartbeat 更新"""
        self._last_heartbeat = time.monotonic()

    # ------------------------------------------------------------------
    # 重複排除 (utterance_id)
    # ------------------------------------------------------------------

    def _is_processed(self, utterance_id: str) -> bool:
        """utterance_id が処理済みか確認 (空文字列は常に未処理扱い)"""
        if not utterance_id:
            return False
        self._cleanup_expired_ids()
        return utterance_id in self._processed_ids

    def _mark_processed(self, utterance_id: str) -> None:
        """utterance_id を処理済みとしてマーク（+ 定期クリーンアップ）"""
        if utterance_id:
            self._processed_ids[utterance_id] = time.monotonic()
            # _is_processed が呼ばれないパスでもメモリ蓄積を防ぐ
            if len(self._processed_ids) > 100:
                self._cleanup_expired_ids()

    def _cleanup_expired_ids(self) -> None:
        """TTL 超過した utterance_id を削除"""
        now = time.monotonic()
        expired = [
            uid for uid, ts in self._processed_ids.items()
            if now - ts > DEDUP_TTL_S
        ]
        for uid in expired:
            del self._processed_ids[uid]

    # ------------------------------------------------------------------
    # コマンドキュー
    # ------------------------------------------------------------------

    async def _enqueue_command(
        self, text: str, utterance_id: str, asr_latency_ms: int = 0,
    ) -> None:
        """bounded キューにコマンドを投入 (満杯時は最古を破棄)"""
        if self._queue.full():
            try:
                discarded = self._queue.get_nowait()
                logger.warning("キュー満杯: 最古コマンド破棄 '%s'", discarded[0])
            except asyncio.QueueEmpty:
                pass
        enqueue_ts = time.monotonic()
        await self._queue.put((text, utterance_id, asr_latency_ms, enqueue_ts))
        logger.debug("コマンドキュー投入: '%s' (size=%d)", text, self._queue.qsize())

    async def _command_worker(self) -> None:
        """コマンドワーカー: キューから直列消費 → brain.process_and_execute + E2E計測"""
        while self._running:
            try:
                item = await asyncio.wait_for(
                    self._queue.get(), timeout=1.0,
                )
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break

            text, utterance_id, asr_latency_ms, enqueue_ts = item

            # 冷却ウィンドウチェック
            now = time.monotonic()
            if now < self._cooldown_until:
                remaining = self._cooldown_until - now
                logger.debug("冷却中: %.0fms 残り, '%s' スキップ",
                             remaining * 1000, text)
                continue

            # 重複排除 (キュー投入から実行までに同一 ID が処理された場合)
            if self._is_processed(utterance_id):
                logger.debug("ワーカー重複排除: utt=%s", utterance_id)
                continue

            self._mark_processed(utterance_id)

            # キュー待機時間
            queue_wait_ms = (now - enqueue_ts) * 1000

            # brain 呼出 + E2E 計測
            logger.info("コマンド実行: '%s'", text)
            brain_start = time.monotonic()
            try:
                result = await self._brain.process_and_execute(text)
                brain_ms = (time.monotonic() - brain_start) * 1000

                # E2E = ASR推論 + キュー待機 + Brain処理+実行
                e2e_ms = asr_latency_ms + queue_wait_ms + brain_ms

                # E2E タイミングログ出力
                self._write_e2e_log(
                    command=text,
                    route=result.reasoning or "",
                    asr_ms=asr_latency_ms,
                    queue_ms=queue_wait_ms,
                    brain_ms=brain_ms,
                    e2e_ms=e2e_ms,
                    api_code=result.api_code,
                    execution_status=result.execution_status or "",
                    utterance_id=utterance_id,
                )

                logger.info(
                    "E2E計測: '%s' → e2e=%.0fms (asr=%dms + queue=%.0fms + brain=%.0fms)",
                    text, e2e_ms, asr_latency_ms, queue_wait_ms, brain_ms,
                )

                # 結果表示 (E2E データ付き)
                if self._on_result:
                    try:
                        self._on_result("result", text, result)
                        self._on_result("e2e_timing", text, {
                            "asr_ms": asr_latency_ms,
                            "brain_ms": brain_ms,
                            "e2e_ms": e2e_ms,
                        })
                    except Exception:
                        pass

                logger.info("コマンド結果: response='%s', api=%s, status=%s",
                            result.response[:50] if result.response else "",
                            result.api_code,
                            result.execution_status)

            except Exception as e:
                brain_ms = (time.monotonic() - brain_start) * 1000
                logger.error("コマンド実行失敗: '%s': %s (brain=%.0fms)", text, e, brain_ms)

    # ------------------------------------------------------------------
    # ハートビート watchdog
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # E2E タイミングログ
    # ------------------------------------------------------------------

    def _write_e2e_log(
        self,
        command,        # type: str
        route,          # type: str
        asr_ms,         # type: float
        queue_ms,       # type: float
        brain_ms,       # type: float
        e2e_ms,         # type: float
        api_code,       # type: Optional[int]
        execution_status,  # type: str
        utterance_id,   # type: str
    ):
        # type: (...) -> None
        """E2E タイミングを JSONL ファイルに記録"""
        try:
            entry = {
                "ts": datetime.now().isoformat(),
                "command": command,
                "route": route,
                "asr_ms": round(asr_ms, 1),
                "queue_ms": round(queue_ms, 1),
                "brain_ms": round(brain_ms, 1),
                "e2e_ms": round(e2e_ms, 1),
                "api_code": api_code,
                "execution_status": execution_status,
                "utterance_id": utterance_id,
            }
            log_path = os.path.normpath(_E2E_LOG_PATH)
            log_dir = os.path.dirname(log_path)
            if not os.path.isdir(log_dir):
                os.makedirs(log_dir, exist_ok=True)
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception as e:
            logger.debug("E2E ログ書込失敗: %s", e)

    # ------------------------------------------------------------------
    # ハートビート watchdog
    # ------------------------------------------------------------------

    async def _heartbeat_watchdog(self) -> None:
        """ハートビート監視: タイムアウト時に再接続を促す"""
        while self._running:
            try:
                await asyncio.sleep(5.0)
            except asyncio.CancelledError:
                break

            elapsed = time.monotonic() - self._last_heartbeat
            if elapsed > HEARTBEAT_TIMEOUT_S:
                logger.warning(
                    "ハートビートタイムアウト: %.0fs 経過 (閾値=%ds)",
                    elapsed, HEARTBEAT_TIMEOUT_S,
                )
                # consumer_task をキャンセル → 再接続ループに入る
                if self._consumer_task and not self._consumer_task.done():
                    self._consumer_task.cancel()
                    # consumer_loop の外側 while が再接続する
                    self._consumer_task = asyncio.ensure_future(
                        self._result_consumer_loop()
                    )
                self._last_heartbeat = time.monotonic()  # リセット
