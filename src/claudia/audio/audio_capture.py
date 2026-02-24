#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB 麦克風音声キャプチャ — arecord 非同期サブプロセス

AT2020USB-XP (hw:2,0, 44100Hz) → resample 16kHz → UDS audio.sock
フレーム単位 (960 bytes = 30ms @ 16kHz 16bit mono) で ASR サーバーに送信。
"""

import asyncio
import logging
import os
import re
import time
from typing import Optional

from .pcm_utils import resample_pcm_int16
from .asr_service.ipc_protocol import AUDIO_SOCKET, connect_uds

logger = logging.getLogger("claudia.audio.capture")

# PCM 定数
SRC_RATE = 44100       # USB マイクネイティブレート
DST_RATE = 16000       # ASR サーバー入力レート
FRAME_SAMPLES = 480    # 30ms @ 16kHz
FRAME_BYTES = FRAME_SAMPLES * 2  # 960 bytes (16-bit mono)

# arecord から 1 回に読むサイズ: 100ms @ 44100Hz 16bit mono = 8820 bytes
ARECORD_CHUNK_SAMPLES = 4410  # 100ms
ARECORD_CHUNK_BYTES = ARECORD_CHUNK_SAMPLES * 2  # 8820 bytes

# 再試行設定
RETRY_DELAY_INIT = 1.0
RETRY_DELAY_MAX = 30.0


class AudioCapture:
    """USB マイク音声キャプチャ

    arecord 非同期サブプロセスで PCM を取得し、16kHz にリサンプル後、
    UDS audio socket にフレーム単位で送信する。

    Args:
        device: ALSA デバイス名 (例: "hw:2,0")。None で自動検出。
        mock: True の場合、無音フレームを生成 (マイク不要)
        socket_path: 音声ソケットパス (デフォルト: ipc_protocol.AUDIO_SOCKET)
    """

    def __init__(
        self,
        device: Optional[str] = None,
        mock: bool = False,
        socket_path: str = AUDIO_SOCKET,
    ) -> None:
        self._explicit_device = device or os.environ.get("CLAUDIA_AUDIO_DEVICE")
        self._device = self._explicit_device
        self._mock = mock
        self._socket_path = socket_path
        self._running = False
        self._process: Optional[asyncio.subprocess.Process] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._frame_buffer = b""

    # ------------------------------------------------------------------
    # 公開 API
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """メインループ: デバイス検出 → ソケット接続 → キャプチャ

        USB 抜き差し時は指数バックオフでリトライ。
        """
        self._running = True
        retry_delay = RETRY_DELAY_INIT

        while self._running:
            try:
                # 1. デバイス検出 (mock の場合スキップ)
                if not self._mock and not self._device:
                    self._device = await self._discover_device()

                # 2. ソケット接続
                await self._connect_socket()

                # 3. キャプチャループ
                await self._capture_loop()

                # 正常終了 (running=False)
                break

            except (ConnectionError, OSError) as e:
                if not self._running:
                    break
                logger.warning("キャプチャエラー: %s — %.1fs 後リトライ", e, retry_delay)
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RETRY_DELAY_MAX)
                # 明示指定がなければ再検出 (指定デバイスは保持)
                if not self._explicit_device:
                    self._device = None

            except asyncio.CancelledError:
                break

            except Exception as e:
                if not self._running:
                    break
                logger.error("予期せぬキャプチャエラー: %s", e, exc_info=True)
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, RETRY_DELAY_MAX)

        await self._cleanup()

    async def shutdown(self) -> None:
        """優雅なシャットダウン"""
        logger.info("AudioCapture シャットダウン中...")
        self._running = False
        await self._cleanup()

    # ------------------------------------------------------------------
    # デバイス検出
    # ------------------------------------------------------------------

    async def _discover_device(self) -> str:
        """arecord -l 解析で AT2020 USB マイクを自動検出

        Returns:
            ALSA デバイス名 (例: "hw:2,0")

        Raises:
            RuntimeError: デバイスが見つからない場合
        """
        try:
            proc = await asyncio.create_subprocess_exec(
                "arecord", "-l",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
            stdout, _ = await proc.communicate()
            output = stdout.decode("utf-8", errors="replace")
        except FileNotFoundError:
            raise RuntimeError("arecord コマンドが見つかりません (alsa-utils 未インストール?)")

        # card N: ... [AT2020...] の行を探す
        pattern = re.compile(r"card\s+(\d+):.*AT2020", re.IGNORECASE)
        for line in output.splitlines():
            m = pattern.search(line)
            if m:
                card_num = m.group(1)
                device = "hw:{},0".format(card_num)
                logger.info("AT2020 USB マイク検出: %s", device)
                return device

        raise RuntimeError(
            "AT2020 USB マイクが見つかりません。"
            "CLAUDIA_AUDIO_DEVICE 環境変数でデバイスを指定してください。"
        )

    # ------------------------------------------------------------------
    # ソケット接続
    # ------------------------------------------------------------------

    async def _connect_socket(self) -> None:
        """audio.sock に接続"""
        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

        _, writer = await connect_uds(self._socket_path, retries=10, delay=1.0)
        self._writer = writer
        self._frame_buffer = b""
        logger.info("audio.sock 接続完了: %s", self._socket_path)

    # ------------------------------------------------------------------
    # キャプチャループ
    # ------------------------------------------------------------------

    async def _capture_loop(self) -> None:
        """arecord → 読取 → リサンプル → フレーム送信"""
        if self._mock:
            await self._mock_capture_loop()
            return

        # arecord 起動
        self._process = await asyncio.create_subprocess_exec(
            "arecord",
            "-D", self._device,
            "-f", "S16_LE",
            "-r", str(SRC_RATE),
            "-c", "1",
            "-t", "raw",
            "--buffer-size", "8192",
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.DEVNULL,
        )
        logger.info("arecord 起動: device=%s, rate=%d", self._device, SRC_RATE)

        loop = asyncio.get_event_loop()

        try:
            while self._running and self._process.returncode is None:
                # 非同期で 100ms 分読取
                data = await self._process.stdout.readexactly(ARECORD_CHUNK_BYTES)

                # リサンプル (CPU バウンド → スレッドプール)
                import numpy as np
                samples_in = np.frombuffer(data, dtype=np.int16)
                samples_out = await loop.run_in_executor(
                    None, resample_pcm_int16, samples_in, SRC_RATE, DST_RATE,
                )
                resampled_bytes = samples_out.tobytes()

                # フレームバッファに追加 → 完全フレーム送信
                await self._send_frames(resampled_bytes)

        except asyncio.IncompleteReadError:
            if self._running:
                logger.warning("arecord 予期せず終了 (USB 切断?)")
                raise ConnectionError("arecord プロセス終了")
        finally:
            await self._kill_process()

    async def _mock_capture_loop(self) -> None:
        """Mock モード: 30ms 間隔で無音フレームを送信"""
        silent_frame = b"\x00" * FRAME_BYTES
        logger.info("mock キャプチャモード開始")

        while self._running:
            try:
                self._writer.write(silent_frame)
                await self._writer.drain()
            except (ConnectionError, OSError):
                if self._running:
                    raise
                break
            await asyncio.sleep(0.03)  # 30ms

    # ------------------------------------------------------------------
    # フレーム送信
    # ------------------------------------------------------------------

    async def _send_frames(self, resampled_bytes: bytes) -> None:
        """残差バッファと合わせて完全 960 byte フレームを送信"""
        self._frame_buffer += resampled_bytes

        while len(self._frame_buffer) >= FRAME_BYTES:
            frame = self._frame_buffer[:FRAME_BYTES]
            self._frame_buffer = self._frame_buffer[FRAME_BYTES:]
            try:
                self._writer.write(frame)
                await self._writer.drain()
            except (ConnectionError, OSError) as e:
                if self._running:
                    raise ConnectionError("audio.sock 書き込み失敗: {}".format(e))
                break

    # ------------------------------------------------------------------
    # クリーンアップ
    # ------------------------------------------------------------------

    async def _kill_process(self) -> None:
        """arecord プロセスを終了"""
        if self._process and self._process.returncode is None:
            try:
                self._process.terminate()
                try:
                    await asyncio.wait_for(self._process.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    self._process.kill()
                    await self._process.wait()
            except ProcessLookupError:
                pass
            self._process = None

    async def _cleanup(self) -> None:
        """全リソース解放"""
        await self._kill_process()

        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
            self._writer = None

        self._frame_buffer = b""
