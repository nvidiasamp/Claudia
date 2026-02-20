#!/bin/bash
# Claudia Production Brain 启动脚本

echo "=================================="
echo "🤖 Claudia Production Brain Launcher"
echo "=================================="
echo ""

# 设置环境（从脚本位置推导项目根目录，避免硬编码）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 统一 Python 解释器（避免 conda/base 环境缺依赖导致误回退到 Mock）
# 可通过 CLAUDIA_PYTHON_BIN 覆盖
PYTHON_BIN="${CLAUDIA_PYTHON_BIN:-/usr/bin/python3}"
if [ ! -x "$PYTHON_BIN" ]; then
    echo "❌ Python解释器不可用: $PYTHON_BIN"
    exit 1
fi

# 注入 Unitree SDK2 Python 源码路径（兼容两个常见安装位置）
for SDK_PATH in "$SCRIPT_DIR/unitree_sdk2_python" "$HOME/unitree_sdk2_python"; do
    if [ -d "$SDK_PATH" ] && [[ ":${PYTHONPATH:-}:" != *":$SDK_PATH:"* ]]; then
        export PYTHONPATH="$SDK_PATH${PYTHONPATH:+:$PYTHONPATH}"
    fi
done

# 配置CycloneDDS环境 - 关键！
export CYCLONEDDS_HOME="$SCRIPT_DIR/cyclonedds/install"
export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 配置网络连接 - 使用官方推荐的内联配置方式
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'

echo "🔧 網路配置:"
echo "   本機IP: $(ip addr show eth0 | grep '192.168.123' | awk '{print $2}' | cut -d'/' -f1)"
echo "   機器人IP: 192.168.123.161 (Go2)"
echo "   DDS配置: eth0"
echo "   Python: $PYTHON_BIN ($($PYTHON_BIN -V 2>&1))"
echo ""

# 注意: 不再 source setup_cyclonedds.sh（它使用 $HOME/cyclonedds/install，
# 可能覆盖上面设置的 CYCLONEDDS_HOME，造成路径不一致）
# setup_cyclonedds.sh 仅用于首次安装 CycloneDDS，不在运行时加载

# ======================================================================
# デフォルト設定値 (環境変数未設定時のみ適用)
# ======================================================================

: "${CLAUDIA_WAKE_WORD_ENABLED:=0}"
: "${COMMANDER_WAKEUP_ANIMATION:=0}"
: "${BRAIN_MODEL_7B:=claudia-7b:v2.0}"
: "${BRAIN_ROUTER_MODE:=dual}"
: "${CLAUDIA_ASR_MODEL:=base}"
: "${SAFETY_ALLOW_HIGH_RISK:=0}"
: "${CLAUDIA_AUDIO_DEVICE:=auto}"

# ======================================================================
# ヘルパー関数
# ======================================================================

ensure_action_model() {
    # PR2: Action モデル確認・作成 (非 legacy 時のみ)
    if [ "$BRAIN_ROUTER_MODE" != "legacy" ]; then
        local action_model="${BRAIN_MODEL_ACTION:-claudia-action-v3}"
        if ! ollama list 2>/dev/null | grep -q "$action_model"; then
            echo "📦 Action モデル作成中: $action_model ..."
            ollama create "$action_model" -f models/ClaudiaAction_v3.0
            echo "✅ Action モデル作成完了"
        else
            echo "✅ Action モデル確認済み: $action_model"
        fi
        echo "   ルーティング: $BRAIN_ROUTER_MODE"
        echo ""
    fi
}

config_toggle_boolean() {
    # $1 = variable name, $2 = display label
    local var_name="$1"
    local current="${!var_name}"
    if [ "$current" = "1" ]; then
        export "$var_name=0"
        echo "  → $2: OFF"
    else
        export "$var_name=1"
        echo "  → $2: ON"
    fi
}

config_select_router_mode() {
    echo ""
    echo "  ルーティングモード選択:"
    echo "    1) dual   — Action channel 優先 (デフォルト)"
    echo "    2) legacy — 7B フルモデルのみ"
    echo "    3) shadow — Legacy + Action 比較ログ"
    read -p "  選択 [1/2/3]: " rm_choice
    case $rm_choice in
        1) export BRAIN_ROUTER_MODE=dual;   echo "  → dual" ;;
        2) export BRAIN_ROUTER_MODE=legacy; echo "  → legacy" ;;
        3) export BRAIN_ROUTER_MODE=shadow; echo "  → shadow" ;;
        *) echo "  (変更なし)" ;;
    esac
}

config_select_asr_model() {
    echo ""
    echo "  ASR モデル選択:"
    echo "    1) base   — 高速, CPU ~2-3s (デフォルト)"
    echo "    2) small  — バランス, CPU ~5-8s"
    echo "    3) medium — 高精度, CPU ~10-15s"
    read -p "  選択 [1/2/3]: " asr_choice
    case $asr_choice in
        1) export CLAUDIA_ASR_MODEL=base;   echo "  → base" ;;
        2) export CLAUDIA_ASR_MODEL=small;  echo "  → small" ;;
        3) export CLAUDIA_ASR_MODEL=medium; echo "  → medium" ;;
        *) echo "  (変更なし)" ;;
    esac
}

config_select_llm_model() {
    echo ""
    echo "  利用可能な Ollama モデル:"
    local models
    models=$(ollama list 2>/dev/null | tail -n +2 | awk '{print $1}')
    if [ -z "$models" ]; then
        echo "    (Ollama 未接続またはモデルなし)"
        return
    fi
    local i=1
    local model_array=()
    while IFS= read -r model; do
        model_array+=("$model")
        local marker=""
        if [ "$model" = "$BRAIN_MODEL_7B" ]; then
            marker=" (現在)"
        fi
        echo "    $i) ${model}${marker}"
        i=$((i + 1))
    done <<< "$models"
    read -p "  選択 [1-$((i-1))]: " llm_choice
    if [[ "$llm_choice" =~ ^[0-9]+$ ]] && [ "$llm_choice" -ge 1 ] && [ "$llm_choice" -le "${#model_array[@]}" ]; then
        export BRAIN_MODEL_7B="${model_array[$((llm_choice-1))]}"
        echo "  → $BRAIN_MODEL_7B"
    else
        echo "  (変更なし)"
    fi
}

config_select_mic_device() {
    echo ""
    echo "  マイクデバイス設定:"
    echo "    1) auto — 自動検出 (AT2020USB-XP 優先)"
    echo "    2) 手動入力 (例: hw:2,0)"
    read -p "  選択 [1/2]: " mic_choice
    case $mic_choice in
        1) export CLAUDIA_AUDIO_DEVICE=auto; echo "  → auto" ;;
        2)
            read -p "  デバイス名: " dev
            if [ -n "$dev" ]; then
                export CLAUDIA_AUDIO_DEVICE="$dev"
                echo "  → $dev"
            else
                echo "  (変更なし)"
            fi
            ;;
        *) echo "  (変更なし)" ;;
    esac
}

show_config_panel() {
    local wake_label="OFF"
    [ "$CLAUDIA_WAKE_WORD_ENABLED" = "1" ] && wake_label="ON"
    local anim_label="OFF"
    [ "$COMMANDER_WAKEUP_ANIMATION" = "1" ] && anim_label="ON"
    local risk_label="OFF"
    [ "$SAFETY_ALLOW_HIGH_RISK" = "1" ] && risk_label="ON"

    echo ""
    echo "  =================================="
    echo "    Claudia 設定パネル"
    echo "  =================================="
    echo "    1) 唤醒词            : $wake_label"
    echo "    2) 起動アニメーション : $anim_label"
    echo "    3) LLM モデル        : $BRAIN_MODEL_7B"
    echo "    4) ルーティング      : $BRAIN_ROUTER_MODE"
    echo "    5) ASR モデル        : $CLAUDIA_ASR_MODEL"
    echo "    6) 高リスク動作      : $risk_label"
    echo "    7) マイクデバイス    : $CLAUDIA_AUDIO_DEVICE"
    echo "  =================================="
    echo "    q) 戻る"
    echo "  =================================="
}

config_panel_loop() {
    while true; do
        show_config_panel
        read -p "  設定 [1-7/q]: " cfg_choice
        case $cfg_choice in
            1) config_toggle_boolean CLAUDIA_WAKE_WORD_ENABLED "唤醒词" ;;
            2) config_toggle_boolean COMMANDER_WAKEUP_ANIMATION "起動アニメーション" ;;
            3) config_select_llm_model ;;
            4) config_select_router_mode ;;
            5) config_select_asr_model ;;
            6) config_toggle_boolean SAFETY_ALLOW_HIGH_RISK "高リスク動作" ;;
            7) config_select_mic_device ;;
            q|Q) break ;;
            *) echo "  (無効な選択)" ;;
        esac
    done
}

launch_in_tmux() {
    # tmux 可用性チェック
    if ! command -v tmux &>/dev/null; then
        echo "  ❌ tmux がインストールされていません"
        echo "  インストール: sudo apt install tmux"
        return
    fi

    local session_name="claudia"

    # 既存セッションチェック
    if tmux has-session -t "$session_name" 2>/dev/null; then
        echo ""
        echo "  既存の tmux セッション '$session_name' が見つかりました"
        echo "    a) アタッチ (既存セッションに接続)"
        echo "    k) 終了して新規作成"
        echo "    c) キャンセル"
        read -p "  選択 [a/k/c]: " tmux_action
        case $tmux_action in
            a|A) exec tmux attach-session -t "$session_name" ;;
            k|K) tmux kill-session -t "$session_name"; echo "  既存セッション終了" ;;
            *) echo "  キャンセル"; return ;;
        esac
    fi

    # モード選択
    echo ""
    echo "  tmux 後台モード:"
    echo "    1) 語音 + シミュレーション"
    echo "    2) 語音 + 実機"
    echo "    3) キーボード + シミュレーション"
    echo "    4) キーボード + 実機"
    read -p "  選択 [1-4]: " tmux_mode

    local cmd_args=""
    local py_script=""
    case $tmux_mode in
        1) py_script="voice_commander.py"; cmd_args="--daemon" ;;
        2) py_script="voice_commander.py"; cmd_args="--hardware --daemon" ;;
        3) py_script="production_commander.py"; cmd_args="" ;;
        4) py_script="production_commander.py"; cmd_args="--hardware" ;;
        *) echo "  無効な選択"; return ;;
    esac

    # ランチャースクリプト生成 (環境変数を安全に保存)
    local launcher="/tmp/claudia_tmux_$$.sh"
    {
        echo "#!/bin/bash"
        echo "cd '$SCRIPT_DIR'"
        # Claudia 設定
        echo "export CLAUDIA_WAKE_WORD_ENABLED='$CLAUDIA_WAKE_WORD_ENABLED'"
        echo "export COMMANDER_WAKEUP_ANIMATION='$COMMANDER_WAKEUP_ANIMATION'"
        echo "export BRAIN_MODEL_7B='$BRAIN_MODEL_7B'"
        echo "export BRAIN_ROUTER_MODE='$BRAIN_ROUTER_MODE'"
        echo "export CLAUDIA_ASR_MODEL='$CLAUDIA_ASR_MODEL'"
        echo "export SAFETY_ALLOW_HIGH_RISK='$SAFETY_ALLOW_HIGH_RISK'"
        # DDS / SDK
        echo "export CYCLONEDDS_HOME='$CYCLONEDDS_HOME'"
        echo "export LD_LIBRARY_PATH='$LD_LIBRARY_PATH'"
        echo "export RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION'"
        echo "export PYTHONPATH='$PYTHONPATH'"
        # CYCLONEDDS_URI は改行+引用符を含むため printf %q 経由
        printf "export CYCLONEDDS_URI=%q\n" "$CYCLONEDDS_URI"
        echo "'$PYTHON_BIN' $py_script $cmd_args"
        echo "echo '--- セッション終了 (Enter で閉じる) ---'"
        echo "read"
    } > "$launcher"
    chmod +x "$launcher"

    tmux new-session -d -s "$session_name" "bash '$launcher'"

    echo ""
    echo "  ✅ tmux セッション '$session_name' 起動完了"
    echo ""
    echo "  操作:"
    echo "    接続: tmux attach -t $session_name"
    echo "    切断: Ctrl+B → D"
    echo "    終了: tmux kill-session -t $session_name"
    echo ""
    read -p "  今すぐ接続? [Y/n]: " attach_now
    if [[ "$attach_now" != [nN] ]]; then
        exec tmux attach-session -t "$session_name"
    fi
}

# ======================================================================
# 実行モード
# ======================================================================

execute_mode() {
    local mode="$1"

    # auto デバイスは unset して Python 側の自動検出に委ねる
    if [ "$CLAUDIA_AUDIO_DEVICE" = "auto" ]; then
        unset CLAUDIA_AUDIO_DEVICE
    fi

    # 全モード共通: Action モデル確認
    ensure_action_model

    case $mode in
        1)
            echo ""
            echo "✅ 起動: キーボード + シミュレーション"
            echo ""
            if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
                echo "⚠️ Python ollama 未検出、HTTP フォールバック予熱"
            fi
            "$PYTHON_BIN" production_commander.py
            ;;
        2)
            echo ""
            echo "⚠️  実機モード — 機器人接続を確認してください"
            read -p "確認? [y/N]: " confirm
            if [[ $confirm == [yY] ]]; then
                if ! "$PYTHON_BIN" -c "import unitree_sdk2py, cyclonedds.idl" >/dev/null 2>&1; then
                    echo ""
                    echo "❌ ハードウェア依存の import 失敗 (unitree_sdk2py / cyclonedds.idl)"
                    echo "   Python: $PYTHON_BIN"
                    echo "   PYTHONPATH: ${PYTHONPATH:-<empty>}"
                    echo ""
                    echo "修正手順:"
                    echo "1) export PYTHONPATH=$SCRIPT_DIR/unitree_sdk2_python:\$PYTHONPATH"
                    echo "2) CycloneDDS ライブラリパス確認: $CYCLONEDDS_HOME/lib"
                    echo "3) import テスト: $PYTHON_BIN -c 'import unitree_sdk2py, cyclonedds.idl; print(\"OK\")'"
                    echo ""
                    echo "MockSportClient への誤回退を防ぐため停止しました。"
                    exit 1
                fi
                echo ""
                echo "✅ 起動: キーボード + 実機"
                echo ""
                if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
                    echo "⚠️ Python ollama 未検出、HTTP フォールバック予熱"
                fi
                "$PYTHON_BIN" production_commander.py --hardware
            else
                echo "キャンセル"
            fi
            ;;
        3)
            echo ""
            echo "✅ 起動: 語音 + シミュレーション"
            echo ""
            if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
                echo "⚠️ Python ollama 未検出、HTTP フォールバック予熱"
            fi
            "$PYTHON_BIN" voice_commander.py
            ;;
        4)
            echo ""
            echo "⚠️  語音実機モード — 機器人 + USB マイク接続を確認してください"
            read -p "確認? [y/N]: " confirm
            if [[ $confirm == [yY] ]]; then
                if ! "$PYTHON_BIN" -c "import unitree_sdk2py, cyclonedds.idl" >/dev/null 2>&1; then
                    echo ""
                    echo "❌ ハードウェア依存の import 失敗 (unitree_sdk2py / cyclonedds.idl)"
                    echo "   Python: $PYTHON_BIN"
                    echo "   PYTHONPATH: ${PYTHONPATH:-<empty>}"
                    echo ""
                    echo "MockSportClient への誤回退を防ぐため停止しました。"
                    exit 1
                fi
                echo ""
                echo "✅ 起動: 語音 + 実機"
                echo ""
                if ! "$PYTHON_BIN" -c "import ollama" >/dev/null 2>&1; then
                    echo "⚠️ Python ollama 未検出、HTTP フォールバック予熱"
                fi
                "$PYTHON_BIN" voice_commander.py --hardware
            else
                echo "キャンセル"
            fi
            ;;
        *)
            echo "無効なモード: $mode"
            exit 1
            ;;
    esac
}

# ======================================================================
# メイン: 直接起動パラメータ or 対話メニュー
# ======================================================================

if [ "$1" = "--voice" ]; then
    execute_mode 3
    exit 0
elif [ "$1" = "--voice-hw" ]; then
    execute_mode 4
    exit 0
fi

# 対話メニューループ
while true; do
    echo "运行モード選択:"
    echo "  1) キーボード + シミュレーション"
    echo "  2) キーボード + 実機"
    echo "  3) 語音 + シミュレーション"
    echo "  4) 語音 + 実機"
    echo "  c) 設定パネル"
    echo "  t) 後台モード (tmux)"
    echo ""
    read -p "選択 [1/2/3/4/c/t]: " choice

    case $choice in
        1|2|3|4)
            execute_mode "$choice"
            break
            ;;
        c|C)
            config_panel_loop
            ;;
        t|T)
            launch_in_tmux
            break
            ;;
        *)
            echo "  無効な選択"
            echo ""
            ;;
    esac
done
