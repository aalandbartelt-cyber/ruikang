#!/bin/bash
# ============================================================
# 睿抗机器人 一键急停脚本
# 同时停止 Python 视觉 + C++ 主控
# ============================================================

echo "========================================"
echo "  睿抗机器人 一键急停"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "========================================"

kill_by_pidfile() {
    local pidfile=$1
    local label=$2
    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile")
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "[STOP] 终止 $label (PID=$pid)..."
            kill "$pid" 2>/dev/null
            sleep 1
            if kill -0 "$pid" 2>/dev/null; then
                echo "[STOP] $label 未响应，强制终止 (SIGKILL)..."
                kill -9 "$pid" 2>/dev/null
            fi
            echo "[STOP] $label 已停止"
        else
            echo "[STOP] $label (PID=$pid) 已不在运行"
        fi
        rm -f "$pidfile"
    else
        echo "[STOP] 未找到 $label 的 PID 文件"
    fi
}

kill_by_pidfile /tmp/ruikang_vision.pid "Python视觉"
kill_by_pidfile /tmp/ruikang_cpp.pid   "C++主控"

# 兜底清理
echo "[STOP] 兜底检查..."
pkill -f "main_vision.py" 2>/dev/null && echo "[STOP] 兜底清理: main_vision.py" || true
pkill -f "RobotMain"       2>/dev/null && echo "[STOP] 兜底清理: RobotMain"       || true

echo "[STOP] 急停完成"
echo "========================================"
