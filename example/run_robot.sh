#!/bin/bash
# ============================================================
# 睿抗机器人 一键启动脚本
# 同时启动 Python 视觉（main_vision.py） + C++ 主控（RobotMain）
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VISION_DIR="$SCRIPT_DIR/vision_py"
CPP_BUILD_DIR="$SCRIPT_DIR/cpp_core/build"
CPP_BINARY="$CPP_BUILD_DIR/RobotMain"

echo "========================================"
echo "  睿抗机器人 一键启动"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "========================================"

# ---- 1. 检查 / 编译 C++ 可执行文件 ----
if [ ! -f "$CPP_BINARY" ]; then
    echo "[BUILD] C++ 可执行文件不存在，开始编译..."
    cd "$SCRIPT_DIR/cpp_core"
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    if [ ! -f "$CPP_BINARY" ]; then
        echo "[ERROR] 编译失败，请检查 CMake 输出"
        exit 1
    fi
    echo "[BUILD] 编译完成"
    cd "$SCRIPT_DIR"
fi

# ---- 2. 启动 Python 视觉进程 ----
echo "[START] 启动 Python 视觉..."
cd "$VISION_DIR"
python3 main_vision.py > /tmp/ruikang_vision.log 2>&1 &
VISION_PID=$!
echo "[START] 视觉 PID: $VISION_PID  (日志: /tmp/ruikang_vision.log)"

# 等待视觉初始化完成
sleep 2

# ---- 3. 启动 C++ 主控 ----
echo "[START] 启动 C++ 主控..."
cd "$SCRIPT_DIR"
"$CPP_BINARY" > /tmp/ruikang_cpp.log 2>&1 &
CPP_PID=$!
echo "[START] 主控 PID: $CPP_PID  (日志: /tmp/ruikang_cpp.log)"

# ---- 4. 写入 PID 文件 ----
echo "$VISION_PID" > /tmp/ruikang_vision.pid
echo "$CPP_PID"   > /tmp/ruikang_cpp.pid

echo "========================================"
echo "  机器人已启动"
echo "  视觉 PID : $VISION_PID"
echo "  主控 PID : $CPP_PID"
echo "  停止脚本 : ./stop_robot.sh"
echo "  视觉日志 : tail -f /tmp/ruikang_vision.log"
echo "  主控日志 : tail -f /tmp/ruikang_cpp.log"
echo "========================================"

# 等待任意子进程退出
wait -n 2>/dev/null
echo ""
echo "[WARN] 有进程异常退出！请检查日志："
echo "  cat /tmp/ruikang_vision.log"
echo "  cat /tmp/ruikang_cpp.log"
