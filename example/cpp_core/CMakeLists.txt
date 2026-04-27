# C++ 编译配置
cmake_minimum_required(VERSION 3.10)
project(RobotControlCore)

# 设置 C++ 标准为 C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# ==========================================
# 只寻找宇树主 SDK，放弃寻找不规范的 DDS 包
# ==========================================
find_package(unitree_sdk2 REQUIRED)

# ==========================================
# 🌟 暴力包含头文件：直接指向你 find 出来的路径！
# ==========================================
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UNITREE_SDK2_INCLUDE_DIRS} 
    /usr/local/include/ddscxx     # 👈 核心救命代码：强制告诉编译器去这里找 DDS
)

# 自动递归搜索 src 目录下所有的 .cpp 文件
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

set(SOURCES 
    ${SRC_FILES}
    main.cpp
)

# 生成可执行文件
add_executable(RobotMain ${SOURCES})

# ==========================================
# 链接库 
# ==========================================
target_link_libraries(RobotMain 
    pthread 
    unitree_sdk2                  
    ddscxx      # 👈 直接链接 DDS 动态库名称
    ddsc        # 👈 直接链接 DDS 动态库名称
)