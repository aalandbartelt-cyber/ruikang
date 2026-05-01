# ==========================================
# RobotControlCore - 睿抗多模态巡检主控
# ==========================================
cmake_minimum_required(VERSION 3.10)
project(RobotControlCore CXX)

# ==========================================
# C++ 标准：C++17（向前兼容 C++14）
# ==========================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==========================================
# 编译选项：调试符号 + 优化 + 严格警告
# ==========================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g")   # Release 也保留 -g，便于现场 gdb

# ==========================================
# 寻找宇树主 SDK（不寻找不规范的独立 DDS 包）
# ==========================================
find_package(unitree_sdk2 REQUIRED)

# ==========================================
# 头文件目录
# ==========================================
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${UNITREE_SDK2_INCLUDE_DIRS}
    /usr/local/include/ddscxx           # CycloneDDS C++ binding
)

# ==========================================
# 编译时打印关键诊断信息（现场报错快速定位）
# ==========================================
message(STATUS "================================================")
message(STATUS "[Build] Project: ${PROJECT_NAME}")
message(STATUS "[Build] CXX Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "[Build] Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "[Build] Unitree SDK Include: ${UNITREE_SDK2_INCLUDE_DIRS}")
message(STATUS "[Build] CycloneDDS Include: /usr/local/include/ddscxx")
message(STATUS "================================================")

# ==========================================
# 自动递归搜索源文件
# ==========================================
file(GLOB_RECURSE SRC_FILES 
    "src/*.cpp"
)

set(SOURCES
    ${SRC_FILES}
    main.cpp
)

# 打印参与编译的文件（5.1 现场如果忘记加新文件能立刻发现）
message(STATUS "[Build] Source files:")
foreach(SRC ${SOURCES})
    message(STATUS "    ${SRC}")
endforeach()
message(STATUS "================================================")

# ==========================================
# 可执行文件
# ==========================================
add_executable(RobotMain ${SOURCES})

# ==========================================
# 链接库
# ==========================================
target_link_libraries(RobotMain
    pthread
    unitree_sdk2
    ddscxx
    ddsc
)

# ==========================================
# 安装规则（可选，方便 deploy）
# ==========================================
install(TARGETS RobotMain
    RUNTIME DESTINATION bin
)