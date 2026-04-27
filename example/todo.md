4.24-4.25:
1\M:
(已完成)在 Gitee 上创建团队代码仓库，把 V1.0 的空目录结构完完整整地传上去，让 Y 和 X Clone 下来在里面填空。
（已完成）引入 `nlohmann/json.hpp` 库，写好 `udp_receiver.cpp`。在自己电脑上写个 Python 脚本疯狂给它发 UDP 数据，测试 C++ 接收多线程会不会卡死，数据解析是否正确。
（已完成）写好状态机的基类和 `state_01_init.cpp`、`state_02_start_obs.cpp` 两个最基础的状态切换逻辑。

2\Y：
（已完成）搭建vs code代码环境
（已完成）搭建好 OpenCV 环境。写好读取本地视频/图片的测试代码。
（已完成）写出 `line_tracker.py`，实现读取一张带有黑线的图片，能稳定输出黑线中心偏离图像中心的像素差（负数偏左，正数偏右）。
（已完成）写好 `udp_sender.py`，用 Python 自己发给自己测试，确保 JSON 格式完全符合 V1.0 协议

3\X：
（已完成）研读宇树 SDK2 的文档或官方 Example，找到对应“控制移动(Move)”、“跳跃(Jump)”和“阻尼模式/急停”的 API。
（已完成）在本地电脑上搭好 `cpp_core` 的目录，把 `CMakeLists.txt` 写好，确保能通过 `cmake ..` 和 `make` 编译出可执行文件（输出一句 Hello World也OK）。
（已完成）编写 `robot_driver.cpp` 的骨架，把 SDK 的调用代码填进去（有报错没关系，先理清逻辑）
