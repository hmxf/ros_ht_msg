# Version Log

源项目地址：https://github.com/dslab-agrobot/AgRobot2
本项目地址：https://github.com/hmxf/ros_ht_msg

工作概要：
整合了 https://github.com/dslab-agrobot/AgRobot2/src/software/ros_ht/src 下的绝大多数文件，添加了开发相关的所有文档资料。

具体内容：

- 经对比，ros_ht_msg 包无任何实质性修改，因此本次工作基于原始 ros_ht_msg 包，添加了多个平台下的最新版 libcontrolcan.so 和 controlcan.h 并使用软链接将其添加到原本所在路径。其中 controlcan.h 无架构依赖，libcontrolcan.so 需要根据 CPU 架构手动修改软链接指向的目标，目前的源码中默认指向 aarch64 架构。

- test 包中包含的内容均为测试脚本，因此本工作将其移动至 example 目录下并修改了文件名称以突出其示例作用。

- pub_sim 包中包含的 pub_ht_move.py 脚本被保留并移动至 scripts 目录下，其余文件由于其功能为滑台控制，与底盘控制无关，因此从本项目中移除。
