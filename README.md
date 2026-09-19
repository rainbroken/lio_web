# lio_web：通过 Web 网段实时显示 IteraLIO 点云

这是一个 ROS 2 Humble Python 包。它订阅 IteraLIO 的 `sensor_msgs/PointCloud2`，在 ROS 机上完成体素采样、限频和 `int16` 量化，再用同一个 HTTP/WebSocket 端口提供浏览器页面。

## 启动

在 `/home/rain/ros_ws`：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_web
source install/setup.bash
ros2 launch lio_web lio_web.launch.py
```

在同一 Web 网段的电脑浏览器访问 `http://<ROS主机IP>:8765/`。查看 ROS 主机地址：

```bash
ip -br addr
```

如果 LIO 发布的是其他话题，例如只看去畸变帧：

```bash
ros2 launch lio_web lio_web.launch.py \
  pointcloud_topic:=/lio/cloud_deskewed max_points:=12000 rate_hz:=3 voxel_size:=0.20
```

## 带宽和速率

默认每帧最多 20,000 点，单点 7 字节，加 24 字节头，约 137 KiB/帧；5 Hz 时约 5.6 Mbit/s（WebSocket/TCP 额外开销未计）。实际带宽可直接看页面的帧大小和帧率。建议预留链路带宽的 30% 以上，多个浏览器客户端会各自占用一份带宽。

参数含义：

* `max_points`：每帧点数上限，先增大 `voxel_size` 再增大此值；
* `rate_hz`：服务端限频，慢客户端不会积压历史帧，只显示最新帧；
* `voxel_size`：米，决定体素去重和量化步长，也决定精度；
* `bind_host`：默认 `0.0.0.0` 允许 Web 网段访问；只本机调试可设 `127.0.0.1`。

常用局域网配置：

```bash
ros2 launch lio_web lio_web.launch.py max_points:=10000 rate_hz:=3 voxel_size:=0.20
```

协议使用 `LIO1` 二进制帧，不传 JSON/完整 ROS 消息。浏览器只依赖原生 WebGL；若要跨不可信网络使用，应在反向代理层增加认证和 HTTPS。当前服务只提供只读点云，不暴露 ROS 参数或命令接口。
