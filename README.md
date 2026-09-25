# lio_web：通过 Web 网段实时显示 IteraLIO 点云

网页采用 `index.html` 工作台布局，实际运行页面是 `web/index.html`。实时数据来自
`/lio/global_map` 点云（无地图帧时回退到 `/lio/cloud_colored`）和 `/lio/odom` 里程计，显示扫描时长、累积里程、平移速度、原始点云消息点数、位置及 XY 轨迹。扫描时长从本次里程计首帧的 ROS 时间戳计算，回放暂停时不走时；里程计时间回退时重置。点云颜色、点大小、网格、视角和保存 PNG 都是浏览器本地显示功能；
页面默认隐藏坐标网格，右下角 XYZ 坐标轴随视角旋转；页面不会暂停或修改 ROS 扫描。没有 ROS 数据时对应指标保持等待状态，不显示演示数值。

这是一个 ROS 2 Humble Python 包。优先显示 IteraLIO 的 `/lio/global_map` 彩色全局地图；地图暂时没有帧时自动显示 `/lio/cloud_colored` 的实时单帧点云。服务在 ROS 机上完成体素采样、限频和 `int16` 量化，再用同一个 HTTP/WebSocket 端口提供浏览器页面。

## 启动

在 `/home/rain/ros_ws`：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_web
source install/setup.bash
ros2 launch lio_web lio_web.launch.py
```

独立启动的 Web 参数放在 `src/lio_web/config/web.yaml`，联合启动使用
`src/lio_bringup/config/web.yaml`。例如修改 `max_points`
控制每次发送给浏览器的最多点数，修改 `voxel_size` 控制 Web 采样精度。
修改后重新启动 Web 服务即可；从源码修改配置后，首次需按上面的命令
使用 `--symlink-install` 构建。启动时的 `max_points:=...` 等参数仍可临时覆盖配置。
这些参数只影响 Web 展示和传输，不改变 LIO 的配准或地图存档。

也可以直接运行仓库中的脚本；脚本不依赖当前工作目录，按 `Ctrl+C` 或关闭脚本时会清理它启动的 ROS 节点：

```bash
/home/rain/ros_ws/lio_web/launch.sh
```

启动参数会透传给 ROS launch，例如更换端口：

```bash
/home/rain/ros_ws/lio_web/launch.sh port:=8766
```

在同一 Web 网段的电脑浏览器访问 `http://<ROS主机IP>:8765/`。查看 ROS 主机地址：

```bash
ip -br addr
```

如果 LIO 发布的是其他话题，例如只看去畸变帧：

```bash
ros2 launch lio_web lio_web.launch.py \
  pointcloud_topic:=/lio/cloud_deskewed odometry_topic:=/lio/odom \
  max_points:=12000 rate_hz:=3 voxel_size:=0.20
```

## 带宽和速率

默认每帧最多 200,000 点，单点 9 字节，加 24 字节头；完整帧约 1.8 MB。编码在后台线程执行，忙时跳过新帧，避免点云编码阻塞里程计回调。多个浏览器客户端会各自占用一份带宽。

要查看全局地图，LIO 的 `debug.publish_global_map` 必须启用，并按机器性能设置 `preview.global_map_max_points`；当前快照频率是 0.2 Hz，约每 5 秒一帧。地图帧暂不可用时，Web 自动显示彩色单帧点云，页面的数据源会标出当前话题。网页对地图使用 15 秒无新帧提示，避免把正常发布间隔当作断流。里程计 `/lio/odom` 是另一条数据链路。

参数含义：

* `max_points`：每帧点数上限，先增大 `voxel_size` 再增大此值；
* `pointcloud_topic` / `fallback_pointcloud_topic`：优先点云及回退点云话题；将回退话题设为空字符串可关闭回退；
* `rate_hz`：服务端限频，慢客户端不会积压历史帧，只显示最新帧；
* `voxel_size`：米，决定体素去重和量化步长，也决定精度；
* `trajectory_max_points`：Web 轨迹点数上限，默认 2000。轨迹每移动至少 0.05 m 记录一点；超出上限时保留最近四分之一的点，并逐步抽稀旧轨迹，优先保留转弯。轨迹只存于 Web 进程内，重启后重新开始；
* `bind_host`：默认 `0.0.0.0` 允许 Web 网段访问；只本机调试可设 `127.0.0.1`。

常用局域网配置：

```bash
ros2 launch lio_web lio_web.launch.py max_points:=10000 rate_hz:=3 voxel_size:=0.20
```

协议使用带 RGB 三通道的 `LIO2` 二进制帧，不传 JSON/完整 ROS 消息；浏览器仍兼容旧的灰度 `LIO1` 帧。浏览器只依赖原生 WebGL；若要跨不可信网络使用，应在反向代理层增加认证和 HTTPS。当前服务只提供只读点云，不暴露 ROS 参数或命令接口。
