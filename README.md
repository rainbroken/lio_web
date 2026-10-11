# lio_web：通过 Web 网段实时显示 IteraLIO 点云

联合启动启用 `optimized_dense_preview: true`：回环后台线程复用密集彩色归档的逐帧位姿修正，原子生成 `cloud_color_optimized.pcd`，通过 `/lio/optimized_colored_archive` 发布时间戳与路径（仅文件通知，不是第二份全量点云话题）。Web 固定该文件 inode，后台建立磁盘 SQLite 块索引，概览限 `max_points`；`/tile?...&generation=N` 按需读取 5 m 块并进行 1 cm 去重，过期版本返回 409。浏览器预加载当前视野中心附近 27 块，使用两路请求和 `tile_draw_max_points` 预算，随后在同一事件循环内切换概览、细节和轨迹。预加载失败保留旧版本。

成本与边界：优化会重读/写一次密集归档并建立磁盘索引，优化快照准备期间保持旧画面，因此不是零延迟。需要 LIVO 彩色归档、相机数据、回环成功和 Web 与 LIO 共享文件系统；没有彩色归档时应关闭该选项使用原优化快照。后台只在捕获归档边界时持写入锁，坐标修正阶段允许继续追加扫描。连续优化合并待处理索引任务。优化时的空间坐标修正沿用离线 PCD 的关键帧插值算法；索引和块缓存不会让全量地图常驻内存。最终前端密度仍受视野距离降采样和点数预算影响。

预览顶部显示 ScanForgeApp，左侧箭头通过 `scanforge://back` 请求 Android App 返回设备首页，不停止扫描。右上角齿轮打开设置抽屉：实时数据显示、接收速率图、轨迹开关、保存点云、保存视图；复位/网格/相机/全屏按钮在画布右下角。手机需安装 0.1.3 或之后的 App 才能处理网页返回箭头；独立浏览器使用浏览器返回按钮。

网页采用 `index.html` 工作台布局，实际运行页面是 `web/index.html`。实时数据来自
`/lio/global_map` 点云（无地图帧时回退到 `/lio/cloud_colored`）和 `/lio/odom` 里程计，显示扫描时长、累积里程、平移速度、原始点云消息点数、位置及 XY 轨迹。扫描时长从本次里程计首帧的 ROS 时间戳计算，回放暂停时不走时；里程计时间回退时重置。点云颜色、点大小、网格、视角和保存 PNG 都是浏览器本地显示功能；
页面默认隐藏坐标网格，右下角 XYZ 坐标轴随视角旋转；页面不会暂停或修改 ROS 扫描。没有 ROS 数据时对应指标保持等待状态，不显示演示数值。

优化显示模式在原始里程计时间回退超过 0.5 秒或里程计发布节点身份变化时清理上一轮预览缓存，恢复原始轨迹和地图接收，直到本轮产生优化结果。会话切换丢弃仍在后台编码的旧点云；仅重新连接手机不会删除当前地图或开启扫描。

这是一个 ROS 2 Humble Python 包。优先显示 IteraLIO 的 `/lio/global_map` 彩色全局地图；地图暂时没有帧时自动显示 `/lio/cloud_colored` 的实时单帧点云。服务在 ROS 机上完成体素采样、限频和 `int16` 量化，再用同一个 HTTP/WebSocket 端口提供浏览器页面。

App 可通过同一端口的 `GET /api/device` 和 `GET /api/status` 连接。接口返回 JSON；状态中的时长和地图点数取自现有 ROS 遥测，扫描生命周期返回 `unknown`，不会把收到消息推断为扫描任务正在运行。`device_name`、`device_model`、`device_serial` 是可配置的设备标识；电量、版本、温度等没有真实来源的字段返回 null。`capabilities` 明确声明扫描控制和任务管理尚未接入，相关接口返回 JSON 501。部署后重启 lio_web，再在 App 中重新连接；只更新 APK 无法给旧设备服务增加接口。


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

联合启动的 Web 配置默认订阅 `/lio/optimized_map`。首个回环优化前显示原始 `/lio/global_map` 点云和 `/lio/odom` 轨迹；收到优化路径后显示 `/lio/optimized_path`，收到优化地图快照后切换到 `/lio/optimized_map` 并清除原始地图缓存。该优化点云只有 XYZ 和 intensity，没有彩色归档的 RGB；离线保存的 `cloud_color_optimized.pcd` 是单独的产物。将 `src/lio_bringup/config/web.yaml` 中 `pointcloud_topic` 改为 `/lio/global_map` 可始终查看原始地图和里程计轨迹。

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
