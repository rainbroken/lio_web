# lio_web：通过 Web 网段实时显示 IteraLIO 点云

默认 `save_pcd.save_pcd_on_shutdown: false`：退出不导出 PCD，原始归档和评估文件仍保留。Web“保存点云”调用 `/lio/save_pcd`，有已提交优化结果时只保存优化完整点云，否则只保存原始完整点云；优化导出失败不回退为原始版本。扫描输入结束后需保持 LIO/Web 运行直至保存成功，退出全部节点后不能再通过 Web 保存。扫描中保存的是当前已归档前缀，导出占用处理锁，建议停止输入后保存。彩色文件分别为 `cloud_color_optimized.pcd` 或 `cloud_color.pcd`；此前手动保存的其他版本不会被删除。

联合启动启用 `optimized_dense_preview: true`：回环后台线程复用最终导出的逐帧位姿插值，只原子生成 `optimized_preview.json` 块清单，通过 `/lio/optimized_colored_archive` 发布时间戳与路径。清单包含 `colored_points.bin` 的不可变前缀、每帧局部包围盒和优化变换；原始点云保持局部坐标，不再为了预览生成完整优化 PCD 或 SQLite 点索引。Web 对局部包围盒变换后的八个角建立版本包围盒，概览只读取采样点，`/tile?...&generation=N` 筛选候选帧并按需变换、1 cm 去重，最多缓存 16 MiB 已编码块，过期版本返回 409。浏览器附近 27 块预加载总等待上限 8 秒，随后切换优化概览和可用细节，失败块继续补载。回环后每约 5 秒更新块清单以包含新增扫描；完整优化 PCD 仍在手动保存/退出保存时生成。旧 PCD 通知格式仍兼容。

边界：块以一帧扫描为单位，稀疏分布或大范围扫描的包围盒可能覆盖很多区域，某个细节请求仍需读取多个候选帧。`cpp_web` 的 C++ 读取器合并相邻采样，临时读取缓冲最多 64 KiB；采样间距超过 32 点时仍逐点 `pread`，避免为稀疏概览读取整份归档。细节处理每批最多 8192 点，不把完整归档放入内存。后续仍可建立局部子块索引或缓存跨版本采样点。该实现未验证 RK3576 性能，不保证彩色队列零丢帧。

成本与边界：Web 仍需读取清单、准备概览并按需读取归档，准备期间保持旧画面，因此不是零延迟。块清单链路不在 Web 中重写完整优化点云或建立全点 SQLite 索引；旧 PCD 通知链路仍需扫描文件建立磁盘索引。需要 LIVO 彩色归档、相机数据、优化结果和 Web 与 LIO 共享文件系统；没有彩色归档时应关闭该选项使用原优化快照。连续优化合并待处理预览任务。优化时的空间坐标修正沿用清单中的帧变换；索引和块缓存不会让全量地图常驻内存。最终前端密度仍受视野距离降采样和点数预算影响。

预览顶部显示 ScanForgeApp，左侧箭头通过 `scanforge://back` 请求 Android App 返回设备首页，不停止扫描。右上角齿轮打开设置抽屉：实时数据显示、轨迹开关、保存点云、保存视图；复位/网格/相机/全屏按钮在画布右下角。接收频率和传输带宽直接显示在实时数据中，不再提供速率图或单独开关。手机需安装 0.1.3 或之后的 App 才能处理网页返回箭头；独立浏览器使用浏览器返回按钮。

网页采用 `index.html` 工作台布局，实际运行页面是 `web/index.html`。实时数据来自
`/lio/global_map` 点云（无地图帧时回退到 `/lio/cloud_colored`）和 `/lio/odom` 里程计，显示扫描时长、累积里程、平移速度、原始点云消息点数、位置及 XY 轨迹。扫描时长从本次里程计首帧的 ROS 时间戳计算，回放暂停时不走时；里程计时间回退时重置。点云颜色、点大小、网格、视角和保存 PNG 都是浏览器本地显示功能；
页面默认隐藏坐标网格，右下角 XYZ 坐标轴随视角旋转；页面不会暂停或修改 ROS 扫描。没有 ROS 数据时对应指标保持等待状态，不显示演示数值。

优化显示模式在原始里程计时间回退超过 0.5 秒或里程计发布节点身份变化时清理上一轮预览缓存，恢复原始轨迹和地图接收，直到本轮产生优化结果。会话切换丢弃仍在后台编码的旧点云；仅重新连接手机不会删除当前地图或开启扫描。

这是一个 ROS 2 Humble C++17/Python 混合包，使用 `ament_cmake` 构建。优先显示 IteraLIO 的 `/lio/global_map` 彩色全局地图；地图暂时没有帧时自动显示 `/lio/cloud_colored` 的实时单帧点云。C++ 扩展完成体素采样、归档点变换、去重、读取和 `int16` 编码；Python 保留 ROS 订阅、任务调度、限频、缓存及 HTTP/WebSocket 服务。浏览器继续使用 JavaScript/WebGL。

## cpp_web 的处理链路与验证

`lio_web/cloud.py` 只将 PointCloud2 字段和原始字节缓冲交给 `src/native.cpp`，不在 Python 中遍历点，也不先复制整份 ROS 点云。实时概览、增量帧和 ROS 返回的细节块均走该入口。支持 PointCloud2 八种数值字段类型、RGB/RGBA 位字段、大小端及组织点云的行填充；截断记录和越界字段会拒绝。缺少 XYZ 的消息仍返回空结果。

`optimized_blocks.py` 保留清单校验、版本包围盒筛选及 16 MiB 编码缓存，逐点读取、变换、1 cm 去重和编码交给 C++。概览保留分批取消检查。旧 PCD 通知的 `optimized_tiles.py` 保留 NumPy/SQLite 索引构建，采样读取、细节读取和编码交给同一扩展。读取通过已打开的文件描述符，仍固定通知对应的快照/不可变前缀。

C++ 点处理和 I/O 期间释放 GIL；Python 每批接收 NumPy 数组，归档点以 float64 表示，因此批次内存比原始 20 字节点记录更大。扩展必须构建成功，不提供静默 Python 回退。显示点数预算、采样顺序、颜色与反射率规则、后台忙时跳帧、版本 409 和请求并发限制沿用原设计。实时点云仍以固定量化步长编码并钳制到 int16，超大范围可能饱和；优化归档概览会自动增大量化步长。迁移语言不会提高地图密度或修复坐标/时间同步问题。

2026-10-13 在本机 i7-1360P / x86_64 / Release 的合成规则网格测试，20 万输入点、概览上限 20 万、细节上限 3 万。与修改前 `e9a8d78` 比较，排除预热，取 3 次中位数；下列每组输入的二进制结果逐字节一致：

| 工作 | 原 Python | C++ 热点 | 加速比 |
| --- | ---: | ---: | ---: |
| 实时采样与编码 | 1037.15 ms | 25.87 ms | 40.09× |
| 优化块概览 | 95.30 ms | 12.33 ms | 7.73× |
| 优化细节块，绕过编码缓存 | 96.97 ms | 3.30 ms | 29.41× |
| 旧 PCD 概览 | 444.16 ms | 9.64 ms | 46.06× |
| 旧 PCD 细节块 | 49.38 ms | 3.39 ms | 14.58× |

同样设置下将输入增至 150 万点（概览预算仍为 20 万），实时采样/编码为 3201.16→166.51 ms，约 19.23×；优化块概览为 89.30→14.27 ms。抽样步长、点分布、重复率和缓存命中率会改变加速比。

这是合成数据和热文件缓存的局部耗时，不包含首次磁盘访问、PCD 索引构建、网络传输、浏览器绘制或 LIO 负载。没有真实扫描回放、RK3576 或 Android WebView 验证，不能将这些数字作为设备帧率或端到端提升。

构建并 source 安装环境后，在工作空间根目录复现：

```bash
colcon test --packages-select lio_web --event-handlers console_direct+
colcon test-result --test-result-base build/lio_web --verbose
python3 src/lio_web/scripts/benchmark_native.py --points 200000 --repeats 3
python3 src/lio_web/scripts/benchmark_native.py --points 1500000 --repeats 3
```

回归测试覆盖 Python 基准编码、采样上限、RGB/反射率、大小端/行填充、异常输入、归档读取、跨块变换与去重、快照版本、API 转发及合成 ROS→后台编码→WebSocket 发送。ROS 链路测试使用独立 DDS domain 177 和随机测试话题，不依赖扫描仪。

App API 实现已迁入 `lio_communicate`，默认端口 8766，合约见 `../lio_communicate/PROTOCOL.md`。
本服务默认端口仍为 8765，提供网页、点云和图像预览。`GET /api/device`、`GET /api/status`
仅兼容转发到配置项 `app_api_base_url`（默认 `http://127.0.0.1:8766`），不会生成另一份设备数据；
客户端认证头原样转发，通信服务未启动时返回 JSON 503。其他 `/api` 请求返回 JSON 410。
设备名称、型号、序列号配置现位于 lio_communicate。旧里程计时间跨度不再冒充任务时长，
任务生命周期未知时由通信服务返回 unknown。独立启动 lio_web 不会自动启动设备 API。


## 启动

在工作空间根目录 `/home/rain/ros_ws/handler_ws`，先安装依赖（新增 `pybind11-dev`、`python3-dev`；也可使用 `rosdep install --from-paths src/lio_web --ignore-src -r -y`）：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select lio_web \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch lio_web lio_web.launch.py
```

独立启动的 Web 参数放在 `src/lio_web/config/web.yaml`，联合启动使用
`src/lio_bringup/config/web.yaml`。例如修改 `max_points`
控制每次发送给浏览器的最多点数，修改 `voxel_size` 控制 Web 采样精度。
修改后重新启动 Web 服务即可；从源码修改配置后，首次需按上面的命令
使用 `--symlink-install` 构建。启动时的 `max_points:=...` 等参数仍可临时覆盖配置。
这些参数只影响 Web 展示和传输，不改变 LIO 的配准或地图存档。

从旧 `ament_python` 版本首次切换时，可给上述构建命令加 `--cmake-clean-cache`。Python/C++ 必须来自同一工作空间；只设置源码 `PYTHONPATH` 不能找到安装目录中的扩展。启动日志应显示 `Point-cloud backend: C++17 native kernels`。

例如更换端口：

```bash
ros2 launch lio_web lio_web.launch.py port:=8775
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

实时数据中的传输带宽按收到的有效载荷字节数除以实际采样时间计算，使用十进制 MB/s（1 MB = 1,000,000 字节），覆盖 WebSocket 点云、相机图像、遥测和 HTTP 地图块。它不包含协议开销、其他页面或其他应用流量，也不是网络带宽利用率百分比。概览点云接收频率以相邻帧的实际接收间隔计算，单位 Hz，不是雷达采集或屏幕刷新频率；持续无新帧后显示 0。地图增量和细节块计入带宽，不计入概览帧频率。

默认每帧最多 200,000 点，当前 `LIO3` 单点 10 字节（int16 XYZ + RGB + 反射率），加 24 字节头；完整帧约 2.0 MB。编码在后台线程的 C++ 扩展执行，忙时跳过新帧，多个浏览器客户端会各自占用一份带宽。迁移没有降低协议字节数，网络拥塞仍需调整点数/频率或传输策略。

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

协议使用带 RGB 和反射率的 `LIO3` 概览、`LID3` 增量和 `LIT3` 细节帧；浏览器继续兼容旧协议。点云不传 JSON/完整 ROS 消息。浏览器只依赖原生 WebGL；若要跨不可信网络使用，应在反向代理层增加认证和 HTTPS。保存点云接口仍调用既有 ROS 服务。
