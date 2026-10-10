# LIO 接收与串行处理

`lio_node` 使用一个 ROS 接收线程和一个长期运行的 `lio_worker`。接收回调只校验
IMU 数值、时间戳并入队；点云转换、排序、IMU 配包、去畸变、滤波、地图和发布都
在工作线程执行。工作线程通过条件变量等待输入，退出时先唤醒，再 join，最后释放
估计器。OpenMP 点匹配和 ikd-tree 自身重建线程沿用原来的配置。

输入使用 reliable；LiDAR depth 10，IMU depth 与 `imu_queue_capacity` 一致（默认 4096）。`body_cloud`、里程计、TF 的源时间戳、坐标系和
点云密度沿用原行为；初始化期间仍不发布定位输出。可选输出开关、节流及算法
迭代上限不因这次线程修改改变。

## 输入边界

以下参数可写在配置 YAML 或通过 `--ros-args -p` 覆盖，运行后只读。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `lidar_queue_capacity` | 2 | 待处理扫描上限，包含正在转换或等待 IMU 的候选 |
| `imu_queue_capacity` | 4096 | 未消费 IMU 上限及 DDS 接收深度；应用缓冲超限报错退出 |
| `timing_trace_path` | 空字符串 | 可选 CSV 路径；默认关闭 |

第三个待处理扫描到达时，默认删除全部旧候选，只保留新扫描。已进入核心处理的
扫描会完成；随后用保留的完整 IMU 历史处理最新候选。队列内身份验证与 IMU 提取
在同一把锁下完成，被删除的候选不能消费 IMU。大点云释放和转换在锁外执行。
上限约束的是应用层待处理队列，DDS 还会保留各自接收深度以内的输入消息，不能把应用队列容量当成整个进程的消息上限。

重复源时间戳会计数并拒绝；源时间倒退、非有限 IMU 或容量超限会记录
原因并以非零状态退出。空扫描、`point_num` 不匹配及非法转换点云会跳过且不消费
IMU。退出必须等待正在执行的核心结束，因此不保证任意地图规模下的固定退出时限。
LiDAR 可以跳帧，但定位在真实持续过载下的精度仍需要单独评估。

不再限制相邻 IMU 消息的最大时间间隔，也不因该间隔退出。Livox 驱动未做
PTP/GPS 同步时可能使用主机处理数据包的时间戳，接收积压可形成长间隔和随后
的短间隔。配包仍等待最新 IMU 时间达到扫描结束时间，保留跨越跳帧的全部
IMU 历史；处理器沿用上一帧边界 IMU 进行积分和去畸变。真实采样缺失仍可能
影响定位精度，输入新鲜度和定位可信度由车辆定位链路独立判断。

Humble 的纯接收器暂停实验复现了 IMU DDS depth 10 下恢复后停流；同一程序只
把 IMU depth 改为 4096 后完整接收了源消息。扩大 IMU 接收深度与应用传播缓存
一致，用于减少接收侧丢失；完整恢复仍依赖源发布者的历史与实际中间件配置。
整进程暂停后的输入交付可能滞后于进程恢复；扩大接收深度不提供固定恢复时限。

## 调试与验证

启用计时路径前应创建其父目录。计时记录最多 65536 个事件，超限数量写为
`trace_dropped`；接收和工作线程只写内存，join 后才输出 CSV。关闭时不分配记录
缓冲。记录包含源扫描身份、接收/转换/IMU 就绪/核心/输出时刻、队列深度、核心耗时、
实际迭代数及点数。节点退出打印接收、重复、跳帧、无效、消费和处理计数。

```bash
colcon build --packages-select fastlio2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select fastlio2 --ctest-args -R 'test_(imu_deskew|input_buffer|timing_trace)' --output-on-failure
colcon test-result --verbose
```

真实 ROS 生命周期测试需在独立 ROS domain 中运行，且已 source ROS、Livox 消息
及 fastlio2 的 overlay：

```bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 python3 src/FASTLIO2_ROS2/fastlio2/test/test_lio_runtime.py \
  --binary install/fastlio2/lib/fastlio2/lio_node \
  --config src/FASTLIO2_ROS2/fastlio2/config/lio.yaml -v
```

队列并发测试可以独立使用 TSan；数值和生命周期测试可使用 ASan/UBSan 构建。
回放比较应保持扫描身份、全部 IMU、配置、编译选项和输出订阅者一致。有跳帧时
需要以同一批选中扫描的串行结果为参照，不能把与全扫描轨迹的差异当作线程误差。
