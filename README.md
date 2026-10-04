# FASTLIO2 ROS2

本工作区维护的 fork：https://github.com/EleSheep-moving/FASTLIO2_ROS2 。
AIMSRacer 的 FAST-LIO 修改应提交和推送到此 fork；`origin` 的 fetch/push 均指向它，
主仓库 `.gitmodules` 也使用此地址。原始上游为 https://github.com/liangheming/FASTLIO2_ROS2 。

## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

## 详细说明
### 1.编译 LIVOX-SDK2
```shell
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### 2.编译 livox_ros_driver2
```shell
mkdir -r ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 3.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### OpenMP 点云匹配

`fastlio2` 默认 `MP_EN=ON`、`MP_PROC_NUM=2`，ARM 和 x86 使用相同的显式配置。
启用时必须找到 OpenMP，编译器和链接器通过 `OpenMP::OpenMP_CXX` 获得支持，
避免配置显示开启但编译器实际忽略并行指令。
匹配标志使用独立字节 `std::vector<std::uint8_t>`，避免 `vector<bool>` 位压缩导致
不同点的并发写入互相覆盖。数组在并行循环前分配，循环结束后串行汇总约束。

```shell
colcon build --symlink-install --packages-select fastlio2 --cmake-args -DMP_EN=ON -DMP_PROC_NUM=2
```

本地验证覆盖相邻标志交错写入、存储字/线程分块边界、odometry 和可选输出。
这些新增测试与实验程序未随本次提交发布。线程数增加是否改善整帧耗时，
需要在实际工作负载下测量。

旋转测量雅可比使用 IMU 坐标下的点 `R_il * p_lidar + t_il`，旧公式的
`t_wi` 会使收敛依赖地图原点。在两线程、迭代上限 10 的独立真实运动扫描
对照中，修正后平均迭代由 5.57 降为 3.37，核心处理均值由 30.80 ms 降为
19.44 ms；逐点匹配占核心耗时约 88%–91%。这些结果不包含 ROS 排队、发布
及全栈负载，不能据此解释事故中的全部累计延迟。

`ieskf_max_iter: 5` 当前仍未传给 `IESKF::setMaxIter`，实际默认上限是 10。
此项接线问题尚未修改。更新源码后需要重新构建，源码修正不会自动进入旧二进制。

### 点云输出开销

LIO 优先发布 odometry，再处理可选输出。`publish_body_cloud` 默认启用，以保留
PGO/ICP 的输入；`publish_world_cloud`、`publish_path` 默认关闭。两个点云输出均
在坐标变换之前检查开关和订阅者，无订阅者就不分配、变换、序列化输出点云。
ROS 参数可覆盖配置 YAML，启动后只读。仅需 odometry 时可同时关闭两个点云开关。

world cloud 启用后默认最多 5 Hz、2000 点，采用等间隔抽取，不增加体素滤波计算。
源扫描时间戳和内部估计点云保持不变。body cloud 保留原始频率与密度；不要在使用
PGO/localizer 时关闭它。可选路径最多 1 Hz、1000 个位姿。RViz 的点云显示默认关闭。

主仓库的 `fastlio_rear.yaml` 默认关闭 world cloud；已知地图一致性检查使用
同时间戳的 body cloud 和原始 LIO odometry。world cloud 仅用于可选可视化。

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数，由于ROS2中的timer、subscriber以及service的回调实际上运行在同一个线程上，在电脑性能不是好的时候，会出现调用阻塞的情况，建议使用线程并发的方式将耗时的回调独立出来(如timerCB)来提升性能
