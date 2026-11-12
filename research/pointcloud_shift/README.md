# ROS1/ROS2 点云域偏移研究起点（独立于赛车主链）

> 状态：实验性研究基线（2026-11）。与孙宏宇老师公开的 3D 点云开放环境感知、
> 检索增强、测试时适应研究主题存在技术交集；**本仓没有实现或复现 Point-Cache**。
> 合成指标只能说明实验脚本可用，绝不是现实精度证据。

## 目标与边界

对现有 src/sensors/lidar_cluster 的 PCL 预处理/聚类提出可量化的问题：
在相同场景下施加点丢失、XYZ 噪声、远距稀疏化后，候选锥桶的检出率和中心误差
如何变化？与一个冻结的参考特征缓存比较，几何特征距离如何变化？

研究脚本**不订阅或发布 ROS topic，不改动 C++ 生产感知、规划、控制、安全、赛道几何或 #92 门禁**。
实验中的空间哈希聚类和阈值分类器是简单基线，**不是已有 PCL 实现的等效替代**。
所有代码放在 research/pointcloud_shift/，不进入 colcon 构建。

## 不需要 ROS2 rosbag

有 ROS1 .bag 即可：用第三方 rosbags 的 AnyReader 直接读
/velodyne_points (sensor_msgs/PointCloud2) 的 XYZ，转换为研究用 JSONL。
无需 ROS 1/2 安装。读取器支持 x/y/z FLOAT32/FLOAT64、大小端及行填充；
自定义消息能否读取视 bag 中类型定义而定。

原始 bag 和研究导出**始终保留在本机**，data/、results/ 和 *.jsonl 被忽略。
没有在 GitHub 伪造用户实车数据，也没有声称已经读取真实 bag。

### 0. 没有任何数据时，先跑合成基线

从仓库根目录运行（Python >= 3.10，纯标准库）：

~~~bash
python3 -m unittest discover -s research/pointcloud_shift/tests -v
python3 research/pointcloud_shift/run.py synthetic \
  --seed 42 --frames 12 \
  --out research/pointcloud_shift/results/synthetic_seed42.json
~~~

会执行四种场景：clean、xyz_noise（高斯坐标扰动）、dropout（随机丢点）、
far_sparse（远距更稀疏）。合成场景有 **3 个锥桶、地面和一根干扰柱**，
因此可计算一对一匹配 TP/FP/FN、Precision、Recall、F1 与中心误差。
每次评测使用确定的 seed；冻结缓存由**独立随机种子**的干净帧建立，不使用测试标签。

mean_nearest_reference_distance 是手工几何特征
（log(点数)、高度、二维半径）到最近参考簇的加权距离。
**缓存只做检索诊断，不修改分类、不更新模型；它不是测试时自适应性能指标。**

### 1. 从 ROS1 .bag 导出 XYZ（不需要 ros2 bag）

~~~bash
python3 -m pip install 'rosbags>=0.11,<1'
python3 research/pointcloud_shift/rosbag_to_frames.py \
  --bag "/path/to/your_2025_track.bag" \
  --topic /velodyne_points \
  --max-frames 100 --step-frames 3 --max-points 2500 \
  --out research/pointcloud_shift/data/ros1_velodyne.jsonl
~~~

默认每 3 帧保留 1 帧，各帧最多保留 2500 个 XYZ 点，这是初期诊断限制；
等间隔采样可能有空间偏差，**不用于正式密度/精度结论**。正式实验需要
保留完整帧并预注册合理的采样协议。

若 bag 内找不到目标话题，程序会列出可用 topic；可用 --topic 改为实际
PointCloud2 话题。它不会自动把 /INS/ASENSING_INS 等自定义消息转换成 ROS2 消息。
每行记录 frame_id/timestamp_ns/source_topic/sensor_frame/points，
不包括原始强度、ring、逐点时间、TF 或位姿；**不能代替高保真传感器日志**。

### 2. 对未标注真实帧做只读诊断

~~~bash
python3 research/pointcloud_shift/run.py bag \
  --input research/pointcloud_shift/data/ros1_velodyne.jsonl \
  --support-frames 3 \
  --out research/pointcloud_shift/results/ros1_diagnostics.json
~~~

只输出每帧点数、簇数量、几何候选数量、参考特征距离。
**无人工真值标签绝不输出 Precision / Recall / F1**。车在运动时同一锥桶
在 LiDAR 坐标系的位置不断变化，参考缓存距离不是已校正的域偏移量；
它也会同时反映场景变化，不能解释为传感器退化。

### 3. 如确实必须在 ROS2 中回放

可以另外转换原始 bag，不需要改动本分支代码：

~~~bash
rosbags-convert --src "/path/to/your_2025_track.bag" --dst "/path/to/ros2_bag"
ros2 bag info /path/to/ros2_bag
~~~

转换前备份原始 bag；未索引的 ROS1 bag 需先重建索引。
自定义消息、TF、INS 话题、QoS 等仍需逐项核对，
**容器格式转换不等于 ROS2 闭环语义正确**。

参考：
- https://ternaris.gitlab.io/rosbags/topics/rosbag1.html
- https://ternaris.gitlab.io/rosbags/topics/convert.html

## 后续真正的研究路线（以下均未实现）

1. **数据协议**：拿到 ROS1 bag 的 /velodyne_points、标定和 INS/TF；
   人工标注小样本锥桶中心；划分 train/support/test，防止测试泄漏。
2. **公平基线**：实测现有 lidar_cluster 的 PCL 检测输出，并统一标注、
   滤波、坐标系、精度、召回率、时延口径。
3. **检索增强与测试时自适应**：先复现
   https://github.com/auniquesun/Point-Cache 的原始设置，再研究点云编码器、
   缓存更新、置信度和缓存污染控制；当前 FeatureMemory 只是最简数据接口原型。
4. **闭环验证**：回接 ROS2 前需独立测试数据、消融、漏检/误检、延迟预算、
   失效回退；不直接改变 /sensors/cones/raw 等已有契约。

**当前验收范围**：离线纯 Python 单测、合成数据 CLI、ROS1/ROS2 PointCloud2
离线读取接口、严格的未标注诊断报告。无实际 bag 时不能声称通过真实数据验收。
