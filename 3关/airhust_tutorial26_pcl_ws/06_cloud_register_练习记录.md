# 3.6 点云哪来的 练习记录

对应文档：`docs/06-cloud-register.md`
运行环境：Ubuntu / ROS Noetic / zsh，学生镜像仿真

## 一、任务目标

前面几关读的是 PCD 文件——固定、静态、已经处理好的点云。这一节回答一个问题：实际运行时，后面要订阅的 `/cloud_registered` **是谁发的、怎么来的**。

本节**不用写代码**，只做两件事：

1.  通过讲义搞清点云的来源；
2.  启动仿真，检查 `/cloud_registered` 的发布节点、消息类型、坐标系、发布频率。

## 二、点云的两条来源

前面几关：
- `loadPCDFile` 读一个文件，内容固定。

实际运行：
- 雷达持续扫描 → 驱动把每帧点发布到 ROS 话题，例如 `/livox/lidar`（原始点，雷达自身坐标系）。
- 我们处理时订阅的是 **`/cloud_registered`**——已经配准处理过的成品，消息类型 `sensor_msgs/PointCloud2`。

补两个底子：

- **话题（topic）**：ROS 里数据流动的管道，以 `/` 开头。一方发（发布者），一方读（订阅者），靠话题名对接。
- **`sensor_msgs/PointCloud2`**：ROS 标准点云消息，和 PCD 内容等价，但持续流动，还多带一个 `header`（时间戳 + `frame_id`）。

### 1. 实机：FAST-LIO / FAST-LIO2

输入：**雷达点云 + IMU**（惯性测量单元，测加速度和角速度，更新快、短时准）。

做三件事：

1. 估计雷达**位姿**（现在在哪、朝哪）；
2. **补偿扫描期间的运动影响**——扫一帧的工夫飞机在动，不校正的话同一面墙会被扫成斜线（deskew）；
3. 把当前帧的点**转换到参考坐标系**，发布到 `/cloud_registered`。

FAST-LIO2 是改进版。它不会自己跑，要**手动执行实机对应的启动脚本或 launch 文件**把它拉起来，等话题开始发布才能订阅。

> 讲义折叠块：作者先用 PCL 的 `ICP`（迭代最近点）、`NDT`（正态分布变换）自己配准原始点云，结果计算耗时较长，稀疏、带噪声的点云也经常对不齐，所以改用 FAST-LIO 处理原始数据，自己只订阅 `/cloud_registered`。这是背景，不是任务。

### 2. 仿真：tutorial_gazebo 的真值方案

讲义举的是本地 `catkin_ws/src/tutorials/tutorial_gazebo` 包：

- 节点 `cloud_register_mapper` 利用 **Gazebo 提供的模型真值**（仿真里知道每个模型的精确位置，等于开卷考试）处理雷达点云，发布 `/cloud_registered`。
- 该包的 `sim-new.launch` 已经包含这个节点，启动后直接订阅即可。

> 讲义折叠块：这个节点只是用真值里的位置给点坐标**加平移偏移**，`frame_id` 设成 `world`。完整的坐标转换**还包含旋转**——飞机转向后要结合雷达输出的坐标方向检查这一步，否则点会朝错方向。

**关键区分**：3.7 用的是学生镜像里的 `simulation` 包，属于**另一套仿真环境**。它的发布节点要看对应启动文件和 `rostopic info /cloud_registered` 的实际输出为准，别把 `tutorial_gazebo` 的节点名照搬过来。

### 3. 坐标系：`frame_id`

| 环境 | 坐标系 `frame_id` |
| --- | --- |
| 本地 FAST-LIO | `camera_init` |
| `tutorial_gazebo` 真值节点 | `world` |
| 学生镜像 `simulation` | 看实测消息头（本次是 `world`） |

rviz 里 **Fixed Frame 填消息头里实际的 `frame_id`**。如果填别的系，就得提供它与点云坐标系之间的 **TF 变换**（ROS 记录各坐标系相对位置的机制），否则点云不显示或位置错乱。

## 三、一帧点云 vs 多帧累计

这一节最核心的概念：

- 发布的 `/cloud_registered` **只包含当前帧**的点。飞机换一个位置，下一帧就是新位置观测到的点。
- 想把不同位置看到的场景**合成一份点云**：**转换到同一坐标系是合并前的准备**（配准已由上游做好），**把多帧点保存在一起才是累计**（下一节程序要做的）。
- 位姿和时间对应准确时，同一个静止物体在各帧的点才能互相重叠对齐；**移动物体**会在累计结果里留下多个时刻的位置，这是正常现象。
- 下一节订阅连续点云，把**累计 + 滤波 + 聚类**串成一个处理节点。

## 四、启动仿真与实测

### 启动命令

教程 3.7「启动仿真」给的命令：

```shell
roslaunch simulation sim.launch
```

启动前确认 `roscore` 在跑。若提示 `roscore cannot run as another roscore/master is already running`，说明已有 master 在运行，属正常状态，直接用即可，不用再起第二个。

仿真启动时会出现几条无害警告，不用管：

- `missing <robotNamespace>` → 自动设默认值
- `libcurl: (7) Couldn't connect to server` → 连不上外网下载模型，但已用本地 CSV 文件
- `context mismatch in svga_surface_destroy` → 图形渲染警告

### 检查话题的四条命令

```shell
rostopic info /cloud_registered
rostopic type /cloud_registered
rostopic echo -n 1 /cloud_registered/header
rostopic hz /cloud_registered
```

一条一条敲，每条单独按回车。环境已在 `~/.zshrc` 配好，不用每次手动 source。

### 我实测的结果

```
Type: sensor_msgs/PointCloud2
Publishers:
*/cloud_register_mapping(http://airhust:46061/)
Subscribers:
*/rostopic_219575_1795703391420(http://airhust:39277/)
seq: 30
stamp:
secs: 3
nsecs: 104000000
frame_id: "world"
```

```
rostopic hz /cloud_registered
subscribed to [/cloud_registered]
average rate: 10.173
min: 0.075s max: 0.118s std dev: 0.01502s window: 10
```

| 项目 | 结果 |
| --- | --- |
| 发布节点 | `/cloud_register_mapping` |
| 消息类型 | `sensor_msgs/PointCloud2` |
| frame_id | `world` |
| 发布频率 | 约 10 Hz（帧间隔 75~118 ms） |

`WARNING: may be using simulated time` 是仿真时间的正常提示，不用管。

发布节点 `cloud_register_mapping` + `frame_id: world`，说明这套仿真对应讲义里的**真值方案**（不是 FAST-LIO，FAST-LIO 那套用 `camera_init`）。所以 rviz 的 Fixed Frame 填 `world`。

## 五、rviz 查看

Add → By topic → PointCloud2，选 `/cloud_registered`，Fixed Frame 填 `world`。

## 六、小结

- 这一节不动代码，只认识点云来源：`/cloud_registered` 由仿真的发布节点持续发布，我们订阅成品。
- 实机的成品来自 FAST-LIO（雷达 + IMU 配准），仿真的成品来自真值节点（模型精确位置）。
- 每条消息只有当前一帧；累计要把多帧点转到同一坐标系后存到一起，这是下一节的事。
- 实测确认了本环境的发布节点、类型、坐标系、频率，为 3.7 的订阅程序提供参数。
