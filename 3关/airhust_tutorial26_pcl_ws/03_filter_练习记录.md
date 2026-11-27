# 3.3 滤波 练习记录

对应源码：`src/tutorial_pcl_basic/src/03_filter.cpp`
运行环境：Ubuntu 20.04 / ROS Noetic / PCL，zsh

## 一、任务目标

`demo_room_noisy.pcd` 是在 `demo_room.pcd` 上加了冗余和噪声得到的：每个点复制三份并加 2mm 抖动，另撒了 4000 个随机点和 800 个从雷达位置往外散的离群点。

要在 `03_filter.cpp` 里补全三个函数，走完这条流水线：

```
读点云 → 体素降采样 → 统计去噪 → 取关心的区域 → 存点云
```

## 二、补全内容

三个处理函数统一模式：`setInputCloud` → 设参数 → `filter(output)`，结果直接写入调用方提供的 `output`，函数结尾不再赋值覆盖。

### 1. `downsample` —— VoxelGrid 降采样

```cpp
pcl::VoxelGrid<pcl::PointXYZI> grid;
grid.setInputCloud(cloud);
grid.setLeafSize(leaf, leaf, leaf);   // leaf = 0.05 m
grid.filter(output);
```

把空间切成 5cm 的格子，每格用重心作代表点。输入是 `PointXYZI` 点云，所以模板参数必须写 `PointXYZI`，不能写 `PointXYZ`——两者是不同的类型，`setInputCloud` 不接受。

### 2. `denoise` —— StatisticalOutlierRemoval 去噪

```cpp
pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
sor.setInputCloud(cloud);
sor.setMeanK(mean_k);                 // 20
sor.setStddevMulThresh(stddev);       // 1.0
sor.filter(output);
```

对每个点统计 20 个邻居的平均距离，超过全局均值 1.0 个标准差的点判为离群点删掉。

### 3. `region` —— CropBox 区域裁剪

```cpp
pcl::CropBox<pcl::PointXYZI> crop;
crop.setInputCloud(cloud);
crop.setMin(Eigen::Vector4f(1.8f, 1.8f, 0.60f, 1.0f));
crop.setMax(Eigen::Vector4f(3.5f, 4.5f, 1.20f, 1.0f));
crop.filter(output);
```

只保留落在桌子和桌上零件所在盒子里的点。`Vector4f` 第四个分量是齐次坐标占位，固定填 `1.0`。

## 三、运行结果

命令：`./run 3`

```
原始点数        : 342000
按体素大小合并重复点
体素降采样后    : 91064
按邻居距离去除离群点
统计离群点去除后: 90311
存到 .../out/03_denoised.pcd
按给定范围提取区域
区域内点数      : 3982
存到 .../out/03_region.pcd
```

| 步骤 | 实测点数 | 讲义示例 | 说明 |
| --- | --- | --- | --- |
| 原始 | 342000 | 342000 | 加了冗余和噪声的数据 |
| 体素降采样 | 91064 | 91064 | 精确一致 |
| 统计去噪 | 90311 | 90311 | 精确一致 |
| 区域裁剪 | 3982 | 3987 | 差 5 个点 |

输出文件：

- `out/03_denoised.pcd`
- `out/03_region.pcd`

复核命令（可选）：

```shell
pcl_viewer data/demo_room_noisy.pcd out/03_denoised.pcd
```

叠着看，远处散在空中的点消失，墙面和桌面保留。

## 四、结果分析

三个步骤的点数变化符合预期：

- 降采样从 342000 降到 91064，去掉了约 73% 的冗余点。原始点云里每个点被复制三份并加了 2mm 抖动，这些点仍落进同一个格子，合并后自然大幅减少。
- 去噪又删掉 753 个点，对应讲义里撒进去的随机点和离群点。它们周围没有邻居，平均距离明显偏大。
- 区域裁剪后只剩 3982 个点，把地面、天花板和远处墙面都排除了。

区域点数比讲义示例少 5 个（3982 vs 3987），属于正常波动：不同 PCL 版本对恰好落在边界上的点判定可能不同，`setMin` / `setMax` 传的是 `float`，边界点受浮点精度影响。相对误差约 0.13%，不影响结果。

## 五、遇到的问题

### 代码上的三个错误

1. `output.height 0= 1;` —— 多了个字符 `0`，语法错误。改成 `output.height = 1;`。
2. `VoxelGrid<pcl::PointXYZ>` 和 `StatisticalOutlierRemoval<pcl::PointXYZ>` —— 输入是 `PointXYZI`，模板参数不匹配，编译报错。都改成 `PointXYZI`。
3. `filter(output)` 之后又留了 `output = *cloud;`，把刚算好的结果整体覆盖回原样。降采样函数没有这句所以正常，去噪和区域裁剪有这句，点数会退回上一步。两行都删掉。

关于第 3 点：`output = *cloud;` 在原骨架里是占位语句，作用是让 TODO 没做时程序也能跑通。一旦写了真正的 `filter(output)`，它就成了覆盖语句，必须删掉。判断标准很简单——函数里有没有真正产生结果的代码。

### `readCloud` 里 empty 检查的位置

把 `if (output.empty())` 放在函数开头是错的：此时 `output` 是刚创建的空点云，检查必然成立，程序还没读文件就抛异常退出。要放在清洗完 NaN/Inf 之后，检查"是不是全被删光了"。

### 环境配置：zsh 要 source setup.zsh

报错：

```
devel/setup.bash:.:8: no such file or directory: /home/.../airhust_tutorial26_pcl_ws/setup.sh
rospack find tutorial_pcl_basic  →  package not found
```

原因是 shell 用的是 zsh，却 source 了 bash 专用的 `devel/setup.bash`。该脚本内部用 `${BASH_SOURCE[0]}` 定位自己，这个变量在 zsh 里为空，于是它把当前目录当成了脚本目录，去找工作区根目录的 `setup.sh`（不存在），环境变量没设上，`rospack` 自然找不到包。

解决：

```shell
source devel/setup.zsh
```

并把 `~/.zshrc` 里的 `setup.bash` 改成 `setup.zsh`，以后新开终端自动生效。讲义 `docs/index.md` 里本来写的就是 `source devel/setup.zsh`。

## 六、拓展问题：改变体素边长后，参数还是同一组吗

讲义的拓展问题问：把 `leaf` 从 5cm 改成 10cm 或 2cm，`meanK` 和 `stddev` 还用 20 和 1.0 吗？

结论：**不是同一组，`meanK` 比 `stddev` 更需要跟着调。**

### 为什么 meanK 要变：它定的是"个数"，不是"距离"

`VoxelGrid` 输出的不是原始点，而是每个格子的重心。格子边长一变，输出点云的密度就变了：

| leaf | 格子体积（相对 5cm） | 点数 | 点距 |
| --- | --- | --- | --- |
| 2 cm | 1/8 | 显著增多 | 约 2 cm |
| 5 cm | 1（基准） | 91064 | 约 5 cm |
| 10 cm | 8 倍 | 约 1/8 | 约 10 cm |

而 `meanK = 20` 固定的是邻居**数量**，不是物理半径。密度一变，这 20 个邻居铺开的空间范围完全不同：

- **点稀疏时（10cm）**：20 个邻居要跑到很远才够数，平均距离可能跨过了桌面和地面，不再代表"局部表面"，表面点和离群点的距离差异被抹平，去噪灵敏度下降。应**减小 `meanK`**（10~15）。
- **点密集时（2cm）**：20 个邻居挤在很小一块区域，全落在同一个平面上，平均距离小、涨落也小，阈值变紧，容易把正常点误删。应**增大 `meanK`**（30~50）。

一个直观换算：`meanK` 个邻居大致分布在半径 `r ≈ (3·meanK / 4π·密度)^(1/3)` 的球里。密度随 `leaf³` 反比变化，想让 `r` 保持不变，`meanK` 就得按密度反比调整。实际不必这么极端，因为离群点判定看的是相对差异，但方向是对的：`leaf` 变大就减小 `meanK`，`leaf` 变小就增大 `meanK`。

### 为什么 stddev 也要重新看

`stddev` 是"均值 + N 倍标准差"的阈值，是**相对量**，对密度变化有一定自适应性——这正是它比绝对距离阈值好的地方。但它建立在 `meanK` 算出的平均距离分布合理的前提下：

- `meanK` 太小 → 每个点的平均距离本身波动大 → 标准差大 → 阈值宽松 → 删不动
- `meanK` 太大 → 邻居跨表面 → 分布混乱 → 该删的没删

所以调参顺序是：**先根据 `leaf` 定 `meanK`，再从 `stddev = 1.0` 起调**，观察删掉的点数和 `pcl_viewer` 里的效果。

### 一个容易忽略的连锁效应

`leaf` 变大时，`VoxelGrid` 会在降采样阶段就把一部分离群点"洗掉"——离群点若和表面点落在同一个 10cm 格子里，求重心时就被平均掉了。`leaf = 2cm` 时网格细，离群点更容易完整保留到去噪阶段。

也就是说，不同 `leaf` 下喂给 `StatisticalOutlierRemoval` 的输入本身就不同（离群点数量和分布都变了），参数更不能照搬。

### 小结

讲义给的 `leaf = 0.05 / meanK = 20 / stddev = 1.0` 是**配套的一组**，针对当前点密度调出来的。改 `leaf` 就得重新调后面两个。判断标准不是点数好不好看，而是 `pcl_viewer` 里该保留的零件还在、该删的散点没了。

## 七、小结

`Filter<PointT>` 和 `FilterIndices<PointT>` 的调用方式一致，四个滤波类都是"设置输入、设置参数、计算输出"三步。模板参数必须和点云类型一致，`filter(output)` 是写入型接口，结尾不要再有赋值覆盖。

参数不是固定的：`leaf` 决定保留多少细节，`mean_k` 和 `stddev` 决定去噪力度。换一份数据要重新调参数，先看点数变化，再看该保留的物体还在不在。
