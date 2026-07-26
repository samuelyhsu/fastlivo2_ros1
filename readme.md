# FAST-LIVO2 ROS1

包含所有依赖，基于简答智能硬件复现FAST-LIVO2算法，适用于ROS1系统

## 安装官方驱动

- 激光雷达：livox mid360s 测试时官方sdk还没有支持，暂使用内置修改版本
- 工业相机：机器视觉工业相机客户端MVS（Linux） https://www.hikrobotics.com/cn/machinevision/service/download/?module=0

## 运行

设备供电

```bash
# windows 执行，将相机usb转移到wsl中
usbipd list
usbipd bind --busid 1-19
usbipd attach --wsl --busid 1-19
```

```bash
# 查看设备 确认相机usb已经转移过来
lsusb
```

```bash
# 运行激光雷达节点
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

```bash
# 运行相机节点
source devel/setup.bash
roslaunch mvs_ros_pkg mvs_camera_trigger.launch
```

```bash
# 运行算法
source devel/setup.bash
roslaunch fast_livo mapping_mid360.launch
```

```bash
# 录制数据
source devel/setup.bash
rosbag record -O dataset/mid360s_mvs_$(date +%F_%H-%M-%S).bag --lz4 /livox/lidar /livox/imu /left_camera/image
# 回放数据
rosbag play dataset/livo_2026-07-25-22-50-36.bag
# 查看话题频率
rostopic hz /livox/lidar /livox/imu /left_camera/image

# 数据可视化
roslaunch rosbridge_server rosbridge_websocket.launch
```

## 回归测试

验证改动前后算法效果有没有变化。

```bash
# 改代码前：用指定数据源跑 3 轮 1x，生成基线轨迹 + 阈值（约 3.5 分钟）
./test/regress.sh baseline dataset/livo_2026-07-25-22-50-36.bag -n mid360-52s

# 改代码后：跑 1 轮 8x， 与基线对比
./test/regress.sh check -n mid360-52s --rate 8

./test/regress.sh list                     # 查看已有基线
./test/regress.sh check -n 名字 --rate 1    # 用 1x 对比
./test/regress.sh baseline <bag> -r 5      # 基线跑 5 轮
```

退出码：`0`=PASS `1`=FAIL `2`=环境错误 `3`=结果不可信（检测到积压）。

### 一套数据一套基线

`-n` 名字对应 `test/baseline/名字/` 一个目录，多套基线可并存：

```bash
./test/regress.sh baseline dataset/livo_2026-07-25-22-50-36.bag -n mid360-52s
./test/regress.sh baseline dataset/avia_office.bag -n avia-office \
  --cfg avia.yaml --cam-cfg camera_pinhole.yaml

./test/regress.sh check -n avia-office     # 数据与配置自动跟着基线走
```

check 只给基线名，不给 bag：bag 路径、`--cfg`、`--cam-cfg` 都记在基线的
`meta.json` 里，脚本据此回放对应数据、加载对应配置（显式传 `--cfg/--cam-cfg`
可覆盖）。bag 的字节数与各话题消息数也会校验，换过数据会直接报"数据源不一致"。

只有一套基线时 `-n` 可省略；有多套时不指定会报错并列出可选名字。

### 判据就是"位姿完全不变"

同一份数据重复运行，输出**逐字节一致**，所以判据是精确相等：基线阶段跑 N 轮两两
对比测出的噪声为 0，乘安全系数后阈值仍是 0，任何位移都会被判为回归。

实测（3 轮 1x + 1x/5x/8x 各一轮 check）：位置 RMSE、最大偏差、最大姿态偏差
全部为 0.0000，轨迹文件 `cmp` 逐字节相同。回放倍速不影响结果，8x 下有约 1.2s
排空但处理的帧完全一致。OMP 线程数也不影响：`OMP_THREAD_LIMIT=1` 压到单线程
（computeJacobian 0.0015s vs 4 线程 0.0006s）跑出的轨迹与 4 线程基线完全相同。

判定用位置 RMSE、最大偏差、最大姿态偏差三项。末帧偏差（位置与姿态）只打印不判定
——它恒 ≤ 对应的最大偏差，能报的问题最大偏差都能报。

### 闭环误差（只看不判）

测试数据通常绕一圈回到起点，而 FAST-LIVO2 的世界系锚定在首帧，所以末点离原点多远、
姿态偏了多少，直接就是这一趟的累计漂移。baseline 和 check 都会多打一张表：

```text
Loop closure   (not judged, this is algorithm accuracy)
  metric                       measured   baseline
  Distance from origin [m]       0.0358     0.0358
  Rotation from origin [deg]     1.7785     1.7785
```

**不参与成败判定**：它衡量的是算法精度，与本工具要回答的"改动有没有改变行为"是两个
问题——拿它做判据只会把算法本来就有的漂移误报成回归。想看改动让精度变好还是变坏时，
对比这两列即可。中位轨迹的值也记进 `meta.json` 的 `closure` 字段。

若某条轨迹的首点离原点超过 5cm，会额外提醒一句：那说明它不是从原点起步的，
这两个数就不能当闭环误差看。

> **这份可复现性是 2026-07-26 修掉两个未定义行为之后才有的**，此前同倍速重跑两次
> 末帧就差约 5cm、最大偏差约 18cm。两个 bug 都来自上游 FAST-LIVO2：
>
> 1. [vio.cpp `updateState`](src/FAST-LIVO2/src/vio.cpp#L1677) 越界读图像缓冲区。
>    算雅可比的梯度模板最远取到 `±5*scale`（`scale = 1 << (level + search_level)`，
>    最大 32），而选点时的 `border` 按 `(patch_size_half+1) << patch_pyrimid_level`
>    = 80 算，少算了 `search_level` 这一层。靠近图像边缘的点因此读到 `img.data`
>    分配范围之外的相邻堆内存，每次运行内容都不同。全程约 0.7% 的迭代命中。
> 2. [vio.cpp `updateReferencePatch`](src/FAST-LIVO2/src/vio.cpp#L1080) 用未初始化
>    的值算 NCC。`mean_` 是跨帧缓存，但原代码只在未命中时给局部变量 `ref_mean` /
>    `other_mean` 赋值，命中时它们是未初始化的栈值却照样参与运算（每帧约 1400 次）。
>    NCC 决定 `pt->ref_patch` 选谁，直接影响后续光度误差。
>
> 曾被归咎的 `#pragma omp parallel for reduction(+ : error, n_meas)` 不是原因：
> 单线程重跑两轮照样分叉，而修完两个 UB 后 4 线程完全可复现。`error` 只参与
> `error <= last_error` 的收敛判断，状态更新走的 `z` / `H_sub` 按下标写入本身确定。
> 该归约在原理上仍是个刀尖——若换机器后出现无法解释的偏差，它是第一个怀疑对象。

### 一个必须知道的限制

**测不出精细调参的差异——这条已过期，需重测。** 它是在"噪声底噪约 3cm"的前提下
测出来的：当时把 `point_filter_num` 2→4 或 `max_iterations` 5→1，轨迹差异小于
底噪，工具报 PASS。现在底噪是 0，任何真实改动都会被捕捉到，该结论大概率不再成立。
