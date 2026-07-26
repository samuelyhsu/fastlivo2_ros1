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
# 改代码前：用指定数据源跑 3 轮 1x，生成基线轨迹 + 噪声阈值（约 3.5 分钟）
./test/regress.sh baseline dataset/livo_2026-07-25-22-50-36.bag -n mid360-52s

# 改代码后：跑 1 轮 5x 与基线对比（约 25 秒）
./test/regress.sh check

./test/regress.sh list                # 查看已有基线
./test/regress.sh check --rate 1      # 用 1x 对比
./test/regress.sh baseline <bag> -r 5 # 基线跑 5 轮
```

退出码：`0`=PASS `1`=FAIL `2`=环境错误 `3`=结果不可信（检测到积压）。

### 判据不是"位姿完全不变"

FAST-LIVO2 的输出**不可复现**：[vio.cpp](src/FAST-LIVO2/src/vio.cpp#L1631) 用了
`#pragma omp parallel for reduction(+ : error, n_meas)`，4 线程浮点归约的合并顺序
run-to-run 不固定，而 `error` 参与迭代收敛判断，某帧跨过阈值即翻转、之后一路发散。
实测同倍速重跑两次，末帧就差约 5cm、最大偏差约 18cm。

所以脚本比的是"偏差是否落在噪声基线内"。基线阶段跑 N 轮两两对比测出噪声，
乘安全系数 2 得到阈值；判定用位置 RMSE、最大偏差、最大姿态偏差三项
（末帧偏差方差过大，只打印不判定）。

### 两个必须知道的限制

**测不出精细调参的差异。** 实测把 `point_filter_num` 2→4（点云降采样加倍）
或 `max_iterations` 5→1，产生的轨迹差异都小于噪声底噪，工具报 PASS；
而外参平移错 2 米能稳定报 FAIL。它用于捕捉"改坏了"级别的回归。

**FAIL 会自动复测一次。** 算法的不确定性是重尾的——13 轮实测中有 1 轮偏差达中位数的
18 倍，且 1x/5x 都会出现。单轮误报率约 8%，故两轮都 FAIL 才判定回归。
同理，若基线里混进这样一轮，阈值会被抬高一个数量级，脚本会检测并提示重建基线。

5 倍加速回放本身不影响结果：实测 5x 与 1x 的偏差分布完全重叠（5x 中位 0.124m、
1x 中位 0.145m），排空耗时均为 ~0.01s 无积压，有效帧数完全一致。
