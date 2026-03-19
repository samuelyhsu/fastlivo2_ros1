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
