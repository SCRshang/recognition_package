# awakelion_sentry

> [!NOTE]
>
> This repository is based on `ROS2`, and current branch is `humble`.

## Usage

### build

```shell
git clone --recursive https://github.com/AwakeLion-Robot-Lab/awakelion-sentry-ros2/tree/humble 
colcon build --symlink-install
```

### submodule update

```shell
git submodule update
```

## Hierarchy

```shell
├── sentry_auto_aim
├── sentry_bringup
├── sentry_decision
├── sentry_description
├── sentry_localization
├── sentry_navigation
└── sentry_postprocessing
```

### sentry_auto_aim

* auto aim package based on `rm_vision` and nerual network.

### sentry_bringup

* bringup  package includes param configuration, launch file and so on.

### sentry decision

* decision package based on [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) ([BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2) is not included).

### sentry_description

* robot description via `xacro` and broadcast static TF accordingly.
* integrate sensors and their information to broadcast TF and publish odometry, pointcloud information.

### sentry_localization

* open source LIO algorithm [Point-LIO](https://github.com/hku-mars/Point-LIO), this package is forked from PolarBear Team [point_lio repo](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/641424bf9d924f5ba0bd87ca9d91a4f148384925).

### sentry_navigation

* communication like topic communication to other ros2 nodes and serial communication to hardware.
* elevation filter algorithm to select non-ground pointcloud via normal of points (TBD).
* customized plan planner algorithm which is capable to omni kinematics model (TBD).
* pcd file handler to generate map file or transformed pcd file.

### sentry_postprocessing

#### fake_vel_transform

* `cmd_vel` publisher package to correct `cmd_vel` while gimbal is spinning. This package is from PolarBear Team [fake_vel_transform package](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/tree/main/fake_vel_transform) with few changes.

> 1. remove `cmd_spin_topic`.
> 2. publish `cmd_vel` with refined `linear/x` and `linear/y`, without `angular/z`.
>
> these changes is capable for `omni` kinematics model.

#### patchworkpp

* an efficient pointcloud segmentation package forked from

#### pointcloud_registration

* a package include pointcloud registration algorithm like customized ICP, [small_gicp](https://github.com/koide3/small_gicp), [KISS_Matcher](https://github.com/MIT-SPARK/KISS-Matcher/).

> [!NOTE]
>
> these algorithm is from aw_algorithm_utils (TBD), which is an integrated and flexible package.




# 醒狮-哨兵

> [!笔记]
>
> 此分支用于`ROS2-humble`, 目前, `non-ROS` 版本待定:).

## 食用指南

### 构建项目

```shell
git clone --recursive https://github.com/AwakeLion-Robot-Lab/awakelion-sentry-ros2/tree/humble 
colcon build --symlink-install
```

### 子模块更新

```shell
git submodule update
```

## 目录结构

```shell
├── sentry_auto_aim
├── sentry_bringup
├── sentry_decision
├── sentry_description
├── sentry_localization
├── sentry_navigation
└── sentry_postprocessing
```

### sentry_auto_aim（哨兵自瞄）

* 自瞄包是基于 `rm_vision` 和 “神经网络”的.

### sentry_bringup（哨兵配置）

* 此目录包含参数配置、启动文件等.

### sentry decision（哨兵决策）

* 决策包基于：[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (但它并不包含：[BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)).

### sentry_description（哨兵描述）

* 通过`xacro`对机器人进行描述，并相应地，传递静态的TF坐标变换信息。
* 集成传感器及其信息以推送TF坐标信息，并发布里程计、点云信息。

### sentry_localization（哨兵定位）

* 开源LIO算法[Point-LIO](https://github.com/hku-mars/Point-LIO)。
* 这个包是从PolarBear团队的[point_lio repo](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/641424bf9d924f5ba0bd87ca9d91a4f148384925)里分支而来的。

### sentry_navigation（哨兵导航）

* 与其他ros2节点的主题通信和与硬件的串行通信等通信。
* 通过点法线（TBD）选择非地面点云的高程滤波算法。
* 定制计划器算法（TBD）。

### sentry_postprocessing（哨兵后处理）

### dependencies包
* 暂时没有什么东西...

#### fake_vel_transform

* `cmd_vel` 发布者包，用于在万向轮旋转的时候纠正 `cmd_vel`。 这个包来自PolarBear团队的[fake_vel_transform package](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/tree/main/fake_vel_transform)，有一点改动。

> 1. 移除了 `cmd_spin_topic`.
> 2. 发布 `cmd_vel` ，其中包含改进后的 `linear/x` 和 `linear/y`,没有 `angular/z`.
>
> 这些变化适用于 `全向` 运动学模型。

#### patchworkpp（点云切割）

* 一个高效的点云分割包。

#### pointcloud_registration（点云配准）

* 包含点云配准算法的包，例如，特定的ICP, [small_gicp](https://github.com/koide3/small_gicp), [KISS_Matcher](https://github.com/MIT-SPARK/KISS-Matcher/).

> [!NOTE]
>
> 这些算法来自aw_algorithm_utils（TBD），它是一个集成和定制的包。
