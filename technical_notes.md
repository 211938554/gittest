- [<center> 前言 / Preface </center>](#center-前言--preface-center)
- [<center> ROS </center>](#center-ros-center)
  - [ROS TF/TF2 相关](#ros-tftf2-相关)
    - [Transformations - Overview](#transformations---overview)
    - [Transformations - APIs](#transformations---apis)
    - [Coordinate Frame Conventions](#coordinate-frame-conventions)
    - [Publish Static Transforms](#publish-static-transforms)
    - [常见坐标系 `base_link`, `odom`, `map` 等](#常见坐标系-base_link-odom-map-等)
- [<center> Sensor输入 </center>](#center-sensor输入-center)
  - [Realsense SDK](#realsense-sdk)
    - [Realsense Ros](#realsense-ros)
      - [Realsense Ros 基本信息](#realsense-ros-基本信息)
    - [Realsense SDK 底层功能 APIs](#realsense-sdk-底层功能-apis)
      - [图像、深度图滤波](#图像深度图滤波)
      - [T265 Mapping and Localization](#t265-mapping-and-localization)
    - [Realsense Multi-Cam 多相机同步](#realsense-multi-cam-多相机同步)
  - [RGB-D 相机 (Realsense D435)](#rgb-d-相机-realsense-d435)
    - [2020.02.02 RGB-D 相机对齐的检查](#20200202-rgb-d-相机对齐的检查)
    - [2020.01.29 Realsense 深度图配准的一些额外参考](#20200129-realsense-深度图配准的一些额外参考)
    - [2020.01.28 深度图配准原理/综述](#20200128-深度图配准原理综述)
  - [双目相机 (Realsense T265 & Zed2)](#双目相机-realsense-t265--zed2)
  - [激光雷达](#激光雷达)
  - [Intel Realsense Depth 系列 (D435, T265, SR305)](#intel-realsense-depth-系列-d435-t265-sr305)
  - [IMU (Intel Realsense D435i and T265)](#imu-intel-realsense-d435i-and-t265)
    - [IMU 读数值](#imu-读数值)
- [<center> 编程规范 / Programming Guide </center>](#center-编程规范--programming-guide-center)
  - [Python](#python)
- [<center> 数据集 / Dataset </center>](#center-数据集--dataset-center)
    - [2021.02.07 Image Segmentation 数据集及算法性能比对](#20210207-image-segmentation-数据集及算法性能比对)
    - [2021.02.03 RGB-D 扩展数据集](#20210203-rgb-d-扩展数据集)
    - [2021.01.16 立体匹配, 深度重建 相关数据集(及算法性能对比)](#20210116-立体匹配-深度重建-相关数据集及算法性能对比)
- [<center> 模块: 点云处理 </center>](#center-模块-点云处理-center)
  - [基于点云的 SLAM 流程, Kinect Fusion 及相关开源](#基于点云的-slam-流程-kinect-fusion-及相关开源)
  - [PCL (Point Cloud Library) 点云滤波相关](#pcl-point-cloud-library-点云滤波相关)
  - [PCL (Point Cloud Library) 点云分割/聚类相关](#pcl-point-cloud-library-点云分割聚类相关)
    - [2020.02.03 算法原理和论文](#20200203-算法原理和论文)
    - [2020.02.02 点云分割实践](#20200202-点云分割实践)
  - [Open3D 及 Python Package](#open3d-及-python-package)
    - [Overview](#overview)
- [<center> 模块: 图像分割、平面辅助 </center>](#center-模块-图像分割平面辅助-center)
  - [图像分割](#图像分割)
    - [2020.02.07 Combination of Graphcut and Superpixel](#20200207-combination-of-graphcut-and-superpixel)
    - [2020.02.05 Following: 学术大牛的主页， 论文(及代码)](#20200205-following-学术大牛的主页-论文及代码)
    - [2020.01.27 基于 OpenCV 的已有实现(Graph-Cut, SuperPixel)](#20200127-基于-opencv-的已有实现graph-cut-superpixel)
    - [2020.01.20 DPPTAM 的实现参考 (图像分割辅助稠密重建)](#20200120-dpptam-的实现参考-图像分割辅助稠密重建)
- [<center> 模块: 深度图 / 视差图 处理 </center>](#center-模块-深度图--视差图-处理-center)
  - [深度图补全](#深度图补全)
    - [2020.01.28 入门](#20200128-入门)
- [<center> 模块: CV基础 </center>](#center-模块-cv基础-center)
  - [BELID：增强的高效的局部图像描述符](#belid增强的高效的局部图像描述符)
- [<center> 其它 / Miscs </center>](#center-其它--miscs-center)
  - [YAML 读写](#yaml-读写)
- [<center> The End </center>](#center-the-end-center)
----

# <center> 前言 / Preface </center> #

----

# <center> ROS </center> #

----

## ROS TF/TF2 相关

----
关于 tf 配置的一些配置， 了解一下

**<font color="ff7500">注意:</font>**
当前 Koda 中使用的是 tf ， 而非 tf2， 请注意一下


### Transformations - Overview
参见 [ros wiki, overview/transformations](http://wiki.ros.org/tf/Overview/Transformations)， 里面再次讲解了一些 Transform, Frame 的概念， 待核对

以下基础概念， **<font color="ff7600">非常重要</font>**

**Frames and Points**
> A frame is a coordinate system. <font color="0000ff">Coordinate systems in ROS are always in 3D, and are right-handed, with X forward, Y left, and Z up</font>.
> 
> Points within a frame are represented using tf::Point, which is equivalent to the bullet type btVector3. The coordinates of a point p in a frame W are written as Wp.

**Frame Poses**

**Frame poses as Point Mappings**
以上两项先参考下图， 后续再整理。 总结来说， 与我们之前在 SLAM 中的理解一致， $T_A^B$ 既可表示 pose 姿态， 也表示从 A 系到 B 系的坐标变换

![figure08](technical_notes_attachment/figure08.ros_frame_and_poses.png)


**Transforms基础**:
tf 中的一些概念澄清， 参见 [blog, ROS中TF(坐标系转换)原理与使用](https://www.cnblogs.com/sxy370921/p/11726691.html)
- <font color="ff7500">注意区分坐标系变换 (frame transform) 和 坐标变换 (coordinate/coefficents transform)</font>。后面我们统一用以下术语:
  - `frame transform`: 等价于 `coordinate transform, bias transform` 基变换， 坐标系变换 这几个术语， <font color="ff7500">通常用 $B$ 矩阵表示</font>
  - `coordinate transform`: 等价于 `coefficients transform, data transform` 坐标变换 这几个术语， <font color="ff7500">通常用 $T$ 矩阵表示</font>
- `parent`、`child` frame是在描述<font color="0000ff">坐标系变换(frame transform)/基变换</font>时的概念，<font color="0000ff">`parent` 是原坐标系， `child` 是变换后的坐标系。 通常用 $B$来表达坐标系变换(frame transform)/基变换， 为 $B_{parent}^{child}$
- `source_frame`, `target_frame` 是对应我们熟悉的坐标变换(coordinate transform)来说， 即 $T_{source\_frame}^{target\_frame}$</font>，这个时候这个变换描述的是坐标系变换，也是 `source_frame` 坐标系在 `target_frame` 坐标系下的描述。
- 综上, 我们有如下对应关系。
  $$
  B_{parent}^{child} = T_{child}^{parent}
  $$


### Transformations - APIs
```cpp
void tf::TransformListener::lookupTransform (const std::string &target_frame, const std::string &source_frame, const ros::Time &time, StampedTransform &transform) const 
```
注意:
- 它返回的结果， 按照我们的习惯规范计法， 是 $T_{source}^{target}$， 即数据坐标从 `source frame--> target frame` 的坐标变换(coodinate/coefficients transform), 也是 `source frame` 的位姿在 `target frame` 中的表达
  > `lookupTransform()` is a lower level method which returns the transform between two coordinate frames.
  > Which if applied to data, will transform data in the source_frame into the target_frame. See [geometry/CoordinateFrameConventions#Transform_Direction](http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction)
- 从 ros 比较拗口的 坐标系变换 (frame transform)的角度来说，是反的。 **这个所谓的坐标系变换 (frame transform)<font color="0000ff">对应向量几何中的基变换， 对应 $B_{target}^{source}$</font>**
  > <font color="ff00ff">The direction of the transform returned will be from the target_frame to the source_frame</font>. 


### Coordinate Frame Conventions

接上节， `lookupTransform()` 的返回值为 `geometry_msgs/TransformStamped` 格式， 如下
> 参见 [ros wiki, Coordinate Frame Conventions](http://wiki.ros.org/geometry/CoordinateFrameConventions)
```
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
```

关于这个 message, Transform Direction 分析如下:
- 按照[上节](#overview---transformations), 从坐标变换(coodinate transform)的角度来说， `lookupTransform(target_frame, source_frame)` 的返回值为我们习惯记法的$T_{source}^{target}$， 也即 `source frame` 在 `target frame` 中的姿态(pose)表达
- 那么从ros 拗口的 坐标系变换(frame transform) / 或者基变换的角度来说, 即表达从 `target_frame` 倒 `source_frame` 的基变换
- 而 `geometry_msgs/TransformStamped中的 header.frame_id 以及 child_frame_id`， 按照 ros 拗口的说法， 是从坐标系变换(frame transform) 的角度来表达的， 表达的是**从 `header.frame_id` 到 `child_frame_id` 的<font color="ff6d00">`frame transform` $B^{child\_frame\_id}_{header.frameid}$， 等价于 `coordinate transform` $T_{child\_frame\_id}^{header.frameid}$</font>**
  > Transforms are specified <font color="ff00ff">in the direction such that they will transform the **coordinate frame** "frame_id" into "child_frame_id"</font> in tf::StampedTransform and [geometry_msgs/TransformStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html). 
- 这样子， 综合上面的信息， 可知:  
  `header.frame_id == target_frame`
  `child_frame_id == source_frame`
  > This is <font color="ff00ff">the same transform which will take data from "child_frame_id" into "frame_id".</font>

### Publish Static Transforms
参见: [ros wiki, static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher)

**IMPORTANT!!!** 
- 在 `static_transform_publisher` 中填写的值， 注意是 <font color="0000ff">坐标变换 (coordinate transform)</font>， 是从 `frame_id --> child_frame_id`的基变换， 从另一个角度说， <font color="0000ff">是坐标变换 $T_{child\_frame\_id}^{frame\_id}$ </font>, 
  ```python
  ## Publish a static coordinate transform(from child_frame_id to frame_id) to tf2 using an x/y/z offset in meters and yaw/pitch/roll in radians. (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X).
  static_transform_publisher x y z yaw pitch roll frame_id child_frame_id

  ## Publish a static coordinate transform(from child_frame_id to frame_id) to tf2 using an x/y/z offset in meters and quaternion.
  static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
  ```
- 使用示例
  ```
  <launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="1 0 0 0 0 0 1 link1_parent link1" />
  </launch>
  ```
- 佐证， 参见 [ros answers, static transformer question](https://answers.ros.org/question/253178/static-transformer-question/)
  如下配置， 表达 $T_{camera\_link}^{base\_link}$, 也即 `camera_link` 在 `base_link` 中的姿态表达
  ```
  node pkg="tf" type="static_transform_publisher" name="base_to_kinect_broadcaster" args="-0.17 0.04 0.1975 0 0 0 /base_link /camera_link 100" />
  ```
  > You need to identify the origin of the camera wrt to the base_link. So (x,y,z) is the origin of the camera's coordinate system in base_link coordinate system.
  > <font color="0000ff">Eg: If (0,0,0) in camera's coordinate system is (10,-2,5.5) in the base_link coordinate system, then (x,y,z) is (10,-2,5.5)</font>. Same with r,p,y
  
### 常见坐标系 `base_link`, `odom`, `map` 等
先参见
- **IMPORTANT**: [blog, ROS里基本坐标系的理解：map,odom,base_link,base_laser ](https://blog.csdn.net/flyinsilence/article/details/51854123)
- reference: [blog, ROS中odom、map坐标系的理解](https://blog.csdn.net/u012686154/article/details/88174195)

最常用的就是map，odom，base_link，base_laser坐标系，这也是开始接触gmapping的一些坐标系。
- map: 地图坐标系
  > 顾名思义，一般设该坐标系为固定坐标系（fixed frame），一般与机器人所在的世界坐标系一致。
- base_link:机器人本体坐标系
  > 与机器人中心重合，当然有些机器人(PR 2)是base_footprint,其实是一个意思。
- odom：里程计坐标系
  > 这里要区分开odom topic，这是两个概念，一个是坐标系，一个是根据编码器（或者视觉等）计算的里程计。
  但是两者也有关系，odom topic 转化得位姿矩阵是odom-->base_link的tf关系。
- 这时可有会有疑问，odom和map坐标系是不是重合的？
  > （这也是我写这个博客解决的主要问题）可以很肯定的告诉你，机器人运动开始是重合的。
  > 但是，随着时间的推移是不重合的，而出现的偏差就是里程计的累积误差。
  > <font color="0000ff">那map-->odom的tf怎么得到?就是在一些校正传感器合作校正的package比如amcl会给出一个位置估计（localization），这可以得到map-->base_link的tf，所以估计位置和里程计位置的偏差也就是odom与map的坐标系偏差。所以，如果你的odom计算没有错误，那么map-->odom的tf就是0.</font>

注:
尽管直觉上map和odom都应该和base_link连接，但是这是不行的，由于每个坐标系只能有一个父坐标系

官方文档 Ros Wiki 上的说明， 参见 [ros wiki, REP 105, Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html#id18)
- base_link
  > The coordinate frame called base_link is rigidly attached to the mobile robot base. The base_link can be attached to the base in any arbitrary position or orientation; for every hardware platform there will be a different place on the base that provides an obvious point of reference. Note that REP 103 [1] specifies a preferred orientation for frames.
- odom
  > <font color="0000ff">The coordinate frame called odom is a world-fixed frame. 
  The pose of a mobile platform in the odom frame can drift over time, without any bounds.</font> 
  This drift makes the odom frame useless as a long-term global reference. However, the pose of a robot in the odom frame is guaranteed to be continuous, meaning that the pose of a mobile platform in the odom frame always evolves in a smooth way, without discrete jumps.
  > <font color="0000ff">In a typical setup the odom frame is computed based on an odometry source, such as wheel odometry, visual odometry or an inertial measurement unit.
  > The odom frame is useful as an accurate, short-term local reference, but drift makes it a poor frame for long-term reference.</font>
- map
  > <font color="0000ff">The coordinate frame called map is a world fixed frame, with its Z-axis pointing upwards.</font>
  > The pose of a mobile platform, relative to the map frame, should not significantly drift over time. The map frame is not continuous, meaning the pose of a mobile platform in the map frame can change in discrete jumps at any time.
  > <font color="0000ff">In a typical setup, a localization component constantly re-computes the robot pose in the map frame based on sensor observations, therefore eliminating drift, but causing discrete jumps when new sensor information arrives.</font>
  > <font color="0000ff">The map frame is useful as a long-term global reference, but discrete jumps in position estimators make it a poor reference frame for local sensing and acting.</font>
- Relationship between Frames
  > We have chosen a tree representation to attach all coordinate frames in a robot system to each other. Therefore each coordinate frame has one parent coordinate frame, and any number of child coordinate frames. The frames described in this REP are attached as follows:
  > ![figure09](technical_notes_attachment/figure09.ros_frames_relationship.png)
  > <font color="0000ff">The `map` frame is the parent of `odom`, and `odom` is the parent of `base_link`. Although intuition would say that both `map` and `odom` should be attached to `base_link`</font>, <font color="ff00ff">this is not allowed because each frame can only have one parent.</font>
- odom Frame Consistency
  > TODO: TBA

# <center> Sensor输入 </center> #

----
基于 ToF 的 RGB-D 相机， 与激光雷达 Lidar 的区别是什么， 参见 [zhihu, TOF摄像机可以替代Flash激光雷达吗？](https://zhuanlan.zhihu.com/p/146751368)
> 基本成像原理上ToF Camera与LiDAR相同，都采用飞行时间测距技术。不同之处在于应用的场景、结构设计和发射光波长存在不同。
> 再有就是机械扫描激光雷达是360度覆盖的，TOF相机一般水平FOV为55度。

## Realsense SDK

----
关注 sdk 开源到什么程度

### Realsense Ros

官方相关的文档：
- [official, RealSense Ros Main Page](http://wiki.ros.org/RealSense)
  - [official, Realsense Ros, Documentation](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)
  - [official, Realsense Ros, Tutorial](http://wiki.ros.org/ROS/Tutorials)
  - [official, Realsense Ros, Release Note](https://github.com/IntelRealSense/realsense-ros/blob/development/README.md)


**<font color="ff7500">注意:</font>**
关于 Realsense Ros 中的 `xxx_frame_id` 、坐标系、 Transform 等， 请参见 [Ros 的统一规范](#ros-tftf2-相关)

#### Realsense Ros 基本信息
配置项、Node、Topic、Service 等基本信息
参见官方文档 [offcial, realsense ros, launch parameters](https://github.com/IntelRealSense/realsense-ros#launch-parameters)。 现摘取几项重要的项

**Service**:
- `reset` : 
  > Cause a hardware reset of the device. Usage: rosservice call /camera/realsense2_camera/reset

**配置项：**
- `tf_prefix`: By default all frame's ids have the same prefix - camera_. This allows changing it per camera.
- 
**Topoics**:
- `odom_frame_id`: 按照这项的描述， t265 的 pose topic 是发布在这个 frame 下的
  > defines the origin coordinate system in ROS convention (X-Forward, Y-Left, Z-Up). pose topic defines the pose relative to that system.
- `topic_odom_in`: 按照这个描述？ 它可以输入速度信息， 提高其 odometry 的鲁棒性吗？
  > For T265, add wheel odometry information through this topic. The code refers only to the twist.linear field in the message.
- 所有可用的 frameid:
  > All the rest of the frame_ids can be found in the template launch file: [nodelet.launch.xml](https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/launch/includes/nodelet.launch.xml)
  > ![figure05](./technical_notes_attachment/figure05.realsense_ros_frame_ids.png)
- 不同 frame id 的区别与联系， 见 [about frameid](https://github.com/IntelRealSense/realsense-ros#about-frame-id)
  > ![figure06](technical_notes_attachment/figure06.realsense_ros_frame_id_groups.png)
- t265 的独特 topic, 参见 [using t265](https://github.com/IntelRealSense/realsense-ros#using-t265)
  - `/camera/odom/sample`
  - `/camera/accel/sample`
  - `/camera/gyro/sample`
  - `/camera/fisheye1/image_raw`
  - `/camera/fisheye2/image_raw`

问题疑问:
- 从 frameid 来看， 既有 `pose_frame_id`, 又有 `pose_optical_frame_id` 已经 `odom_frame_id`， 有什么区别？
- t265 输出的 topic 中, `/camera/odom/sample` 和 `/odom` 有什么区别


### Realsense SDK 底层功能 APIs

#### 图像、深度图滤波
参考一下里面的计算:
- [spatial-filter.cpp](https://github.com/IntelRealSense/librealsense/blob/master/src/proc/spatial-filter.cpp)
- [temporal-filter.cpp](https://github.com/IntelRealSense/librealsense/blob/master/src/proc/temporal-filter.cpp)
- [occlusion-filter.cpp](https://github.com/IntelRealSense/librealsense/blob/master/src/proc/occlusion-filter.cpp)
- [hole-filling-filter.cpp](https://github.com/IntelRealSense/librealsense/blob/master/src/proc/hole-filling-filter.cpp)

#### T265 Mapping and Localization
从文档和代码接口上来看， T265 有两个重要的辅助功能， 如果能够成功使用的话， 能有效提高其 Odometry 精确度
- topic: `odom_in`， 用于向 T265 提供轮速计信息。 参见文档页面：
  - [github pullrequest, t265 wheel odometry API](https://github.com/IntelRealSense/librealsense/pull/3462)
    这里面较好的描述了输入 odometry 的配置信息， 以及一些坐标系的说明
  - 启动参数 `topic_odom_in`, `calib_odom_file` 等
    , 见 [launch-parameters](https://github.com/IntelRealSense/realsense-ros#launch-parameters)
- `import localization map`， 用于向 T265 导入地图， 以提供在芯片上的 relocalization 功能。 参见文档页面：
  - **Important**: [Import/Export T265 localization map API](https://github.com/IntelRealSense/librealsense/pull/3324)
    这个是讲从 API 的角度如何做
  - **Important**: [example rs-ar-advanced Sample](https://github.com/IntelRealSense/librealsense/tree/master/examples/ar-advanced)
    完整的示例， 讲如何实现导入、导出地图， 接受重定位回调， 设置 anchor 节点等
    > - Import/export localization data file(s),
    > - Receive a relocalization event callback,
    > - Set/get static node to add/read landmark to/from the localization map.
  - [Exporting and importing localisation map for T265 from realsense-viewer](https://github.com/IntelRealSense/librealsense/issues/4526)， 这个是讲从 viewer 上如何实现
  - [T265 Localization in existing map](https://github.com/IntelRealSense/realsense-ros/issues/940)
  

### Realsense Multi-Cam 多相机同步
参考一下这个文档中的链接：[blog, How to get the same timestamp data from two cameras](https://support.intelrealsense.com/hc/en-us/community/posts/360050894574-How-to-get-the-same-timestamp-data-from-two-cameras)

可以硬件同步， 也可以考虑软件同步， 在文档中有一些讲解

TODO: 了解细节

## RGB-D 相机 (Realsense D435)

----
### 2020.02.02 RGB-D 相机对齐的检查

其它:
- [blog, 深度图转伪彩色图（python）](https://www.cnblogs.com/BambooEatPanda/p/9921446.html)

### 2020.01.29 Realsense 深度图配准的一些额外参考
参见
- **Major**: [blog, realsense SDK2.0学习：：（三）D435深度图片对齐到彩色图片-代码实现](https://blog.csdn.net/dieju8330/article/details/85300976)
- [blog, Kinect深度图与RGB摄像头的标定与配准](https://blog.csdn.net/aichipmunk/article/details/9264703)


### 2020.01.28 深度图配准原理/综述
参见
- [blog, 深度图像配准（Registration）原理](https://www.cnblogs.com/cv-pr/p/5769617.html)
- [blog, 一起做RGB-D SLAM (2)](https://www.cnblogs.com/gaoxiang12/p/4652478.html)
  讲到RGB-D 需要配准的一些问题
  > 实际Kinect里（或其他rgb-d相机里）直接采到的RGB图和深度图可能会有些<font color="ff00ff">小问题</font>：
  > - 有一些时差（约几到十几个毫秒）。这个时差的存在，会产生“RGB图已经向右转了，怎么深度图还没转”的感觉哦。
  > - 光圈中心未对齐。因为深度毕竟是靠另一个相机获取的，所以深度传感器和彩色传感器参数可能不一致。
  > - 深度图里有很多“洞”。因为RGB-D相机不是万能的，它有一个探测距离的限制啦！太远或太近的东西都是看不见的呢。关于这些“洞”，我们暂时睁一只眼闭一只眼，不去理它。以后我们也可以靠双边bayes滤波器去填这些洞。但是！这是RGB-D相机本身的局限性。软件算法顶多给它修修补补，并不能完全弥补它的缺陷。

## 双目相机 (Realsense T265 & Zed2)

----
T265 出来的 pose
- $pose.rotation$ 对应 $R^G_C$, 其中 $C$为 Camera坐标系, $G$为 初始坐标系， $y$轴与重力对齐后的
- $pose.translation$ 对应 $^Gt_C$, 也就对应着$^GP_C$
- 因此 $pose.rotation$ 和 $pose.translation$ 可以直接拼接为变换矩阵
  $$T^G_C = \begin{bmatrix}
    R^G_C & ^Gt_C \\
    0 & 1
  \end{bmatrix}
  = \begin{bmatrix}
    pose.rotation & pose.translation \\
    0 & 1
  \end{bmatrix}$$矩阵
> 参见官方文档 [official, t265 coordinate system](https://dev.intelrealsense.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera#section-41b-coordinate-systems)
> The pose provided by the T265 is provided relative to the center of the two cameras, as shown in Figure 4b. 
> <font color="0000ff">The pose is then the transformation from this reference frame to a gravity aligned reference frame centered on the starting position</font>, with Y pointing up, -Z pointing forward relative to the initial camera axis, and X pointing to the right (see Figure 4c).

## 激光雷达

----
一个大概了解， 参考
- [激光雷达的几线几线是啥意思？ - 三耳郎的回答 - 知乎](https://www.zhihu.com/question/405457595/answer/1330562982)
- [csdn, 激光雷达的性能指标](https://blog.csdn.net/weijimin1/article/details/93512218)

Velodyne 是一个做激光雷达的公司
目前国内外主要的激光雷达厂商见下表所示

国外
![figure04](technical_notes_attachment/figure03.lidar_manufactures_foreign.png)

国内
![figure04](technical_notes_attachment/figure04.lidar_manufactures_mainland.png)

Koda 原型上的激光雷达为 [velodyne vlp-16 puck](https://velodynelidar.com/products/puck/)

## Intel Realsense Depth 系列 (D435, T265, SR305)

----
参考 [csdn, intel RealSense摄像头比较](https://blog.csdn.net/qq_30113467/article/details/94401552)， 这里面做了较为全面一些的比较

## IMU (Intel Realsense D435i and T265)

----
### IMU 读数值

----
**<font color="ff4500">注意:</font>**
IMU 加速度计的读值， 是作用力的值， 因此转到世界坐标系后需要减去重力值 g
- 参见 [blog, 加速度计和陀螺仪指南(很详细的介绍)](https://blog.csdn.net/lovewubo/article/details/9084291)

关于 Realsense D435i, 其 IMU 与相机坐标系如下
- 参见: [official, d435i imu coord system](https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/)
  > ![figure02](technical_notes_attachment/figure02.realsense_d435i_coord_system.png)
- 以及 [github issue tracking, Transformation Matrices, D435i](https://github.com/IntelRealSense/realsense-ros/issues/1449)

# <center> 编程规范 / Programming Guide </center> #

----
## Python

----
以 google 的 python 编程规范/编程规格 为基准， 参见:
- [offcial doc, Google Python Style Guide](http://eiichiroi.github.io/google-styleguide-ja/pyguide.html#Comments)

将一些最常用的规范记录在此
- 命名规范, [naming](http://eiichiroi.github.io/google-styleguide-ja/pyguide.html#Naming)
- 注释， 特别是 doc string 注释的格式， [comments](http://eiichiroi.github.io/google-styleguide-ja/pyguide.html#Comments)


# <center> 数据集 / Dataset </center> #

----
### 2021.02.07 Image Segmentation 数据集及算法性能比对
Szeliski 大牛推荐:
- **Classic**: [dataset, The Berkeley Segmentation Dataset and Benchmark][dataset, The Berkeley Segmentation Dataset and Benchmark]
  > 这个应该是比较经典的数据集， 可以看看。

[dataset, The Berkeley Segmentation Dataset and Benchmark]: https://www2.eecs.berkeley.edu/Research/Projects/CS/vision/bsds/

### 2021.02.03 RGB-D 扩展数据集

从 open3d 的页面 [offcial, open3d rgbd images](http://www.open3d.org/docs/release/tutorial/geometry/rgbd_image.html) 中看到有另外两个数据集
- [Redwood dataset](http://redwood-data.org/)
  > 还未细看其中的数据情况。 但其数据格式与 realsense d435 的数据格式很类似， 比如: 
  > The Redwood format stored depth in a 16-bit single channel image. The integer value represents the depth measurement in millimeters
- [SUN RGB-D: A RGB-D Scene Understanding Benchmark Suite](http://rgbd.cs.princeton.edu/)
  > 这个数据集有比较类似于我们室内场景的
  > ![figure01](./technical_notes_attachment/figure01.rgbd_sun_dataset_indoor.png)

### 2021.01.16 立体匹配, 深度重建 相关数据集(及算法性能对比)

主要看权威性的数据集。 对我们来说， 主要关注它们在室内、建筑场景的表现

在这些标准数据集的基础上， 他们还汇总了不同算法的性能表现， 可以参考一下
- [Kitti Benchmark](http://www.cvlibs.net/datasets/kitti/index.php)
  > 参考其  stereo, depth 等方面的结果， TestImage 4, 5 有建筑场景
- [Middlebury Benchmark](https://vision.middlebury.edu/stereo/)
  > 主要参考其 stereo, MRF 方面的结果。 Classroom, Hoop5, Living Room, Stairs 这几个数据集有室内我们常见的平面场景
- [Robust Vision Benchmark](http://www.robustvision.net/)
  > 待参考， 我也还没细看这里面都是些啥
- [Sintel Benchmark](http://sintel.is.tue.mpg.de/)
  > 这个主要是光流的性能， 暂时不用太关注

从这几个数据集中的， 可以看到性能较好（针对室内和平面场景）， 又有代码的， 目前主要还是这两个
> 其它很多排名靠前的是 ml/dl 的， 对我们实际应用的意义不大
- [github, t-taniai/LocalExpStereo](https://github.com/t-taniai/LocalExpStereo)
  > 里面有给出 Maxflow code by Boykov and Kolmogorov [Code v3.01] [Code v3.04]， 非常好
- SGBM opencv implementation

# <center> 模块: 点云处理 </center> #

----
## 基于点云的 SLAM 流程, Kinect Fusion 及相关开源

----
Kinect Fusion 应当是一个标杆， 但算法没开源， 相关开源实现在我的理解中应当有:
- Elastic Fusion
- PCL Kinfu 模块

## PCL (Point Cloud Library) 点云滤波相关

----
感觉与图像中的滤波一样非常重要， 但是现在还不是特别理解， 要看一下。

综述总结参考:
- [blog, 【PCL】—常用的点云滤波算法详解](https://blog.csdn.net/lemonxiaoxiao/article/details/106071516)

## PCL (Point Cloud Library) 点云分割/聚类相关

----
PCL 主要参考:
- Tutorial: [offical, tutorial, segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/#segmentation)
- Reference: [official, Module segmentation](https://pointclouds.org/documentation/group__segmentation.html)
  > 从中可以看到， 在 PCL 点云分割部分， 有多种分割算法：
  > - `pcl::GrabCut` 用于前景背景提取
  > - `pcl::LCCPSegmentation`
  > - `pcl::RegionGrowing, pcl::RegionGrowingRGB` 区域增长分割算法
  > - `pcl::SACSegmentation, pcl::SACSegmentationFromNormals` 。<font color="0000ff">Sample Concensus Algorithm</font>。其中有适合平面分割的
  > - `pcl::SeededHueSegmentation`
  > - `pcl::SegmentDifferences`
  > - `pcl::SupervoxelClustering` 超体素分割


### 2020.02.03 算法原理和论文
暂时还没有去深入探究论文， 这里先列一下别人的总结， 待后面细看
- [blog, 传统的点云分割方法及PCL中的分割模块](https://blog.csdn.net/lemonxiaoxiao/article/details/105583091)
  > 很多<font color="ff4500">仍然是基于 graph cut， 因此对 graph cut, mrf 等基础方法的理解非常重要</font>
- [blog, PCL-欧式聚类算法详解](https://blog.csdn.net/lemonxiaoxiao/article/details/106061265)
- [blog, PCL-RANSAC点云分割算法详解](https://blog.csdn.net/lemonxiaoxiao/article/details/106073092)

### 2020.02.02 点云分割实践
主要参考 PCL 官方 Tutorial, 使用 pclpy 的现有函数进行辅助分割
- [Tutorial, Plane model segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html#planar-segmentation)
- [Tutorial, Region growing segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html#region-growing-segmentation)
- [Tutorial, Color-based region growing segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_rgb_segmentation.html#region-growing-rgb-segmentation)
  > 区域自增长的分割， 以及基于颜色的辅助
- [Tutorial， Euclidean Cluster Extraction](https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction)
  > 基于几何的分割
- [Tutorial, Conditional Euclidean Clustering](https://pcl.readthedocs.io/projects/tutorials/en/latest/conditional_euclidean_clustering.html#conditional-euclidean-clustering)
- > 基于几何的分割
- [Tutorial, Difference of Normals Based Segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/don_segmentation.html#don-segmentation)
- [Tutorial, Clustering of Pointclouds into Supervoxels - Theoretical primer](https://pcl.readthedocs.io/projects/tutorials/en/latest/supervoxel_clustering.html#supervoxel-clustering)
  > 超体素分割
- [Tutorial, Identifying ground returns using ProgressiveMorphologicalFilter segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/progressive_morphological_filtering.html#progressive-morphological-filtering)
- [Tutorial, Filtering a PointCloud using ModelOutlierRemoval](https://pcl.readthedocs.io/projects/tutorials/en/latest/model_outlier_removal.html#model-outlier-removal)

备注:
- [Min-Cut Based Segmentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/min_cut_segmentation.html#min-cut-segmentation) 主要用于二分分割， 比如前景背景分割， 因此暂时不用去管它

## Open3D 及 Python Package

----
因为要从 RGB-D 直接转点云， 本身可以自己写， 但是比较土。 PCL 库中只有 PointCloud -> RangeImage， 没有反过来的部分。
看了一下 stack-overflow 的介绍， Open3D 的维护好想比较好， 而且有比较靠谱的 python wrapper, 可以支持直接从 conda 或 pip 安装
- 参见 [stack overflow, Generate point cloud from depth image](https://stackoverflow.com/questions/59590200/generate-point-cloud-from-depth-image)

此外， Open3d 还支持在 python 上直接查看点云， 比较好。 因此进行

### Overview
官方页面:
- [github, Open3D: A Modern Library for 3D Data Processing](https://github.com/intel-isl/Open3D)
- [official, open3d getting started](http://www.open3d.org/docs/release/getting_started.html)
- [offcial, open3d docs](http://www.open3d.org/docs/release/)
- [offcial, open3d python api](http://www.open3d.org/docs/release/index.html#python-api-index) 
- [offcial, open3d 主页](http://www.open3d.org/)

介绍性文档:
- [zhihu, 3D可视化神器之Open3D](https://zhuanlan.zhihu.com/p/57215172)
- [csdn, 教程：Python Open3d 完成 ICP 点云配准](https://blog.csdn.net/weixin_42488182/article/details/105196148)


# <center> 模块: 图像分割、平面辅助 </center> #

----

我们设想， 将利用平面等先验信息辅助深度补全、3维重建等， 因此这个是一个比较基础的功能

## 图像分割

----
Szeliski 的书中也讲到了， 因为图像分割的领域和实际应用都非常宽广， 推荐通过在 经典数据集 上去查看不同算法的性能比较， 比如: [Berkeley Segmentation Dataset and Benchmark][dataset, The Berkeley Segmentation Dataset and Benchmark]


附录:
这里有一篇博客， 介绍了图像分割方面的相应论文发展， 有空的时候可以看看
- [blog, 超像素分割技术发展情况梳理(Superpixel Segmentation）--计算机视觉专题3](https://blog.csdn.net/anshan1984/article/details/8918167)

### 2020.02.07 Combination of Graphcut and Superpixel

在数据集
### 2020.02.05 Following: 学术大牛的主页， 论文(及代码)

关于 Graphcut， 以下三位大牛的主页可以进行深入跟踪, 俩人的主页上就有大量的code。包括maxflow/min-cut、stereo algorithms等算法：
- [Prof. Boykov, code page](https://vision.cs.uwaterloo.ca/code/)
- [Prof. Kolmogorov](http://pub.ist.ac.at/~vnk/software.html)
- [康奈尔大学的graphcuts研究主页, Prof. Ramin Zabih](http://www.cs.cornell.edu/~rdz/)

附录:
中文的博客有部分总结， 可以参考:
- [csdn, 【图像处理】图像分割之（一~四）GraphCut，GrabCut函数使用和源码解读（OpenCV）](https://blog.csdn.net/KYJL888/article/details/78253829)

### 2020.01.27 基于 OpenCV 的已有实现(Graph-Cut, SuperPixel)

前面说到的 [Efficient Graph-Based Image Segmentation](http://fcv2011.ulsan.ac.kr/files/announcement/413/IJCV(2004)%20Efficient%20Graph-Based%20Image%20Segmentation.pdf)， <font color="ff4500">在 OpenCV 上有对应实现， 并且还有其它的一些2D分割方式， 可以尝试一下</font>

一、图分割的方式
> 主要参见 [official doc, opencv ximgproc segmentation](https://docs.opencv.org/4.1.0/d5/df0/group__ximgproc__segmentation.html)模块
- [GraphSegmentation](https://docs.opencv.org/4.1.0/dd/d19/classcv_1_1ximgproc_1_1segmentation_1_1GraphSegmentation.html#details)
  > 其中几个参数的意义:
  > Sigma：先对原图像进行高斯滤波去噪，sigma即为高斯核的
  > k: 控制合并后的区域的大小，见前文
  > min: 后处理参数，分割后会有很多小区域，当区域像素点的个数小于min时，选择与其差异最小的区域合并。
  - [应用示例: python 代码](https://blog.gtwang.org/programming/opencv-graph-based-segmentation-tutorial/)
- [SelectiveSearchSegmentation](https://docs.opencv.org/4.1.0/d6/d6d/classcv_1_1ximgproc_1_1segmentation_1_1SelectiveSearchSegmentation.html)
  - 示例代码参见 [github, opencv_contrib, selectivesearchsegmentation_demo.py](https://github.com/opencv/opencv_contrib/blob/master/modules/ximgproc/samples/selectivesearchsegmentation_demo.py)
  

二、超像素的方式
Super Pixel 原理
- 参见 [blog, 超像素(Superpixel)理解](https://blog.csdn.net/Linear_Luo/article/details/52588515)
- 有趣直观并且带有源代码的是 [SLIC Superpixel](https://infoscience.epfl.ch/record/177415)，使用K-means的聚类方法，分割的效果很好

OpenCV 实现
> super-pixel 这种方式听起来与点云中的 super-voxel 方式好像蛮有关联的
> 主要参见 [official doc, opencv ximgproc, super pixel](https://docs.opencv.org/4.1.0/df/d6c/group__ximgproc__superpixel.html)
- [LSC (Linear Spectral Clustering) SuperPixels](https://docs.opencv.org/4.1.0/d5/da0/classcv_1_1ximgproc_1_1SuperpixelLSC.html#details)
- [SEEDS (Superpixels Extracted via Energy-Driven Sampling)](https://docs.opencv.org/4.1.0/df/d81/classcv_1_1ximgproc_1_1SuperpixelSEEDS.html#details)
- [SLIC (Simple Linear Iterative Clustering) SuperPixels](https://docs.opencv.org/4.1.0/d3/da9/classcv_1_1ximgproc_1_1SuperpixelSLIC.html#details)

OpenCV 调用示例：
- 应用示例, [Python - Opencv实现图像超像素分割（SLIC、SEEDS、LSC](https://blog.csdn.net/qq_40268412/article/details/103915197)



三、基于谱分割的方法:
TBA: 这种方法好像还在研究前沿， 但在 cs168 中已经看到部分其应用和威力， 后续可以跟踪一下这方面

### 2020.01.20 DPPTAM 的实现参考 (图像分割辅助稠密重建)

----
DPPTAM 是使用平面信息来辅助稠密(虽然是基于单目)， 但因为其代码是开源公布了的， 因此还是有所帮助
> : Dense Piecewise Planar Tracking and Mapping from a Monocular Sequence


看它的代码， 里面的分割部分基于 [Efficient Graph-Based Image Segmentation](http://fcv2011.ulsan.ac.kr/files/announcement/413/IJCV(2004)%20Efficient%20Graph-Based%20Image%20Segmentation.pdf) 论文实现

github 上还有较多对应的实现，如
- [davidstutz/graph-based-image-segmentation](https://github.com/davidstutz/graph-based-image-segmentation)
- [suryanshkumar/GraphSegmentation](https://github.com/suryanshkumar/GraphSegmentation)
- 其它参见: [github search, efficient graph based image segmentation](https://github.com/search?q=Efficient+graph-based+image+segmentation)

# <center> 模块: 深度图 / 视差图 处理 </center> #

----
基于不同输入得到的 深度图/视差图， 经常可以做进一步的处理， 如空洞填充、深度补全等， 输入包括
- 激光雷达扫描
- RGB-D 这类相机直接得到
- 双目/多目立体匹配得到

本节包含这上面

## 深度图补全

----

### 2020.01.28 入门
参考
- [zhihu, 原理性介绍， 计算机视觉方向简介：深度图补全](https://zhuanlan.zhihu.com/p/42084058)

# <center> 模块: CV基础 </center> #

----
## BELID：增强的高效的局部图像描述符
大致介绍参考 [blog, BELID：增强的高效的局部图像描述符](https://mp.weixin.qq.com/s/y5WLCaF4N54F4Mt1NxQGtQ)

对应论文参考: [BEBLID: Boosted Efficient Binary Local Image Descriptor](https://raw.githubusercontent.com/iago-suarez/BEBLID/master/BEBLID_Boosted_Efficient_Binary_Local_Image_Descriptor.pdf)

opencv 中的实现参考: [cv::xfeatures2d::BEBLID Class Reference](https://docs.opencv.org/4.5.1/d7/d99/classcv_1_1xfeatures2d_1_1BEBLID.html#details)

# <center> 其它 / Miscs </center> #

----
## YAML 读写

用 Python 操作 yaml 文件， 详细指导见 [important, blog, python操作yaml说明](https://www.jb51.net/article/184299.htm)
如每个参数的含义和用法：
- `stream`
  > 指定由于输出YAML流的打开的文件对象。默认值为 None，表示作为函数的返回值返回。
- `default_flow_style`
  > 是否默认以流样式显示序列和映射。默认值为 None，表示对于不包含嵌套集合的YAML流使用流样式。设置为 True 时，序列和映射使用块样式。
- `default_style`
  > 默认值为 None。表示标量不使用引号包裹。设置为 '"' 时，表示所有标量均以双引号包裹。设置为 "'" 时，表示所有标量以单引号包裹。
- `canonical`
  > 是否以规范形式显示YAML文档。默认值为 None，表示以其他关键字参数设置的值进行格式化，而不使用规范形式。设置为 True 时，将以规范形式显示YAML文档中的内容。
- `width`
  > 表示每行的最大宽度。默认值为 None，表示使用默认的宽度80。
  
# <center> The End </center> #