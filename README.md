task05
----

主要文件介绍
---

  **config**
  
  存放的是相机参数的配置文件camera_params.yaml, 可以通过修改该文件来改变搜索相机ip号或sn号，以及相机读取图片时的参数设置（包括exposure_time， gain，frame_rate， pixel_format）。
  
  
  **include/hik_camera**
  
  1.armor_classifier.hpp 装甲板分类器的头文件
  
  2.hik_camera_driver.hpp 海康相机的驱动头文件

  3.light_strip_detector.hpp 灯条检测器的头文件

  4.pose_estimator.hpp 装甲板中心坐标计算的头文件
  
  
  **launch**
  
  启动节点。
  
  
  **src**
  
  1.hik_camera_driver.cpp:定义初始化设备与收集图像相关功能的cpp文件，同时加入了可以读取本地视频文件并上传到话题的功能。
  
  2.hik_camera_node.cpp：驱动海康相机收集或者调用本地视频流的主节点。

  3.armor_classifier.cpp:定义了识别装甲板数字图案所需要的函数。

  4.light_strip_detector.cpp：进行灯条识别，图像装甲板roi查找，标记识别数字与装甲板，并发布图像到processed_image（only for test,可以注释掉）和 image_with_light_strips两个topic上。
  
  
  **CMakeLists.txt**
  
  指导节点编译。


  **number_recognition_model**
  
  包含了训练模型的代码，划分训练集的代码（原本包括数据集，但是由于数据集较大，因此未上传）

  
  **packege.xml**
  
  包含对pkg描述，作者信息以及所需依赖。

  **(另外，还需要hik的SDK连接库)**




编译配置节点
---

***首先说明：CmakeLists.txt文件中使用的是include文件夹与MVS_SDK文件的绝对路径，运行前需要手动修改路径以正确链接MVS_SDK的源文件与include文件中的.hpp头文件***

运行之前，检查你的电脑上是否有CMakeLists.txt里所要求的所有依赖库，若没有需要自行下载。部分连接库使用的绝对路径，改成自己库所在的位置。

运行之前还有几点需要说明的事情：

*1*
-
因为在标定相机的时候使用的相机只支持MONO格式，因此若要使用相机进行检验，需要进行以下操作：
*1. 将camera_params.yaml中的pixel_format改为Mono8，将source_type改为hik。*

*2. 在light_strip_detector.cpp中找到以下两行：
```
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
```

```
auto annotated_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", annotated_image).toImageMsg();
```

将第一行代码encoding改为“MONO8”，第二行的改成“mono8”。如果需要调用单通道编码视频同理。

*2*
-
在light_strip_detector中加入了计算装甲板坐标的逻辑，目前处于注释状态，若要计算装甲版相机坐标系坐标取消注释即可，但是建议在代码应用于检测装甲板数字的时候注释掉这一段（否则有可能会造成卡顿的现象。）

***在以上事项均作好之后开始构建编译。***
-

首先找到你的ros2。
```
source /opt/ros/humble/setup.zsh
```

然后就可以启动构建pkg:

```
colcon build --packages-select hik_camera
```
(这里会弹出一些警告，无伤大雅，忽视即可)

构建成功后。

```
source install/setup.zsh
```

之后导入下列路径：
```
export LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH
```

然后使用launch文件来运行节点：

```
ros2 launch hik_camera hik_camera.launch.py
```

你就可以看到rviz2窗口弹出，接下来你就可以在窗口中点击add，然后你就可以查看三个话题。

*1* image_raw 相机取流或者本地视频文件取流并上传的原图像。

*2* image_processed 处理后的灰度图像，可以在light_strip_detector.cpp中注释掉。

*3* image_with_light_strips 标记出来了装甲板的位置以及装甲版上的图案（这里解释一下：6代表outpost，7代表guard，8代表base）。

