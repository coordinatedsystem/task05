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

```
source /opt/ros/humble/setup.zsh   
```
如若没有警告，

接着source install文件里面的setup.py:

```
source install/setup.zsh 
```

然后就可以启动构建pkg了:

```
colcon build --packages-select hik_camera
```
(这里会弹出一些警告，无伤大雅，忽视即可)

最后运行节点：

```
ros2 run hik_camera hik_camera_node
```

之后节点就会开始运行,但不会弹出rviz2窗口。这是你需要手动输入参数并且手动打开rviz2。

更好的方法是使用launch文件来运行节点：

```
ros2 launch hik_camera hik_camera.launch.py
```

你就可以看到rviz2窗口弹出，接下来你就可以在窗口中点击add，然后by topic/image_raw/image来查看图像。

接收和查看节点
---
新建一个终端，然后先source你的ros2：

```
source /opt/ros/humble/setup.zsh
```

首先可以打开node list查看是否节点正在运行：

```
ros2 node list
```

然后查看是否有名为hik_camera的node。

无论你使用的是直接运行节点，还是使用launch文件运行节点，那么你就可以在这个命令行改变参数,如:

```
ros2 param set /hik_camera camera_sn 00F26632041
ros2 param set /hik_camera exposure_time 5000.0
ros2 param set /hik_camera gain 20.0
ros2 param set /hik_camera frame_rate 10.0
```

之后查看话题：

```
ros2 topic list
```

然后查看是否有名为image_raw的话题。

这是如果你监听image_raw的话题的话：

```
ros2 topic echo /image_raw
```

会看到一串数字编码，说明相机正在向你的电脑传输图片数据。

在这串数字窗口中，你可以找到图片的详细信息。
