


# Multi-Modal Lidar Dataset for Benchmarking General-PurposeLocalization and Mapping Algorithms 

<div align=center>
<img src="./imgs/intro.jpg" width="800px">

</div>
<p align="left">Figure 1.  (Left) Front view of the multi-modal data acquisition system. Next to each sensor, we show the individual coordinate frames for the generated point clouds. (Right) Samples of map data form different dataset sequences. From left to right and top to down, we display maps generated from a forest, an urban area, an open road and a large indoors hall, respectively.</p>


<div align=center>
<img src="./imgs/data_sample.jpg" width="800px">
</div>

<p align="left"> Figure 2. Our dataset was captured by a rich suite of sensors. Subsets of the data from the *Indoor04* sequence are visualized here. The leftmost column shows the lidar data from Avia and Horizon; the second column shows the lidar data from OS1 and OS0; the third column shows the data from the VLP-16 and depth image from L515. The rightmost column shows the RGB image from L515 and range images from 0S1 and OS0..</p>

## ABSTRACT:

This is a novel multi-modal lidar dataset with sensors showcasing different scanning modalities (spinning and solid-state), sensing technologies, and lidar cameras. The focus of the dataset is on low-drift odometry, with ground truth data available in both indoors and outdoors environment with sub-millimeter accuracy from a motion capture (MOCAP) system. For comparison in longer distances, we also include data recorded in larger spaces indoors and outdoors.The dataset contains point cloud data from spinning lidars and solid-state lidars. Also, it provides range images from high resolution spinning lidars, RGB and depth images from a lidar camera, and inertial data from built-in IMUs.

Keywords:Dataset, Multi-model, Multi-scenario, SLAM

## MAIN CONTRIBUTIONS: 

   * A dataset with data from 5 different lidar sensors and one lidar camera in a variety of environments. This is, to our knowledge, the most diverse dataset in terms of lidar sensors for these environments. The dataset includes spinning lidars with 16 (Velodyne VLP-16), 64 (Ouster OS1-64) and 128 (Ouster OS1-128) channels and different vertical FoVs. Two different solid-state lidars (Livox Horizon and Livox Avia) with different scanning patterns and FoVs are also included. A lidar camera (RealSense L515) provides RGB images and lidar-aided depth images. Low-resolution images with depth, near-infrared and laser reflectivity data from the Ouster sensors complete the dataset. 

   * The dataset includes sequences with MOCAP-based ground truth in both indoors and outdoors environments. This is, to the best of our knowledge, the first lidar dataset to provide such accurate ground truth in forest environments in addition to indoor areas, albeit the limited trajectory length.


   * In addition to the MOCAP-labeled data, the dataset includes other sequences in large indoor halls, roads, and forest paths. The wide variety of sensors enables comparison between lidar odometry and mapping algorithms to an extent that was not possible before, with both general-purpose and sensor-specific approaches.

   * Based on the presented dataset, we provide a baseline comparison of the state-of-the-art in lidar odometry, localization and mapping. We compare the odometry lift as well as the quality of the maps obtained with different sensors and different algorithms.


## Updates 
2022.03.01   uploads initial dataset
 

## 1.LICENSE
This work is licensed under MIT license. International License and is provided for academic purpose. If you are interested in our project for commercial purposes, please contact us on qingqli@utu.fi for further communication.  

 
## 2.SENSOR SETUP
### 2.1 Acquisition Platform
Physical drawings and schematics of the ground robot is given below. The unit of the figures is centimeter.

<div align=center>
<img src="./imgs/scales.jpg" width="800px">
</div>
<p align="left">Figure 3. Our data collecting platform, front view RGB (left),top view (middle) and front view (right).</p>



### 2.2 Sensor parameters
Sensor specification for the presented dataset. Angular resolution is configurable in the OS1-64 (varying the vertical FoV). Livoxlidars have a non-repetitive scan pattern that delivers higher angular resolution with longer integration times. Range is based on manufacturerinformation, with values corresponding to 80% Lambertian reflectivity and 100 klx sunlight, except for the L515 lidar camera.
<div align=center>
<img src="./imgs/sensors.jpg" width="600px">
</div>

 

The rostopics of our rosbag sequences are listed as follows:

* VLP-16 LIDAR : `/velodyne_points  sensor_msgs/PointCloud2`  
* OS0 LIDAR :  `/os_cloud_node/imu           : sensor_msgs/Imu  `  ,       
            ` /os_cloud_node/points               : sensor_msgs/PointCloud2 `,
            `/img_node/nearir_image               : sensor_msgs/Image  ` ,      
            ` /img_node/range_image               : sensor_msgs/Image  `  ,     
            ` /img_node/reflec_image              : sensor_msgs/Image   `  ,   

* OS1 LIDAR :`   /os_cloud_nodee/imu        : sensor_msgs/Imu ` ,         
             `/os_cloud_nodee/points        : sensor_msgs/PointCloud2  `,           
            `/img_nodee/nearir_image          : sensor_msgs/Image `,        
            ` /img_nodee/range_image           : sensor_msgs/Image `,       
            ` /img_nodee/reflec_image       : sensor_msgs/Image `,        
            ` /img_nodee/signal_image        : sensor_msgs/Image ` ,  

* Horizon LIDAR : `/livox/imu : sensor_msgs/Imu`            
                `/livox/lidar : livox_ros_driver/CustomMsg ` 
* AVIA LIDAR : 
    `/avia/livox/imu  sensor_msgs/Imu `,
    `/avia/livox/lidar  livox_ros_driver/CustomMsg`,
* L515 LIDAR CAMERA: 
    `/cam_1/color/image_raw             : sensor_msgs/Image `        
    `/cam_1/depth/image_rect_raw            : sensor_msgs/Image `
* MOCAP SYSTEM:  
    `/vrpn_client_node/optitest/pose    6471 msgs    : geometry_msgs/PoseStamped`
 
 

## 3.DATASET SEQUENCES
 

<!-- <div align=center> -->

<!-- <img src="https://github.com/sjtuyinjie/mypics/blob/main/dynamic.gif" width="400px">
</div> -->

<!-- <p align="left">Figure 3. A sample video with fish-eye image(both forward-looking and sky-pointing),perspective image,thermal-infrared image,event image and lidar odometry</p> -->
 

<div align=center>
<img src="./imgs/data_sequences.jpg" width="400px">
<p align="center">List of data sequences in our dataset (V: Velodyne VLP-16, H:Livox Horizon, A:Livox Avia, O_0: Ouster OS0, O_1: Ouster OS1.</p>
</div>

### 3.1 Main Dataset

Sequence Name|Collection Date|Total Size|Duration|Features|Rosbag 
--|:--|:--:|--:|--:|--: 
Forest01|2022-02-08|21.9g|62s|Winter,Square|(Rosbag: Uploading)  
Forest02|2022-02-08|22.4g|73s|Windter,Straight|(Rosbag: Uploading)  
Forest03|2021-09-28|7.3g|717s|Autumn|(Rosbag: Uploading)  
Indoor01|2022-04-27|49.3g|114s|day|(Rosbag: Uploading)  
Indoor02|2022-02-21|16.7g|42.3s|day|(Rosbag: Uploading)  
Indoor03|2021-02-21|19.2g|46.7s|day|(Rosbag: Uploading)  
Indoor04(Hall)|2022-02-09|92.8g|248s|day|(Rosbag: Uploading)  
Indoor05(Corridor)|2022-02-09|141.5g|551s|day|(Rosbag: Uploading)  
Road01|2022-02-20|47.6g|110s|day|(Rosbag: Uploading)  
Road02|2022-02-20|212.7g|487s|day|(Rosbag: Uploading)  

### 3.2 Other Data
Sequence Name|Collection Date|Total Size|Duration|Features|Sensors|Rosbag 
--|:--|:--:|--:|--:|--:|--: 
LidarsCali|2022-02-11|21.9g|19.1s|room| |(Rosbag: Uploading)  


## 4. Tested SLAM Result
### 4.1 Trajectory Result
We teseted  some well-known SLAM systems as below:
 
[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM),
 
[FAST_LIO](https://github.com/hku-mars/FAST_LIO),

[LIO_LIVOX](https://github.com/Livox-SDK/LIO-Livox),
 
  
<div align=center>
<img src="./imgs/traj_result.jpg" width="800px">
<p align="center">Estimated trajectories. Top row: Indoor01, Indoor02, Indoor03, Forest02. Bottom row: Forest01, Road02, Indoor04, Indoor05.</p>
</div>
 

### 4.2 Mapping Quality Result
<div align=center>
<img src="./imgs/parking_signal.jpg" width="800px">
<p align="center">Qualitative comparison of the mapping quality. Top row showm the rgb image, map LIOL Horizon, FLIO OS0. Bottom row shows a parking signs in rgb image, and mapping result from Horizon-based LIOL, Horizon, Avia, OS0, and OS1-based FLIO, and Velodyne's LeGo-LOAM maps, respectively.</p>
</div>

 
## 5.DEVELOPMENT TOOLKITS 
### 5.1 Frame id reset
Rosbag recoreds message with their raw frame_id. If user need to show or run multiple lidar same time, we use [srv_tools](https://github.com/srv/srv_tools) to change frame_id of each topics. 
  
### 5.2 Evaluation
We use open-source tool [evo](https://github.com/MichaelGrupp/evo) for evalutation. 

To evaluate LIDAR SLAM,type 
~~~
evo_ape tum optk.txt {SLAM_result}.txt -a -p
~~~

### 5.3 Calibration 
For IMU intrinsics,visit [Imu_utils](https://github.com/gaowenliang/imu_utils)
 
For extrinsics between cameras and LIVOX Lidar, visit [
livox_camera_lidar_calibration](https://github.com/Livox-SDK/livox_camera_lidar_calibration)  

 

## 6.ACKNOWLEGEMENT
This research work is supported by the Academy of Finland's AeroPolis project (Grant 348480) and the Finnish Foundation for Technology Promotion (Grants 7817 and 8089).