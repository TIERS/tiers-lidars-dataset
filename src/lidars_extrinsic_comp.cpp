 #include <time.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h> 
 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/filters/statistical_outlier_removal.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>  
 

#include <fstream>
#include <chrono>
#include <string>
#include <Eigen/Dense>
  
typedef  pcl::PointXYZ   PointType;
 
class LidarsCalibrater{
    private: 
        ros::NodeHandle nh; 
        ros::Subscriber sub_avia;
        ros::Subscriber sub_hori;
        ros::Subscriber sub_velo; 
        ros::Subscriber sub_os0;
        ros::Subscriber sub_os1; 

        ros::Publisher pub_avia;
        ros::Publisher pub_hori;
        ros::Publisher pub_velo;
        ros::Publisher pub_os0;
        ros::Publisher pub_os1; 

        tf::TransformBroadcaster tf_br;

        // avia tf
        int avia_itegrate_frames = 5;
        pcl::PointCloud<PointType> avia_igcloud;
        Eigen::Matrix4f avia_tf_matrix; 
        bool avia_tf_initd = false;

        // Hori TF
        int hori_itegrate_frames = 5;
        pcl::PointCloud<PointType> hori_igcloud;
        Eigen::Matrix4f hori_tf_matrix; 
        bool hori_tf_initd = false;

        // OS0 TF
        Eigen::Matrix4f os0_tf_matrix; 
        bool os0_tf_initd = false;
        pcl::PointCloud<PointType> os0_cloud;
        bool os0_received = false;

        // OS1 TF
        Eigen::Matrix4f os1_tf_matrix; 
        bool os1_tf_initd = false;
  
        // Velo TF
        Eigen::Matrix4f velo_tf_matrix; 
        bool velo_tf_initd = false;
 


    public:
        LidarsCalibrater(){ 
            sub_avia = nh.subscribe<sensor_msgs::PointCloud2>("/livox_avia", 1000, &LidarsCalibrater::avia_cloud_handler, this); 
            sub_hori = nh.subscribe<sensor_msgs::PointCloud2>("/livox_horizon", 1000, &LidarsCalibrater::hori_cloud_handler, this);
            sub_velo = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1000, &LidarsCalibrater::velo_cloud_handler, this);
            sub_os0 = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 1000, &LidarsCalibrater::os0_cloud_handler, this); 
            sub_os1 = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_nodee/points", 1000, &LidarsCalibrater::os1_cloud_handler, this); 
 
            pub_avia    = nh.advertise<sensor_msgs::PointCloud2>("/a_avia", 1);
            pub_hori    = nh.advertise<sensor_msgs::PointCloud2>("/a_horizon", 1);
            pub_velo    = nh.advertise<sensor_msgs::PointCloud2>("/a_velo", 1);
            pub_os0     = nh.advertise<sensor_msgs::PointCloud2>("/a_os0", 1);
            pub_os1     = nh.advertise<sensor_msgs::PointCloud2>("/a_os1", 1);   

            // avia_itegrate_frames = 10;
            // hori_itegrate_frames = 10;
        };
         

        ~LidarsCalibrater(){};

        void matrix_to_transform(Eigen::Matrix4f & matrix, tf::Transform & trans){
            tf::Vector3 origin;
            origin.setValue(static_cast<double>(matrix(0,3)),static_cast<double>(matrix(1,3)),static_cast<double>(matrix(2,3)));

            tf::Matrix3x3 tf3d;
            tf3d.setValue(static_cast<double>(matrix(0,0)), static_cast<double>(matrix(0,1)), static_cast<double>(matrix(0,2)),
            static_cast<double>(matrix(1,0)), static_cast<double>(matrix(1,1)), static_cast<double>(matrix(1,2)),
            static_cast<double>(matrix(2,0)), static_cast<double>(matrix(2,1)), static_cast<double>(matrix(2,2)));

            tf::Quaternion tfqt;
            tf3d.getRotation(tfqt);

            trans.setOrigin(origin);
            trans.setRotation(tfqt);
        }
 
        void removeFarPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out, float thres) 
        {
            if (&cloud_in != &cloud_out) {
                cloud_out.header = cloud_in.header;
                cloud_out.points.resize(cloud_in.points.size());
            }

            size_t j = 0;

            for (size_t i = 0; i < cloud_in.points.size(); ++i) {
                if (cloud_in.points[i].x * cloud_in.points[i].x +
                        cloud_in.points[i].y * cloud_in.points[i].y +
                        cloud_in.points[i].z * cloud_in.points[i].z > thres * thres)
                    continue;
                cloud_out.points[j] = cloud_in.points[i];
                j++;
            }
            if (j != cloud_in.points.size()) {
                cloud_out.points.resize(j);
            }

            cloud_out.height = 1;
            cloud_out.width = static_cast<uint32_t>(j);
            cloud_out.is_dense = true;
        }

        void avia_cloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            if(!os0_received) return;
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            if(avia_itegrate_frames > 0)
            {
                avia_igcloud += full_cloud_in; 
                avia_itegrate_frames--; 
                // ROS_INFO_STREAM("Avia cloud integrating: " << avia_itegrate_frames);
                return;
            }else
            {
                if(!avia_tf_initd){
                    Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
                    Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
                    Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());

                    Eigen::Translation3f init_trans(0.0,0.0,0.0);
                    Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix(); 

                     ROS_INFO("\n\n\n  Calibrate Avia ...");
                    calibrate_PCLICP(avia_igcloud.makeShared(), os0_cloud.makeShared(), avia_tf_matrix, false); 
                    Eigen::Matrix3f rot_matrix = avia_tf_matrix.block(0,0,3,3);
                    Eigen::Vector3f trans_vector = avia_tf_matrix.block(0,3,3,1);
                    
                    // tf::Transform t_transform;
                    // matrix_to_transform(avia_tf_matrix,t_transform);
                    // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "avia_frame", "base_link"));
        
                    std::cout << "Avia -> base_link " << trans_vector.transpose()
                        << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "avia_frame"
                        << " /" << "base_link" << " 10" << std::endl;

                    // publish result
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (avia_igcloud , out_cloud, avia_tf_matrix);

                    sensor_msgs::PointCloud2 avia_msg;
                    pcl::toROSMsg(out_cloud, avia_msg);
                    avia_msg.header.stamp = ros::Time::now();
                    avia_msg.header.frame_id = "base_link"; 
                    pub_avia.publish(avia_msg); 

                    avia_tf_initd = true;
                }else
                {
                    // tf::Transform t_transform;
                    // matrix_to_transform(avia_tf_matrix,t_transform);
                    // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "avia_frame", "base_link"));
        
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , out_cloud, avia_tf_matrix);

                    sensor_msgs::PointCloud2 avia_msg;
                    pcl::toROSMsg(out_cloud, avia_msg);
                    avia_msg.header.stamp = ros::Time::now();
                    avia_msg.header.frame_id = "base_link"; 
                    pub_avia.publish(avia_msg); 
                }
            } 
        }
 
        void hori_cloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            if(!os0_received) return;
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            if(hori_itegrate_frames > 0)
            {
                hori_igcloud += full_cloud_in; 
                hori_itegrate_frames--; 
                // ROS_INFO_STREAM("hori cloud integrating: " << hori_itegrate_frames);
                return;
            }else
            {
                if(!hori_tf_initd){
                    Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
                    Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
                    Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());

                    Eigen::Translation3f init_trans(0.0,0.0,0.0);
                    Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
 
                    // pcl::transformPointCloud (full_cloud , cloud_out, transformation_matrix);
                     ROS_INFO("\n\n\n  Calibrate Horizon ...");
                    calibrate_PCLICP(hori_igcloud.makeShared(), os0_cloud.makeShared(), hori_tf_matrix, false); 
                    Eigen::Matrix3f rot_matrix = hori_tf_matrix.block(0,0,3,3);
                    Eigen::Vector3f trans_vector = hori_tf_matrix.block(0,3,3,1);
                    
                    // tf::Transform t_transform;
                    // matrix_to_transform(hori_tf_matrix,t_transform);
                    // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "hori_frame", "base_link"));
        
                    std::cout << "hori -> base_link " << trans_vector.transpose()
                        << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "hori_frame"
                        << " /" << "base_link" << " 10" << std::endl;

                    // publish result
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (hori_igcloud , out_cloud, hori_tf_matrix);

                    sensor_msgs::PointCloud2 hori_msg;
                    pcl::toROSMsg(out_cloud, hori_msg);
                    hori_msg.header.stamp = ros::Time::now();
                    hori_msg.header.frame_id = "base_link"; 
                    pub_hori.publish(hori_msg); 

                    hori_tf_initd = true;
                }else
                {
                    // tf::Transform t_transform;
                    // matrix_to_transform(hori_tf_matrix,t_transform);
                    // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "hori_frame", "base_link"));
        
                    pcl::PointCloud<PointType>  out_cloud;
                    pcl::transformPointCloud (full_cloud_in , out_cloud, hori_tf_matrix);

                    sensor_msgs::PointCloud2 hori_msg;
                    pcl::toROSMsg(out_cloud, hori_msg);
                    hori_msg.header.stamp = ros::Time::now();
                    hori_msg.header.frame_id = "base_link"; 
                    pub_hori.publish(hori_msg); 
                }
            }  
        }
 
        void velo_cloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        { 
            if(!os0_received) return;
            
            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rot_z( 0.0 , Eigen::Vector3f::UnitZ());

            Eigen::Translation3f init_trans(0.0,0.0,0.0);
            Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
           
            if(!velo_tf_initd){ 
                pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);

                 ROS_INFO("\n\n\n  Calibrate Velo ...");
                calibrate_PCLICP(full_cloud_in.makeShared(), os0_cloud.makeShared(), velo_tf_matrix); 
                // velo_tf_matrix = velo_tf_matrix * init_tf;
                Eigen::Matrix3f rot_matrix = velo_tf_matrix.block(0,0,3,3);
                Eigen::Vector3f trans_vector = velo_tf_matrix.block(0,3,3,1);
                
                // tf::Transform t_transform;
                // matrix_to_transform(velo_tf_matrix,t_transform);
                // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "velo_sensor", "base_link"));
    
                std::cout << "velo -> base_link " << trans_vector.transpose()
                    << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "velo_sensor"
                    << " /" << "base_link" << " 10" << std::endl;

                // publish result
                pcl::PointCloud<PointType>  out_cloud;
                pcl::transformPointCloud (full_cloud_in , out_cloud, velo_tf_matrix);

                sensor_msgs::PointCloud2 velo_msg;
                pcl::toROSMsg(out_cloud, velo_msg);
                velo_msg.header.stamp = ros::Time::now();
                velo_msg.header.frame_id = "base_link"; 
                pub_velo.publish(velo_msg); 

                velo_tf_initd = true;
            }else
            {
                // tf::Transform t_transform;
                // matrix_to_transform(velo_tf_matrix,t_transform);
                // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "velo_sensor", "base_link"));
    
                pcl::PointCloud<PointType>  out_cloud;
                pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);
                pcl::transformPointCloud (full_cloud_in , out_cloud, velo_tf_matrix);

                sensor_msgs::PointCloud2 velo_msg;
                pcl::toROSMsg(out_cloud, velo_msg);
                velo_msg.header.stamp = ros::Time::now();
                velo_msg.header.frame_id = "base_link"; 
                pub_velo.publish(velo_msg);  
            }
        }

        void os0_cloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        { 

            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rot_z(  M_PI / 4 , Eigen::Vector3f::UnitZ()); 
            Eigen::Translation3f init_trans(0.0,0.0,0.0);
            Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();

            Eigen::Matrix3f rot_matrix = init_tf.block(0,0,3,3);
            Eigen::Vector3f trans_vector = init_tf.block(0,3,3,1);

            if(!os0_tf_initd){
                 std::cout << "OS0 -> base_link " << trans_vector.transpose()
                    << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "os0_sensor"
                    << " /" << "base_link" << " 10" << std::endl;

                os0_tf_initd = true;
            }
            
            pcl::PointCloud<PointType>  out_cloud;
            pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);
            
            os0_cloud.clear();
            os0_cloud += full_cloud_in;  
            os0_received = true;
            // pcl::transformPointCloud (full_cloud_in , out_cloud, os0_tf_matrix);

            sensor_msgs::PointCloud2 os0_msg;
            pcl::toROSMsg(os0_cloud, os0_msg);
            os0_msg.header.stamp = ros::Time::now();
            os0_msg.header.frame_id = "base_link"; 
            pub_os0.publish(os0_msg); 
  
        }


        void os1_cloud_handler(const sensor_msgs::PointCloud2ConstPtr& pointCloudIn)
        {
            if(!os0_received) return;

            pcl::PointCloud<PointType>  full_cloud_in;
            pcl::fromROSMsg(*pointCloudIn, full_cloud_in);
            pcl_conversions::toPCL(pointCloudIn->header, full_cloud_in.header); 
             
            Eigen::AngleAxisf init_rot_x( 0.0 , Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf init_rot_y( 0.0 , Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf init_rot_z(  -M_PI / 4 , Eigen::Vector3f::UnitZ());

            Eigen::Translation3f init_trans(0.0,0.0,0.0);
            Eigen::Matrix4f init_tf = (init_trans * init_rot_z * init_rot_y * init_rot_x).matrix();
           
            if(!os1_tf_initd){ 
                pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);

                ROS_INFO("\n\n\n  Calibrate OS1 ...");
                calibrate_PCLICP(full_cloud_in.makeShared(), os0_cloud.makeShared(), os1_tf_matrix); 
                os1_tf_matrix = os1_tf_matrix * init_tf;
                Eigen::Matrix3f rot_matrix = os1_tf_matrix.block(0,0,3,3);
                Eigen::Vector3f trans_vector = os1_tf_matrix.block(0,3,3,1);
                
                // tf::Transform t_transform;
                // matrix_to_transform(os1_tf_matrix,t_transform);
                // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "os1_sensor", "base_link"));
    
                std::cout << "OS1 -> base_link " << trans_vector.transpose()
                    << " " << rot_matrix.eulerAngles(2,1,0).transpose() << " /" << "os1_sensor"
                    << " /" << "base_link" << " 10" << std::endl;

                // publish result
                pcl::PointCloud<PointType>  out_cloud;
                pcl::transformPointCloud (full_cloud_in , out_cloud, os1_tf_matrix);

                sensor_msgs::PointCloud2 os1_msg;
                pcl::toROSMsg(out_cloud, os1_msg);
                os1_msg.header.stamp = ros::Time::now();
                os1_msg.header.frame_id = "base_link"; 
                pub_os1.publish(os1_msg); 

                os1_tf_initd = true;
            }else
            {
                // tf::Transform t_transform;
                // matrix_to_transform(os1_tf_matrix,t_transform);
                // tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), "os1_sensor", "base_link"));
    
                pcl::PointCloud<PointType>  out_cloud;
                // pcl::transformPointCloud (full_cloud_in , full_cloud_in, init_tf);
                pcl::transformPointCloud (full_cloud_in , out_cloud, os1_tf_matrix);

                sensor_msgs::PointCloud2 os1_msg;
                pcl::toROSMsg(out_cloud, os1_msg);
                os1_msg.header.stamp = ros::Time::now();
                os1_msg.header.frame_id = "base_link"; 
                pub_os1.publish(os1_msg);  
            }
 
 
        }

        void calibrate_PCLICP(pcl::PointCloud<PointType>::Ptr source_cloud,
        pcl::PointCloud<PointType>::Ptr target_cloud, Eigen::Matrix4f &tf_marix, bool save_pcd =true)
        {
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::ratio<1, 1000>> time_span =
            std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);

            std::cout << "------------checking PCL GICP---------------- "<< std::endl;
            int pCount = source_cloud->size();

            pcl::PointCloud<PointType>::Ptr ds_cloud_in (new pcl::PointCloud<PointType> );
            pcl::PointCloud<PointType>::Ptr dstf_cloud_in (new pcl::PointCloud<PointType> );
            pcl::PointCloud<PointType>::Ptr ds_cloud_out (new pcl::PointCloud<PointType> ); 
            
            // // Remove the noise points
            pcl::PointCloud<PointType>::Ptr cloudin_filtered (new pcl::PointCloud<PointType>); 
            // pcl::StatisticalOutlierRemoval<PointType> sor;
            // sor.setInputCloud (source_cloud);
            // sor.setMeanK (50);
            // sor.setStddevMulThresh (1.0);
            // sor.filter (*cloudin_filtered); 
            // ROS_INFO_STREAM("CloudIn-> Removing noise clouds "<< source_cloud->size() << " "<<  cloudin_filtered->size() );
            removeFarPointCloud(*source_cloud, *cloudin_filtered, 50);
            ROS_INFO_STREAM("CloudIn-> Removing noise clouds "<< source_cloud->size() << " "<<  cloudin_filtered->size() );
 
            // Remove the noise points
            pcl::PointCloud<PointType>::Ptr cloudout_filtered (new pcl::PointCloud<PointType>);  
            // sor.setInputCloud (target_cloud);
            // sor.setMeanK (50);
            // sor.setStddevMulThresh (1.0);
            // sor.filter (*cloudout_filtered);
            // ROS_INFO_STREAM("CloudOut-> Removing noise clouds "<< target_cloud->size() << " "<<  cloudout_filtered->size() ); 
            removeFarPointCloud(*target_cloud, *cloudout_filtered, 50);
            ROS_INFO_STREAM("CloudOut-> Removing noise clouds "<< target_cloud->size() << " "<<  cloudout_filtered->size() ); 
        
            // Create the filtering object
            pcl::VoxelGrid< PointType> vox;
            vox.setInputCloud (cloudin_filtered);
            vox.setLeafSize (0.05f, 0.05f, 0.05f);
            vox.filter(*ds_cloud_in);
            // std::cout << "Source DS: " << source_cloud->size() << " ->  " << ds_cloud_in->size()<< std::endl;

            // Create the filtering object 
            vox.setInputCloud (cloudout_filtered);
            vox.setLeafSize (0.05f, 0.05f, 0.05f);
            vox.filter(*ds_cloud_out);
            // std::cout << "Target DS: " << target_cloud->size() << " -> " << ds_cloud_out->size()<< std::endl;
            
            // Init Rotation:
            // std::cout << "Rotating DS source  .... " << ds_cloud_in->size()<< std::endl; 
            // Eigen::Quaterniond rotate_tf =  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
            //                                 * Eigen::AngleAxisd(0,  Eigen::Vector3d::UnitY())
            //                                 * Eigen::AngleAxisd( M_PI/4, Eigen::Vector3d::UnitZ());
            // Eigen::Vector3d    trans_tf {0,0,0}; 
            // *dstf_cloud_in =  *dstf_cloud_in + *(transformCloud(*ds_cloud_in, rotate_tf, trans_tf));
            *dstf_cloud_in =  *dstf_cloud_in + *ds_cloud_in; 

            std::cout << "GICP start  .... " << ds_cloud_in->size() << " to "<< ds_cloud_out->size()<< std::endl;
            pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
            gicp.setTransformationEpsilon(0.001);
            gicp.setMaxCorrespondenceDistance(2);
            gicp.setMaximumIterations(500);
            gicp.setRANSACIterations(12);  
            gicp.setInputSource(dstf_cloud_in);
            gicp.setInputTarget(ds_cloud_out);

            pcl::PointCloud<PointType>::Ptr transformedP (new pcl::PointCloud<PointType>);

            t1 = std::chrono::steady_clock::now();
            gicp.align(*transformedP);
            t2 = std::chrono::steady_clock::now();
            time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
            // std::cout << "PCL gicp.align Time: " << time_span.count() << " ms."<< std::endl;
            std::cout << "has converged: " << gicp.hasConverged() << " score: " <<
                gicp.getFitnessScore() << std::endl; 

            auto transformation_matrix =  gicp.getFinalTransformation ();
            // std::cout << "transformation_matrix:\n"<<transformation_matrix << std::endl;
            // std::cout << std::endl;

            auto cloudSrc = dstf_cloud_in;
            auto cloudDst = ds_cloud_out;

            pcl::PointCloud<PointType> input_transformed;
            pcl::transformPointCloud (*cloudSrc, input_transformed, transformation_matrix);
        
            Eigen::Matrix3f rot_matrix = transformation_matrix.block<3,3>(0,0);
            Eigen::Vector3f euler = rot_matrix.eulerAngles(2, 1, 0); 
            // std::cout << "Rotation    : "<<  euler.x() << " " << euler.y() << " " << euler.z()<< std::endl;   
            // std::cout << "Rotation_matrix:\n"<<rot_matrix << std::endl; 

            tf_marix = transformation_matrix;

            if(save_pcd)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSrcRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size(),1));
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDstRGB (new pcl::PointCloud<pcl::PointXYZRGB>(cloudDst->size(),1));
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudALL (new pcl::PointCloud<pcl::PointXYZRGB>(cloudSrc->size() + cloudDst->size(),1));
                
                // Fill in the CloudIn data
                for (int i = 0; i < cloudSrc->size(); i++)
                {
                    pcl::PointXYZRGB &pointin = (*cloudSrcRGB)[i];
                    pointin.x = (input_transformed)[i].x;
                    pointin.y = (input_transformed)[i].y;
                    pointin.z = (input_transformed)[i].z;
                    pointin.r = 255;
                    pointin.g = 0;
                    pointin.b = 0;
                    (*cloudALL)[i] = pointin;
                }
                for (int i = 0; i < cloudDst->size(); i++)
                {
                    pcl::PointXYZRGB &pointout = (*cloudDstRGB)[i];
                    pointout.x = (*cloudDst)[i].x;
                    pointout.y = (*cloudDst)[i].y;
                    pointout.z = (*cloudDst)[i].z;
                    pointout.r = 0;
                    pointout.g = 255;
                    pointout.b = 255;
                    (*cloudALL)[i+cloudSrc->size()] = pointout;
                } 
                pcl::io::savePCDFile<pcl::PointXYZRGB> ("/home/qing/icp_ICP.pcd", *cloudALL);
            }
        } 
};

int 
main(int argc, char **argv)
{  
    ros::init(argc, argv, "Lidar Calibrate"); 

    LidarsCalibrater swo; 
    ROS_INFO("\033[1;32m---->\033[0m Lidar Calibrate Started.");  

    ros::Rate r(10);
    while(ros::ok()){   

        ros::spinOnce(); 
        r.sleep(); 
  
    }


    return 0;
}
