import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg      import Odometry
import tf
  

def opti_cb(data): 
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    f = open("ptitrack.csv", "a")
    f.write("{},{},{},{},{},{},{},{},{},{},{}, \n".format(data.header.stamp,  data.pose.position.x  
                                            , data.pose.position.y,  data.pose.position.z
                                            , roll, pitch, yaw 
                                            , data.pose.orientation.x , data.pose.orientation.y 
                                            , data.pose.orientation.z , data.pose.orientation.w ))
    # print(data.header.stamp,  data.pose.position.x  , data.pose.position.y,  data.pose.position.z)
    f.close()

def loam_cb(data): 
    print("-> Odom reveived .. ", data.header.stamp )
    f = open("loamPose.csv", "a")

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    f.write("{},{},{},{},{},{},{},{},{},{},{}, \n".format(data.header.stamp,  data.pose.pose.position.x  
                                            , data.pose.pose.position.y,  data.pose.pose.position.z
                                            , roll, pitch, yaw 
                                            , data.pose.pose.orientation.x , data.pose.pose.orientation.y 
                                            , data.pose.pose.orientation.z , data.pose.pose.orientation.w ))
    # print(data.header.stamp,  data.pose.pose.position.x   , data.pose.pose.position.y,  data.pose.pose.position.z )
     
    f.close()

def listener(): 
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/Odometry", Odometry, loam_cb) 
    rospy.Subscriber("/aft_mapped_to_init", Odometry, loam_cb)                  # lego-loam
    rospy.Subscriber("/livox_odometry_mapped", Odometry, loam_cb)               # lio-livox
    rospy.Subscriber("/vrpn_client_node/UWBTest/pose", PoseStamped, opti_cb)    # optrick
    rospy.Subscriber("/vrpn_client_node/optitest/pose", PoseStamped, opti_cb)   # optrick
    rospy.spin()

if __name__ == '__main__':
    listener()