
#!/usr/bin/env python3

from tkinter import X
import rospy
import struct

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import numpy as np
import ros_numpy as rnp
import sensor_msgs
import message_filters
import math
from mpl_toolkits import mplot3d
import time


import matplotlib.pyplot as plt

class pointcloud_subscriber:
    def __init__(self):

        rospy.init_node('listener', anonymous=True)

        print("Entered class")
        self.fig = plt.figure()
        self.ax = plt.axes(projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        

        # rospy.Subscriber("/ns1/velodyne_points", PointCloud2, self.listener_callback)
        self.lidar1_sub = message_filters.Subscriber('/ns1/velodyne_points', PointCloud2)
        self.lidar2_sub = message_filters.Subscriber('/ns2/velodyne_points', PointCloud2)
        self.pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar1_sub, self.lidar2_sub], 10, 0.00999) # Changed code filters/ subscribers/ fs, queue_size, slop/ delay (in seconds) with which messages can be synchronized
        # self.ts = message_filters.TimeSynchronizer([self.lidar1_sub, self.lidar2_sub], 50) 
        self.ts.registerCallback(self.listener_callback)
        print("Callback ended")
        # plt.show()
        rospy.spin()



    def listener_callback(self, cloud1, cloud2):
        print("Listening START")
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        # msg.__class = PointCloud2
        # offset_sorted = {f.offset: f for f in msg.fields}
        # msg.fields = [f for(_, f) in sorted(offset_sorted.items())]
        print("CLoud1 message timings: ", cloud1.header)
        print("CLoud2 message timings: ", cloud2.header)
        # time.sleep(20000)
        cloud1_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(cloud1,remove_nans=True) #vlp16_port
        cloud2_xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(cloud2,remove_nans=True) #vlp16_starboard
        
        x = 0
        y = -1.320 #Extracted from tf
        z = 0
        
        tf_1_to_2 = [[1,   0,   0,   x],
                     [0,   1,   0,   y],
                     [0,   0,   1,   z],
                     [0,   0,   0,   1]]
        print("Cloud1 XYZ array is of size: ", cloud1_xyz.shape)
        print("Cloud2 XYZ array is of size: ", cloud2_xyz.shape)

        cloud2_xyz_wrt_1 = np.array([x, y, z]) + cloud2_xyz
        print("Cloud2 wrt 1 XYZ array is of size: ", cloud2_xyz_wrt_1.shape)
        print("Original cloud2: ", cloud2_xyz[0])
        print("Transformed cloud2: ", cloud2_xyz_wrt_1[0])

        final_wrt_1 = np.concatenate((cloud1_xyz, cloud2_xyz_wrt_1))
        final_wrt_1 = np.unique(final_wrt_1, axis=0)
        print("Final cloud: ", final_wrt_1.shape)

        #final_cloud = rnp.point_cloud2.array_to_pointcloud2(final_wrt_1, stamp=None, frame_id='vlp16_port')
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]

        header = Header()
        header.frame_id = "vlp16_port"
        header.stamp = rospy.Time.now()

        x = final_wrt_1[:,0].T
        y = final_wrt_1[:,1].T
        z = final_wrt_1[:,2].T

        # print(x, x.shape)
        # z = 0.5 * np.sin(2*x - count/10.0) * np.sin(2*y)
        points = np.array([x,y,z,z]).reshape(4,-1).T

        pc2 = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(pc2)

        print("Final XYZ to PCL2 conversion completed")

        # self.pub.publish(pc2)
        print("Publishing completed")



def main(args=None):
    points_filteration = pointcloud_subscriber()


if __name__ == '__main__':
    main()