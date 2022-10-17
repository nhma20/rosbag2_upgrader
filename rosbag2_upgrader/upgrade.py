import struct
import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from sensor_msgs.msg import Image, PointCloud2, PointField
import std_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading
import sys
import os
import time
import subprocess
import signal


#/home/nm/Downloads/rosbag2_2022_10_14-11_14_30 # T265
#/home/nm/Downloads/PL_MAPPER_bags/bag_uncompressed # both img and pcl

global parsing_radar_done
parsing_radar_done = 0
global parsing_img_done
parsing_img_done = 0

class DataLoader(Node):
    def __init__(self, bag_path, img_topic, mmw_topic):
        super().__init__('data_loader')
        

        self.bag_path = bag_path#'/home/nm/Downloads/PL_MAPPER_bags/sim_topics'

        self.img_topic = img_topic
        self.mmw_topic = mmw_topic



        self.img_pub_topic = img_topic
        self.mmw_pub_topic = mmw_topic
        

        self.img_publisher_ = self.create_publisher(Image, self.img_pub_topic, 10)
        self.mmw_publisher_ = self.create_publisher(PointCloud2, self.mmw_pub_topic, 10)

        self.bridge = CvBridge()

        self.img_msgs = []
        self.mmw_msgs = []


    def pcl_to_numpy(self, pcl_msg):

        points = []

        n_points = int(len(pcl_msg.data)/3/4)

        for i in range(n_points):
            point = [0,0,0]

            for j in range(3):
                point[j] = struct.unpack("f", pcl_msg.data[i*12+j*4:i*12+(j+1)*4])[0]

            points.append(point)
        
        arr = np.asarray(points)

        return arr


    def load_data(self):
        # create reader instance and open for reading
        with Reader(self.bag_path) as reader:

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():

                if connection.topic == self.img_topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    self.img_msgs.append(msg)

                elif connection.topic == self.mmw_topic:
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    self.mmw_msgs.append(msg)

        print("Finished loading messages")


    def republish_img(self):   
        print("Republishing images...")

        now_start_time = 0
        now_start_sec = 0
        now_start_nanosec = 0
        orig_start_sec = 0
        orig_start_nanosec = 0
        now_nano_delta = 0
        now_sec_delta = 0
        orig_nano_delta = 0
        orig_sec_delta = 0
        now_delta = 0
        orig_delta = 0
        it = 0

        # iterate over messages
        for i in range(len(self.img_msgs)):

            if it == 0:
                now_start_time = self.get_clock().now().to_msg()
                now_start_sec = now_start_time.sec
                now_start_nanosec = now_start_time.nanosec
                orig_start_sec = self.img_msgs[0].header.stamp.sec
                orig_start_nanosec = self.img_msgs[0].header.stamp.nanosec
            else:
                orig_sec_delta = self.img_msgs[i].header.stamp.sec - orig_start_sec
                orig_nano_delta = self.img_msgs[i].header.stamp.nanosec - orig_start_nanosec
                orig_delta = orig_sec_delta*1000 + orig_nano_delta/1000000

            # wait the correct amount of time before publishing
            while(not(now_delta > orig_delta)):
                now_time = self.get_clock().now().to_msg()
                now_sec_delta = now_time.sec - now_start_sec
                now_nano_delta = now_time.nanosec - now_start_nanosec
                now_delta = now_sec_delta*1000 + now_nano_delta/1000000       

            it = it + 1

            # convert original image to numpy array
            cv_image = self.bridge.imgmsg_to_cv2(self.img_msgs[i], desired_encoding='passthrough')

            # create new message with numpy array as data
            img_msg = Image()
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
            img_msg.header = std_msgs.msg.Header()
            img_msg.header.stamp = self.get_clock().now().to_msg() #msg.header.stamp
            img_msg.header.stamp.sec = self.img_msgs[i].header.stamp.sec
            img_msg.header.stamp.nanosec = self.img_msgs[i].header.stamp.nanosec
            img_msg.header.frame_id = self.img_msgs[i].header.frame_id
            img_msg.height = self.img_msgs[i].height
            img_msg.width = self.img_msgs[i].width
            img_msg.encoding = self.img_msgs[i].encoding
            img_msg.is_bigendian = self.img_msgs[i].is_bigendian
            img_msg.step = self.img_msgs[i].step
            self.img_publisher_.publish(img_msg)

        global parsing_img_done
        parsing_img_done = 1
        print("Images finished")


    def republish_mmw(self):
        print("Republishing radar data...")

        now_start_time = 0
        now_start_sec = 0
        now_start_nanosec = 0
        orig_start_sec = 0
        orig_start_nanosec = 0
        now_nano_delta = 0
        now_sec_delta = 0
        orig_nano_delta = 0
        orig_sec_delta = 0
        now_delta = 0
        orig_delta = 0
        it = 0

        # iterate over messages
        for i in range(len(self.mmw_msgs)):

            if it == 0:
                now_start_time = self.get_clock().now().to_msg()
                now_start_sec = now_start_time.sec
                now_start_nanosec = now_start_time.nanosec
                orig_start_sec = self.mmw_msgs[0].header.stamp.sec
                orig_start_nanosec = self.mmw_msgs[0].header.stamp.nanosec
            else:
                orig_sec_delta = self.mmw_msgs[i].header.stamp.sec - orig_start_sec
                orig_nano_delta = self.mmw_msgs[i].header.stamp.nanosec - orig_start_nanosec
                orig_delta = orig_sec_delta*1000 + orig_nano_delta/1000000

            # wait the correct amount of time before publishing
            while(not(now_delta > orig_delta)):

                now_time = self.get_clock().now().to_msg()
                now_sec_delta = now_time.sec - now_start_sec
                now_nano_delta = now_time.nanosec - now_start_nanosec
                now_delta = now_sec_delta*1000 + now_nano_delta/1000000       

            it = it + 1

            # create new message with numpy array as data
            pcl_msg = PointCloud2()
            pcl_msg.header = std_msgs.msg.Header()
            pcl_msg.header.stamp = self.get_clock().now().to_msg() #msg.header.stamp
            pcl_msg.header.stamp.sec = self.mmw_msgs[i].header.stamp.sec
            pcl_msg.header.stamp.nanosec = self.mmw_msgs[i].header.stamp.nanosec
            pcl_msg.header.frame_id = self.mmw_msgs[i].header.frame_id
            pcl_msg.height = self.mmw_msgs[i].height
            pcl_msg.width = self.mmw_msgs[i].width
            pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
            pcl_msg.point_step = self.mmw_msgs[i].point_step #size of 1 point (float32 * dimensions (3 when xyz))
            pcl_msg.row_step = pcl_msg.point_step*pcl_msg.width  # only 1 row because unordered
            pcl_msg.is_dense = True
            pcl_msg.data = self.mmw_msgs[i].data.tostring()

            self.mmw_publisher_.publish(pcl_msg)

        global parsing_radar_done
        parsing_radar_done = 1
        print("Radar data finished")



def main(argv=None):

    if len(sys.argv) == 5:
        bag_path = sys.argv[1] 
        img_topic = sys.argv[2]
        mmw_topic = sys.argv[3]
        bag_dir = sys.argv[4]
    elif len(sys.argv) > 5:
        print("Too many arguments given. Usage: \n ros2 run rosbag2_upgrader upgrade <bag_path> <camera_topic> <radar_topic>")
        exit()
    elif len(sys.argv) < 5:
        print("Too few arguments given. Usage: \n ros2 run rosbag2_upgrader upgrade <bag_path> <camera_topic> <radar_topic>")
        exit()


    #init
    rclpy.init()
    minimal_publisher = DataLoader(bag_path=bag_path, img_topic=img_topic, mmw_topic=mmw_topic)
    minimal_publisher.load_data()

    recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_dir, img_topic, mmw_topic])

    time.sleep(0.5)

    mmw_thread = threading.Thread(target=minimal_publisher.republish_mmw)
    cam_thread = threading.Thread(target=minimal_publisher.republish_img)

    mmw_thread.start()
    cam_thread.start()


    global parsing_img_done
    global parsing_radar_done
    while(parsing_img_done != 1 or parsing_radar_done != 1):
        time.sleep(0.5)

    #shutdown
    mmw_thread.join()
    cam_thread.join()
    recorder_pid = recorder.pid
    os.kill(recorder_pid, signal.SIGINT) # ctrl+c to ros2 bag record

    print("Done")
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    exit()
    



if __name__ == '__main__':
    main()
