import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from sensor_msgs.msg import Image
import std_msgs.msg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading


class DataLoader(Node):
    def __init__(self):
        super().__init__('data_loader')
        
        self.publisher_ = self.create_publisher(Image, '/camera/fisheye1/image_raw', 10)

        self.bridge = CvBridge()

        self.img_msgs = []
        self.mmw_msgs = []


    def load_data(self):
        # create reader instance and open for reading
        with Reader('/home/nm/Downloads/rosbag2_2022_10_14-11_14_30') as reader:

            # iterate over messages
            for connection, timestamp, rawdata in reader.messages():

                if connection.topic == '/camera/fisheye1/image_raw':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    self.img_msgs.append(msg)

                elif connection.topic == '/iwr6843_pcl':
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

            #print("Now: ", now_delta, " Orig: ", orig_delta)

            # wait the correct amount of time before publishing
            while(not(now_delta > orig_delta)):
                now_time = self.get_clock().now().to_msg()
                now_sec_delta = now_time.sec - now_start_sec
                now_nano_delta = now_time.nanosec - now_start_nanosec
                now_delta = now_sec_delta*1000 + now_nano_delta/1000000       


            it = it + 1
            #print("it ", it)

            # convert original image to numpy array
            cv_image = self.bridge.imgmsg_to_cv2(self.img_msgs[i], desired_encoding='passthrough')

            # create new message with numpy array as data
            img_msg = Image()
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
            img_msg.header = std_msgs.msg.Header()
            img_msg.header.stamp = self.get_clock().now().to_msg() #msg.header.stamp
            #print(self.get_clock().now().to_msg())
            img_msg.header.stamp.sec = self.img_msgs[i].header.stamp.sec
            img_msg.header.stamp.nanosec = self.img_msgs[i].header.stamp.nanosec
            img_msg.header.frame_id = self.img_msgs[i].header.frame_id
            img_msg.height = self.img_msgs[i].height
            img_msg.width = self.img_msgs[i].width
            img_msg.encoding = self.img_msgs[i].encoding
            img_msg.is_bigendian = self.img_msgs[i].is_bigendian
            img_msg.step = self.img_msgs[i].step
            self.publisher_.publish(img_msg)

        print("Images finished")


    def republish_mmw(self):
        print("Republishing radar data...")
        print("Radar data finished")


def main():
    #init
    rclpy.init()
    minimal_publisher = DataLoader()
    minimal_publisher.load_data()

    mmw_thread = threading.Thread(target=DataLoader.republish_mmw)
    mmw_thread.start()

    minimal_publisher.republish_img()

    print("Done")
    rclpy.spin_once(minimal_publisher)

    #shutdown
    mmw_thread.join()
    minimal_publisher.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()