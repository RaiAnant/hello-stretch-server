import cv2
import rospy

import numpy as np

from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge, CvBridgeError
from rospy_tutorials.msg import Floats

import sys
sys.path.append("/home/hello-robot/record3d/")
import demo 

NODE_NAME = 'gopro_node'
IMAGE_PUBLISHER_NAME = '/gopro_image'
DEPTH_PUBLISHER_NAME = '/gopro_depth'

app = demo.DemoApp()
app.connect_to_device(dev_idx=0)
print('connected')

class ImagePublisher (object):
    def __init__(self, app):
        # Initializing camera
        self.app = app
        # Initializing ROS node
        rospy.init_node(NODE_NAME)
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(IMAGE_PUBLISHER_NAME, Image, queue_size = 1)
        self.depth_publisher = rospy.Publisher(DEPTH_PUBLISHER_NAME, numpy_msg(Floats), queue_size = 1)

    def publish_image_from_camera(self):
        rate = rospy.Rate(28)
        while True:

            image, depth, pose = self.app.start_process_image()

            image = np.moveaxis(image, [0], [1])[...,::-1,::-1]
            depth = np.moveaxis(depth, [0], [1])[...,::-1,::-1]

            # Creating a CvBridge and publishing the data to the rostopic
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            self.image_publisher.publish(self.image_message)
            self.depth_publisher.publish(depth)

            # Stopping the camera
            if cv2.waitKey(1) == 27:
                break

            rate.sleep()

        cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_publisher = ImagePublisher(app)
    #print('calling publisher')
    camera_publisher.publish_image_from_camera()
    #print('publisher end')
