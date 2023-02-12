from .tensor_subscriber import TensorSubscriber
from .hello_robot import HelloRobot
import rospy
from std_msgs.msg import Int64
import random
import pickle
import argparse
from multiprocessing import Value
PING_TOPIC_NAME = 'run_model_ping'


parser = argparse.ArgumentParser()
parser.add_argument('--lift', type=float, default=0.5, help='position of robot lift')
parser.add_argument('--arm', type=str, default=0.02, help='arm position')
parser.add_argument('--base', type=float, default=0.0, help='position of robot base')
parser.add_argument('--yaw', type=float, default=0.0, help='position of robot wrist yaw')
parser.add_argument('--pitch', type=float, default=0.0, help='position of robot wrist pitch')
parser.add_argument('--roll', type=float, default=0.0, help='position of robot wrist roll')
parser.add_argument('--gripper', type=float, default=1.0, help='position of robot gripper')

args = parser.parse_args()
params = vars(args)

def listner(hello_robot = None, stop_listening = Value('b', False)):

    try:
        rospy.init_node('Acting_node')
    except rospy.exceptions.ROSException:
        print('node already initialized')
    
    print('setting up publisher')
    publisher = rospy.Publisher(PING_TOPIC_NAME, Int64, queue_size=1)
    
    tensor_sub_object = TensorSubscriber()
    rate = rospy.Rate(5)
    hello_robot = HelloRobot() if hello_robot is None else hello_robot
    print('homing')
    hello_robot.home()
    print('homed')
    while not stop_listening.value:
        # x = input()
        
        uid = random.randint(0,30000)
        publisher.publish(Int64(uid))
        print('published', uid)
        waiting = True
        while waiting and not stop_listening.value:
            print('waiting 1')
            if ((tensor_sub_object.tr_data_offset == uid) and (tensor_sub_object.rot_data_offset == uid) \
                and (tensor_sub_object.gr_data_offset == uid)) or (tensor_sub_object.home_data_offset==uid):

                waiting = False

            rate.sleep()

        if stop_listening.value:
            break
        elif tensor_sub_object.home_data_offset==uid:
            hello_robot.home()
            rate.sleep()
        else:
            hello_robot.move_to_pose(tensor_sub_object.translation_tensor, tensor_sub_object.rotational_tensor, tensor_sub_object.gripper_tensor)
            rate.sleep()
    
    print('stopped listening', stop_listening.value)
        

if __name__ == '__main__':
    hello_robot = HelloRobot()
    hello_robot.initialize_home_params(params['lift'], params['arm'], params['base'], 
                                        params['yaw'], params['pitch'], params['roll'], params['gripper'])
    listner(hello_robot)

