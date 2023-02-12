from camera import DemoApp, ImagePublisher
from robot import listner, HelloRobot
import cv2
from multiprocessing import Process, Value
import time


def camera_process(app):
    camera_publisher = ImagePublisher(app)
    camera_publisher.publish_image_from_camera()

def robot_process(hello_robot, stop_listening):
    print("robot process started")
    listner(hello_robot, stop_listening)

def thread_manager():
    # app = DemoApp()
    # while app.stream_stopped:
    #     try:
    #         app.connect_to_device(dev_idx=0)
    #     except RuntimeError as e:
    #         print(e)
    #         print("Retrying to connect to device, make sure the device is connected...")
    #         time.sleep(2)
    # t1 = Thread(target=camera_process, args=(app,))
    hello_robot = HelloRobot()
    hello_robot.initialize_home_params(home_lift = 0.5)
    stop_listening = Value('b', False)
    # t2 = Process(target=robot_process, args=(hello_robot, stop_listening, ))
    # t2.start()
    robot_process(hello_robot, stop_listening)
    
    # # import pdb; pdb.set_trace()
    # try:
    #     camera_process(app)
    # except cv2.error as e:
    #     print(e)
    #     print("The device was connected but the stream didn't start, trying to reconnect...")

    # while not app.stream_stopped:
    #     #sleep for 2 second
    #     time.sleep(2)
    #stop the thread t2
    time.sleep(10)
    print("stopping the thread")
    stop_listening.value = True
    # t2.join()
    # thread_manager()


if __name__ == '__main__':
    thread_manager()