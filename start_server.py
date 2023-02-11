from camera import demo, publisher
from robot import listner, hello_robot

from threading import Thread
import time

def camera_process(app):
    print('connected')
    camera_publisher = publisher.ImagePublisher(app)
    camera_publisher.publish_image_from_camera()

def robot_process(hello_robot):
    listner(hello_robot)

def thread_manager():
    app = demo.DemoApp()
    app.connect_to_device(dev_idx=0)
    t1 = Thread(target=camera_process, args=(app,))
    t2 = Thread(target=robot_process, args=(hello_robot,))
    t1.start()
    t2.start()
    while not app.stream_stopped and t2.is_alive():
        #sleep for 2 second
        time.sleep(2)
    thread_manager()


if __name__ == '__main__':
    thread_manager()