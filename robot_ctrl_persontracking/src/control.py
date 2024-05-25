import sys
import numpy as np
import cv2
import rospy
import message_filters

from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist

# width 640

class RobotControl:
    def __init__(self):
        self.image_cb = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.yolov5_cb = message_filters.Subscriber('/yolov5/detections', BoundingBoxes)
        self.ts = message_filters.TimeSynchronizer([self.image_cb, self.yolov5_cb], 10)
        self.ts.registerCallback(self.callback)
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()  # twist 객체를 멤버 변수로 초기화
        self.person_detect_time = rospy.get_time()  # 마지막 감지 시간을 멤버 변수로 초기화

    def callback(self, img_msg, bbox_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        
        img_cv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            img_cv = img_cv.byteswap().newbyteorder()
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)

        # twist 객체를 초기화하는 부분을 제거하고 멤버 변수를 사용
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        current_time = rospy.get_time()

        for bbox in bbox_msg.bounding_boxes:
            print(bbox.Class)
            cv2.rectangle(img_cv, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
            if bbox.Class == "person":
                x_mid = (bbox.xmax + bbox.xmin)//2
                if 222<x_mid<444:
                    self.twist.linear.x = 0.25
                elif 0<x_mid<=222:
                    self.twist.angular.z = 0.05
                else:
                    self.twist.angular.z = -0.05
                self.person_detect_time = current_time  # 감지 시간 업데이트

        # 감지 후 3초가 지났는지 확인
        if (current_time - self.person_detect_time > 3):
            print("current time:", current_time)
            print("detect time:", self.person_detect_time)
            self.twist.angular.z = 0.1

        self.pub.publish(self.twist)    
        cv2.imwrite('aaa.jpg', img_cv)

if __name__ == "__main__":
    rospy.init_node("robot_control", anonymous=True)
    control = RobotControl()
    rospy.spin()
