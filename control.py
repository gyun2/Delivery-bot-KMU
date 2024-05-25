# import sys
# import numpy as np
# import cv2
# import rospy
# import message_filters
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from sensor_msgs.msg import Image
# from detection_msgs.msg import BoundingBox, BoundingBoxes
# from geometry_msgs.msg import Twist

# # width 640

# def send_goal():
#     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
#     client.wait_for_server()

#     goal = MoveBaseGoal()
#     goal.target_pose.header.frame_id = "map"
#     goal.target_pose.header.stamp = rospy.Time.now()
#     goal.target_pose.pose.position.x = 0.5
#     goal.target_pose.pose.orientation.w = 1.0

#     client.send_goal(goal)
#     wait = client.wait_for_result()
#     if not wait:
#         rospy.logerr("Action server not available!")
#         rospy.signal_shutdown("Action server not available!")
#     else:
#         return client.get_result()

# class RobotControl:
#     def __init__(self):
#         self.image_cb = message_filters.Subscriber('/d400/color/image_raw', Image)
#         self.yolov5_cb = message_filters.Subscriber('/yolov5/detections', BoundingBoxes)
#         self.ts = message_filters.TimeSynchronizer([self.image_cb, self.yolov5_cb], 10)
#         self.ts.registerCallback(self.callback)
        
#         self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#         self.twist = Twist()  # twist 객체를 멤버 변수로 초기화
#         self.person_detect_time = rospy.get_time()  # 마지막 감지 시간을 멤버 변수로 초기화

#     def callback(self, img_msg, bbox_msg):
#         dtype = np.dtype("uint8")
#         dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        
#         img_cv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
#         if img_msg.is_bigendian == (sys.byteorder == 'little'):
#             img_cv = img_cv.byteswap().newbyteorder()
#         img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)

#         # twist 객체를 초기화하는 부분을 제거하고 멤버 변수를 사용
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = 0.0
#         current_time = rospy.get_time()

#         for bbox in bbox_msg.bounding_boxes:
#             print(bbox.Class)
#             cv2.rectangle(img_cv, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
#             if bbox.Class == "bottle":
#                 # result = send_goal()
#                 # print("1")
#                 x_mid = (bbox.xmax + bbox.xmin)//2
#                 if 222<x_mid<444:
#                     self.twist.linear.x = 0.25
#                 elif 0<x_mid<=222:
#                     self.twist.angular.z = 0.05
#                 else:
#                     self.twist.angular.z = -0.05
#                 self.person_detect_time = current_time  # 감지 시간 업데이트

#         # 감지 후 3초가 지났는지 확인
#         if (current_time - self.person_detect_time > 3):
#             print("current time:", current_time)
#             print("detect time:", self.person_detect_time)
#             self.twist.angular.z = 0.1

#         self.pub.publish(self.twist)    
#         cv2.imwrite('aaa.jpg', img_cv)

# if __name__ == "__main__":
#     rospy.init_node("robot_control", anonymous=True)
#     control = RobotControl()
#     rospy.spin()

import sys
import numpy as np
import cv2
import rospy
import message_filters
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist


coords_map={
    "5041": (-7.794, 6.902, 1.0),
    "503" : (-1, 2, 1.0)
}



class RobotControl:
    def __init__(self):
        # Initialize Action Client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.image_cb = message_filters.Subscriber('/d400/color/image_raw', Image)
        self.yolov5_cb = message_filters.Subscriber('/yolov5/detections', BoundingBoxes)
        self.ts = message_filters.TimeSynchronizer([self.image_cb, self.yolov5_cb], 10)
        self.ts.registerCallback(self.callback)

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()  # Initialize twist object as a member variable
        self.person_detect_time = rospy.get_time()  # Initialize last detection time as a member variable

    def send_goal(self, t):
        x,y,w = t
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def callback(self, img_msg, bbox_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        img_cv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            img_cv = img_cv.byteswap().newbyteorder()
        img_cv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)

        # Reset twist object
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        current_time = rospy.get_time()

        curr_number= []
        for bbox in bbox_msg.bounding_boxes:
            print(bbox.Class)
            curr_number.append((int(bbox.xmin), bbox.Class))
            cv2.rectangle(img_cv, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255, 0, 0), 2)
            # if bbox.Class == "bottle":
            #     result = self.send_goal()
            #     if result:
            #         rospy.loginfo("Goal successfully sent and reached")
            #     else:
            #         rospy.logerr("Failed to reach the goal")
        curr_number.sort() 
        print("result:", end=" ")
        result_str = ""
        for i in range(len(curr_number)):
            print(curr_number[i][1], end='')
            result_str+=curr_number[i][1]
        print()
        
        dest = coords_map.get(result_str, "not_found")
        if dest == "not_found":
            print("not found!!")
        else:
            result = self.send_goal(dest)
            if result:
                rospy.loginfo("Goal successfully sent and reached")
            else:
                rospy.logerr("Failed to reach the goal")


        self.pub.publish(self.twist)
        cv2.imwrite('aaa.jpg', img_cv)

if __name__ == "__main__":
    rospy.init_node("robot_control", anonymous=True)
    control = RobotControl()
    rospy.spin()