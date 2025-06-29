#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.pub = rospy.Publisher('/person_detected', Bool, queue_size=10)
        self.bridge = CvBridge()
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # ✅ 订阅实际的图像话题名（根据你运行 astra.launch 的输出为准）
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.loginfo("🎥 CameraNode has started.")
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            detected = len(faces) > 0
            self.pub.publish(Bool(data=detected))

            if detected:
                rospy.loginfo("👤 Face detected")

            # ✅ 可视化框出人脸
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # ✅ 显示画面
            cv2.imshow("Jupiter Camera - Face Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("❌ CV error: %s", str(e))

if __name__ == '__main__':
    try:
        CameraNode()
    except rospy.ROSInterruptException:
        pass
