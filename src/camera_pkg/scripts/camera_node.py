#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import numpy as np

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.bridge = CvBridge()
        self.face_pub = rospy.Publisher('/person_detected', Bool, queue_size=10)
        self.gesture_pub = rospy.Publisher('/hand_gesture', Bool, queue_size=10)  # True=OK, False=Open Palm

        # 人脸检测器
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # MediaPipe 手部模型
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False,
                                         max_num_hands=1,
                                         min_detection_confidence=0.5,
                                         min_tracking_confidence=0.5)

        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.loginfo("🎥 CameraNode with Face + Hand detection started.")
        rospy.spin()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 人脸检测
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
            self.face_pub.publish(Bool(data=len(faces) > 0))
            if len(faces) > 0:
                rospy.loginfo("👤 Face detected")
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 手势检测
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb_frame)
            gesture_detected = None

            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    gesture_detected = self.classify_gesture(hand_landmarks)
                    if gesture_detected is not None:
                        self.gesture_pub.publish(Bool(data=gesture_detected))
                        gesture_name = "OK" if gesture_detected else "Open Palm"
                        rospy.loginfo(f"✋ Hand gesture: {gesture_name}")
                    mp.solutions.drawing_utils.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )

            # 显示图像
            cv2.imshow("Camera - Face + Gesture Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"❌ Error in image callback: {str(e)}")

    def classify_gesture(self, landmarks):
        """
        判断是否为 OK 或张开手掌：
        - OK: 拇指指尖靠近食指指尖
        - 张开: 手指都张开，拇指不贴近其他指尖
        """
        thumb_tip = landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]

        # 计算拇指和食指距离
        dist_thumb_index = np.linalg.norm(np.array([thumb_tip.x, thumb_tip.y]) - np.array([index_tip.x, index_tip.y]))
        dist_thumb_middle = np.linalg.norm(np.array([thumb_tip.x, thumb_tip.y]) - np.array([middle_tip.x, middle_tip.y]))

        # OK: 距离很小（0.05 是经验值）
        if dist_thumb_index < 0.05:
            return True
        # 张开手掌：拇指远离中指
        elif dist_thumb_middle > 0.1:
            return False
        else:
            return None  # 无法识别

if __name__ == '__main__':
    try:
        CameraNode()
    except rospy.ROSInterruptException:
        pass
