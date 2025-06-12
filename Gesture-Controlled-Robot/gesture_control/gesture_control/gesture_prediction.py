import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
import math
import joblib

from custom_msgs.msg import Input

class GesturePredictionNode(Node):
    def __init__(self, name = 'GesturePredictionNode'):
        super().__init__(name)
        print("My name is:", name)
        self.clf = joblib.load("/home/tasodio4/ESA/ROS_PRACTICE/Gesture-Controlled-Robot/gesture_control/models/gesture_classifier.pkl")
        self.class_names = np.load("/home/tasodio4/ESA/ROS_PRACTICE/Gesture-Controlled-Robot/gesture_control/models/class_names.npy")
        

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1)
        self.mp_draw = mp.solutions.drawing_utils

        self.gesture_pub = self.create_publisher(Input, '/cmd_vel', 10)
        print(self.class_names)
        self.gesture_prediction()

    def gesture_prediction(self):
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(image)

            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    landmarks = []
                    for lm in hand_landmarks.landmark:
                        landmarks.extend([lm.x, lm.y, lm.z])

                    prediction = self.clf.predict([landmarks])[0]
                    confidence = self.clf.predict_proba([landmarks])[0][prediction]

                    gesture = self.class_names[prediction]


                    gesture_msg = Input()

                    if gesture == 'rock':
                        gesture_msg.linear_vel = 1.0
                        gesture_msg.angular_vel = 0.0
                        gesture_msg.duration = 0.1
                    elif gesture == 'paper':
                        gesture_msg.linear_vel = 0.0
                        gesture_msg.angular_vel = math.pi / 4
                        gesture_msg.duration = 0.1

                    elif gesture == 'scissors':
                        gesture_msg.linear_vel = -1.0
                        gesture_msg.angular_vel = 0.0
                        gesture_msg.duration = 0.1

                    
                    self.gesture_pub.publish(gesture_msg)
                    cv2.putText(frame, f"{gesture} ({confidence:.2f})", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

            cv2.imshow("Gesture Prediction", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

def main(args = None):
    rclpy.init(args=args)       

    gesture = GesturePredictionNode()

    rclpy.spin(gesture)


    gesture.destroy_node()

    rclpy.shutdown()



