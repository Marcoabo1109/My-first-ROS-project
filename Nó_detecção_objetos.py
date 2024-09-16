#!/bb/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.object_pub = rospy.Publisher('/detected_object', Point, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Converte a imagem para HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define o intervalo de cor para detecção (exemplo: vermelho)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Cria uma máscara
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Encontra contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Pega o maior contorno
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Publica a posição do objeto
                object_position = Point(x=cx, y=cy, z=0)
                self.object_pub.publish(object_position)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    detector = ObjectDetector()
    detector.run()
