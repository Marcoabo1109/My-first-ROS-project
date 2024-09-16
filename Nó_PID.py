#!/bb/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller')
        self.object_sub = rospy.Subscriber('/detected_object', Point, self.object_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Parâmetros PID (ajuste conforme necessário)
        self.Kp = 0.1
        self.Ki = 0.01
        self.Kd = 0.05

        self.previous_error = 0
        self.integral = 0

        # Centro da imagem (ajuste conforme a resolução da sua câmera)
        self.center_x = 320
        self.center_y = 240

    def object_callback(self, msg):
        # Calcula o erro
        error_x = self.center_x - msg.x
        error_y = self.center_y - msg.y

        # Calcula os termos PID
        proportional = self.Kp * error_x
        self.integral += error_x
        integral = self.Ki * self.integral
        derivative = self.Kd * (error_x - self.previous_error)

        # Calcula o controle
        control = proportional + integral + derivative

        # Cria a mensagem de comando
        cmd = Twist()
        cmd.angular.z = control  # Assume que o controle é para a rotação do hovercraft
        
        # Publica o comando
        self.cmd_pub.publish(cmd)

        self.previous_error = error_x

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = PIDController()
    controller.run()
