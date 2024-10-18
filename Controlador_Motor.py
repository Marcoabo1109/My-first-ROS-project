#!/bb/bin/env python3

import rospy
import pigpio
import time
from geometry_msgs.msg import Twist
from typing import Tuple, Optional
from motors import ESC

class HovercraftControl:
    """
    Controla dois motores ESC para um hovercraft, implementando controle diferencial
    através da distribuição de potência entre os motores.
    """
    
    # Constantes de configuração
    MIN_PWM = 1000       # PWM mínimo (motor parado)
    MAX_PWM = 2000       # PWM máximo (potência total)
    CENTER_PWM = 1500    # PWM central (neutro)
    STOP_THRESHOLD = 0.05
    MAX_POWER = 100.0    # Potência máxima em porcentagem
    
    def __init__(self, left_pin: int = 13, right_pin: int = 19) -> None:
        """
        Inicializa o controlador do hovercraft.
        
        Args:
            left_pin (int): GPIO pin para o motor esquerdo
            right_pin (int): GPIO pin para o motor direito
        """
        rospy.init_node('hovercraft_controller', anonymous=True)
        
        # Configuração dos subscribers
        self.control_sub = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.motor_control,
            queue_size=1
        )
        
        # Inicialização dos ESCs
        try:
            self.esc_left = ESC(pin=left_pin)
            self.esc_right = ESC(pin=right_pin)
            self._arm_escs()
        except Exception as e:
            rospy.logerr(f"Erro ao inicializar ESCs: {str(e)}")
            raise
        
        # Estado dos motores
        self.left_power = 0.0   # Potência atual do motor esquerdo (%)
        self.right_power = 0.0  # Potência atual do motor direito (%)
        
        # Configuração de diagnóstico
        self.diagnostics_pub = rospy.Publisher(
            '/hovercraft_diagnostics', 
            Twist, 
            queue_size=1
        )
        
        rospy.loginfo("Controlador do hovercraft inicializado com sucesso!")
    
    def _arm_escs(self) -> None:
        """Arma ambos os ESCs com tratamento de erro."""
        try:
            self.esc_left.arm()
            self.esc_right.arm()
            rospy.loginfo("ESCs armados com sucesso")
        except Exception as e:
            rospy.logerr(f"Falha ao armar ESCs: {str(e)}")
            raise
    
    def calculate_motor_powers(self, throttle: float, steering: float) -> Tuple[float, float]:
        """
        Calcula a distribuição de potência para os motores baseado no throttle e steering.
        
        Args:
            throttle: Velocidade linear desejada (-1 a 1)
            steering: Velocidade angular desejada (-1 a 1)
            
        Returns:
            Tuple[float, float]: Potência para motor esquerdo e direito em porcentagem
        """
        # Converte throttle para potência base (0 a 100%)
        base_power = abs(throttle) * self.MAX_POWER
        
        # Calcula o fator de distribuição baseado no steering (-1 a 1)
        # steering > 0 favorece giro para direita, < 0 para esquerda
        power_diff = abs(steering) * base_power
        
        if throttle >= 0:  # Movimento para frente
            if steering >= 0:  # Virando à direita
                left_power = base_power
                right_power = base_power - power_diff
            else:  # Virando à esquerda
                left_power = base_power - power_diff
                right_power = base_power
        else:  # Movimento para trás
            if steering >= 0:  # Virando à direita
                left_power = -base_power
                right_power = -(base_power - power_diff)
            else:  # Virando à esquerda
                left_power = -(base_power - power_diff)
                right_power = -base_power
        
        # Garante que as potências estejam entre -100% e 100%
        left_power = max(-self.MAX_POWER, min(self.MAX_POWER, left_power))
        right_power = max(-self.MAX_POWER, min(self.MAX_POWER, right_power))
        
        return left_power, right_power
    
    def power_to_pwm(self, power: float) -> int:
        """
        Converte potência percentual (-100 a 100) para valor PWM (1000 a 2000).
        
        Args:
            power: Potência do motor em porcentagem (-100 a 100)
            
        Returns:
            int: Valor PWM correspondente
        """
        if abs(power) < self.STOP_THRESHOLD:
            return self.CENTER_PWM
        
        # Mapeia potência para PWM
        if power >= 0:
            return int(self.CENTER_PWM + (power / 100.0) * (self.MAX_PWM - self.CENTER_PWM))
        else:
            return int(self.CENTER_PWM + (power / 100.0) * (self.CENTER_PWM - self.MIN_PWM))
    
    def motor_control(self, msg: Twist) -> None:
        """
        Callback para processar mensagens Twist e controlar os motores do hovercraft.
        
        Args:
            msg: Mensagem Twist contendo velocidades linear e angular
        """
        try:
            # Extrai velocidades do comando
            throttle = msg.linear.x
            steering = msg.angular.z
            
            # Verifica se deve parar
            if abs(throttle) < self.STOP_THRESHOLD and abs(steering) < self.STOP_THRESHOLD:
                self.stop_motors()
                return
            
            # Calcula potências dos motores
            self.left_power, self.right_power = self.calculate_motor_powers(throttle, steering)
            
            # Converte potência para PWM e aplica aos motores
            left_pwm = self.power_to_pwm(self.left_power)
            right_pwm = self.power_to_pwm(self.right_power)
            
            self.esc_left.control(left_pwm)
            self.esc_right.control(right_pwm)
            
            # Publica diagnóstico
            self.publish_diagnostics()
            
        except Exception as e:
            rospy.logerr(f"Erro no controle dos motores: {str(e)}")
            self.stop_motors()
    
    def stop_motors(self) -> None:
        """Para ambos os motores de forma segura."""
        try:
            self.esc_left.halt()
            self.esc_right.halt()
            self.left_power = 0.0
            self.right_power = 0.0
        except Exception as e:
            rospy.logerr(f"Erro ao parar motores: {str(e)}")
    
    def publish_diagnostics(self) -> None:
        """Publica informações de diagnóstico dos motores."""
        diagnostics = Twist()
        diagnostics.linear.x = self.left_power   # Potência motor esquerdo
        diagnostics.linear.y = self.right_power  # Potência motor direito
        self.diagnostics_pub.publish(diagnostics)
    
    def shutdown(self) -> None:
        """Desliga os motores e limpa recursos."""
        self.stop_motors()
        rospy.loginfo("Desligando controlador do hovercraft...")

if __name__ == '__main__':
    try:
        controller = HovercraftControl()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro fatal: {str(e)}")
