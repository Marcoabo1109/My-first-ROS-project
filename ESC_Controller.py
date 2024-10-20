import pigpio
import time
import rospy
from geometry_msgs.msg import Twist
from typing import Tuple, Optional

class ESC:
    """Controlador ESC aprimorado com recursos de segurança e inicialização adequada"""
    
    # Constantes da classe
    MIN_PULSE_WIDTH = 1000    # Largura de pulso mínima em microssegundos
    MAX_PULSE_WIDTH = 2000    # Largura de pulso máxima em microssegundos
    CENTER_PULSE = 1500       # Largura de pulso em posição neutra
    ARM_DELAY = 2.0          # Atraso para a sequência de armamento em segundos
    
    def __init__(self, pin: int, frequency: int = 50, simulation: bool = False):
        """
        Inicializa o controlador ESC
        
        Args:
            pin: Número do pino GPIO
            frequency: Frequência PWM em Hz
            simulation: Executar em modo de simulação (sem hardware necessário)
        """
        self.pin = pin
        self.frequency = frequency
        self.simulation = simulation
        self.current_pulse = self.MIN_PULSE_WIDTH
        self.is_armed = False
        
        if not self.simulation:
            try:
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    raise RuntimeError("Falha ao conectar ao daemon pigpio")
                
                # Configura a frequência PWM
                self.pi.set_PWM_frequency(self.pin, self.frequency)
                # Estado inicial seguro
                self.pi.set_servo_pulsewidth(self.pin, self.MIN_PULSE_WIDTH)
                
            except Exception as e:
                raise RuntimeError(f"Falha na inicialização do ESC no pino {pin}: {str(e)}")
    
    def arm(self) -> None:
        """
        Executa a sequência de armamento do ESC com verificações de segurança
        """
        if self.simulation:
            self.is_armed = True
            return
            
        try:
            # Redefinir para o estado conhecido
            self.pi.set_servo_pulsewidth(self.pin, self.MIN_PULSE_WIDTH)
            time.sleep(self.ARM_DELAY)
            
            # Sequência de armamento
            self.pi.set_servo_pulsewidth(self.pin, self.MAX_PULSE_WIDTH)
            time.sleep(self.ARM_DELAY)
            self.pi.set_servo_pulsewidth(self.pin, self.MIN_PULSE_WIDTH)
            time.sleep(self.ARM_DELAY)
            
            self.is_armed = True
            self.current_pulse = self.MIN_PULSE_WIDTH
            
        except Exception as e:
            self.is_armed = False
            raise RuntimeError(f"Falha no armamento do ESC no pino {self.pin}: {str(e)}")
    
    def control(self, pulse_width: int) -> None:
        """
        Controla o ESC com a largura de pulso
        
        Args:
            pulse_width: Largura de pulso em microssegundos (1000-2000)
        """
        if not self.is_armed:
            raise RuntimeError("ESC não armado! Chame arm() primeiro")
            
        # Verificação de limites
        pulse_width = max(self.MIN_PULSE_WIDTH, min(self.MAX_PULSE_WIDTH, pulse_width))
        
        if self.simulation:
            self.current_pulse = pulse_width
            return
            
        try:
            self.pi.set_servo_pulsewidth(self.pin, pulse_width)
            self.current_pulse = pulse_width
        except Exception as e:
            raise RuntimeError(f"Falha no controle do ESC no pino {self.pin}: {str(e)}")
    
    def halt(self) -> None:
        """Para o ESC de forma segura"""
        if self.simulation:
            self.current_pulse = self.MIN_PULSE_WIDTH
            return
            
        try:
            self.pi.set_servo_pulsewidth(self.pin, self.MIN_PULSE_WIDTH)
            self.current_pulse = self.MIN_PULSE_WIDTH
        except Exception as e:
            raise RuntimeError(f"Falha ao parar o ESC no pino {self.pin}: {str(e)}")
    
    def cleanup(self) -> None:
        """Libera os recursos do GPIO"""
        if not self.simulation:
            try:
                self.halt()
                self.pi.stop()
            except Exception as e:
                raise RuntimeError(f"Falha ao limpar os recursos do ESC no pino {self.pin}: {str(e)}")

class HovercraftControl:
    """
    Controla dois motores ESC para um hovercraft com direção diferencial
    """
    
    def __init__(self, left_pin: int = 13, right_pin: int = 19, 
                 simulation: bool = False) -> None:
        """
        Inicializa o controlador do hovercraft
        
        Args:
            left_pin: Pino GPIO para o motor esquerdo
            right_pin: Pino GPIO para o motor direito
            simulation: Executar em modo de simulação
        """
        # Inicializa o nó ROS
        rospy.init_node('hovercraft_controller', anonymous=True)
        
        self.simulation = simulation
        
        # Inicializa os ESCs com modo de simulação, se necessário
        try:
            self.esc_left = ESC(pin=left_pin, simulation=simulation)
            self.esc_right = ESC(pin=right_pin, simulation=simulation)
            self._arm_escs()
        except Exception as e:
            rospy.logerr(f"Falha ao inicializar os ESCs: {str(e)}")
            raise
        
        # Inscritores e publicadores ROS
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, 
                                          self.motor_control, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/hovercraft_diagnostics', 
                                             Twist, queue_size=1)
        
        # Estado do motor
        self.left_power = 0.0
        self.right_power = 0.0
        
        # Timeout de segurança para mensagens de comando
        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(1.0)  # Timeout de 1 segundo
        
        # Inicia o temporizador de segurança
        self.timer = rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        rospy.loginfo("Controlador de hovercraft inicializado com sucesso!")
    
    def safety_check(self, event):
        """Verificação periódica de segurança para parar os motores se não houver comandos recebidos"""
        if (rospy.Time.now() - self.last_cmd_time) > self.cmd_timeout:
            rospy.logwarn_throttle(1, "Timeout de comando - parando os motores")
            self.stop_motors()
    
    def _arm_escs(self) -> None:
        """Arma ambos os ESCs com tratamento adequado de erros"""
        try:
            self.esc_left.arm()
            self.esc_right.arm()
            rospy.loginfo("ESCs armados com sucesso")
        except Exception as e:
            rospy.logerr(f"Falha ao armar os ESCs: {str(e)}")
            raise
    
    def motor_control(self, msg: Twist) -> None:
        """
        Processa as mensagens Twist e controla os motores
        
        Args:
            msg: Mensagem Twist contendo as velocidades linear e angular
        """
        try:
            self.last_cmd_time = rospy.Time.now()
            
            # Extrai as velocidades
            throttle = msg.linear.x
            steering = msg.angular.z
            
            # Calcula a potência dos motores
            self.left_power, self.right_power = self.calculate_motor_powers(
                throttle, steering)
            
            # Converte para PWM e aplica
            left_pwm = self.power_to_pwm(self.left_power)
            right_pwm = self.power_to_pwm(self.right_power)
            
            self.esc_left.control(left_pwm)
            self.esc_right.control(right_pwm)
            
            # Publica os dados de diagnóstico
            self.publish_diagnostics()
            
        except Exception as e:
            rospy.logerr(f"Erro no controle dos motores: {str(e)}")
            self.stop_motors()
    
    def calculate_motor_powers(self, throttle: float, 
                             steering: float) -> Tuple[float, float]:
      """ Calcula a distribuição de potência para os motores baseado no throttle e steering (diferencial).
        
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
    def stop_motors(self) -> None:
        """Para ambos os motores de forma segura"""
        try:
            self.esc_left.halt()
            self.esc_right.halt()
            self.left_power = 0.0
            self.right_power = 0.0
        except Exception as e:
            rospy.logerr(f"Erro ao parar os motores: {str(e)}")
    
    def shutdown(self) -> None:
        """Desligamento seguro do controlador"""
        self.stop_motors()
        if hasattr(self, 'timer'):
            self.timer.shutdown()
        self.esc_left.cleanup()
        self.esc_right.cleanup()
        rospy.loginfo("Controlador de hovercraft desligado com sucesso")

if __name__ == '__main__':
    try:
        controller = HovercraftControl()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Erro fatal: {str(e)}")
