import pigpio
import time

class ESC:
    def __init__(self, pwm_pin, freq=50):
        # Frequência  de 50Hz (aparentemente essa é a frequencia padrão para ESC)para controle de ESC
        """Inicializa o controle da ESC usando pigpio"""
        self.pwm_pin = pwm_pin
        self.freq = freq
        
        # Inicializa o pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Falha ao conectar no pigpio")
            
        # Define a frequência inicial do PWM
        self.pi.set_PWM_frequency(self.pwm_pin, self.freq)
        
        # Define a largura de pulso inicial como 0 (1000µs = 1ms = parado)
        self.pi.set_servo_pulsewidth(self.pwm_pin, 1000)
        
    def calibrate(self):
        """Calibra o ESC"""
        input('Conecte a bateria e pressione Enter')
        input('Desconecte a bateria e pressione Enter')
        
        print("Calibrando ESC - Máximo...")
        self.set_speed(100)  # Define a velocidade máxima para calibração
        input("Conecte a bateria e pressione Enter")
        time.sleep(2)
        
        print("Calibrando ESC - Mínimo...")
        self.set_speed(0)  # Define a velocidade mínima para calibração
        time.sleep(2)
        input("Desconecte a bateria e pressione Enter")
        print("ESC calibrado")
        
    def set_speed(self, speed):
        """
        Define a velocidade do ESC (0-100%)
        Mapeia a velocidade para a largura de pulso entre 1000µs (0%) e 2000µs (100%)
        """
        # Garante que a velocidade esteja dentro dos limites
        speed = max(0, min(100, speed))
        
        # Mapeia 0-100% para 1000-2000µs para duty cycle entre 5% e 10%
        # 5% duty cycle -> 1ms (ESC parada)
        # 10% duty cycle -> 2ms (ESC a toda velocidade)

        pulse_width = self.map_speed_to_pulse_width(speed)
        self.pi.set_servo_pulsewidth(self.pwm_pin, pulse_width)
        
    def map_speed_to_pulse_width(self, speed):
        """Mapeia a velocidade (0-100%) para largura de pulso (1000-2000µs)"""
        return 1000 + (speed * 1000 / 100)
        
    def stop(self):
        """Para o ESC"""
        self.pi.set_servo_pulsewidth(self.pwm_pin, 1000)
        
    def cleanup(self):
        """Limpa os recursos do pigpio"""
        self.stop()
        self.pi.stop()

if __name__ == "__main__":
    # Exemplo de uso
    esc1 = ESC(pwm_pin=18)  # GPIO 18
    esc2 = ESC(pwm_pin=22)  # GPIO 22
    
    try:
        # Controle de velocidade de exemplo
        esc1.set_speed(40)
        esc2.set_speed(0.6)
        time.sleep(5)
        
        # Descomente para testar outras funcionalidades
        # # Testa a faixa completa
        # esc1.set_speed(100)
        # esc2.set_speed(100)
        # time.sleep(2)
        # esc1.set_speed(0)
        # esc2.set_speed(0)
        # time.sleep(2)
        
        # # Exemplo de calibração
        # esc1.calibrate()
        
        # # Teste de velocidade gradual
        # print("Velocidade 50%")
        # esc1.set_speed(50)
        # time.sleep(5)
        
        # print("Velocidade 100%")
        # esc1.set_speed(100)
        # time.sleep(5)
        
        # print("Parando o ESC")
        # esc1.stop()
        
    except KeyboardInterrupt:
        pass
    finally:
        esc1.cleanup()
        esc2.cleanup()
