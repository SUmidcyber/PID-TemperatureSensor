import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value

        P = self.Kp * error

        # Integral Term
        self.integral += error
        I = self.Ki * self.integral

        derivative = error - self.prev_error
        D = self.Kd * derivative

        # PID Control Çıktı
        control_output = P + I + D

        self.prev_error = error

        return control_output

class TemperatureSensor:
    def read_temperature(self):
        return 25
        # Sıcaklık Sensörü
class Heater:
    def control(self, output):
        print("Heater", output)
        # Isıtıcı output
def main():
    #PID Parametresi
    Kp = 1.0
    Ki = 0.1
    Kd = 0.01
    
    # Sıcaklık ayarı
    setpoint_temperature = 30.0

    pid_controller = PIDController(Kp, Ki, Kd)
    # PID Controllerimizi veriyoz

    # Sensör ve Heater objeclerimiz
    temperature_sensor = TemperatureSensor()
    heater = Heater()

    while True:
        # Okuma sensörü 
        current_temperature = temperature_sensor.read_temperature()

        # PID siyalini hesablayın
        control_signal = pid_controller.compute(setpoint_temperature, current_temperature)

        # Isıtıcı kontrol sinyalı
        heater.control(control_signal)

        # Kontrol döngüsü arasında bir gecikme simulasyonu
        time.sleep(1)

if __name__ == "__main__":
    main()
