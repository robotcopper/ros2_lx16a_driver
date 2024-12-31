#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pylx16a.lx16a import LX16A
from ros2_lx16a_driver.srv import GetServoInfo

class LX16AController(Node):
    def __init__(self):
        super().__init__('lx16a_controller')

        # Initialisation du bus LX-16A
        try:
            LX16A.initialize("/dev/ttyUSB0")  # Remplacez par votre port série
            self.servos = {}
        except Exception as e:
            self.get_logger().error(f"Erreur d'initialisation de la communication : {e}")
            self.destroy_node()
            return

        # Création du service GetServoInfo
        self.srv = self.create_service(GetServoInfo, '/get_servo_info', self.handle_get_servo_info)

        self.get_logger().info('Nœud LX-16A prêt.')

    def get_servo(self, servo_id):
        """Retourne une instance de servo donnée, la crée si nécessaire."""
        if servo_id not in self.servos:
            try:
                self.servos[servo_id] = LX16A(servo_id)
                self.get_logger().info(f"Servo {servo_id} ajouté.")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de l'ajout du servo {servo_id} : {e}")
                return None
        return self.servos[servo_id]

    def handle_get_servo_info(self, request, response):
        """Service pour obtenir les informations détaillées d'un servo."""
        servo_id = int(request.id)  # ID du servo demandé
        servo = self.get_servo(servo_id)

        if servo is None:
            self.get_logger().error(f"Servo {servo_id} non disponible.")
            return response  # Laisse la réponse vide si le servo n'est pas trouvé

        try:
            # Récupération des informations du servo
            response.angle_offset = int(servo.get_angle_offset()) 
            response.angle_min = int(servo.get_angle_limits()[0]) 
            response.angle_max = int(servo.get_angle_limits()[1])
            response.voltage_min = float(servo.get_vin_limits()[0]) 
            response.voltage_max = float(servo.get_vin_limits()[1]) 
            response.temperature_limit = float(servo.get_temp_limit()) 
            motor_mode = servo.is_motor_mode(poll_hardware=True)  
            response.motor_mode = motor_mode  
            if motor_mode:
                response.motor_speed = servo.get_motor_speed()  
            response.torque_enabled = servo.is_torque_enabled()  
            response.led_enabled = servo.is_led_power_on() 
            response.led_error_temp = servo.get_led_error_triggers()[0] 
            response.led_error_voltage = servo.get_led_error_triggers()[1]  
            response.led_error_locked = servo.get_led_error_triggers()[1]  
            response.current_temperature = float(servo.get_temp())  
            response.input_voltage = float(servo.get_vin()) 
            if not motor_mode:
                response.physical_angle = servo.get_physical_angle() 
                response.commanded_angle = servo.get_commanded_angle() 

            self.get_logger().info(f"Informations du servo {servo_id} récupérées.")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la récupération des infos du servo {servo_id} : {e}")
            # Réponse vide si une erreur se produit
            return response

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LX16AController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Arrêt du nœud.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
