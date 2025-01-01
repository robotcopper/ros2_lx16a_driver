#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pylx16a.lx16a import LX16A
from ros2_lx16a_driver.srv import GetLX16AInfo, SetLX16AParams, SetLX16ATorqueLed 
from std_msgs.msg import Float32MultiArray, Int32MultiArray  # Message utilisé pour la commande de position

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

        # Création des services
        self.srv_get = self.create_service(GetLX16AInfo, '/lx16a_get_info', self.handle_get_info)
        self.srv_set_params = self.create_service(SetLX16AParams, '/lx16a_set_params', self.handle_set_params)
        self.srv_set_torque_led = self.create_service(SetLX16ATorqueLed, '/lx16a_set_torque_led', self.handle_set_torque_led)

        # Création du subscriber pour la commande de position
        self.position_subscriber = self.create_subscription(
            Float32MultiArray,  # Type de message
            '/cmd_pose_lx16a',  # Nom du topic
            self.handle_position_command,  # Callback
            10  # Taille de la file d'attente
        )

        # Création du subscriber pour la commande de vitesse
        self.position_subscriber = self.create_subscription(
            Int32MultiArray,  # Type de message
            '/cmd_vel_lx16a',  # Nom du topic
            self.handle_velocity_command,  # Callback
            10  # Taille de la file d'attente
        )

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

    def handle_get_info(self, request, response):
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

    def handle_set_params(self, request, response):
        """Service pour définir les paramètres d'un servo."""
        servo_id = int(request.id)
        servo = self.get_servo(servo_id)

        if servo is None:
            self.get_logger().error(f"Servo {servo_id} non disponible.")
            response.success = False
            return response
        
        # default_values = {
        #     'new_id': 0,
        #     'angle_offset': 0,
        #     'angle_min': 0,
        #     'angle_max': 240,
        #     'voltage_min': 4500.0,
        #     'voltage_max': 12000.0,
        #     'temperature_limit': 85.0,
        #     'motor_mode': False,
        #     'motor_speed': 0.0,
        #     'torque_enabled': True,
        #     'led_enabled': True,
        #     'led_flash_condition': 1
        # }

        try:
            # Appliquer les paramètres reçus
            servo.set_id(request.new_id)
            servo.set_angle_offset(request.angle_offset)
            servo.set_angle_limits(request.angle_min, request.angle_max)
            servo.set_vin_limits(int(request.voltage_min), int(request.voltage_max))
            servo.set_temp_limit(int(request.temperature_limit))
                # print(request.motor_mode)
                # if request.motor_mode:
                #     servo.motor_mode(1000)
                # else:
                #     servo.servo_mode()
            servo.set_led_error_triggers(request.led_error_temp,request.led_error_voltage,request.led_error_locked)
            
            self.get_logger().info(f"Paramètres du servo {servo_id} mis à jour.")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la mise à jour des paramètres du servo {servo_id} : {e}")
            response.success = False
        
        return response
    
    def handle_set_torque_led(self, request, response):
        """Service pour définir le torque et controller la led d'un servo."""
        servo_id = int(request.id)
        servo = self.get_servo(servo_id)

        if servo is None:
            self.get_logger().error(f"Servo {servo_id} non disponible.")
            response.success = False
            return response

        try:
            # Appliquer les paramètres reçus
            if request.torque_enabled:
                servo.enable_torque()
            else:
                servo.disable_torque()
            if request.led_enabled:
                servo.led_power_on()
            else:
                servo.led_power_off()

            self.get_logger().info(f"Paramètres du servo {servo_id} mis à jour.")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la mise à jour des paramètres du servo {servo_id} : {e}")
            response.success = False
        
        return response

    def handle_position_command(self, msg):
        """Callback pour gérer la commande de position des servos."""
        if len(msg.data) == 0:
            self.get_logger().warn("Aucune commande de position reçue.")
            return

        # Appliquer la commande de position à chaque servo
        for i, position in enumerate(msg.data):
            servo = self.get_servo(i)
            if servo is not None:
                try:
                    servo.servo_mode()
                    servo.move(position)  # Méthode pour déplacer le servo à la position donnée
                    self.get_logger().info(f"Servo {i} déplacé à la position {position}.")
                except Exception as e:
                    self.get_logger().error(f"Erreur lors du déplacement du servo {i} : {e}")

    def handle_velocity_command(self, msg):
        """Callback pour gérer la commande de vitesse des servos."""
        if len(msg.data) == 0:
            self.get_logger().warn("Aucune commande de vitesse reçue.")
            return

        # Appliquer la commande de vitesse à chaque servo
        for i, velocity in enumerate(msg.data):
            servo = self.get_servo(i)
            if servo is not None:
                try:
                    servo.motor_mode(velocity) # Méthode pour comander le servo à la vitesse donnée
                    self.get_logger().info(f"Servo {i} commandé à la vitesse {velocity}.")
                except Exception as e:
                    self.get_logger().error(f"Erreur lors du déplacement du servo {i} : {e}")

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


