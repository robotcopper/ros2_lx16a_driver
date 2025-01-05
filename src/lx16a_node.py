#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from pylx16a.lx16a import LX16A
from ros2_lx16a_driver.srv import GetLX16AInfo, SetLX16AParams, SetLX16ATorque
from std_msgs.msg import Float32MultiArray, Int32MultiArray

class LX16AController(Node):
    """
    ROS 2 Node for controlling LX-16A servos. 

    This node provides services to get servo information, set parameters, and control torque.
    It also subscribes to position and velocity commands to move the servos in real-time.
    """

    def __init__(self):
        super().__init__('lx16a_controller')


        # Declare ROS 2 parameters with default values
        self.declare_parameter('default_angle_offset', 0)
        self.declare_parameter('default_angle_min', 0)
        self.declare_parameter('default_angle_max', 240)
        self.declare_parameter('default_voltage_min', 4500)
        self.declare_parameter('default_voltage_max', 12000)
        self.declare_parameter('default_temperature_limit', 85)
        self.declare_parameter('log_level', 0)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('scan_connected_servos', False)


        # Store default parameter values
        self.default_values = {
            'angle_offset': self.get_parameter('default_angle_offset').value,
            'angle_min': self.get_parameter('default_angle_min').value,
            'angle_max': self.get_parameter('default_angle_max').value,
            'voltage_min': self.get_parameter('default_voltage_min').value,
            'voltage_max': self.get_parameter('default_voltage_max').value,
            'temperature_limit': self.get_parameter('default_temperature_limit').value,
        }


        # Set logger verbosity level
        self.get_logger().set_level(self.get_parameter('log_level').value)


        # Initialize communication with LX-16A servos
        try:
            LX16A.initialize(self.get_parameter('port').value)
            self.servos = {}
            if self.get_parameter('scan_connected_servos').value:
                self.scan_connected_servos()
        except Exception as e:
            self.get_logger().error(f"Error initializing communication: {e}")
            self.destroy_node()
            return


        # Create ROS 2 services
        self.srv_get = self.create_service(GetLX16AInfo, '/lx16a_get_info', self.handle_get_info)
        self.srv_set_params = self.create_service(SetLX16AParams, '/lx16a_set_params', self.handle_set_params)
        self.srv_set_torque = self.create_service(SetLX16ATorque, '/lx16a_set_torque', self.handle_set_torque)


        # Create ROS 2 subscribers
        self.position_subscriber = self.create_subscription(
            Float32MultiArray, '/cmd_pose_lx16a', self.handle_position_command, 10 
        )
        self.velocity_subscriber = self.create_subscription(
            Int32MultiArray, '/cmd_vel_lx16a', self.handle_velocity_command, 10
        )


        self.get_logger().info('LX-16A driver ready')


    def get_servo(self, servo_id):
        """
        Retrieve and create an instance of a servo with a given ID.
        
        Parameters:
            servo_id (int): ID of the servo.

        Returns:
            LX16A: Servo instance or None if an error occurs.
        """
        if servo_id not in self.servos:
            try:
                self.servos[servo_id] = LX16A(servo_id)
                self.get_logger().info(f"Servo {servo_id} added.")
            except Exception as e:
                self.get_logger().error(f"Error adding servo {servo_id}: {e}")
                return None
        return self.servos[servo_id]


    def handle_get_info(self, request, response):
        """
        Handle the GetLX16AInfo service to retrieve servo information.

        Parameters:
            request (GetLX16AInfo.Request): Service request.
            response (GetLX16AInfo.Response): Service response.

        Returns:
            GetLX16AInfo.Response: Populated response with servo details.
        """
        servo_id = int(request.id)
        servo = self.get_servo(servo_id)


        if servo is None:
            self.get_logger().error(f"Servo {servo_id} not available.")
            return response  
        

        try:
            # Retrieve detailed information about the servo
            response.angle_offset = int(servo.get_angle_offset()) 
            response.angle_min = int(servo.get_angle_limits()[0]) 
            response.angle_max = int(servo.get_angle_limits()[1])
            response.voltage_min = float(servo.get_vin_limits()[0]) 
            response.voltage_max = float(servo.get_vin_limits()[1]) 
            response.temperature_limit = float(servo.get_temp_limit()) 
            motor_mode = servo.is_motor_mode(poll_hardware=True)  
            response.motor_mode = motor_mode  
            if motor_mode:
                response.motor_speed = int(servo.get_motor_speed(poll_hardware=True))  
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


            self.get_logger().info(f"Information for servo {servo_id} retrieved.")
        except Exception as e:
            self.get_logger().error(f"Error retrieving info for servo {servo_id}: {e}")


        return response


    def handle_set_params(self, request, response):
        """
        Handle the SetLX16AParams service to update servo parameters.
        """
        servo_id = int(request.id)
        servo = self.get_servo(servo_id)

        if servo is None:
            self.get_logger().error(f"Servo {servo_id} not available.")
            response.success = False
            return response

        try:
            # Apply the parameters received or use the default values
            if request.led_enabled:
                servo.led_power_on()
            else:
                servo.led_power_off()
            servo.set_id(request.new_id if 0 <= request.new_id <= 253 else servo_id)
            servo.set_angle_offset(request.angle_offset if -30 <= request.angle_offset <= 30 else self.default_values['angle_offset'])
            servo.set_angle_limits(
                request.angle_min if 0 <= request.angle_min <= 240 else self.default_values['angle_min'],
                request.angle_max if 0 <= request.angle_max <= 240 else self.default_values['angle_max']
            )
            servo.set_vin_limits(
                int(request.voltage_min) if 4500 <= request.voltage_min <= 12000 else int(self.default_values['voltage_min']),
                int(request.voltage_max) if 4500 <= request.voltage_min <= 12000 else int(self.default_values['voltage_max'])
            )
            servo.set_temp_limit(int(request.temperature_limit) if 50 <= request.temperature_limit <= 100  else int(self.default_values['temperature_limit']))
            servo.set_led_error_triggers( request.led_error_temp, request.led_error_voltage, request.led_error_locked)

            self.get_logger().info(f"Servo {servo_id} parameters updated.")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error updating parameters for servo {servo_id}: {e}")
            response.success = False
        
        return response
    

    def handle_set_torque(self, request, response):
        """
        Handle the SetLX16ATorque service to enable or disable servo torque.
        """
        servo_id = int(request.id)
        servo = self.get_servo(servo_id)

        if servo is None:
            self.get_logger().error(f"Servo {servo_id} not available.")
            response.success = False
            return response

        try:
            # Apply the received parameters
            if request.torque_enabled:
                servo.enable_torque()
            else:
                servo.disable_torque()


            self.get_logger().info(f"Servo {servo_id} parameters updated.")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error updating parameters for servo {servo_id}: {e}")
            response.success = False
        
        return response


    def handle_position_command(self, msg):
        """
        Handle position commands received on the '/cmd_pose_lx16a' topic.
        """
        if len(msg.data) == 0:
            self.get_logger().warn("No position command received.")
            return

        # Apply position command to each servo
        for i, position in enumerate(msg.data):
            servo = self.get_servo(i)
            if servo is not None:
                try:
                    servo.servo_mode()
                    servo.move(position)  # Method to move the servo to the given position
                    self.get_logger().info(f"Servo {i} moved to position {position}°.")
                except Exception as e:
                    self.get_logger().error(f"Error moving servo {i}: {e}")


    def handle_velocity_command(self, msg):
        """
        Handle velocity commands received on the '/cmd_vel_lx16a' topic.
        """
        if len(msg.data) == 0:
            self.get_logger().warn("No velocity command received.")
            return

        # Apply velocity command to each servo
        for i, velocity in enumerate(msg.data):
            servo = self.get_servo(i)
            if servo is not None:
                try:
                    servo.motor_mode(velocity) # Method to control the servo at the given velocity
                    self.get_logger().info(f"Servo {i} commanded to velocity {velocity}.")
                except Exception as e:
                    self.get_logger().error(f"Error commanding servo {i}: {e}")


    def scan_connected_servos(self):
        """
        Scan and log all connected servos on the LX-16A bus.
        """
        connected_ids = []
        for servo_id in range(254):
            try:
                LX16A(servo_id)
                connected_ids.append(servo_id)
            except Exception:
                continue

        if connected_ids:
            self.get_logger().info(f"\033[32;1mConnected servos: {connected_ids}\033[0m")
        else:
            self.get_logger().warn("\033[31mNo servos detected.\033[0m")
        return connected_ids
    

def main(args=None):
    """
    Main function to initialize and spin the ROS 2 node.
    """
    rclpy.init(args=args)
    node = LX16AController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.destroy_node()  # Destroy the node
            rclpy.shutdown()     # Shutdown if the context is still valid


if __name__ == '__main__':
    main()


    