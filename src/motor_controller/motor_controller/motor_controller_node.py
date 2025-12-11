#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time
import math

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parámetros del robot (ajusta según tu robot)
        self.declare_parameter('wheel_radius', 0.033)  # 3.3cm en metros
        self.declare_parameter('wheel_separation', 0.18)  # 18cm en metros
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_rpm', 100.0)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Inicializar comunicación serial
        try:
            self.serial_connection = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1
            )
            time.sleep(2)  # Esperar a que Arduino se inicialice
            self.get_logger().info(f'Conectado a Arduino en {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al conectar con Arduino: {e}')
            raise
        
        # Suscriptor para comandos de velocidad
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publicador para datos de encoders
        self.encoder_pub = self.create_publisher(
            Float32MultiArray,
            'encoder_data',
            10)
        
        # Publicador para RPM actuales
        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            'motor_rpm',
            10)
        
        # Hilo para lectura serial
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        # Temporizador para verificar conexión
        self.timer = self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info('Motor Controller Node iniciado')
    
    def check_connection(self):
        """Verifica que la conexión serial esté activa"""
        if not self.serial_connection.is_open:
            self.get_logger().warn('Conexión serial perdida')
    
    def cmd_vel_callback(self, msg):
        """Convierte Twist a velocidades de ruedas y envía a Arduino"""
        # Extraer velocidades lineales y angulares
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Cinemática diferencial inversa
        # v_left = (v - ω * L/2) / r
        # v_right = (v + ω * L/2) / r
        v_left_wheel = linear - (angular * self.wheel_separation / 2.0)
        v_right_wheel = linear + (angular * self.wheel_separation / 2.0)
        
        # Convertir velocidad lineal (m/s) a angular (rad/s)
        w_left = v_left_wheel / self.wheel_radius
        w_right = v_right_wheel / self.wheel_radius
        
        # Convertir rad/s a RPM
        # RPM = (rad/s * 60) / (2 * π)
        left_rpm = (w_left * 60.0) / (2.0 * math.pi)
        right_rpm = (w_right * 60.0) / (2.0 * math.pi)
        
        # Limitar RPM máximas
        left_rpm = max(min(left_rpm, self.max_rpm), -self.max_rpm)
        right_rpm = max(min(right_rpm, self.max_rpm), -self.max_rpm)
        
        # Enviar comandos a Arduino
        command = f"SPEED {left_rpm:.2f} {right_rpm:.2f}\n"
        try:
            self.serial_connection.write(command.encode('utf-8'))
            self.get_logger().debug(f'Enviado: {command.strip()}')
        except Exception as e:
            self.get_logger().error(f'Error al enviar comando: {e}')
    
    def read_serial_data(self):
        """Lee datos de encoders desde Arduino"""
        buffer = ""
        while rclpy.ok():
            try:
                if self.serial_connection.in_waiting:
                    # Leer datos serial
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # Procesar líneas completas
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            self.process_serial_line(line)
            except Exception as e:
                self.get_logger().error(f'Error en lectura serial: {e}')
                time.sleep(0.1)
    
    def process_serial_line(self, line):
        """Procesa una línea recibida desde Arduino"""
        try:
            if line.startswith('ENCODER'):
                # Formato: ENCODER left_pulses right_pulses
                parts = line.split()
                if len(parts) == 3:
                    left_pulses = float(parts[1])
                    right_pulses = float(parts[2])
                    
                    # Publicar datos de encoders
                    encoder_msg = Float32MultiArray()
                    encoder_msg.data = [left_pulses, right_pulses]
                    self.encoder_pub.publish(encoder_msg)
                    
                    self.get_logger().debug(f'Encoders: L={left_pulses}, R={right_pulses}')
            
            elif line.startswith('RPM'):
                # Formato: RPM left_rpm right_rpm
                parts = line.split()
                if len(parts) == 3:
                    left_rpm = float(parts[1])
                    right_rpm = float(parts[2])
                    
                    # Publicar RPM actuales
                    rpm_msg = Float32MultiArray()
                    rpm_msg.data = [left_rpm, right_rpm]
                    self.rpm_pub.publish(rpm_msg)
            
            elif line.startswith('ARDUINO READY'):
                self.get_logger().info('Arduino reporta READY')
            
            elif line.startswith('ERROR'):
                self.get_logger().warn(f'Arduino error: {line}')
            
            elif line.startswith('OK'):
                self.get_logger().debug(f'Arduino: {line}')
                
        except Exception as e:
            self.get_logger().error(f'Error procesando línea: {line}, Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        motor_controller = MotorController()
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Cerrar conexión serial al salir
        if 'motor_controller' in locals():
            motor_controller.serial_connection.close()
            motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
