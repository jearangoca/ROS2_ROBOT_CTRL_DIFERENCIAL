#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Parámetros configurables
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('speed_step', 0.05)  # Incremento de velocidad
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_step = self.get_parameter('speed_step').value
        
        # Publicador de comandos de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Velocidades actuales
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        self.get_logger().info('=== NODO DE TELEOPERACIÓN ===')
        self.print_instructions()
        
        # Guardar configuración original del terminal
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Iniciar control por teclado
        self.run_keyboard_control()
    
    def print_instructions(self):
        instructions = """
        CONTROLES:
        ----------
        W       : Mover hacia adelante
        S       : Mover hacia atrás
        A       : Girar a la izquierda
        D       : Girar a la derecha
        Q/E     : Aumentar/disminuir velocidad lineal
        Z/C     : Aumentar/disminuir velocidad angular
        Espacio : Detener robot
        X       : Salir
        
        Velocidad lineal actual: {:.2f} m/s
        Velocidad angular actual: {:.2f} rad/s
        """.format(self.linear_speed, self.angular_speed)
        
        print(instructions)
    
    def get_key(self):
        """Lee una tecla sin esperar Enter"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_twist(self):
        """Publica el mensaje Twist actual"""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular
        
        self.publisher.publish(twist)
        
        # Mostrar estado actual
        print(f"\rVel: Lin={self.current_linear:.2f} m/s, Ang={self.current_angular:.2f} rad/s", end="")
    
    def run_keyboard_control(self):
        """Bucle principal de control por teclado"""
        print("\nPresiona teclas para controlar. 'X' para salir.")
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == 'w' or key == 'W':
                    # Adelante
                    self.current_linear = self.linear_speed
                    self.current_angular = 0.0
                
                elif key == 's' or key == 'S':
                    # Atrás
                    self.current_linear = -self.linear_speed
                    self.current_angular = 0.0
                
                elif key == 'a' or key == 'A':
                    # Girar izquierda
                    self.current_linear = 0.0
                    self.current_angular = self.angular_speed
                
                elif key == 'd' or key == 'D':
                    # Girar derecha
                    self.current_linear = 0.0
                    self.current_angular = -self.angular_speed
                
                elif key == 'q' or key == 'Q':
                    # Aumentar velocidad lineal
                    self.linear_speed += self.speed_step
                    print(f"\nVelocidad lineal aumentada: {self.linear_speed:.2f} m/s")
                
                elif key == 'e' or key == 'E':
                    # Disminuir velocidad lineal
                    self.linear_speed -= self.speed_step
                    if self.linear_speed < 0.05:
                        self.linear_speed = 0.05
                    print(f"\nVelocidad lineal disminuida: {self.linear_speed:.2f} m/s")
                
                elif key == 'z' or key == 'Z':
                    # Aumentar velocidad angular
                    self.angular_speed += self.speed_step
                    print(f"\nVelocidad angular aumentada: {self.angular_speed:.2f} rad/s")
                
                elif key == 'c' or key == 'C':
                    # Disminuir velocidad angular
                    self.angular_speed -= self.speed_step
                    if self.angular_speed < 0.05:
                        self.angular_speed = 0.05
                    print(f"\nVelocidad angular disminuida: {self.angular_speed:.2f} rad/s")
                
                elif key == ' ':
                    # Detener
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                
                elif key == 'x' or key == 'X':
                    # Salir
                    print("\nSaliendo...")
                    break
                
                elif key:
                    # Tecla no reconocida
                    print(f"\nTecla no reconocida: {key}")
                    continue
                
                # Publicar comando si hubo cambio
                if key:
                    self.publish_twist()
        
        except Exception as e:
            self.get_logger().error(f'Error en teleoperación: {e}')
        
        finally:
            # Detener robot antes de salir
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.publish_twist()
            
            # Restaurar configuración del terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nTerminal restaurado. ¡Adiós!")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopNode()
        # No usamos spin porque el control está en run_keyboard_control
        # Solo mantenemos el nodo vivo
        while rclpy.ok():
            rclpy.spin_once(teleop_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if 'teleop_node' in locals():
            teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
