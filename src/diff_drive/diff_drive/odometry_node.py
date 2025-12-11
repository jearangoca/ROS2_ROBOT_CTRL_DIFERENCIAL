#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Parámetros del robot (ajusta según tu robot)
        self.declare_parameter('wheel_radius', 0.033)  # 3.3cm en metros
        self.declare_parameter('wheel_separation', 0.18)  # 18cm en metros
        self.declare_parameter('ticks_per_rev', 360.0)  # Pulsos por revolución
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        # Variables de estado
        self.x = 0.0  # Posición X (metros)
        self.y = 0.0  # Posición Y (metros)
        self.theta = 0.0  # Orientación (radianes)
        
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()
        
        # Covarianza (ajusta según tu robot)
        self.odom_pose_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]
        
        self.odom_twist_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]
        
        # Configurar QoS para mejor fiabilidad
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Suscriptor para datos de encoders
        self.encoder_sub = self.create_subscription(
            Float32MultiArray,
            'encoder_data',
            self.encoder_callback,
            qos_profile)
        
        # Publicador de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Broadcaster de TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Temporizador para publicar odometría incluso sin nuevos datos
        self.timer = self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        self.get_logger().info('Odometry Node iniciado')
        self.get_logger().info(f'Parámetros: radio rueda={self.wheel_radius}m, separación={self.wheel_separation}m')
    
    def encoder_callback(self, msg):
        """Procesa datos de encoders y calcula odometría"""
        if len(msg.data) < 2:
            self.get_logger().warn('Datos de encoder incompletos')
            return
        
        current_left_ticks = msg.data[0]
        current_right_ticks = msg.data[1]
        current_time = self.get_clock().now()
        
        # Calcular diferencia de tiempo
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            self.get_logger().warn('Delta time inválido')
            return
        
        # Calcular diferencia de pulsos
        delta_left = current_left_ticks - self.last_left_ticks
        delta_right = current_right_ticks - self.last_right_ticks
        
        # Actualizar contadores
        self.last_left_ticks = current_left_ticks
        self.last_right_ticks = current_right_ticks
        self.last_time = current_time
        
        # Calcular distancia recorrida por cada rueda (metros)
        left_distance = 2.0 * math.pi * self.wheel_radius * delta_left / self.ticks_per_rev
        right_distance = 2.0 * math.pi * self.wheel_radius * delta_right / self.ticks_per_rev
        
        # Calcular desplazamiento del robot
        distance = (right_distance + left_distance) / 2.0
        theta_change = (right_distance - left_distance) / self.wheel_separation
        
        # Velocidades lineales y angulares
        v = distance / dt
        w = theta_change / dt
        
        # Actualizar posición (integración simple)
        if abs(theta_change) < 0.0001:
            # Movimiento recto
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Movimiento curvilíneo
            radius = distance / theta_change
            icc_x = self.x - radius * math.sin(self.theta)
            icc_y = self.y + radius * math.cos(self.theta)
            
            self.x = math.cos(theta_change) * (self.x - icc_x) - math.sin(theta_change) * (self.y - icc_y) + icc_x
            self.y = math.sin(theta_change) * (self.x - icc_x) + math.cos(theta_change) * (self.y - icc_y) + icc_y
            self.theta += theta_change
        
        # Normalizar ángulo entre -pi y pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Guardar velocidades para publicación
        self.last_v = v
        self.last_w = w
        
        self.get_logger().debug(f'Posición: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.1f}°')
    
    def publish_odometry(self):
        """Publica mensaje de odometría y transformada TF"""
        # Crear mensaje de odometría
        odom_msg = Odometry()
        current_time = self.get_clock().now()
        
        # Encabezado
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientación (convertir ángulo a quaternion)
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q
        
        # Covarianza de posición
        odom_msg.pose.covariance = self.odom_pose_covariance
        
        # Velocidad (si está disponible)
        if hasattr(self, 'last_v') and hasattr(self, 'last_w'):
            odom_msg.twist.twist.linear.x = self.last_v
            odom_msg.twist.twist.angular.z = self.last_w
        else:
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.angular.z = 0.0
        
        # Covarianza de velocidad
        odom_msg.twist.covariance = self.odom_twist_covariance
        
        # Publicar odometría
        self.odom_pub.publish(odom_msg)
        
        # Publicar transformada TF
        self.publish_tf_transform(odom_msg)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convierte ángulos de Euler a quaternion"""
        q = Quaternion()
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def publish_tf_transform(self, odom_msg):
        """Publica transformada TF desde odom a base_link"""
        t = TransformStamped()
        
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        odometry_node = OdometryNode()
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error en nodo de odometría: {e}')
    finally:
        if 'odometry_node' in locals():
            odometry_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
