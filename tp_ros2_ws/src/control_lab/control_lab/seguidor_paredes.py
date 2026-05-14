import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class SeguidorParedes(Node):
    def __init__(self):
        super().__init__('seguidor_paredes_nodo')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription_scan = self.create_subscription(
            LaserScan, '/diff_drive/scan', self.scan_callback, qos_sensor)

        self.subscription_odom = self.create_subscription(
            Odometry, '/diff_drive/odometry', self.odom_callback, qos_sensor)

        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.cmd = Twist()

        # PARAMETROS DE NAVEGACION
        self.Dist_Obj = 1.6           
        self.Dist_frontal = 1.5       
        self.Vel_lineal = 0.3         
        self.Vel_ang_emergencia = 0.5 

        # PARAMETROS PID
        self.Kp = 1.5   # Proporcional
        self.Kd = 0.5   # Derivativo
        self.Ki = 0.08  # Integrador
        
        # Inicializo los valores del error
        self.error_anterior = 0.0
        self.error_integral = 0.0  
        
        self.tiempo_anterior = self.get_clock().now().nanoseconds

        # Estado del Robot
        self.estado = 'Seguir'
        self.yaw_actual = 0.0
        self.yaw_objetivo = 0.0
        self.debug_init = False

        self.get_logger().info(f'Nodo PID listo. Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        t3 = +2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        t4 = +1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw_actual = math.atan2(t3, t4)

    def normalizar_angulo(self, angulo):
        while angulo > math.pi: angulo -= 2.0 * math.pi
        while angulo < -math.pi: angulo += 2.0 * math.pi
        return angulo

    def scan_callback(self, msg):
        if not self.debug_init:
            self.get_logger().info('¡Recibiendo datos del sensor!')
            self.debug_init = True

        rangos = np.array(msg.ranges, dtype=np.float32)
        rangos[np.isinf(rangos)] = msg.range_max
        n = len(rangos)
        centro = n // 2

        frente = np.mean(rangos[centro - 10 : centro + 10])      
        derecha = np.min(rangos[0 : n//4]) 

        # MAQUINA DE ESTADOS 
        # ESTADO 1: GIRO DE EMERGENCIA
        if self.estado == 'GIRAR EMERGENCIA':
            error_yaw = self.normalizar_angulo(self.yaw_objetivo - self.yaw_actual)
            
            if abs(error_yaw) > 0.05: 
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.Vel_ang_emergencia if error_yaw > 0 else -self.Vel_ang_emergencia
                self.publisher_.publish(self.cmd)
            else:
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.estado = 'Seguir'
                
                # Al terminar el giro, guardo la memoria en 0
                self.error_anterior = 0.0 
                self.error_integral = 0.0 
                
                self.get_logger().info('Giro completado se sigue.')
            return

        # ESTADO 2: Deteccion de obstaculos
        if frente < self.Dist_frontal:
            self.estado = 'GIRAR EMERGENCIA'
            self.yaw_objetivo = self.normalizar_angulo(self.yaw_actual + (math.pi / 2))
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().warn(f'Pared enfrente ({frente:.2f}m). Iniciando giro 90°.')
            return

        # ESTADO 3: SEGUIMIENTO DE PARED con PID
        
        #  Error 
        error_actual = self.Dist_Obj - derecha
        
        # Tiempo
        tiempo_actual = self.get_clock().now().nanoseconds
        dt = (tiempo_actual - self.tiempo_anterior) / 1e9 
        
        if dt > 0:
            # Derivada
            derivada = (error_actual - self.error_anterior) / dt
            
            # Integral
            # Acumulamos el error
            self.error_integral += error_actual * dt
            
            # Anti Winduop
            self.error_integral = np.clip(self.error_integral, -5.0, 5.0)
            
        else:
            derivada = 0.0

        # PID: Kp*P + Kd*D + Ki*I
        giro = (self.Kp * error_actual) + (self.Kd * derivada) + (self.Ki * self.error_integral)

        # Correcion de giros
        if derecha > 2.5:
             giro = np.clip(giro, -0.25, 0.25)
        else:
             giro = np.clip(giro, -0.6, 0.6)

        # Ejecuciion
        self.cmd.linear.x = self.Vel_lineal
        self.cmd.angular.z = float(giro)
        self.publisher_.publish(self.cmd)
        
        # Guardamos valores
        self.error_anterior = error_actual
        self.tiempo_anterior = tiempo_actual
        
        self.get_logger().info(f'Error: {error_actual:.2f}, Prop:{error_actual*self.Kp:.2f}, Der:{derivada*self.Kd:.2f}, Int:{self.error_integral*self.Ki:.2f}')


    def detener_robot(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        try:
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Robot frenado.')
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    nodo = SeguidorParedes()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.detener_robot()
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
