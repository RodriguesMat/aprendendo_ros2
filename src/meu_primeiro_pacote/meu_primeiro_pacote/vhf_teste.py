import rclpy
import numpy 
import tf_transformations
import math
import time 
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class vhf_teste(Node):

    self.espera(0.5)
    self.dr2d2 = 0.0
    self.angulo_robo = 0.0
    self.distancia = 0.0

    def __init__(self):
        super().__init__('vhf_teste')
        self.get_logger().debug ('Definido o nome do nó para "vhf_teste"')

        qos_profile = QoSProfile(depth=10, reliability = QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug ('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug ('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug ('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges
       
    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
    
    def distancia(self):
        destino = [9,9]
        self.distancia = math.dist((self.pose.position.x, self.pose.position.y), destino)
        self.angulo = math.atan2(9 - self.pose.position.x, 9 - self.pose.position.y)

    def espera(self, maxsec):
        start = time.time()
        i = 0
        while i < maxsec:
            i = time.time() - start            
            rclpy.spin_once(self)

    def run(self):

        self.get_logger().debug ('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug ('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x= 0.5,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))
        self.parar          = Twist(linear=Vector3(x= 0.0,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z= 0.0))

        self.get_logger().info ('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        self.get_logger().info ('Entrando no loop princial do nó.')

        momento = 0

        while(rclpy.ok):
            
            self.pose.orientation
            self.pose.position
            _, _, yaw = tf_transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])

            rclpy.spin_once(self)
            
            distancia_direita = numpy.mean(self.laser[0:80])
            distancia_frente = numpy.mean(self.laser[80:100])
            distancia_esquerda = numpy.mean(self.laser[100:180])

            cmd = Twist()
            self.erro_ang = self.angulo_robo - yaw
            self.distancia()

            if self.momento == 0:
                if(abs(self.erro_ang) >= 0.06):
                    cmd.angular.z = 0.4
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('TENTANDO VIRAR PRO 99 CALMA')
                elif( self.distancia <= 3 and abs(self.erro_ang) <= 0.06):
                    self.momento = 2                
                else:
                    cmd.angular.z = 0.0
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('olhando para (9;9), dist=' + str(self.distancia) + 'dr2d2=' + str(self.pose.position.x)+ str(self.pose.position.y))
                    self.momento = 1
        
            elif self.momento == 1: 
                if(self.distancia_frente > self.distancia_direita and self.distancia_frente > self.distancia_esquerda and self.distancia_frente > 1):
                    self.get_logger().info ('seguindo reto')
                    self.momento = 2
                elif(self.distancia_esquerda > self.distancia_direita and self.distancia_esquerda > self.distancia_frente ):
                    cmd.angular.z = 0.5
                    self.get_logger().info ('seguindo a esquerda')
                    self.pub_cmd_vel.publish(cmd)
                elif(self.distancia_direita > self.distancia_frente and self.distancia_direita > self.distancia_esquerda ):
                    cmd.angular.z = -0.5
                    self.get_logger().info ('seguindo a direita')
                    self.pub_cmd_vel.publish(cmd)
                else:
                    cmd.angular.z = 0.5
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().info ('me encontrando')

            elif self.momento == 2: 
                    cmd.linear.x = 0.5
                    self.pub_cmd_vel.publish(cmd)
                    self.get_logger().debug ("distância para o obstáculo" + str(self.distancia_frente))
                
                    if(self.distancia <= 3 and self.distancia >=1 and abs(self.erro_ang) <= 0.06):
                        cmd.linear.x = 0.5
                        self.pub_cmd_vel.publish(cmd)
                        self.get_logger().info ('andando')
                    elif (self.distancia_frente < self.distancia_direita and self.distancia_frente < self.distancia_esquerda or self.distancia_frente < 1):
                        self.get_logger().info ('obstáculo')
                        self.momento = 0
                    if(self.distancia <= 0.8):
                        cmd.angular.z = 0.0
                        cmd.linear.x = 0.0
                        self.pub_cmd_vel.publish(cmd)
                        self.get_logger().info ('destino alcançado')

        self.get_logger().info ('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')
        
# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = vhf_teste()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  
