import rclpy
import numpy 
import time
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from rclpy.duration import Duration
from robot_navigator import BasicNavigator, NavigationResult

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug ('Definido o nome do nó para "R2D2"')

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

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 9.0
        goal_pose.pose.position.y = 9.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        navigator.goToPose(goal_pose)
 
        i = 0
 
  # Keep doing stuff as long as the robot is moving towards the goal
        while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(
                    feedback.distance_remaining) + ' meters.')
 
      # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()
 
      # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)
 
  # Do something depending on the return code
            result = navigator.getResult()
            if result == NavigationResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == NavigationResult.CANCELED:
                print('Goal was canceled!')
            elif result == NavigationResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

        integral = 0.0
        while(rclpy.ok):
            rclpy.spin_once(self)
            distancia_objetivo = 0.5
            distancia_direita = numpy.mean(self.laser[0:10])
            distancia_frente = numpy.mean(self.laser[80:100])
            def wall():
                p_gain = 0.1
                i_gain = 0.00
                d_gain = 0.01 
                integral = 0.0
                error = distancia_objetivo - distancia_direita
                integral = integral + error 
                old_error = error   
                dif_erro = error - old_error
                power = p_gain*error + i_gain*integral + d_gain*dif_erro
                cmd = Twist()
                cmd.linear.x = 0.5
                cmd.angular.z = power
                self.pub_cmd_vel.publish(cmd)
            def virar():
                p_gain = 0.1
                i_gain = 0.00
                d_gain = 0.01 
                integral = 0.0
                error = distancia_objetivo - distancia_frente
                integral = integral + error 
                old_error = error   
                dif_erro = error - old_error
                power = p_gain*error + i_gain*integral + d_gain*dif_erro
                cmd = Twist()
                cmd.linear.x = power
                #cmd.linear.y = 0.5
                cmd.angular.z = 0.5
                self.pub_cmd_vel.publish(cmd)
            if (distancia_frente<distancia_objetivo):
                virar()
                self.get_logger().info ('Virando')
            else:
                wall()
                self.get_logger().info ('Seguindo a parede')
        self.get_logger().info ('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)


    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')
        
# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    goal_pose = PoseStamped() 
  # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
 
    exit(0)
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
   
if __name__ == '__main__':
    main()  
