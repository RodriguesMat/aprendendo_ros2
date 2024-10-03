import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from collections import deque
import numpy as np

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber do odom: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug('Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inicializando a grid para o Wavefront
        self.grid_size = 20  # Define uma grid de 20x20 para exemplo
        self.grid = np.full((self.grid_size, self.grid_size), -1)
        self.start = (self.grid_size - 1, 0)  # Posição inicial na grid (inferior esquerdo)
        self.goal = (0, self.grid_size - 1)   # Posição objetivo na grid (superior direito)

        # Lista para armazenar obstáculos detectados
        self.obstacles = []

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

        # Verifica se há obstáculos à frente
        self.distancia_frente = min(self.laser[80:100])  # Faixa central do laser
        if self.distancia_frente < 1.5:
            self.get_logger().info('Obstáculo detectado! Adicionando à grid de obstáculos.')
            self.adicionar_obstaculo()

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def adicionar_obstaculo(self):
        # Aqui, adicionamos um obstáculo na posição aproximada do robô
        # No caso real, pode-se usar a odometria e as leituras do laser para determinar a posição
        obst_x = self.grid_size // 2  # Simulação aproximada
        obst_y = self.grid_size // 2  # Simulação aproximada
        self.obstacles.append((obst_x, obst_y))
        self.grid[obst_x][obst_y] = 1  # Marca como obstáculo na grid

    def wavefront(self):
        rows = self.grid_size
        cols = self.grid_size

        moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Movimentos permitidos

        queue = deque([self.goal])
        self.grid[self.goal[0]][self.goal[1]] = 0  # Inicializa o objetivo

        while queue:
            x, y = queue.popleft()
            current_value = self.grid[x][y]

            # Explorar as células vizinhas
            for move in moves:
                nx, ny = x + move[0], y + move[1]

                if 0 <= nx < rows and 0 <= ny < cols and self.grid[nx][ny] == -1:
                    self.grid[nx][ny] = current_value + 1
                    queue.append((nx, ny))

    def run(self):
        self.get_logger().debug('Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.get_logger().info('Iniciando o Wavefront')
        self.wavefront()  # Inicializa o algoritmo wavefront

        self.get_logger().info('Executando o loop principal do robô.')
        while rclpy.ok():
            rclpy.spin_once(self)

            # Determina a ação baseada no wavefront
            if self.grid[self.start[0]][self.start[1]] != -1:
                # Move o robô em direção ao objetivo
                self.get_logger().info('Movendo em direção ao objetivo.')
                self.pub_cmd_vel.publish(self.ir_para_frente)
            else:
                self.get_logger().info('Nenhum caminho encontrado ou bloqueado. Parando.')
                self.pub_cmd_vel.publish(self.parar)

            self.get_logger().debug('Atualizando as distâncias lidas pelo laser.')
            if self.distancia_frente < 1.5:
                self.get_logger().info('Obstáculo detectado à frente. Parando.')
                self.pub_cmd_vel.publish(self.parar)
                break

        self.get_logger().info('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)

    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
