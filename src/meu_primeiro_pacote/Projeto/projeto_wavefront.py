import numpy as np
import time
#import rospy 
from rclpy.node import Node
from std_msgs.msg import String
from matplotlib import pyplot as plt

def wavefront(matriz, objetivo):
    linhas, colunas = matriz.shape
    # Cria um campo de onda inicializado com infinito
    campo_onda = np.full((linhas, colunas), np.inf)
    
    # Define a posição do objetivo com valor zero
    campo_onda[objetivo] = 0
    lista_processamento = [objetivo]  # Lista de células a processar

    # Direções de movimentação (cima, baixo, esquerda, direita e diagonais)
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, -1), (1, 1), (-1, -1)]

    # Processa as células enquanto houver
    while lista_processamento:
        atual = lista_processamento.pop(0)  # Pega a próxima célula
        valor_atual = campo_onda[atual]  # Valor atual da célula

        # Verifica as células vizinhas
        for direcao in direcoes:
            vizinho = (atual[0] + direcao[0], atual[1] + direcao[1])
            
            # Verifica se o vizinho está dentro dos limites e é um espaço livre
            if (0 <= vizinho[0] < linhas) and (0 <= vizinho[1] < colunas) and matriz[vizinho] == 1:
                # Atualiza o valor do vizinho se for menor
                if campo_onda[vizinho] > valor_atual + 1:
                    campo_onda[vizinho] = valor_atual + 1  # Aumenta a distância
                    lista_processamento.append(vizinho)  # Adiciona à lista para processar

    return campo_onda  # Retorna o campo de onda

def caminho_mais_curto(campo_onda, inicio, objetivo):
    caminho = [inicio]  # Inicia o caminho com a posição inicial
    direcoes_robo = []  # Lista para armazenar as direções
    atual = inicio  # Célula atual
    # Direções possíveis para movimentação
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, -1), (1, 1), (-1, -1)]
    
    # Continua até alcançar o objetivo
    while True:
        valor_minimo = np.inf  # Inicializa o valor mínimo
        proxima_celula = None  # Próxima célula a ser escolhida
        
        # Verifica as células vizinhas
        for direcao in direcoes:
            vizinho = (atual[0] + direcao[0], atual[1] + direcao[1])
            # Verifica se o vizinho está dentro dos limites
            if (0 <= vizinho[0] < campo_onda.shape[0] and
                0 <= vizinho[1] < campo_onda.shape[1]):
                
                # Verifica se o vizinho é um espaço livre
                if matriz[vizinho] == 1:  # Apenas considera células livres
                    # Verifica se o vizinho tem o menor valor
                    if campo_onda[vizinho] < valor_minimo:
                        valor_minimo = campo_onda[vizinho]  # Atualiza o valor mínimo
                        proxima_celula = vizinho  # Define a próxima célula

        # Se não houver próxima célula, encerra
        if proxima_celula is None or campo_onda[proxima_celula] == np.inf:
            break  # Não há mais caminho

        # Calcula a direção entre a célula atual e a próxima
        direcao = (proxima_celula[0] - atual[0], proxima_celula[1] - atual[1])
        direcoes_robo.append(direcao)  # Adiciona a direção à lista

        caminho.append(proxima_celula)  # Adiciona ao caminho
        atual = proxima_celula  # Atualiza a célula atual
        
        # Se o robô alcançou o objetivo, encerra
        if campo_onda[atual] == 0:  # Alcançou o objetivo
            break

    return caminho, direcoes_robo  # Retorna o caminho e as direções

# Carrega o mapa
with open('my_map_fixed.pgm', 'rb') as pgmf: 
    matriz = plt.imread(pgmf)

# Converte a matriz para um formato binário
matriz = 1.0 * (matriz > 250)  # 0 = obstáculo (preto), 1 = livre (branco)

# Define o objetivo e a posição inicial do robô
goal = (22, 320) 
inicio = (370, 50)  

# Executa o algoritmo de campo de onda
campo_onda = wavefront(matriz, goal)

# Encontra o caminho mais curto e as direções
caminho, direcoes_robo = caminho_mais_curto(campo_onda, inicio, goal)

# Marca o caminho encontrado no mapa
for celula in caminho:
    matriz[celula] = 0.5  # Marca o caminho no mapa

# Exibe as direções
print("Direções para o robô seguir:")
for direcao in direcoes_robo:
    print(direcao)
    #for direcao in sentidos:
        #self.pub_cmd_vel.publish(direcao)
        #rclpy.spin_once(self)
        #time.sleep(0.5)

# Exibe o resultado
plt.imshow(matriz, interpolation='nearest', cmap='hot')
plt.title("Wavefront")
plt.show()
