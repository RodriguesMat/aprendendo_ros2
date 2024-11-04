import numpy as np                   # Para criar arrays
from matplotlib import pyplot as plt # Para leitura do mapa
import random                        # Para contorno
import math                          # Para operações

# Constantes para os campos
katt = 1.0      # Fator de escala do campo atrativo
krep = 500.0    # Fator de escala do campo repulsivo
raio = 100.0    # Distância de influência do obstáculo
delta = 1e-3    # Tolerância para considerar o potencial como zero

# Função para calcular o campo atrativo em direção ao objetivo
def busca_do_objetivo(q, qgoal):
    pitagoras = math.sqrt((q[0] - qgoal[0]) ** 2 + (q[1] - qgoal[1]) ** 2)
    Uatt = 0.5 * katt * (pitagoras ** 2)
    return Uatt

# Função para calcular o campo repulsivo a partir dos obstáculos
def desvio_dos_obstaculos(q, obstacles):
    Urep = 0
    for obstacle in obstacles:
        distancia_obstaculo = math.sqrt((q[0] - obstacle[0]) ** 2 + (q[1] - obstacle[1]) ** 2)
        if distancia_obstaculo <= raio:
            Urep += 0.5 * krep * ((1/distancia_obstaculo) - (1 / raio)) * 2
    return Urep

# Função para calcular o valor total de potencial U em uma posição
def calcular_potencial(q, goal, obstacles):
    Uatt = busca_do_objetivo(q, goal)           # Energia Potencial de Attraction
    Urep = desvio_dos_obstaculos(q, obstacles)  # Potential Energy of Repulsão
    return Uatt + Urep

# Função para encontrar o caminho mais baixo de potencial (gradiente). OBS: após 1000 iterações, ele parará
def campos_potenciais(start, goal, obstacles, max_steps = 1000):
    path = [start]
    current_position = np.array(start)

    for k in range(max_steps):
        # Calcula os vizinhos #----------------------------------------------
        neighbors = []
        movimentos = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, -1), (1, 1), (-1, -1)]

        for dx, dy in movimentos:
            new_x = current_position[0] + dx
            new_y = current_position[1] + dy
    
            if 0 <= new_x < matrix.shape[0] and 0 <= new_y < matrix.shape[1]:  # Verifica se a nova posição está dentro dos limites da matriz
                neighbors.append((new_x, new_y))

        # Avalia o valor de potencial para cada vizinho #--------------------
        potenciais = []
        for neighbor in neighbors:
            potenciais.append(calcular_potencial(neighbor, goal, obstacles))

        # Verifica se o menor potencial é próximo de zero #------------------
        menor_indice = np.argmin(potenciais)
        next_position = neighbors[menor_indice]

        if potenciais[menor_indice] < delta and next_position != goal:
            # Movimento aleatório se o potencial for próximo de zero
            # Escolhe um movimento aleatório (esquerda ou direita)
            if random.choice([True, False]):
                next_position = (current_position[0], current_position[1] - 1)  # Para a esquerda
            else:
                next_position = (current_position[0], current_position[1] + 1)  # Para a direita

        # Guardando o caminho do robô #--------------------------------------
        path.append(tuple(next_position))

        # Atualiza a posição atual #-----------------------------------------
        current_position = np.array(next_position)

        # Verifica se o objetivo foi alcançado, se não foi ele continua, contrário disso, encerra o looping
        if math.sqrt((current_position[0] - goal[0]) ** 2 + (current_position[1] - goal[1]) ** 2) < 1.0:
            break

    return path

# Carrega o mapa
pgmf = open('my_map_fixed.pgm', 'rb')
matrix = plt.imread(pgmf)
pgmf.close()

matrix = 1.0 * (matrix > 250)   # Converte a matriz para binário (0 para obstáculo, 1 para livre)

start = (270, 50)               # Posição inicial do robô
goal = (70, 340)                # Posição do objetivo

# Extrai obstáculos
obstacles = []
for i in range(matrix.shape[0]):
    for j in range(matrix.shape[1]):
        if matrix[i, j] == 0:   # OBS : "0" = obstáculo
            obstacles.append((i, j))

# Executa o algoritmo
path = campos_potenciais(start, goal, obstacles)

# Exibe o caminho encontrado no mapa
for cell in path:
    matrix[int(cell[0]), int(cell[1])] = 0.5

print("abriu o mapa")
plt.imshow(matrix, interpolation='nearest', cmap='hot')
plt.title("Campos Potenciais")
plt.show()
