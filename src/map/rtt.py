import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import random

# Carregando a imagem
with open('my_map_fixed.pgm', 'rb') as pgmf:
    img = plt.imread(pgmf)

# Criando uma cópia da imagem com base em um threshold
image = 1.0 * (img > 240)

# Definindo os pontos de objetivo e do robô
goal = (90, 15)
robot_pos = (25, 300)

# Marcando os pontos na imagem
image[goal[0]][goal[1]] = 0
image[robot_pos[0]][robot_pos[1]] = 0

# Parâmetros do algoritmo
f_crescimento = 12
arvore = [robot_pos]
caminho = []
pais = []
filhos = []
max_interacoes = 800

def encontrar_no_proximo(arvore, ponto_aleatorio):
    menor_dist = float('inf')
    no_proximo = None
    for n in arvore:
        dist = math.dist(n, ponto_aleatorio)
        if dist < menor_dist:
            menor_dist = dist
            no_proximo = n
    return no_proximo

def criar_novo_no(no_proximo, ponto_aleatorio, f_crescimento):
    vetor_direcao = (ponto_aleatorio[0] - no_proximo[0], ponto_aleatorio[1] - no_proximo[1])
    modulo_vetor = math.sqrt(vetor_direcao[0]**2 + vetor_direcao[1]**2)

    if modulo_vetor < f_crescimento:
        return ponto_aleatorio
    
    fator = f_crescimento / modulo_vetor
    novo_no = (int(no_proximo[0] + vetor_direcao[0] * fator), int(no_proximo[1] + vetor_direcao[1] * fator))
    
    return novo_no

for i in range(max_interacoes):
    ponto_aleatorio = (random.randint(0, 350), random.randint(0, 350))  

    no_proximo = encontrar_no_proximo(arvore, ponto_aleatorio)
    novo_no = criar_novo_no(no_proximo, ponto_aleatorio, f_crescimento)

    if image[novo_no[0]][novo_no[1]] == 1.0:
        arvore.append(novo_no)
        caminho.append((no_proximo, novo_no))
        pais.append(no_proximo)
        filhos.append(novo_no)

        if math.dist(novo_no, goal) < f_crescimento:
            print("Objetivo alcançado!")
            break

# Convertendo a imagem para BGR para o OpenCV
image = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)

# Colorindo as linhas da árvore
for (ponto1, ponto2) in caminho:
    cv2.line(image, ponto1[::-1], ponto2[::-1], (0, 0, 255), 1)

# Colorindo o caminho correto
caminho_correto = []
filho_atual = no_proximo
pai_atual = novo_no

while True:
    indice = filhos.index(filho_atual)
    pai_atual = pais[indice]
    caminho_correto.append((pai_atual, filho_atual))
    filho_atual = pai_atual
    if filho_atual == robot_pos:
        break

# Colorindo o caminho final
for (ponto1, ponto2) in caminho_correto:
    cv2.line(image, ponto1[::-1], ponto2[::-1], (255, 0, 0), 1)

plt.imshow(image)
plt.title('Árvore Aleatória de Exploração Rápida (RRT)')
plt.show()
