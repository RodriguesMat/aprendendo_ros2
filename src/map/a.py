import numpy as np
from matplotlib import pyplot as plt

# Função para calcular a distância Euclidiana
def distancia(posicao_atual, destino):
    return np.sqrt((posicao_atual[0] - destino[0]) ** 2 + (posicao_atual[1] - destino[1]) ** 2)

# Função para calcular a probabilidade de ocupação
def probabilidade(matriz, posicao):
    raio = 4
    ocupacao = 0
    for x in range(-raio, raio + 1): 
        for y in range(-raio, raio + 1):  
            vizinho = (posicao[0] + x, posicao[1] + y)
            if 0 <= vizinho[0] < matriz.shape[0] and 0 <= vizinho[1] < matriz.shape[1]:
                ocupacao += 1 - matriz[vizinho]  # Mais próximo dos obstáculos aumenta a ocupação
    
    max_ocupacao = (2 * raio + 1) ** 2  # Máximo de células no raio
    return ocupacao / max_ocupacao

def navegação(matriz, inicio, destino):
    # Fila de nós a explorar
    fila = [(0, inicio)]  # Contém (custo, nó)
    
    # Dicionários para armazenar custo e caminho
    custos = {inicio: 0}
    caminho = {inicio: None}
    
    # Direções possíveis para movimentação
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    while fila:
        # Encontra o nó com menor custo
        _, atual = min(fila, key=lambda x: x[0])
        fila.remove((_, atual))
        
        # Se o objetivo foi alcançado, encerra a busca
        if atual == destino:
            break
        
        for direcao in direcoes:
            vizinho = (atual[0] + direcao[0], atual[1] + direcao[1])
            
            # Verifica se o vizinho está dentro dos limites da matriz e se é um espaço livre
            if 0 <= vizinho[0] < matriz.shape[0] and 0 <= vizinho[1] < matriz.shape[1] and matriz[vizinho] == 1:
                novo_custo = custos[atual] + distancia(atual, vizinho)
                p_ocupacao = probabilidade(matriz, vizinho)
                
                # Atualiza custo se for menor
                if vizinho not in custos or novo_custo < custos[vizinho]:
                    custos[vizinho] = novo_custo
                    prioridade = (novo_custo * p_ocupacao) + distancia(vizinho, destino)  # f(n) = g(n) * Pocc(n) + h(n)
                    fila.append((prioridade, vizinho))
                    caminho[vizinho] = atual

    caminho_final = []
    atual = destino
    while atual is not None:
        caminho_final.append(atual)
        atual = caminho.get(atual)
    
    caminho_final.reverse()
    return caminho_final

# Carrega a matriz do mapa
with open('my_map_fixed.pgm', 'rb') as pgmf:
    matriz = plt.imread(pgmf)

# Converte a matriz para formato binário
matriz = 1.0 * (matriz > 250)  # 0 = obstáculo (preto), 1 = livre (branco)

# Define o objetivo e a posição inicial do robô
goal = (140, 240) 
inicio = (340, 120) 

# Executa o algoritmo A*
caminho_encontrado = navegação(matriz, inicio, goal)

# Marca o caminho encontrado no mapa
for celula in caminho_encontrado:
    matriz[celula] = 0.5  

# Exibe o resultado
plt.imshow(matriz, interpolation='nearest', cmap='hot')
plt.title("A*")
plt.show()
