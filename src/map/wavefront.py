import numpy as np
from matplotlib import pyplot as plt

def wavefront(matriz, objetivo):
    linhas, colunas = matriz.shape
    # Cria um campo de onda inicializado com infinito
    campo_onda = np.full((linhas, colunas), np.inf)
    
    # Define a posição do objetivo com valor zero
    campo_onda[objetivo] = 0
    lista_processamento = [objetivo]  # Lista de células a processar

    # Direções de movimentação (cima, baixo, esquerda, direita)
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1)]

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

# Função para encontrar o caminho mais curto
def caminho_mais_curto(campo_onda, inicio, objetivo):
    caminho = [inicio]  # Inicia o caminho com a posição inicial
    atual = inicio  # Célula atual
    # Direções possíveis para movimentação
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    # Continua até não haver mais células a processar
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
                        # Verifica se o vizinho está na direção do objetivo
                        if verifica_direcao_objetivo(vizinho, objetivo, atual):
                            valor_minimo = campo_onda[vizinho]  # Atualiza o valor mínimo
                            proxima_celula = vizinho  # Define a próxima célula

        # Se não houver próxima célula ou se for um obstáculo, encerra o laço
        if proxima_celula is None or campo_onda[proxima_celula] == np.inf:
            break  # Não há mais caminho

        caminho.append(proxima_celula)  # Adiciona ao caminho
        atual = proxima_celula  # Atualiza a célula atual
        
        # Se o robô alcançou o objetivo, encerra
        if campo_onda[atual] == 0:  # Alcançou o objetivo
            break

    return caminho  # Retorna o caminho encontrado

def verifica_direcao_objetivo(vizinho, objetivo, atual):
    # Calcula a direção para o objetivo
    direcao_objetivo = (objetivo[0] - atual[0], objetivo[1] - atual[1])
    direcao_vizinho = (vizinho[0] - atual[0], vizinho[1] - atual[1])
    
    # Verifica se o vizinho está mais próximo da direção do objetivo
    return (direcao_objetivo[0] * direcao_vizinho[0] >= 0 and
            direcao_objetivo[1] * direcao_vizinho[1] >= 0)

# Carrega o mapa
with open('my_map_fixed.pgm', 'rb') as pgmf: 
    matriz = plt.imread(pgmf)

# Converte a matriz para um formato binário
matriz = 1.0 * (matriz > 250)  # 0 = obstáculo (preto), 1 = livre (branco)

# Define o objetivo e a posição inicial do robô
goal = (40, 80) 
inicio = (370, 50)  

# Executa o algoritmo de campo de onda
campo_onda = wavefront(matriz, goal)

# Encontra o caminho mais curto
caminho = caminho_mais_curto(campo_onda, inicio, goal)

# Marca o caminho encontrado no mapa
for celula in caminho:
    matriz[celula] = 0.5  # Marca o caminho no mapa

# Exibe o resultado
plt.imshow(matriz, interpolation='nearest', cmap='hot')
plt.title("Wavefront")
plt.show()
