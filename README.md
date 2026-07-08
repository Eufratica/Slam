# 🤖 Análise Comparativa de SLAM e Localização Autônoma (AMCL)

**Instituição:** Universidade Federal da Bahia (UFBA)
**Autor:** Gerson Daniel Santos Marques
**Ambiente de Simulação:** ROS Noetic / Gazebo / RViz
**Plataforma Robótica:** Robô Móvel Husky (Clearpath Robotics)

---

## 📑 Índice
1. [Visão Geral do Projeto](#1-visão-geral-do-projeto)
2. [Estrutura do Repositório](#2-estrutura-do-repositório)
3. [Pré-requisitos e Dependências](#3-pré-requisitos-e-dependências)
4. [Metodologia de Execução](#4-metodologia-de-execução)
5. [Processamento de Dados e Cálculos](#5-processamento-de-dados-e-cálculos)
6. [Análise Quantitativa](#6-análise-quantitativa)
7. [Análise Qualitativa dos Mapas](#7-análise-qualitativa-dos-mapas)
8. [Conclusão](#8-conclusão)

---

## 1. Visão Geral do Projeto

Este repositório documenta o processo de avaliação de algoritmos de Mapeamento Simultâneo e Localização (SLAM) em um ambiente simulado. O objetivo central é comparar a qualidade dos mapas gerados pelos métodos **Gmapping** e **Hector SLAM** e, subsequentemente, avaliar como essas diferentes qualidades de mapa impactam a precisão do algoritmo de localização **AMCL (Adaptive Monte Carlo Localization)**.

A validação foi realizada extraindo dados temporais (arquivos `.bag`) da pose estimada pelo filtro de partículas do AMCL e comparando-os diretamente com a pose absoluta (*ground truth*) fornecida pelo simulador Gazebo.

---

## 2. Estrutura do Repositório

Abaixo está a organização dos arquivos críticos para a reprodução deste experimento:

```text
📦 catkin_ws/src/lar_gazebo
 ┣ 📂 launch
 ┃ ┣ 📜 amcl.launch               # Configuração do filtro de partículas e parâmetros de odometria
 ┃ ┣ 📜 mapa_gmapping_final.yaml  # Metadados do mapa (Resolução: 0.05m/px)
 ┃ ┣ 📜 mapa_hector_final.yaml    # Metadados do mapa (Resolução: 0.05m/px)
 ┃ ┣ 🖼️ mapa_gmapping_final.pgm   # Mapa rasterizado Gmapping
 ┃ ┣ 🖼️ mapa_hector_final.pgm     # Mapa rasterizado Hector SLAM
 ┃ ┣ 🗃️ comparacao_gmapping.bag   # Dados brutos de validação (Gmapping)
 ┃ ┗ 🗃️ comparacao_hector.bag     # Dados brutos de validação (Hector)
 ┣ 📂 scripts
 ┃ ┗ 🐍 analisar_rmse.py          # Script Python para extração e cálculo de métricas
 ┗ 📜 README.md                   # Documentação do projeto
```

## 3. Pré-requisitos e Dependências
Para executar os *launch files* e o script de análise de dados, o sistema deve possuir:
* Ubuntu 20.04 LTS com ROS Noetic instalado.
* Pacotes ROS: `gazebo_ros`, `amcl`, `map_server`, `teleop_twist_keyboard`.
* Python 3.8+ com as bibliotecas analíticas:
  
```bash
pip install pandas numpy bagpy
```

## 4. Metodologia de Execução

Os experimentos foram conduzidos rigorosamente sob as mesmas condições para ambos os mapas. O roteiro de execução consiste em instanciar o mundo, carregar o mapa, inicializar o AMCL com *2D Pose Estimate*, movimentar o robô e gravar os tópicos necessários.

### Passo 1: Inicialização do Ambiente
Em um terminal, inicie a simulação do Husky no laboratório:
```bash
roslaunch lar_gazebo lar_husky_sim.launch
```

Passo 2: Inicialização do AMCL
Em um segundo terminal, carregue o sistema de localização apontando para o mapa desejado (Gmapping ou Hector):

```bash
# Exemplo executando o mapa gerado pelo Hector SLAM
roslaunch lar_gazebo amcl.launch map_file:=/caminho/absoluto/mapa_hector_final.yaml
```
No RViz, é estritamente necessário utilizar a ferramenta 2D Pose Estimate para fornecer a estimativa inicial e permitir a convergência das partículas antes do início do movimento.

Passo 3: Coleta de Dados (Rosbag Record)
Para registrar a disparidade entre a crença do robô e a realidade física, os tópicos de pose foram gravados em um terceiro terminal:

```bash
rosbag record -O comparacao_hector.bag /amcl_pose /gazebo/model_states
```
Passo 4: Teleoperação
Movimente o robô pelo ambiente garantindo variações de translação e rotação:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
5. Processamento de Dados e Cálculos
O script analisar_rmse.py foi desenvolvido para ler nativamente as mensagens do ROS, alinhar as séries temporais (que possuem frequências de publicação distintas) utilizando pandas.merge_asof, e calcular as métricas exigidas.

Formulação Matemática
As seguintes equações foram implementadas no script para a avaliação do sistema:

### Formulação Matemática
As seguintes equações foram implementadas no script para a avaliação do sistema:

**1. Erro Euclidiano de Posição (a cada instante $t$):**
$$E_p(t) = \sqrt{(x_{amcl}(t) - x_{gaz}(t))^2 + (y_{amcl}(t) - y_{gaz}(t))^2}$$

**2. RMSE (Root Mean Square Error) de Posição:**
$$RMSE_p = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (E_p(t_i))^2}$$

**3. Erro de Orientação (Yaw):**
Diferença angular mapeada para o intervalo $[-\pi, \pi]$ utilizando quaternions convertidos para ângulos de Euler.


