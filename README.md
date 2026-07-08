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
