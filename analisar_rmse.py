import rosbag
import pandas as pd
import numpy as np
import math

# Função matemática para converter Quaternions em Ângulo de Euler (Yaw/Z)
def get_yaw_from_quat(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

def analisar_bag(bag_file, robot_name='husky'):
    print(f"\n[{'='*40}]")
    print(f"Processando arquivo: {bag_file}")
    print(f"[{'='*40}]\n")
    
    amcl_data = []
    gazebo_data = []
    
    # 1. Leitura Nativa do Rosbag
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            # Lendo as estimativas do AMCL
            for topic, msg, t in bag.read_messages(topics=['/amcl_pose']):
                yaw = get_yaw_from_quat(msg.pose.pose.orientation)
                amcl_data.append({
                    'time': t.to_sec(),
                    'x_amcl': msg.pose.pose.position.x,
                    'y_amcl': msg.pose.pose.position.y,
                    'yaw_amcl': yaw
                })
                
            # Lendo a verdade absoluta (Ground Truth) do Gazebo
            for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
                try:
                    # Encontra o índice do robô na lista de modelos do Gazebo
                    idx = msg.name.index(robot_name)
                    yaw = get_yaw_from_quat(msg.pose[idx].orientation)
                    gazebo_data.append({
                        'time': t.to_sec(),
                        'x_gaz': msg.pose[idx].position.x,
                        'y_gaz': msg.pose[idx].position.y,
                        'yaw_gaz': yaw
                    })
                except ValueError:
                    continue # Ignora se o robô não estiver neste frame
                    
    except Exception as e:
        print(f"Erro ao ler o arquivo {bag_file}: {e}")
        return

    # 2. Conversão para Pandas e Alinhamento Temporal
    df_amcl = pd.DataFrame(amcl_data)
    df_gaz = pd.DataFrame(gazebo_data)
    
    if df_amcl.empty or df_gaz.empty:
        print("Aviso: Dados insuficientes no arquivo. Verifique os tópicos gravados.")
        return

    # Alinha os dados do AMCL com o frame do Gazebo mais próximo no tempo
    df_merged = pd.merge_asof(df_amcl.sort_values('time'), 
                              df_gaz.sort_values('time'), 
                              on='time', 
                              direction='nearest')

    # 3. Cálculos das Métricas Exigidas
    
    # A) Erro de Posição (Distância Euclidiana a cada instante t)
    df_merged['erro_x'] = df_merged['x_amcl'] - df_merged['x_gaz']
    df_merged['erro_y'] = df_merged['y_amcl'] - df_merged['y_gaz']
    df_merged['erro_posicao'] = np.sqrt(df_merged['erro_x']**2 + df_merged['erro_y']**2)
    
    # B) RMSE de Posição
    rmse_pos = np.sqrt(np.mean(df_merged['erro_posicao']**2))
    
    # C) Erro Final de Posição (Último registro gravado)
    erro_final = df_merged['erro_posicao'].iloc[-1]
    
    # D) Erro de Orientação (Diferença absoluta do Yaw)
    # math.remainder garante que a diferença fique no intervalo [-pi, pi]
    df_merged['erro_yaw'] = [math.remainder(a - g, 2*math.pi) for a, g in zip(df_merged['yaw_amcl'], df_merged['yaw_gaz'])]
    rmse_yaw = np.sqrt(np.mean(df_merged['erro_yaw']**2))
    
    # E) Estabilidade (Desvio Padrão do Erro de Posição)
    # Um desvio padrão baixo significa que o AMCL não ficou "pulando", mantendo a margem de erro constante.
    estabilidade = np.std(df_merged['erro_posicao'])

    # 4. Exibição dos Resultados Prontos para o Relatório
    print("MÉTRICAS QUANTITATIVAS CALCULADAS:")
    print("-" * 40)
    print(f"1. Erro Médio de Posição : {np.mean(df_merged['erro_posicao']):.4f} metros")
    print(f"2. RMSE de Posição       : {rmse_pos:.4f} metros")
    print(f"3. Erro Final de Posição : {erro_final:.4f} metros")
    print(f"4. RMSE de Orientação    : {rmse_yaw:.4f} radianos")
    print(f"5. Estabilidade (Desvio) : {estabilidade:.4f} metros")
    print("-" * 40)
    print("-> Uma estabilidade menor indica que o filtro de partículas sofreu menos sobressaltos.\n")

# =======================================================
# EXECUÇÃO
# Substitua 'husky' pelo nome exato do seu modelo no Gazebo, caso seja diferente.
# =======================================================
if __name__ == '__main__':
    # Roda a análise para o Gmapping
    analisar_bag('comparacao_gmapping.bag', robot_name='husky') 
    
    # Roda a análise para o Hector (descomente quando o arquivo existir)
    analisar_bag('comparacao_hector.bag', robot_name='husky')
