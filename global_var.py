class SimConfig:
    C = 2.99792458 * 1e8 # 光速 (m/s)
    F = 2.4 * 1e9 # 周波数 (Hz) - 2.4GHz
    
    P_WINDOW_SIZE = 10
    CHANGE_BEHAVIOR_T = 100 # seconds, 悪意ノードの行動タイプを切り替える周期
    
    NUM_DRONES = 50
    AREA_SIZE = (1600, 800, 800)  # X, Y, Z (m)
    BS_POS = [400, 300, 0]  # 地上局の位置 (m)
    PREPARATION_DURATION = 20  # seconds
    SIM_DURATION = 100  # seconds
    TIME_STEP = 1  # 1秒ごとにシミュレーションを更新
    VELOCITY_RANGE = (8.9, 31.3)  # 20-70 mphをm/sに変換
    COMM_RANGE = 250  # 通信範囲 (m) - 仮定 論文では50-300m 
    INITIAL_TRUST = 0.5
    INITIAL_ENERGY = 26.25 * 3600  # Whをジュールに変換 (仮)
    ENERGY_TX = 460 * 1e-9 # nJ/bitをJ/bitに変換
    ENERGY_RX = 180 * 1e-9 # nJ/bitをJ/bitに変換
 
    PACKET_SIZE = 1024 * 8  # bits
    
    # 信頼度計算の係数 (論文より)
    A_COEFFICIENT = 0.3 # 0.2から0.45の間の値
    L_COEEFFICIENT = 0.5 # 直接信頼値
    M_COEFFICIENT = 0.5 # 間接接信頼値
    B_COEFFICIENT_FITNESS = 0.5 # a*F_i + b*HT_i の 'a'
    B_COEFFICIENT_HYBRID = 0.5 # a*F_i + b*HT_i の 'b'
    
    INIT_SIGMA = 2  # 初期分散の仮定値
    
    E_ELEC = 50e-9      # J/bit (50 nJ/bit), W_e
    E_AMP = 100e-12     # J/bit/m^2 (100 pJ/bit/m^2),  W_a
    
    # ノードタイプの割合
    NODE_TYPE_WEIGHTS = {'good': 0.4, 'neutral': 0.3, 'bad': 0.3}  # good, neutral, bad の割合