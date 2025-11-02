class SimConfig:
    C = 2.99792458 * 1e8 # 光速 (m/s)
    F = 2.4 * 1e9 # 周波数 (Hz) - 2.4GHz
    
    NUM_DRONES = 20
    AREA_SIZE = (800, 600, 400)  # X, Y, Z (m)
    SIM_DURATION = 40  # seconds
    TIME_STEP = 1  # 1秒ごとにシミュレーションを更新
    VELOCITY_RANGE = (8.9, 31.3)  # 20-70 mphをm/sに変換
    COMM_RANGE = 250  # 通信範囲 (m) - 仮定 論文では50-300m 
    INITIAL_TRUST = 0.5
    INITIAL_ENERGY = 26.25 * 3600  # Whをジュールに変換 (仮)
    ENERGY_TX = 0.00000046 * 1e-9 # nJ/bitをJ/bitに変換
    PACKET_SIZE = 4500  # bits
    
    # 信頼度計算の係数 (論文より)
    A_COEFFICIENT = 0.3 # 0.2から0.45の間の値
    L_COEEFFICIENT = 0.5 # 直接信頼値
    M_COEFFICIENT = 0.5 # 間接接信頼値
    B_COEFFICIENT_FITNESS = 0.5 # a*F_i + b*HT_i の 'a'
    B_COEFFICIENT_HYBRID = 0.5 # a*F_i + b*HT_i の 'b'
    
    init_sigma = 9  # 初期分散の仮定値
    d_sigma = 2  # 分散の仮定値
    
    E_ELEC = 50e-9      # J/bit (50 nJ/bit), W_e
    E_AMP = 100e-12     # J/bit/m^2 (100 pJ/bit/m^2),  W_a