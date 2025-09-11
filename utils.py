import math
from global_var import SimConfig
def calculate_signal_strength(config: SimConfig, distance:float): 
    fspl =  20 * math.log10((4 * math.pi * distance * config.F) / config.C)
    ss = (20 - fspl)  # dbm 
    ss = 10 ** (ss / 10) # mWに変換
    return ss
