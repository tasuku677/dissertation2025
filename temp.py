import numpy as np
import math
C = 2.99792458 * 1e8 # 光速 (m/s)
F = 2.4 * 1e9 # 周波数 (Hz) - 2.4GHz

distance = 320  # 距離 (m)

def calculate_signal_strength(distance): 
    fspl =  20 * math.log10((4 * math.pi * distance * F) / C)
    ss = (20 - fspl)  # dbm 
    ss = 10 ** (ss / 10) # mWに変換
    print(fspl, ss)

calculate_signal_strength(distance)