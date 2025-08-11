class UAV:
    """
    UAV class representing a drone in the simulation.
    """
    def __init__(self, id: int, x: float, y: float, z: float, energy: float, ss: float, pdr: float, delay: float,
                 direct_trust: float = 0.5, indirect_trust: float = 0.5, fitness: float = 0.0, final_trust: float = 0.5, malicious: bool = False):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.energy = energy          # 残余エネルギー(0〜1)
        self.ss = ss                  # 信号強度 (0〜100)
        self.pdr = pdr                # パケット配送率 (0〜100)
        self.delay = delay            # 遅延 (ms)
        self.direct_trust = direct_trust
        self.indirect_trust = indirect_trust
        self.fitness = fitness
        self.final_trust = final_trust
        self.malicious = malicious