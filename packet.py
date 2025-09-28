import asyncio
import numpy as np
import random
from typing import Any, Dict

from global_var import SimConfig

class Packet:
    def __init__(self, source_id: int, dest_id: int, data: Any, timestamp: float):
        self.source_id = source_id
        self.dest_id = dest_id
        self.data = data
        self.timestamp = timestamp
        
        
    