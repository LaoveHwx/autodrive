import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autodrive_simu.simulator.road_simu import Simulator

if __name__ == "__main__":
    sim = Simulator()
    sim.run()