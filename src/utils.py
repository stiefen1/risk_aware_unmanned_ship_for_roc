import numpy as np
from submodules.ship_in_transit_simulator.models import StaticObstacle

def load_obstacles(file_path: str) -> list[StaticObstacle]:
    obstacle_txt = np.loadtxt(file_path)
    obstacles_list = []
    for obstacle in obstacle_txt:
        obstacles_list.append(StaticObstacle(obstacle[0], obstacle[1], obstacle[2]))
    return obstacles_list

def heading_sim_to_enc(heading: float) -> float:
    return 90 - heading * 180 / np.pi