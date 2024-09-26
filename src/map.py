# TODO: provide a way to interface ENC with path planning algorithms

from submodules.PathPlanning.Search_based_Planning.Search_2D.env import Env
from dataclasses import dataclass
from shapely import Geometry
from shapely.geometry import Polygon, Point
import numpy as np
from src.wind import WindArray, WindVector

@dataclass
class DiscreteMap:
    def __init__(self, geometry:Geometry, wind:WindArray=None, dx:float=10., dy:float=10.):
        # discretization
        self._dx_m:    float = dx # meters
        self._dy_m:    float = dy # meters
        
        # geometry
        self._geometry = geometry
        self.bounds = geometry.bounds
        self._obs = self._parse_geometry(geometry)

        # wind
        self._wind = wind

        # Environment
        self._motions:list = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                             (1, 0), (1, -1), (0, -1), (-1, -1)]

    def update_obs(self, obs:set):
        self._obs = obs


    def _parse_geometry(self, geometry:Geometry) -> set:
        obs = set()

        x = self.x_range
        y = self.y_range

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        self._binary_map = np.ones((int(self.width/self.dx), int(self.height/self.dy)))
        for x_idx in range(int(self.width/self.dx)):
            for y_idx in range(int(self.height/self.dy)):
                x_m, y_m = self.map_to_world(x_idx, y_idx)
                if geometry.contains(Point(x_m, y_m)):
                    self._binary_map[x_idx, y_idx] = 0
                else:
                    obs.add((x_idx, y_idx))

                # print(f'x_idx: {x_idx}, y_idx: {y_idx}, x_m: {x_m}, y_m: {y_m}, binary_map: {self._binary_map[x_idx, y_idx]}')

        return obs

    def is_obstacle_free(self, x_m:float, y_m:float) -> bool:
        assert self.is_within_bounds(x_m, y_m), f'Impossible to evaluate if point is obstacle free - Point ({x_m:.2f}, {y_m:.2f}) is out of bounds'

        x_idx, y_idx = self.world_to_map(x_m, y_m)
        return self._binary_map[x_idx, y_idx] == 0
    
    def is_within_bounds(self, x_m:float, y_m:float) -> bool:
        return (self.x_min <= x_m <= self.x_max) and (self.y_min <= y_m <= self.y_max)
    
    def map_to_world(self, x_idx:int, y_idx:int) -> tuple:
        return self.x_min + x_idx*self.dx, self.y_min + y_idx*self.dy
    
    def world_to_map(self, x_m:float, y_m:float) -> tuple:
        return int((x_m - self.x_min)/self.dx), int((y_m - self.y_min)/self.dy)

    def obs_map(self) -> set:
        return self._obs

    def env(self) -> Env:
        pass

    def risk_from_discrete_coord(self, x_idx:int, y_idx:int, method:str='radius-level') -> float:
        """Evaluate risk at a given point"""
        x_m, y_m = self.map_to_world(x_idx, y_idx)
        return self.risk_in_world(x_m, y_m, method=method)

    def risk_in_world(self, x_m:float, y_m:float, method:str='radius-level') -> float:
        """Evaluate risk at a given point"""
        if method == 'radius-level':
            return self._risk_radius_level(x_m, y_m)
        elif method == 'wind-cross-distance':
            return self._risk_wind_cross_distance(x_m, y_m)
        else:
            raise ValueError(f'Unknown risk evaluation method: {method}')
        
    def _risk_radius_level(self, x_m:float, y_m:float, radius_at_level_5:float=10., radius_at_level_1:float=1000., radius_distribution:str='lin') -> float:
        """
        Evaluate risk based on distance to closest obstacle. Five risk levels are defined, corresponding
        to five circle drawn around the current position. 
        Using this instead of the distance from closest obstacle is that it is computationnaly cheaper.
        """
        number_of_level:int = 5
        if radius_distribution == 'lin':
            radius = np.linspace(radius_at_level_5, radius_at_level_1, number_of_level)
        elif radius_distribution == 'log':
            radius = np.logspace(np.log10(radius_at_level_5), np.log10(radius_at_level_1), number_of_level)

        # print("GEOMETRY: ", self._geometry)

        # print(f'radius: {radius}, x_m: {x_m}, y_m: {y_m}')
        for i, r in enumerate(radius):
            point = Point(x_m, y_m)
            circle = point.buffer(r).convex_hull
            if circle.intersects(self._geometry.boundary):
                # print(f'number_of_level-level: {i}, intersects: {circle.intersects(self._geometry.boundary)}')
                return number_of_level - i
            
        return 0

    def _risk_wind_cross_distance(self, x_m:float, y_m:float) -> float:
        assert self._wind is not None, 'Wind data is not available'

        wind_at_xy: WindVector = self._wind(x_m, y_m)
        dist_to_closest_obs = self._dist_to_closest_obs(x_m, y_m)

    @property
    def dx(self) -> float:
        return self._dx_m

    @property
    def dy(self) -> float:
        return self._dy_m
    
    @property
    def x_min(self) -> float:
        return self.bounds[0]
    
    @property
    def x_max(self) -> float:
        return self.bounds[2]
    
    @property
    def y_min(self) -> float:
        return self.bounds[1]
    
    @property
    def y_max(self) -> float:
        return self.bounds[3]
    
    @property
    def width(self) -> float:
        return self.bounds[2] - self.bounds[0]
    
    @property
    def height(self) -> float:
        return self.bounds[3] - self.bounds[1]
    
    @property
    def x_range(self) -> int:
        return int(self.width/self.dx)
    
    @property
    def y_range(self) -> int:
        return int(self.height/self.dy)
    
    @property
    def motions(self) -> list:
        return self._motions
    
    @property
    def obs(self) -> set:
        return self._obs

    