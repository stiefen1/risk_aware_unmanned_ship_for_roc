# TODO: represent wind in the simulation.


from typing import Any, Callable
import numpy as np
from shapely.geometry.base import BaseGeometry
from shapely import Point, multipoints
from dataclasses import dataclass
import pandas as pd
from enum import Enum
from scipy.interpolate import griddata

class InterpolationType(Enum):
    INTERP = "interp"
    NEAREST = "nearest"


@dataclass
class WindVector:
    coord: np.ndarray
    wind:  np.ndarray

    @property
    def x(self) -> float:
        return self.coord[0]
    
    @property
    def y(self) -> float:
        return self.coord[1]
    
    @property
    def u(self) -> float:
        return self.wind[0]
    
    @property
    def v(self) -> float:
        return self.wind[1]
    
    @property
    def norm(self) -> float:
        return np.linalg.norm(self.wind)
    
    @property
    def angle(self) -> float:
        return np.arctan2(self.v, self.u)

class WindFunction:
    def __init__(self, func:Callable, domain:BaseGeometry=None) -> None:
        self._func:     Callable = func
        self._domain:   BaseGeometry = domain

    def __call__(self, x:float, y:float, *args: Any, **kwds: Any) -> WindVector:
        """returns a vector f(x,y) = (u(x,y), v(x,y)) that describes the wind at the given point."""
        assert(self._func is not None), "Function not defined"

        if self._domain is not None:
            assert(self._domain.contains(Point(x, y))), "Point is out of domain"

        return WindVector((x, y), self._func(x, y, *args, **kwds))

### THE IDEA WOULD BE TO PASS WINDDATABE TO WINDFUNCTION AS A CALLABLE
### THE WINDFUNCTION CLASS INCLUDES DOMAIN CHECKING


### WINDDATABASE CLASS RECEIVES AN ARRAY OF WIND MEASUREMENT AND IS USED TO QUERY WIND VECTORS AT A SPECIFIC POINT

class WindArray:
    """
    A set of wind vectors, that might be randomly placed in the environment.
    """
    def __init__(self, src:np.ndarray | pd.DataFrame | list, domain:BaseGeometry=None) -> None:
        self._array:np.ndarray = self._parse_dataframe(src)
        self._domain = self._parse_domain(domain)

    def __call__(self, x:float, y:float, interpolation:InterpolationType=InterpolationType.INTERP, *args: Any, **kwds: Any) -> WindVector:
        
        ### FOR INTERPOLATION, USE SCIPY GRIDDATA
        
        if interpolation == InterpolationType.INTERP:
            return self._call_interp(x, y)
        elif interpolation == InterpolationType.NEAREST:
            return self._call_nearest(x, y)
        else:
            raise ValueError(f"Interpolation type {interpolation} not supported")

    def _call_interp(self, x:float, y:float) -> WindVector:
        pass

    def _call_nearest(self, x:float, y:float) -> WindVector:
        pass

    def _compute_domain_from_array(self) -> BaseGeometry:
        self._domain = multipoints(self._array[:, :2]).convex_hull

    def _parse_dataframe(self, src:np.ndarray | pd.DataFrame | list) -> np.ndarray:
        """
        Objective is to end up with a numpy array of dimensions Nx4, where N is the number of wind vectors, and the columns are x, y, u, v.
        """
        if isinstance(src, pd.DataFrame):
            return src.to_numpy()
        
        if isinstance(src, list):
            return np.array(src)
        
        if isinstance(src, np.ndarray):
            return src

    def _parse_domain(self, domain:BaseGeometry) -> BaseGeometry:
        if domain is not None:
            assert(domain.is_valid), "Domain is not valid"
        else:
            domain = self._compute_domain_from_array(self._array)
        
        self._domain: BaseGeometry = domain

    def min(self) -> WindVector:
        pass

    def max(self) -> WindVector:
        pass

    @property
    def domain(self) -> BaseGeometry:
        return self._domain

### WindArrayTimeSeries class stores timestamped map of wind vectors
class WindArrayTimeSeries:
    def __init__(self, src: Any, *args: Any, **kwds: Any) -> None:
        pass

    def __call__(self, time:float, east:float, north:float, *args: Any, **kwds: Any) -> WindVector:
        pass

    def append(self, time:float, wind:WindArray) -> None:
        pass

class WindMap:
    def __init__(self, src: Any, *args: Any, **kwds: Any) -> None:
        pass

    def __call__(self, east:float, north:float, *args: Any, **kwds: Any) -> WindVector:
        pass


if __name__ == "__main__":
    f = ArtificialWindFunction(lambda x, y: np.array([y**2-x, -y-x**2]))
    print(f(x=2, y=5))