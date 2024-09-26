# TODO: represent wind in the simulation.


from typing import Any, Callable
import numpy as np
from shapely.geometry.base import BaseGeometry
from shapely import Point, multipoints
from dataclasses import dataclass
import pandas as pd
from scipy.interpolate import griddata
import matplotlib.pyplot as plt


# PROBABLY WE SHOULD CREATE A MEASUREMENT STATION CLASS THAT PROVIDES US MEASUREMENTS OF WIND AT A SPECIFIC POINT
# FROM THAT WE CREATE AN ARRAY OF MEASUREMENT STATIONS THAT PROVIDES US WIND VECTORS AT DIFFERENT POINTS

# I think for the time being we can just use the current classes to use a set of measurement randomly placed in the xy plane
# and then interpolate the wind vectors at a grid of points.
# The grid is created at each time step when we consider wind may have changed.
# So for instance we update our wind measurement every 5 minutes by sending a request to the measurement stations
# we consider the last measurement as current measurment.

@dataclass
class WindVector:
    coord: np.ndarray
    wind:  np.ndarray
        
    def to_numpy(self) -> np.ndarray:
        return np.array([self.x, self.y, self.u, self.v])

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
    
    @property
    def start(self) -> np.ndarray:
        return self.coord
    
    @property
    def end(self) -> np.ndarray:
        return self.coord + self.wind

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

class WindArray:
    """
    A set of wind vectors, that might be randomly placed in the environment.
    """
    def __init__(self, src:np.ndarray | pd.DataFrame | list | WindFunction, resolution:tuple[float, float]=(100, 100), start:tuple[float, float]=(0., 0.), stop:tuple[float, float]=(1., 1.), domain:BaseGeometry=None) -> None:
        self._resolution = resolution
        self._array:np.ndarray = self._parse_dataframe(src, resolution=resolution, start=start, stop=stop)
        self._domain:BaseGeometry = self._parse_domain(domain)

    def __call__(self, x:float, y:float, method='nearest', *args: Any, **kwds: Any) -> WindVector:
        return self._get_wind_by_interp(x, y, method=method)
    
    def __getitem__(self, index: int) -> WindVector:
        if isinstance(index, int):
            return WindVector(self._array[index, :2], self._array[index, 2:])
        else:
            raise ValueError("Index must be an integer or a tuple of integers")

    def _get_wind_by_interp(self, x:float, y:float, method='nearest') -> WindVector:
        """Return the interpolated wind vector at the given point."""
        u = griddata(self._array[:, :2], self._array[:, 2], (x, y), method=method)
        v = griddata(self._array[:, :2], self._array[:, 3], (x, y), method=method)
        return WindVector(np.array([x, y]), np.array([u, v]))

    def _get_convex_hull_of_array(self) -> BaseGeometry:
        return multipoints(self._array[:, :2]).convex_hull

    def _parse_dataframe(self, src:np.ndarray | pd.DataFrame | list | WindFunction, resolution:tuple[float, float]=None, start:tuple[float, float]=None, stop:tuple[float, float]=None) -> np.ndarray:
        """
        Objective is to end up with a numpy array of dimensions Nx4, where N is the number of wind vectors, and the columns are x, y, u, v.
        """
        if isinstance(src, pd.DataFrame):
            array = src.to_numpy()
        
        elif isinstance(src, list):
            array = np.array(src)
        
        elif isinstance(src, np.ndarray):
            array = src.copy()
            assert(array.shape[1] == 4), "Array must have 4 columns: x, y, u, v"

        elif isinstance(src, WindFunction):
            assert resolution is not None, "Resolution must be provided"
            assert start is not None, "Start must be provided"
            assert stop is not None, "Stop must be provided"

            x = np.linspace(start[0], stop[0], resolution[0])
            y = np.linspace(start[1], stop[1], resolution[1])
            X, Y = np.meshgrid(x, y)
            U = np.zeros_like(X)
            V = np.zeros_like(Y)
            for i in range(X.shape[0]):
                for j in range(X.shape[1]):
                    wind = src(X[i, j], Y[i, j])
                    U[i, j] = wind.u
                    V[i, j] = wind.v
            array = np.hstack((X.flatten()[:, np.newaxis], Y.flatten()[:, np.newaxis], U.flatten()[:, np.newaxis], V.flatten()[:, np.newaxis]))
            print(array.shape)

        else:
            raise ValueError(f"Source type {type(src)} not supported")
        
        return self._clean_array(array)
    
    def _clean_array(self, array:np.ndarray) -> np.ndarray:
        return array[~np.isnan(array).any(axis=1)]

    def _parse_domain(self, domain:BaseGeometry) -> BaseGeometry:
        if domain is not None:
            assert(domain.is_valid), "Domain is not valid"
        else:
            domain = self._get_convex_hull_of_array()
        
        return domain

    def min(self) -> WindVector:
        pass

    def max(self) -> WindVector:
        pass

    def plot(self, domain=True, *args: Any, **kwargs: Any) -> None:
        ax, fig = plt.subplots()
        fig.quiver(self._array[:, 0], self._array[:, 1], self._array[:, 2], self._array[:, 3])

        if domain and self._domain is not None:
            fig.plot(self._domain.boundary.xy[0], self._domain.boundary.xy[1], 'k-')

        return ax, fig
    
    @property
    def shape(self) -> tuple[int, int]:
        return self._array.shape

    @property
    def domain(self) -> BaseGeometry:
        return self._domain
    
    @property
    def array(self) -> np.ndarray:
        return self._array  
    

if __name__ == "__main__":
    print("Creating a WindFunction object that generates wind vectors based on a function")
    
    N = 10
    measurement_station_xy = np.random.rand(N, 2)
    measurement_at_xy = np.zeros((N, 2))
    for i in range(measurement_station_xy.shape[0]):
        # Here I'm creating fake points from a function, but I could read those data from an API
        f = WindFunction(lambda x, y: np.array([y**2-x, -y-x**2]))
        measurement_at_xy[i] = f(measurement_station_xy[i, 0], measurement_station_xy[i, 1]).wind

    w = WindArray(np.hstack((measurement_station_xy, measurement_at_xy)))

    print("Creating a grid of points to interpolate wind vectors")

    grid = np.mgrid[0:1:15j, 0:1:15j]
    xyInterp = np.zeros((grid.shape[1], grid.shape[2], 2))
    uvInterp = np.zeros((grid.shape[1], grid.shape[2], 2))
    for i in range(grid.shape[1]):
        for j in range(grid.shape[2]):
            xyInterp[i, j] = np.array([grid[0, i, j], grid[1, i, j]])
            uvInterp[i, j] = w(grid[0, i, j], grid[1, i, j], method='linear').wind

    print("Instantiating a new WindArray object to interpolate wind vectors")

    wGrid = WindArray(np.hstack((xyInterp.reshape(-1, 2), uvInterp.reshape(-1, 2))))

    print("Plotting the original wind vectors and the interpolated wind vectors")

    ax, fig = wGrid.plot()
    fig.set_title("Interpolated wind vectors")
    w.plot("Original wind vectors")
    plt.show()



##############################################################################################################
################################################# OLD ########################################################
##############################################################################################################


@dataclass
class TimeStampedWindVector:
    time: float
    wind: WindVector

    def to_numpy(self):
        return np.array([self.time, self.wind.x, self.wind.y, self.wind.u, self.wind.v])

    
class ArrayOfTimeStampedWind:
    def __init__(self, array:list[TimeStampedWindVector]) -> None:
        self._array: np.ndarray = np.ndarray((0, 5))
        for wind_vector_at_t in array:
            assert(isinstance(wind_vector_at_t, TimeStampedWindVector)), "Array must contain TimeStampedWindVector objects"
            self.append(wind_vector_at_t)

    def append(self, time_stamped_wind:TimeStampedWindVector) -> None:
        self._array = np.append(time_stamped_wind.to_numpy()[np.newaxis], self._array, axis=0)

    @property
    def N(self) -> int:
        return self._array.shape[0]
    
    
    ### ADD METHOD TO GET WIND AT A SPECIFIC TIME AND POSITION (INTERPOLATED)


    ### WindArrayTimeSeries class stores timestamped map of wind vectors
class WindArrayTimeSeries:
    """
    
    LES DATA QU'ON VA RECEVOIR PROVIENDRONT DE DIFFERENTS CAPTEURS FIXES DANS L'ENVIRONNEMENT
    DESQUELLS ON VA RECUPERER DES MESURES DE VENT A DES INSTANTS PROBABLEMENT DIFFERENTS
    DONC LES POSITIONS NE CHANGENT PAS, MAIS LES INSTANTS AUQUELS ON A MESURE LE VENT SONT DIFFERENTS
    
    """


    def __init__(self, time:list, wind:list[WindArray], *args: Any, **kwds: Any) -> None:
        assert(len(time) == len(wind)), "Time and wind must have the same length"
        
        # Convert time to numpy array and check if it is sorted
        time_np = np.array(time)
        assert(self._is_numpy_sorted(time_np)), "Time must be sorted"
        # self._array: np.ndarray = self._build_time_series_array(np.array(time), wind)
        self._time: np.ndarray      = time_np
        self._wind: list[WindArray] = wind

    def _get_wind_at_xyt(self, x:float, y:float, time:float, method='nearest') -> WindVector:
        """Return the interpolated wind vector at the given (x, y, t) point."""
        # Get previous and next wind vectors at the given point
        previous_wind:          WindArray   = self._get_previous_wind(time)
        next_wind:              WindArray   = self._get_next_wind(time)
        previous_wind_at_xy:    WindVector  = previous_wind(x, y)
        next_wind_at_xy:        WindVector  = next_wind(x, y)
        
        # Compute the interpolated wind vector at the given point
        dt = time - self._get_previous_timestamp(time)
        Dt = self._get_next_timestamp(time) - self._get_previous_timestamp(time)
        assert(Dt != 0), "Time difference is zero"
        alpha:float     = dt/Dt
        uv:np.ndarray   = previous_wind_at_xy.wind + alpha * (next_wind_at_xy.wind - previous_wind_at_xy.wind)

        return WindVector(np.array([x, y]), uv)
    
    def _is_numpy_sorted(self, array:np.ndarray) -> bool:
        """Check if a numpy array is sorted in ascending order."""
        return np.all(array[:-1] <= array[1:])
    
    def __call__(self, x:float, y:float, time:float, method='nearest', *args: Any, **kwds: Any) -> WindVector:
        return self._get_wind_at_xyt(x, y, time, method=method)

    def _get_previous_timestamp(self, time:float) -> float:
        """Get previous timestamp of the time series."""
        return self._time[self._get_previous_timestamp_index(time)]
    
    def _get_next_timestamp(self, time:float) -> float:
        """Get next timestamp of the time series."""
        return self._time[np.where(self._time >= time)[0][0]]
    
    def _get_previous_wind(self, time:float) -> WindArray:
        """Get wind array at the previous timestamp."""
        return self._wind[self._get_previous_timestamp_index(time)]
    
    def _get_next_wind(self, time:float) -> WindArray:
        """Get wind array at the next timestamp."""
        return self._wind[self._get_next_timestamp_index(time)]
    
    def _get_previous_timestamp_index(self, time:float) -> int:
        """Get index of the previous timestamp of the time series."""
        return np.where(self._time <= time)[0][-1]
    
    def _get_next_timestamp_index(self, time:float) -> int:
        """Get index of the next timestamp of the time series."""
        return np.where(self._time >= time)[0][0]
    