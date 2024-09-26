import json, os, numpy as np, time, matplotlib.pyplot as plt, pandas as pd
from seacharts.enc import ENC
from submodules.ship_in_transit_simulator.models import ShipModel, EngineThrottleFromSpeedSetPoint, \
                                                        HeadingByRouteController, StaticObstacle
                                                        

from submodules.PathPlanning.Search_based_Planning.Search_2D.Astar import AStar
from submodules.PathPlanning.Search_based_Planning.Search_2D.Dijkstra import Dijkstra

from submodules.PathPlanning.Search_based_Planning.Search_2D import plotting
from submodules.PathPlanning.Sampling_based_Planning.rrt_2D.rrt import Rrt

from src.utils import load_obstacles, heading_sim_to_enc
from src.map import DiscreteMap
from src.wind import WindArray, WindFunction, WindVector
from examples.main.utils import init_case_study
from shapely.geometry import Polygon, MultiPolygon


def main():
    # Paths
    fileDirName:        str = os.path.dirname(__file__)
    configDirName:      str = os.path.join(fileDirName, "config")
    routeFilePath:      str = os.path.join(configDirName, "route.txt")
    dataDirName:        str = os.path.join("data")

    ### Setup ENC
    size = 9000, 5062
    center = 44300, 6956450
    seabed_depth_for_path_planning = 5
    minkowski_radius = 100
    left_down_corner = (center[0] - size[0]/2, center[1] - size[1]/2)
    right_up_corner = (center[0] + size[0]/2, center[1] + size[1]/2)

    print("Loading ENC...")
    enc = ENC()
    assert seabed_depth_for_path_planning in enc.seabed.keys(), f"Seabed depth {seabed_depth_for_path_planning} not found in ENC, try one of these: {enc.seabed.keys()}"

    # PLOT ALL SEABEDS
    # for i, seabed in enumerate(enc.seabed.values()):
    #     multi: MultiPolygon = seabed.geometry
    #     plt.title(f"Seabed {seabed.depth} m, index: {i}")
    #     for poly in multi.geoms:
    #         x, y = Polygon(poly).exterior.xy
    #         plt.plot(x, y)
    #     plt.show()

    multi: MultiPolygon = enc.seabed[seabed_depth_for_path_planning].geometry.buffer(-minkowski_radius)
    try:
        for poly in multi.geoms:
            x, y = Polygon(poly).exterior.xy
            plt.plot(x, y)
    except:
        # In case all the multi polygons are merged into one
        x, y = Polygon(multi).exterior.xy
        plt.plot(x, y)
    plt.title(f"Seabed {seabed_depth_for_path_planning} m")
    plt.show()        

    print("Discretizing map...")
    discrete_map:   DiscreteMap = DiscreteMap(geometry=enc.seabed[seabed_depth_for_path_planning].geometry.buffer(-minkowski_radius), dx=50, dy=50)

    start = (42800  ,   6958733)
    goal =  (48310  ,   6955500)

    print(f"START:\tObstacle free: {discrete_map.is_obstacle_free(*start)}, within bounds: {discrete_map.is_within_bounds(*start)}")
    print(f"GOAL:\tObstacle free: {discrete_map.is_obstacle_free(*goal)}, within bounds: {discrete_map.is_within_bounds(*goal)}")

    start_discrete = discrete_map.world_to_map(*start)
    goal_discrete = discrete_map.world_to_map(*goal)

    print("Running A*...")
    astar = AStar(start_discrete, goal_discrete, "2-norm", discrete_map)
    plot = plotting.Plotting(start_discrete, goal_discrete, environment=discrete_map)
    path, visited = astar.searching()
    plot.animation(path, visited, "A*")

    # print("Running Dijkstra...")
    # dijkstra = Dijkstra(start_discrete, goal_discrete, "0-norm", discrete_map)
    # path, visited = dijkstra.searching()

    with open(routeFilePath, "w") as f:
        for i, discreteNode in enumerate(path):
            if(i%10 == 0 and i>1):
                node = discrete_map.map_to_world(*discreteNode)
                f.write(f"{int(node[0])}\t{int(node[1])}\n")

    # Load the configuration
    with open(os.path.join(configDirName, "case_study.json")) as f:
        caseStudy_json = json.load(f)

    
    caseStudy_json['initial_pose']['initial_north_position_m'] = int(goal[0])
    caseStudy_json['initial_pose']['initial_east_position_m'] = int(goal[1])

    ### Simulation
    time_step:       float  = 0.5
    sim_duration:    float  = 3600.
    

    out = init_case_study(caseStudy_json, routeFilePath, time_step, sim_duration)
    shipModel:          ShipModel                       = out[0]
    throttleController: EngineThrottleFromSpeedSetPoint = out[1]
    autoPilot:          HeadingByRouteController        = out[2]


    # TODO: Add waypoints to ENC
    with open(routeFilePath, "r") as f:
        route = f.readlines()
        waypoints = []
        for line in route:
            waypoint = line.split()
            waypoints.append((int(waypoint[0]), int(waypoint[1])))
            enc.display.draw_circle((int(waypoint[0]), int(waypoint[1])), 20, "red")

    # TODO: Add wind direction and intensity to ENC
    intensityToViz: float               = 10
    fakeWindFunction: WindFunction      = WindFunction(lambda x, y: - 8. * np.array([((y-center[1]-size[1])/size[1])**2, (-(x-center[0]-size[0])/size[0])**4]))
    wind = WindArray(fakeWindFunction, resolution=(10, 10), start=left_down_corner, stop=right_up_corner)

    for i in range(wind.shape[0]):
        enc.display.draw_arrow(start=wind[i].start, end=wind[i].start + (wind[i].end - wind[i].start) * intensityToViz, color="orange")

    # TODO: Add route to ENC
    enc.display.draw_line(waypoints, 'red', edge_style='--', width=.1)

    # Start ENC
    enc.display.start()

    # Simulation parameters
    time_speed_factor:  float   = 100.0 # Meaning 1 second in real time is 10 second in simulation time
    update_viz_rate:    float   = .5    # Update ENC every second in real time
    update_log_rate:    float   = 5     # Print out simulation data every 30 seconds
    update_route_rate:  float   = 10    # Update route every 10 seconds
    time_to_process:    float   = 0.0   # Time to process the simulation data
    current_wpt_idx:    int     = 0     # Current waypoint

    time_since_last_ship_drawing = 30
    desired_forward_speed_meters_per_second = 3
    integrator_term = []
    heading_list = []
    times = []

    # Main simulation loop
    end, start = 0, 0
    if update_log_rate < time_step:
        print(f"Update log rate is less than time step -> setting it to {time_step:.2f}")
        update_log_rate = time_step
    if update_viz_rate < time_step:
        print(f"Update visualization rate is less than time step -> setting it to {time_step:.2f}")
        update_viz_rate = time_step

    while shipModel.int.time < shipModel.int.sim_time and shipModel:
        start = time.time()
        # Measure position and speed
        north_position: float       = shipModel.north
        east_position:  float       = shipModel.east
        heading:        float       = shipModel.yaw_angle
        speed:          float       = shipModel.forward_speed
        wind:           WindVector  = fakeWindFunction(north_position, east_position)
        
        # Update wind
        shipModel.wind_dir = 180 * wind.angle / np.pi
        shipModel.wind_speed = wind.norm

        # Print out simulation data
        if (shipModel.int.time / time_speed_factor) % update_log_rate == 0:
            print(f"WP{current_wpt_idx}, Time: {shipModel.int.time:.1f}, North: {north_position:.1f}, East: {east_position:.1f}, Heading: {heading:.2f}, Speed: {speed:.1f}, Wind: {wind.norm:.1f} m/s, {shipModel.wind_dir:.1f} deg")

        # Update ENC every second in real time
        if (shipModel.int.time / time_speed_factor) % update_viz_rate == 0:
            # Update vessels pose on ENC
            enc.display.features.vessels_to_file([
                (1, int(north_position), int(east_position), heading_sim_to_enc(heading), "orange")
            ])
            
            enc.display.features.update_vessels()
            enc.display.redraw_plot()
            enc.display.update_plot()

        # Update route on ENC
        if (shipModel.int.time % update_route_rate) == 0:
            enc.display.draw_circle((int(north_position), int(east_position)), 5, "green")

        rudder_angle = autoPilot.rudder_angle_from_route(
            north_position=north_position,
            east_position=east_position,
            heading=heading
        )

        # ACCELERATION
        throttle = throttleController.throttle(
            speed_set_point=desired_forward_speed_meters_per_second,
            measured_speed=speed,
            measured_shaft_speed=speed
        )

        # TODO: PLOT CONTROLLER / STATE TO DEBUG WEIRD BEHAVIOUR
        heading_list.append(heading)

        # Update and integrate differential equations for current time step
        shipModel.store_simulation_data(throttle)
        shipModel.update_differentials(engine_throttle=throttle, rudder_angle=rudder_angle)
        shipModel.integrate_differentials()

        integrator_term.append(autoPilot.navigate.e_ct_int)
        times.append(shipModel.int.time)

        # Progress time variable to the next time step
        shipModel.int.next_time()

        # TODO: Check if the ship has reached final waypoint
        current_wpt_idx, nextWaypointIndex = autoPilot.navigate.next_wpt(current_wpt_idx, north_position, east_position)
        if current_wpt_idx == nextWaypointIndex:
            print(f"Ship has reached final waypoint {current_wpt_idx}")
            enc.display.figure.waitforbuttonpress()
            break


        # Make a drawing of the ship from above every 20 second
        if time_since_last_ship_drawing > 30:
            shipModel.ship_snap_shot()
            time_since_last_ship_drawing = 0
        time_since_last_ship_drawing += shipModel.int.dt

        # Time to process the simulation data
        end = time.time()
        time_to_process = end - start
        
        # Sleep to make the simulation run in real time
        if time_to_process < time_step / time_speed_factor:
            time.sleep(time_step / time_speed_factor - time_to_process)

        


    # Store the simulation results in a pandas dataframe
    results = pd.DataFrame().from_dict(shipModel.simulation_results)

    # Example on how a map-view can be generated
    map_fig, map_ax = plt.subplots()
    map_ax.plot(results['east position [m]'], results['north position [m]'])
    map_ax.scatter(autoPilot.navigate.east, autoPilot.navigate.north, marker='x', color='green')  # Plot the waypoints
    for x, y in zip(shipModel.ship_drawings[1], shipModel.ship_drawings[0]):
        map_ax.plot(x, y, color='black')

    map_ax.set_aspect('equal')

    # Example on plotting time series
    fuel_ifg, fuel_ax = plt.subplots()
    results.plot(x='time [s]', y='power [kw]', ax=fuel_ax)

    int_fig, int_ax = plt.subplots()
    int_ax.plot(times, integrator_term)

    plt.show()


if __name__ == "__main__":
    # TODO: Setup Logging

    main()