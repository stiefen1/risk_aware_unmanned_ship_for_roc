import json, os, numpy as np, time, matplotlib.pyplot as plt, pandas as pd
from seacharts.enc import ENC
from submodules.ship_in_transit_simulator.models import EnvironmentConfiguration, ShipConfiguration, MachineryModeParams, \
                                                        MachineryMode, MachinerySystemConfiguration, SimulationConfiguration, \
                                                        ShipModel, ThrottleControllerGains, EngineThrottleFromSpeedSetPoint, \
                                                        HeadingControllerGains, LosParameters, HeadingByRouteController, \
                                                        StaticObstacle, MachineryModes, SpecificFuelConsumptionWartila6L26, \
                                                        SpecificFuelConsumptionBaudouin6M26Dot3

from submodules.PathPlanning.Search_based_Planning.Search_2D.Astar import AStar
from submodules.PathPlanning.Search_based_Planning.Search_2D.Dijkstra import Dijkstra

from submodules.PathPlanning.Search_based_Planning.Search_2D import plotting
from submodules.PathPlanning.Sampling_based_Planning.rrt_2D.rrt import Rrt

from src.utils import load_obstacles, heading_sim_to_enc
from src.map import DiscreteMap
from src.wind import WindArray, WindFunction, WindVector


def main():
    # Paths
    fileDirName:        str = os.path.dirname(__file__)
    configDirName:      str = os.path.join(fileDirName, "config")
    routeFilePath:      str = os.path.join(configDirName, "route.txt")
    obstacleFilePath:   str = os.path.join(configDirName, 'obstacles.txt')
    dataDirName:        str = os.path.join("data")
    vesselsFilePath:    str = os.path.join(dataDirName, 'vessels.csv')

    ### Setup ENC
    size = 9000, 5062
    center = 44300, 6956450
    left_down_corner = (center[0] - size[0]/2, center[1] - size[1]/2)
    right_up_corner = (center[0] + size[0]/2, center[1] + size[1]/2)

    print("Loading ENC...")
    enc = ENC()

    print("Discretizing map...")
    discreteMap:    DiscreteMap = DiscreteMap(geometry=enc.seabed[1].geometry, dx=100, dy=100)
    
    # start = (42800, 6958733)
    # goal = (44300, 6957433)

    start = (42800  ,   6958733)
    goal =  (48310  ,   6955500)

    print(f"START:\tObstacle free: {discreteMap.is_obstacle_free(*start)}, within bounds: {discreteMap.is_within_bounds(*start)}")
    print(f"GOAL:\tObstacle free: {discreteMap.is_obstacle_free(*goal)}, within bounds: {discreteMap.is_within_bounds(*goal)}")

    startDiscrete = discreteMap.world_to_map(*start)
    goalDiscrete = discreteMap.world_to_map(*goal)

    print("Running A*...")
    astar = AStar(startDiscrete, goalDiscrete, "2-norm", discreteMap)
    path, visited = astar.searching()

    # print("Running Dijkstra...")
    # dijkstra = Dijkstra(startDiscrete, goalDiscrete, "0-norm", discreteMap)
    # path, visited = dijkstra.searching()

    # print(path)

    with open(routeFilePath, "w") as f:
        for i, discreteNode in enumerate(path):
            if(i%2 == 0 and i>1):
                node = discreteMap.map_to_world(*discreteNode)
                f.write(f"{int(node[0])}\t{int(node[1])}\n")

    # plot.animation(path, visited, "A*")

    # Load the configuration
    with open(os.path.join(configDirName, "case_study.json")) as f:
        caseStudy_json = json.load(f)

    ### Environment
    envConfig:          EnvironmentConfiguration    = EnvironmentConfiguration(**caseStudy_json['env']) 
    obstacles_list:     list[StaticObstacle]        = load_obstacles(obstacleFilePath)
    
    caseStudy_json['initial_pose']['initial_north_position_m'] = int(goal[0])
    caseStudy_json['initial_pose']['initial_east_position_m'] = int(goal[1])

    ### Simulation
    timeStep:       float                   = 0.5
    simDuration:    float                   = 3600.
    simConfig:      SimulationConfiguration = SimulationConfiguration(integration_step=timeStep,
                                                                  simulation_time=simDuration,
                                                                  **caseStudy_json['initial_pose'])  

    ### Ship
    shipConfig: ShipConfiguration = ShipConfiguration(**caseStudy_json['ship_config']) 

    # Modes of Operation                                     
    mecModeParams:  MachineryModeParams  = MachineryModeParams(**caseStudy_json['machinery_mode_params'])                       
    mecMode:        MachineryMode        = MachineryMode(params=mecModeParams)
    msoModes:       MachineryModes       = MachineryModes([mecMode])

    # Fuel Consumption
    fuelSpecME: SpecificFuelConsumptionWartila6L26      = SpecificFuelConsumptionWartila6L26()
    fuelSpecDG: SpecificFuelConsumptionBaudouin6M26Dot3 = SpecificFuelConsumptionBaudouin6M26Dot3()

    # Machinery Configuration
    machineryConfig: MachinerySystemConfiguration = MachinerySystemConfiguration(machinery_modes=msoModes,
                                                   specific_fuel_consumption_coefficients_me=fuelSpecME.fuel_consumption_coefficients(),
                                                   specific_fuel_consumption_coefficients_dg=fuelSpecDG.fuel_consumption_coefficients(),
                                                   **caseStudy_json['machinery_system_config'])    
    
    ### Ship Model
    shipModel: ShipModel = ShipModel(ship_config=shipConfig,
                          machinery_config=machineryConfig,
                          environment_config=envConfig,
                          simulation_config=simConfig,
                          **caseStudy_json['initial_ship_states']) 

    ### Controllers
    # Throttle
    throttleControllerGains:    ThrottleControllerGains         = ThrottleControllerGains(**caseStudy_json['throttle_controller_gains'])
    throttleController:         EngineThrottleFromSpeedSetPoint = EngineThrottleFromSpeedSetPoint(
                                                                    gains=throttleControllerGains,
                                                                    max_shaft_speed=shipModel.ship_machinery_model.shaft_speed_max,
                                                                    time_step=timeStep,
                                                                    initial_shaft_speed_integral_error=0
                                                                    )
    
    # Heading
    headingControllerGains = HeadingControllerGains(**caseStudy_json['heading_controller_gains'])

    # Line of Sight
    losGuidanceParameters = LosParameters(**caseStudy_json['los_guidance_parameters'])

    # Heading by Route
    autoPilot = HeadingByRouteController(
        route_name=routeFilePath,
        heading_controller_gains=headingControllerGains,
        los_parameters=losGuidanceParameters,
        time_step=timeStep,
        max_rudder_angle=machineryConfig.max_rudder_angle_degrees * np.pi/180 
    )


    # TODO: Add obstacles to ENC
    for obstacle in obstacles_list:
        enc.display.draw_circle((int(obstacle.n), int(obstacle.e)), obstacle.r, "black")

    # TODO: Add waypoints to ENC
    with open(routeFilePath, "r") as f:
        route = f.readlines()
        waypoints = []
        for line in route:
            waypoint = line.split()
            waypoints.append((int(waypoint[0]), int(waypoint[1])))
            enc.display.draw_circle((int(waypoint[0]), int(waypoint[1])), 20, "red")

    # TODO: Add wind direction and intensity to ENC
    # wind:           np.ndarray          = np.zeros(shape=(windArraySize[0], windArraySize[1], 2), dtype=np.float32)
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
    timeSpeedFactor:        float = 100.0 # Meaning 1 second in real time is 10 second in simulation time
    updateVizRate:          float = .5 # Update ENC every second in real time
    updateLogRate:          float = 5 # Print out simulation data every 30 seconds
    updateRouteRate:        float = 10 # Update route every 10 seconds
    timeToProcess:          float = 0.0 # Time to process the simulation data
    currentWaypointIndex:   int = 0 # Current waypoint

    time_since_last_ship_drawing = 30
    desired_forward_speed_meters_per_second = 3
    integrator_term = []
    heading_list = []
    times = []




    # Main simulation loop
    end, start = 0, 0
    if updateLogRate < timeStep:
        print(f"Update log rate is less than time step -> setting it to {timeStep:.2f}")
        updateLogRate = timeStep
    if updateVizRate < timeStep:
        print(f"Update visualization rate is less than time step -> setting it to {timeStep:.2f}")
        updateVizRate = timeStep

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
        if (shipModel.int.time / timeSpeedFactor) % updateLogRate == 0:
            print(f"WP{currentWaypointIndex}, Time: {shipModel.int.time:.1f}, North: {north_position:.1f}, East: {east_position:.1f}, Heading: {heading:.2f}, Speed: {speed:.1f}, Wind: {wind.norm:.1f} m/s, {shipModel.wind_dir:.1f} deg")

        # Update ENC every second in real time
        if (shipModel.int.time / timeSpeedFactor) % updateVizRate == 0:
            # Update vessels pose on ENC
            enc.display.features.vessels_to_file([
                (1, int(north_position), int(east_position), heading_sim_to_enc(heading), "orange")
            ])
            
            enc.display.features.update_vessels()
            enc.display.redraw_plot()
            enc.display.update_plot()

        # Update route on ENC
        if (shipModel.int.time % updateRouteRate) == 0:
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
        currentWaypointIndex, nextWaypointIndex = autoPilot.navigate.next_wpt(currentWaypointIndex, north_position, east_position)
        if currentWaypointIndex == nextWaypointIndex:
            print(f"Ship has reached final waypoint {currentWaypointIndex}")
            enc.display.figure.waitforbuttonpress()
            break


        # Make a drawing of the ship from above every 20 second
        if time_since_last_ship_drawing > 30:
            shipModel.ship_snap_shot()
            time_since_last_ship_drawing = 0
        time_since_last_ship_drawing += shipModel.int.dt

        # Time to process the simulation data
        end = time.time()
        timeToProcess = end - start
        
        # Sleep to make the simulation run in real time
        if timeToProcess < timeStep / timeSpeedFactor:
            time.sleep(timeStep / timeSpeedFactor - timeToProcess)

        


    # Store the simulation results in a pandas dataframe
    results = pd.DataFrame().from_dict(shipModel.simulation_results)

    # Example on how a map-view can be generated
    map_fig, map_ax = plt.subplots()
    map_ax.plot(results['east position [m]'], results['north position [m]'])
    map_ax.scatter(autoPilot.navigate.east, autoPilot.navigate.north, marker='x', color='green')  # Plot the waypoints
    for x, y in zip(shipModel.ship_drawings[1], shipModel.ship_drawings[0]):
        map_ax.plot(x, y, color='black')
    for obstacle in obstacles_list:
        obstacle.plot_obst(ax=map_ax)

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