import json, os, numpy as np, time
from seacharts.enc import ENC
from submodules.ship_in_transit_simulator.models import EnvironmentConfiguration, ShipConfiguration, MachineryModeParams, \
                                                        MachineryMode, MachinerySystemConfiguration, SimulationConfiguration, \
                                                        ShipModel, ThrottleControllerGains, EngineThrottleFromSpeedSetPoint, \
                                                        HeadingControllerGains, LosParameters, HeadingByRouteController, \
                                                        StaticObstacle, MachineryModes, SpecificFuelConsumptionWartila6L26, \
                                                        SpecificFuelConsumptionBaudouin6M26Dot3

from src.utils import load_obstacles, heading_sim_to_enc

def eval_wind(i: int, j: int) -> np.array:
    return np.array([np.cos(np.pi*i/5000), -np.sin(2*np.pi*j/500)])

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
    enc = ENC()

    # Load the configuration
    with open(os.path.join(configDirName, "case_study.json")) as f:
        caseStudy_json = json.load(f)

    ### Environment
    envConfig:          EnvironmentConfiguration    = EnvironmentConfiguration(**caseStudy_json['env']) 
    obstacles_list:     list[StaticObstacle]        = load_obstacles(obstacleFilePath)
    
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
                                                                    initial_shaft_speed_integral_error=114
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


    desired_forward_speed_meters_per_second = 5
    integrator_term = []
    times = []

    # Simulation parameters
    timeSpeedFactor:        float = 100.0 # Meaning 1 second in real time is 10 second in simulation time
    updateVizRate:          float = .5 # Update ENC every second in real time
    updateLogRate:          float = 5 # Print out simulation data every 30 seconds
    updateRouteRate:        float = 10 # Update route every 10 seconds
    timeToProcess:          float = 0.0 # Time to process the simulation data
    currentWaypointIndex:   int = 0 # Current waypoint

    # TODO: Add obstacles to ENC
    print(obstacles_list)
    for obstacle in obstacles_list:
        print((int(obstacle.e), int(obstacle.n)), obstacle.r)
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
    windResolution: float               = 1000
    windArraySize:  tuple[float, float] = size[0] // windResolution, size[1] // windResolution
    wind:           np.ndarray          = np.zeros(shape=(windArraySize[0], windArraySize[1], 2), dtype=np.float32)
    intensityToViz: float               = 150
    for i in range(windArraySize[0]):
        for j in range(windArraySize[1]):
            x:  float   = i*windResolution + center[0]
            y:  float   = j*windResolution + center[1]
            wind[i, j] = eval_wind(x, y)
            start = np.array([x - size[0]/2, y - size[1]/2])
            stop = start + wind[i, j] * intensityToViz
            enc.display.draw_arrow(start=(start[0], start[1]), end=(stop[0], stop[1]), color="orange")

    # TODO: Add route to ENC
    enc.display.draw_line(waypoints, 'red', edge_style='--', width=.1)

    # Start ENC
    enc.display.start()
    
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
        north_position: float   = shipModel.north
        east_position:  float   = shipModel.east
        heading:        float   = shipModel.yaw_angle
        speed:          float   = shipModel.forward_speed

        # Print out simulation data
        if (shipModel.int.time / timeSpeedFactor) % updateLogRate == 0:
            print(f"Time: {shipModel.int.time:.1f}, North: {north_position:.1f}, East: {east_position:.1f}, Heading: {heading:.2f}, Speed: {speed:.1f}")

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
            enc.display.draw_circle((int(north_position), int(east_position)), 10, "red")


        # Find appropriate rudder angle and engine throttle
        rudder_angle = autoPilot.rudder_angle_from_route(
            north_position=north_position,
            east_position=east_position,
            heading=heading
        )
        throttle = throttleController.throttle(
            speed_set_point=desired_forward_speed_meters_per_second,
            measured_speed=speed,
            measured_shaft_speed=speed
        )

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

        # Time to process the simulation data
        end = time.time()
        timeToProcess = end - start
        
        # Sleep to make the simulation run in real time
        if timeToProcess < timeStep / timeSpeedFactor:
            time.sleep(timeStep / timeSpeedFactor - timeToProcess)


if __name__ == "__main__":
    # TODO: Setup Logging

    main()