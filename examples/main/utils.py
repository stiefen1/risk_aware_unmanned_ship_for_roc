from submodules.ship_in_transit_simulator.models import EnvironmentConfiguration, ShipConfiguration, MachineryModeParams, \
                                                        MachineryMode, MachinerySystemConfiguration, SimulationConfiguration, \
                                                        ShipModel, ThrottleControllerGains, EngineThrottleFromSpeedSetPoint, \
                                                        HeadingControllerGains, LosParameters, HeadingByRouteController, \
                                                        StaticObstacle, MachineryModes, SpecificFuelConsumptionWartila6L26, \
                                                        SpecificFuelConsumptionBaudouin6M26Dot3

import numpy as np

def init_case_study(caseStudy_json: dict, route_filename:str, integration_step: float, simulation_time: float) -> tuple[ShipModel, EngineThrottleFromSpeedSetPoint, HeadingByRouteController]:
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
    
    ### Environment
    envConfig:          EnvironmentConfiguration    = EnvironmentConfiguration(**caseStudy_json['env']) 

    simConfig:      SimulationConfiguration = SimulationConfiguration(integration_step=integration_step,
                                                                  simulation_time=simulation_time,
                                                                  **caseStudy_json['initial_pose'])  
    
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
                                                                    time_step=integration_step,
                                                                    initial_shaft_speed_integral_error=0
                                                                    )
    
    # Heading
    headingControllerGains = HeadingControllerGains(**caseStudy_json['heading_controller_gains'])

    # Line of Sight
    losGuidanceParameters = LosParameters(**caseStudy_json['los_guidance_parameters'])

    # Heading by Route
    autoPilot = HeadingByRouteController(
        route_name=route_filename,
        heading_controller_gains=headingControllerGains,
        los_parameters=losGuidanceParameters,
        time_step=integration_step,
        max_rudder_angle=machineryConfig.max_rudder_angle_degrees * np.pi/180 
    )

    return shipModel, throttleController, autoPilot