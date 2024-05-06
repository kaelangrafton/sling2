# -*- coding: utf-8 -*-
"""
Created on Sat Jan 14 15:11:02 2023

@author: K. Grafton 

Adpated from E. Botero tut_C172.py
"""

import numpy as np
import matplotlib.pyplot as plt

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units

from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from SUAVE.Plots.Performance import *
from SUAVE.Methods.Performance  import payload_range, V_n_diagram

from SUAVE.Input_Output.OpenVSP import write

from ADRpy import airworthiness as aw
from ADRpy import unitconversions as co
from ADRpy import atmospheres as at

def main(): 
    
    vehicle    = vehicle_setup()
    
#    write(vehicle,'Sling_2')
    
    configs = configs_setup(vehicle)
    
    analyses = base_analysis(vehicle)
    
    configs.finalize()
    analyses.finalize()
    
   
    
   
    mission  = mission_setup(analyses,vehicle)
    
    
    results = mission.evaluate()
    
    # run payload diagram
    config = configs.base
    cruise_segment_tag = "cruise"
    reserves = 0.
    payload_range_results = payload_range(config,mission,cruise_segment_tag,reserves)
    
    plot_mission(results)
    
#    weights = analyses.configs.base.weights
#    breakdown = weights.evaluate()  
    
    loads(vehicle)
    
    altitude = 0 * Units.m
    delta_ISA = 20 * Units.degC
    weight = vehicle.mass_properties.max_takeoff
    V_n_diagram(vehicle,analyses,weight, altitude, delta_ISA)
    print("Complete")

def vehicle_setup(): 
      
    vehicle                                     = SUAVE.Vehicle()
    vehicle.tag                                 = 'Sling_2'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    vehicle.mass_properties.max_takeoff         = 700. * Units.kg
    vehicle.mass_properties.takeoff             = 700. * Units.kg
    vehicle.mass_properties.operating_empty     = 460. * Units.kg #including pilot
    vehicle.mass_properties.max_zero_fuel       = 600. * Units.kg    
    
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.8
    
    vehicle.envelope.limit_loads.positive       = 3.5
    vehicle.envelope.limit_loads.negative       = -1.4
    
    
    vehicle.reference_area                      = 11.845 * Units.m**2       
    vehicle.passengers                          = 2
    
    vehicle.envelope.FAR_part_number            = 23
    vehicle.envelope.category                   = 'normal'
    vehicle.envelope.cruise_mach                = 0.09
    
    vehicle.maximum_lift_coefficient            = 1.3 #assumed
    vehicle.minimum_lift_coefficient            = -1 #assumed

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   
    
    wing                                        = SUAVE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.174/1.14 
    wing.areas.reference                        = 11.845 * Units.m**2 
    wing.spans.projected                        = 9.165  * Units.m
    wing.chords.mean                            = 1.3 * Units.m
    wing.chords.root                            = 1.54 * Units.m
    wing.chords.tip                             = 1.14 * Units.m 
    wing.taper                                  = 1.375
    wing.aspect_ratio                           = 7.04
    wing.twists.root                            = 0.0 * Units.degrees #check
    wing.twists.tip                             = 0.0 * Units.degrees #check
    wing.origin                                 = [[2.* Units.m,0,0.22* Units.m]]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False
    wing.dynamic_pressure_ratio                 = 1.0 
    
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 3. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.225/1.5
    segment.dihedral_outboard             = 5 * Units.degrees
    segment.sweeps.quarter_chord          = -1. * Units.degrees
    wing.append_segment(segment)
    
    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Break'
    segment.percent_span_location         = 0.50
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.35/1.374
    segment.thickness_to_chord            = 0.2/1.35
    segment.dihedral_outboard             = 5 * Units.degrees
    segment.sweeps.quarter_chord          = -1. * Units.degrees
    wing.append_segment(segment)

    segment                               = SUAVE.Components.Wings.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 1.14/1.5

    segment.thickness_to_chord            = 0.12
    segment.dihedral_outboard             = 5 * Units.degrees
    segment.sweeps.quarter_chord          = -1. * Units.degrees
    wing.append_segment(segment)

    wing = segment_properties(wing)        
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------                                
    wing                                        = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 7 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.spans.projected                        = 2.825  * Units.m
    wing.areas.reference                        = 2.22 * Units.m**2
    wing.chords.root                            = 0.92 * Units.m
    wing.chords.tip                             = 0.65 * Units.m
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = (wing.spans.projected**2)/ wing.areas.reference
    wing.twists.root                            = -4. * Units.degrees
    wing.twists.tip                             = -4. * Units.degrees
    wing.origin                                 = [[5.6* Units.m,0,0.75 * Units.m]]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9
    

    
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    wing                                        = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 34. * Units.deg
    wing.thickness_to_chord                     = 0.08
    wing.areas.reference                        = 1.18 * Units.m**2
    wing.spans.projected                        = 1.3  * Units.m
    wing.chords.root                            = 1.3 * Units.m
    wing.chords.tip                             = 0.52 * Units.m
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[5.38* Units.m,0,0.69* Units.m]]
    wing.vertical                               = True 
    wing.symmetric                              = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0
    
    wing = SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.number_coach_seats                 = 2.       
    fuselage.tag                                = 'fuselage'    
    fuselage.differential_pressure              = 0. 
    fuselage.width                              = 1.18  * Units.m  
    fuselage.heights.maximum                    = 2.45  * Units.m    
    fuselage.lengths.total                      = 6.675 * Units.m          
    fuselage.lengths.empennage                  = 1.97 * Units.m  
    fuselage.lengths.cabin                      = 2.7 * Units.m
    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 20.396
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 1.1  * Units.m
    fuselage.heights.at_quarter_length          = 0.9 * Units.m
    fuselage.heights.at_three_quarters_length   = 0.64 * Units.m
    fuselage.heights.at_wing_root_quarter_chord = 1.1 * Units.m
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 1.25 * Units.m
    fuselage.origin                             = [[0.33* Units.m,0,0.33 * Units.m]]
    
    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.
    segment.percent_z_location                  = 0.047
    segment.height                              = 0. * Units.m
    segment.width                               = 0.  * Units.m  
    fuselage.Segments.append(segment)       

    # Segment  
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.047
    segment.percent_z_location                  = 0.033
    segment.height                              = 0.463 * Units.m
    segment.width                               = 0.655  * Units.m  
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.161
    segment.percent_z_location                  = 0.028
    segment.height                              = 0.763 * Units.m
    segment.width                               = 1.  * Units.m  
    fuselage.Segments.append(segment)      
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.21
    segment.percent_z_location                  = 0.028
    segment.height                              = 0.8 * Units.m  
    segment.width                               = 1.1      * Units.m  
    fuselage.Segments.append(segment)   

    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.292	
    segment.percent_z_location                  = 0.05
    segment.height                              = 1. * Units.m      
    segment.width                               = 1.307       * Units.m  
    fuselage.Segments.append(segment)   
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.379
    segment.percent_z_location                  = 0.0585
    segment.height                              = 1.24 * Units.m  
    segment.width                               = 1.423       * Units.m  
    fuselage.Segments.append(segment)         
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.615
    segment.percent_z_location                  = 0.0444
    segment.height                              = 0.8 * Units.m  
    segment.width                               = 0.86 * Units.m  
    fuselage.Segments.append(segment)  
    
    # Segment                                   
    segment                                     = SUAVE.Components.Lofted_Body_Segment.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.748
    segment.percent_z_location                  = 0.0444
    segment.height                              = 0.529 * Units.m  
    segment.width                               = 0.457 * Units.m  
    fuselage.Segments.append(segment)    

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------    
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 1.
    net.identical_propellers                    = True
                                                
    # the engine                    
    engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    engine.sea_level_power                  = 73.5 * Units.kilowatts
    engine.flat_rate_altitude               = 0.0
    engine.rated_speed                      = 5800. * Units.rpm
    engine.power_specific_fuel_consumption  = 0.56 #lb/hp-hr 
    net.engines.append(engine)
    
    # the prop
    prop = SUAVE.Components.Energy.Converters.Propeller()
    prop.number_of_blades        = 3.0
    prop.origin                  = [[0.625 * Units.m,0.0,0.625 * Units.m]]
    prop.freestream_velocity     = 60.   * Units.knots
    prop.angular_velocity        = 4500.  * Units.rpm
    prop.tip_radius              = 1.83/2. * Units.m
    prop.hub_radius              = 0.28/2     * Units.m
    prop.design_Cl               = 0.8
    prop.design_altitude         = 9500. * Units.feet
    prop.design_power            = .64 * 73.5 * Units.kilowatts
    prop.variable_pitch          = False
    

    prop.airfoil_geometry        =  ['./Airfoils/NACA_4412.txt'] 
    prop.airfoil_polars          = [['./Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                      './Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                      './Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                      './Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                      './Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ]]

    prop.airfoil_polar_stations  = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]      
    prop                         = propeller_design(prop)   
    
    net.propellers.append(prop)
     
    
    # add the network to the vehicle
    vehicle.append_component(net) 


    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    return vehicle

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                                    = SUAVE.Components.Configs.Config.Container() 
    base_config                                                = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag                                            = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                                     = SUAVE.Components.Configs.Config(base_config)
    config.tag                                                 = 'cruise' 
    configs.append(config)

    return configs

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'


    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    


    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses )

    segment.altitude  = 9500. * Units.feet
    segment.air_speed = 120   * Units.knots
    segment.distance  = 600 * Units.nautical_mile
    
    ones_row                                        = segment.state.ones_row   
    segment.state.numerics.number_control_points    = 16
    segment.state.unknowns.throttle                 = 1.0 * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=5500)
    
    
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip    

    # add to mission
    mission.append_segment(segment)


    return mission

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    ## ------------------------------------------------------------------
    ##  Basic Geometry Relations
    #sizing = SUAVE.Analyses.Sizing.Sizing()
    #sizing.features.vehicle = vehicle
    #analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    
    # Calculate extra drag from landing gear:
    
    main_wheel_width  = 0.26 * Units.m
    main_wheel_height = 0.4 * Units.m
    nose_gear_height  = 0.4 * Units.m
    nose_gear_width   = 0.2 * Units.m
    
    total_wheel       = 2*main_wheel_width*main_wheel_height + nose_gear_width*nose_gear_height
    
    main_gear_strut_height = 0.66 * Units.m
    main_gear_strut_length = 0.86 * Units.m
    nose_gear_strut_height = 0.4 * Units.m
    nose_gear_strut_width  = 0.05 * Units.m
    
    total_strut = 2*main_gear_strut_height*main_gear_strut_length + nose_gear_strut_height*nose_gear_strut_width
    
    # total drag increment area
    drag_area = 1.4*( total_wheel + total_strut)
    
    
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero() 
    aerodynamics.geometry                            = vehicle
    aerodynamics.settings.drag_coefficient_increment = 1.0*drag_area/vehicle.reference_area
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy= SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks #what is called throughout the mission (at every time step))
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    
    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)  
    
    plot_stability_coefficients(results, line_style)

    return




def loads(vehicle):
        
    designbrief = {}
     
    print(vehicle.wings.main_wing.areas.reference)
    
    designdef = {'aspectratio': vehicle.wings.main_wing.aspect_ratio, 'wingarea_m2': vehicle.wings.main_wing.areas.reference, 'weight_n':  vehicle.mass_properties.max_takeoff}
    
    designperf = {'CLmaxclean': vehicle.maximum_lift_coefficient , 'CLminclean': vehicle.minimum_lift_coefficient, 'CLslope': 6.28}
    
    designpropulsion = "piston" # not specifically needed for the V-n diagram here, required simply for 
                                # consistency with other classes and to support features included in later releases 
    
    designatm = at.Atmosphere() # set the design atmosphere to a zero-offset ISA
    
    csbrief={'cruisespeed_keas': 60, 'divespeed_keas': 110,
    'altitude_m': 0,
    'weightfraction': 1, 'certcat': 'norm'}
    
    concept = aw.CertificationSpecifications(designbrief, designdef, designperf, designatm, designpropulsion, csbrief)
    
    points = concept.flightenvelope(textsize=15, figsize_in=[15, 10], show=True)
        
    return

if __name__ == '__main__':
    main()
    plt.show()