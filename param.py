from constants import *
from math import pi as PI
import numpy as np

# deg2rad = 3.14159/180.0

####################################################################################################################
####################################################################################################################
# Experiment Parameters
bike = 'blackbike'                              # Choice of the bike : 'redbike' or 'blackbike'
balancing_controller_structure = 'technion'     # Choice of the balancing controller structure : 'chalmers', 'technion' or 'mdh'
                                                    # Chalmers Controller structure : deltadot = PID(phidot)
                                                    # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
                                                    # MDH Controller structure : delta = PID(phi) ; deltadot = PID(delta)

initial_speed = 3                               # [m/s] Forward speed of the bike

controller_frequency = 100                      # [Hz]Controller frequency
sample_time = 1.0 / controller_frequency        # [s] Sampling time

test_duration = 5555.0                          # [s] Duration of the test
start_up_interval = 5                           # [s] Time to wait before running drive motor after receiving input from user to run the bike
# speed_up_time = test_duration                   # [s] Duration of the test
speed_up_time = 3.0                             # [s] Time to run the drive motor without steering controller to get bike up to speed
walk_time = 2.0                                 # [s] Time to walk the bike while manually controlling steering to get an estimate of the steering angle offset
max_exceed_count = 10                           # Number of times where calculation time can exceed sampling time before aborting experiment

path_tracking = 1                               # 1 = use path tracking ; 0 = do not use path tracking
# path_tracking_engaged = 0
balancing_time = 0.0                            # The time elapsed for balancing before the path tracking is engaged.
path_tracking_structure = 'series'              # 'parallel' : direction and lateral controller in parallel : phiref = PID(heading) + PID(lateral)
                                                # 'series' : direction and lateral controller in series : phiref = PID(heading) ; headingref = PID(lateral)
lateralError_controller = 1                     # 1 = use lateral error controller ; 0 = do not use lateral error controller
heading_controller = 1                          # 1 = use heading controller ; 0 = do not use heading controller
# path_end = ''                                   # WARNING : YOU WILL NEED TO CATCH THE BIKE OR IT MIGHT FALL  - Do nothing when we reach the end of the path
path_end = 'circle'                             # Go into a circle after we reach the end of the reference path
# path_end = 'uturn'                              # Do a U-turn and come back to start of path after we reach the end of the reference path
path_end_circle_rollRef = -6                     # [deg] Constant roll reference to go into a circle after we reach the end of the reference path
path_end_uturn_radius = 15                      # [m] Maximum turning radius of the U-turn to come back to start of path after we reach the end of the reference path
path_end_uturn_stepSize = 1                     # [m] Step size of the path describing the U-turn to come back to start of path after we reach the end of the reference path
path_end_uturn_left = 0                         # 1 = do the U-turn to the left ; 0 = do the U-turn to the right
path_end_uturn_distanceEndPath = 40             # [m] Distance from last point of reference path at which the bike should get back on path in opposite direction after U-turn

# Choice to use more debug outputs
debug = 0               # 1 = debug print outputs are enabled ; 0 = debug print outputs are disabled

# Choice to use Potentiometer
potentiometer_use = 0   # 1 = use potentiometer ; 0 = do not use potentiometer

# Choice to use GPS (for outdoors use only) or not
gps_use = 1             # 1 = use GPS ; 0 = do not use GPS
ntrip_correction = 1    # 1 = Use NTRIP correction to improve accuracy of GPS ; 0 = do not use NTRIP

# Choice to use Laser Ranger (for indoors use on roller only) or not
laserRanger_use = 0     # 1 = use laser rangers ; 0 = do not use laser rangers

# Choice to use Virtual OdometerRanger or not
# Will estimate position and heading from steering angle and velocity
# WARNING : highly imprecise !!!
virtual_odometer = 0

# Roll Reference Tracking:
roll_ref_use = 0
roll_ref_step_imp_flag = 0  # 0 means step, 1 means impulse
rol_ref_periodic = 0

####################################################################################################################
####################################################################################################################
# Reference Path file
# Name of the file path with ".csv" extension. Must be placed in "paths" folder.
# Must be a CSV with 4 columns : time, x, y, heading
# Can also be 'pot' to use the potentiometer as y position on the roller
# Can also be a CSV file with 3 columns : time, lat, lon.
# Can also be 'newest' to load the most recent lat/lon path from "paths".

path_file = 'newest'
# path_file = 'path_20201125-093000.csv'

# path_file = 'ramp_heading_path_test_9secs_5deg.csv'
# path_file = 'step_heading_path_9secs_m5deg.csv'
# path_file = 'step_heading_path_9secs_m10deg.csv'
# path_file = 'step_heading_path_12secs_m5deg.csv'
# path_file = 'step_heading_path_12secs_m10deg.csv'
# path_file = 'straight_path.csv'
# path_file = 'latlon_straight_parking.csv'
# path_file = 'ramp_heading_path_nocoasting_6secs_5deg.csv'

####################################################################################################################
####################################################################################################################
# Reference roll file
# Name of the roll reference file with ".csv" extension. Must be placed in "rollref" folder.
# Must be a CSV with 2 columns : Time, rollref. Must contain "rollref" in the name
# Can also be 'nofile' to use not read a CSV file and use a roll reference defined in the Python code
# rollref_file = 'rollref_test1.csv'
# rollref_file = 'roll_ref_sqr_5.csv'
rollref_file = 'nofile'

# Turn to RIGHT, T is the period
rollref_file = 'roll_ref_StepT3_3deg.csv'
# rollref_file = 'roll_ref_StepT5_3deg.csv'

# Turn to LEFT Deg can be 1-5;
#rollref_file = 'roll_ref_StepT3_-3deg.csv'
#rollref_file = 'roll_ref_StepT5_-3deg.csv'

#rollref_file = 'roll_ref_0.csv'      # Name of the roll reference file with ".csv" extension.


#rollref_file = 'roll_UrefBandStop23_MagPM4LP5.csv'
#rollref_file = 'roll_UrefBandStop23_MagPM7LP5.csv'
#rollref_file = 'roll_UrefBandStop23_MagPM10LP5.csv'
# rollref_file = 'roll_UrefBandStop23_MagPM14LP5.csv'
#rollref_file = 'roll_UrefBandStop23_MagPM16LP5.csv'

####################################################################################################################
####################################################################################################################
# Steering disturbance file
strdistbref_file = 'nofile'
#strdistbref_file = 'str_Udistb3degLP5.csv'
# strdistbref_file = 'str_Udistb6degLP4.csv'
#strdistbref_file = 'str_Udistb10degLP4.csv'
# strdistbref_file = 'str_Udistb_bandstop23_6degLP5.csv'
# strdistbref_file = 'str_Udistb_bandstop23_9degLP5.csv'
# strdistbref_file = 'str_Udistb_bandstop23_7degLP1.csv'
# strdistbref_file = 'str_Udistb_bandstop23_7degLP2.csv'
# strdistbref_file = 'str_Udistb_bandstop23_10degLP5.csv'
#strdistbref_file = 'str_Udistb_bandstop23_14degLP5.csv'
#strdistbref_file = 'str_Udistb_bandstop23_18degLP5.csv'

roll_ref_start_time = 6.0
roll_ref_end_time = 1000.0
if rol_ref_periodic is 1:
    roll_ref_period = (roll_ref_end_time - roll_ref_start_time)*2
roll_ref_Mag = -0.0 * deg2rad
# roll_ref_Mag = -5.0 * deg2rad #  Right side
# roll_ref_Mag = -8.0 * deg2rad
# roll_ref_Mag = 5.0 * deg2rad   # Left Side
#roll_ref_Mag = 8.0 * deg2rad

circle_switch = False # You can't use 1 to replace True!
if circle_switch is True:
    rol_ref_periodic = 1
    roll_ref_start_time1 = 6.0
    roll_ref_Mag1 = -5.0 * deg2rad
    roll_ref1_period = 24.06 # Time elapsed for one circle at 5.0 degs roll when v= 3
    roll_ref_start_time2 = roll_ref_start_time1 + roll_ref1_period
    roll_ref_Mag2 = -8.0 * deg2rad
    roll_ref2_period = 15.0 # for one circle at 8.0 secs v = 3
    roll_ref_totalperiod = roll_ref1_period + roll_ref2_period
    roll_ref_end_time = roll_ref_start_time1 + roll_ref_totalperiod




roll_ref_imp_start_time1 = 4.0
roll_ref_imp_start_time2 = 7.0


roll_ref_imp_Mag = 5.0 * deg2rad


####################################################################################################################
####################################################################################################################
# Load bike specific parameters
if bike == 'blackbike':
    from param_blackbike import *
elif bike == 'redbike':
    from param_redbike import *
else:
    print("Bike choice is not valid : \"%s\"; Using black bike as default.\n" % (bike))
    from param_blackbike import *
