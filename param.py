from constants import *
from math import pi as PI
import numpy as np

deg2rad = 3.14159/180.0
# Experiment Parameters
bike = 'blackbike'                              # Choice of the bike : 'redbike' or 'blackbike'
balancing_controller_structure = 'technion'     # Choice of the balancing controller structure : 'chalmers', 'technion' or 'mdh'
                                                    # Chalmers Controller structure : deltadot = PID(phidot)
                                                    # Technion Controller Structure : phidotref = PID(phi) ; deltadot = PID(phidot,phidotref)
                                                    # MDH Controller structure : delta = PID(phi) ; deltadot = PID(delta)

controller_frequency = 100                      # [Hz]Controller frequency
sample_time = 1.0 / controller_frequency        # [s] Sampling time

path_tracking = 0                              # 1 = use path tracking ; 0 = do not use path tracking
# path_tracking_engaged = 0
balancing_time = 3.0                            # The time elpased for balancing before the path tracking is engaged.
path_tracking_structure = 'parallel'            # 'parallel' : direction and lateral controller in parallel : phiref = PID(heading) + PID(lateral)
                                                # 'series' : direction and lateral controller in series : phiref = PID(heading) ; headingref = PID(lateral)
lateralError_controller = 0                     # 1 = use lateral error controller ; 0 = do not use lateral error controller
heading_controller = 1                          # 1 = use heading controller ; 0 = do not use heading controller
# path_file = 'ramp_heading_path_test_9secs_5deg.csv'                    # Name of the file path with ".csv" extension.
# path_file = 'step_heading_path_9secs_m5deg.csv'
# path_file = 'step_heading_path_9secs_m10deg.csv'
path_file = 'step_heading_path_12secs_m5deg.csv'
# path_file = 'step_heading_path_12secs_m10deg.csv'
# path_file = 'straight_path.csv'
# path_file = 'ramp_heading_path_nocoasting_6secs_5deg.csv'                    # Name of the file path with ".csv" extension.
                                                # Must be placed in "paths" folder.
                                                # Must be a CSV with 4 columns : Time, x, y, heading
                                                # Can also be 'pot' to use the potentiometer as y position on the roller

initial_speed = 4       # [m/s] Forward speed of the bike

test_duration = 5555.0  # [s] Duration of the test
start_up_interval = 5   # [s] Time to wait before running drive motor after receiving input from user to run the bike
speed_up_time = 3.0     # [s] Time to run the drive motor without steering controller to get bike up to speed
max_exceed_count = 10   # Number of times where calculation time can exceed sampling time before aborting experiment

debug = 0               # 1 = debug print outputs are enabled ; 0 = debug print outputs are disabled

# Choice to use Potentiometer
potentiometer_use = 0   # 1 = use potentiometer ; 0 = do not use potentiometer

# Choice to use GPS (for outdoors use only) or not
gps_use = 1           # 1 = use GPS ; 0 = do not use GPS
ntrip_correction = 0    # 1 = Use NTRIP correction to improve accuracy of GPS ; 0 = do not use NTRIP

# Choice to use Laser Ranger (for indoors use on roller only) or not
laserRanger_use = 0     # 1 = use laser rangers ; 0 = do not use laser rangers

# Choice to use Virtual OdometerRanger or not
# Will estimate position and heading from steering angle and velocity
# WARNING : highly imprecise !!!
virtual_odometer = 1

# Roll Reference Tracking:
roll_ref_use = 1
roll_ref_start_time = 4.0
roll_ref_end_time = 7.0
roll_ref_Mag = 2.0 * deg2rad

# Load bike specific parameters
if bike == 'blackbike':
    from param_blackbike import *
elif bike == 'redbike':
    from param_redbike import *
else:
    print "Bike choice is not valid : \"%s\"; Using black bike as default.\n" % (bike)
    from param_blackbike import *
