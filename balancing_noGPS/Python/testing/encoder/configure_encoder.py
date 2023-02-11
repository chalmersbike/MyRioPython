from Adafruit_BBIO.Encoder import RotaryEncoder, eQEP2
import time
'''
NOT A TEST
This file queries the Beaglebone encoder pins and outputs their configuration.

Each channel can be accessed and initialized using its corresponding
channel name constants:

    eQEP0
    eQEP1  # Pins only available when video is disabled
    eQEP2
    eQEP2b # Pins only available when video is disabled
'''

# Instantiate the class to access channel eQEP2, and only initialize
# that channel
# myEncoder = RotaryEncoder(eQEP2)

# Get the current position
# cur_position = myEncoder.position

# Position can also be set as a property
# next_position = 15
# myEncoder.position = next_position

# Reset position to 0
# myEncoder.zero()

# Change mode to relative (default is absolute)
# You can use setAbsolute() to change back to absolute
# Absolute: the position starts at zero and is incremented or
#           decremented by the encoder's movement
# Relative: the position is reset when the unit timer overflows.
# myEncoder.setRelative()

# Read the current mode (0: absolute, 1: relative)
# Mode can also be set as a property
# mode = myEncoder.mode

# Read the current frequency of update
# Returned value is in Hz
# If you want to set the frequency, specify it also in Hz
# freq = myEncoder.frequency

# Disable your eQEP channel
# myEncoder.disable()

# Check if the channel is enabled
# The 'enabled' property is read-only
# Use the enable() and disable() methods to
# safely enable or disable the module
# isEnabled = myEncoder.enabled

###########

myEncoder = RotaryEncoder(eQEP2)
myEncoder.zero()
myEncoder.setAbsolute()

isEnabled = myEncoder.enabled

if isEnabled:
    try:
        print('current mode (0: absolute, 1: relative)')
        print myEncoder.mode
        print('frequency in Hz')
        print myEncoder.frequency
        raw_input("Press Enter to continue...")

        while True:
            cur_position = myEncoder.position
            print cur_position
            time.sleep(0.01)

    except KeyboardInterrupt:
        myEncoder.disable()
        print('Program terminated')
