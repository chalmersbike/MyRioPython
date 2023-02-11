import csv
import time

from CLbike import Bike
from actuators import LinearActuator
from sensors import Potentiometer

bike = Bike(debug=1)
potentiometer = Potentiometer()
linear_actuator = LinearActuator()
RESULTS = open('bromstest.csv', 'wb')
writer = csv.writer(RESULTS)

while True:
    try:
        final_position = float(raw_input('Please enter a final value: '))

        raw_input('Press ENTER to start')

        start_position = potentiometer.read_pot_value()

        writer.writerow(
            ('Time', 'Velocity', 'Reference Position', 'Measured Position'))  # headers

        velocity = 0.0  # init
        time_count = 0.0

        while velocity < 2.5:
            velocity = bike.get_velocity()

        while True:
            pot_position = potentiometer.read_pot_value()
            print 'Potentiometer Value = %f' % pot_position

            if pot_position > final_position:  # we need to retract the actuator
                linear_actuator.set_duty_cycle(-30)  # Values > 0: EXTEND || Values < 0: RETRACT
            else:
                linear_actuator.set_duty_cycle(0)  # STOP
                print 'Reached END value'

            # log some data
            writer.writerow((time_count, bike.get_velocity(), final_position, pot_position))
            print bike.get_velocity()

            time.sleep(0.1)
            time_count += 0.1

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.stop()
        linear_actuator.set_duty_cycle(0)  # STOP

    finally:
        print 'END'
        raw_input('Press ENTER to extend')
        pot_position = potentiometer.read_pot_value()
        while pot_position < 0.7:
            linear_actuator.set_duty_cycle(40)
            pot_position = potentiometer.read_pot_value()
            time.sleep(0.1)
