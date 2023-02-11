import csv
import time
import sys

sys.path.append('../../')
from actuators import LinearActuator
from sensors import Potentiometer

# bike = Bike(debug=1)
potentiometer = Potentiometer()
linear_actuator = LinearActuator()
# RESULTS = open('bromstest%s.csv' % time.strftime("%Y%m%d - %H%M%S"), 'wb')
# writer = csv.writer(RESULTS)
flag = False
count = 1
pot_position = 0

while True:
    try:
        final_position = float(raw_input('Please enter a final value: '))

        raw_input('Press ENTER to start')

        start_position = potentiometer.read_pot_value()

        # writer.writerow(('Reference Position', 'Measured Position'))  # headers

        # velocity = 0.0 # init
        # time_count = 0.0

        # while velocity < 2.5:
        #     velocity = bike.get_velocity()

        while True:
            if flag == False:

                pot_position = potentiometer.read_pot_value()
                print 'Potentiometer Value = %f' % pot_position

                if pot_position > final_position:  # we need to move the actuator
                    linear_actuator.set_duty_cycle(-30)  # Values > 0: EXTEND || Values < 0: RETRACT
                elif pot_position < final_position:
                    linear_actuator.set_duty_cycle(30)  # Values > 0: EXTEND || Values < 0: RETRACT

                if pot_position >= (final_position - final_position * 0.05) and pot_position <= (
                        final_position + final_position * 0.05):
                    linear_actuator.set_duty_cycle(0)  # STOP
                    print 'Reached END value'
                    flag = True
                    # writer.writerow(('Reached Final Position +/- 0.01'))
                    for i in range(0, 30):
                        pot_position = potentiometer.read_pot_value()
                        print pot_position
                        # writer.writerow((count, final_position, pot_position))
                        count += 1
                        time.sleep(0.02)

            time.sleep(0.02)
            count += 1
            # log some data
            # writer.writerow((count, final_position, pot_position))
            # print bike.get_velocity()

            if flag == True:
                raw_input('Press ENTER to run again')
                flag = False
                final_position = float(raw_input('Please enter a final value: '))

            # time_count += 0.1

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        # bike.stop()
        linear_actuator.set_duty_cycle(0)  # STOP
        flag = False
        break

    finally:
        print 'END'
        # raw_input('Press ENTER to extend')
        # pot_position = potentiometer.read_pot_value()
        # while pot_position < 0.7:
        #     linear_actuator.set_duty_cycle(40)
        #     pot_position = potentiometer.read_pot_value()
        #     time.sleep(0.1)
        flag = False
