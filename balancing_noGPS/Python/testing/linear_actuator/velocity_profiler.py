import csv
import time

from CLbike import Bike
from actuators import LinearActuator
from sensors import Potentiometer

bike = Bike(debug=1)
potentiometer = Potentiometer()
linear_actuator = LinearActuator()

ACTUATOR_DISENGAGED_POSITION = 0.5  # %
TIME_CONSTANT = 0.05  # ms
INIT_VELOCITY = 3.0  # m/s
ACCELERATION = -0.625  # m/s^2
LOWER_BOUND = 0.1  # m/s
UPPER_BOUND = 0.3  # m/s

RESULTS = open('velocity_profiler%s.csv' % time.strftime("%Y%m%d - %H%M%S"), 'wb')
writer = csv.writer(RESULTS)


def calc_duty_cycle(error_velocity):
    if LOWER_BOUND < error_velocity < UPPER_BOUND:
        duty_cycle = (100.0 / 0.3) * error_velocity - 166.67
    elif -UPPER_BOUND < error_velocity < -LOWER_BOUND:
        duty_cycle = (100.0 / 0.3) * error_velocity + 166.67
    elif error_velocity > UPPER_BOUND:
        duty_cycle = 100
    elif error_velocity < -UPPER_BOUND:
        duty_cycle = -100
    else:
        duty_cycle = 0
    return duty_cycle


def extend_actuator():
    pot_position = potentiometer.read_pot_value()
    while pot_position < ACTUATOR_DISENGAGED_POSITION:
        linear_actuator.set_duty_cycle(100)
        pot_position = potentiometer.read_pot_value()
        time.sleep(0.02)
    linear_actuator.set_duty_cycle(0)  # STOP


def main():
    try:

        flag = False

        raw_input('Press ENTER to move the actuator to the starting position')
        while not flag:
            pot_position = potentiometer.read_pot_value()
            print 'Pot Position = %f' % pot_position

            if pot_position > ACTUATOR_DISENGAGED_POSITION:
                linear_actuator.set_duty_cycle(-50)
            if pot_position < ACTUATOR_DISENGAGED_POSITION:
                linear_actuator.set_duty_cycle(50)
            elif pot_position >= (ACTUATOR_DISENGAGED_POSITION - 0.01) and pot_position <= (
                    ACTUATOR_DISENGAGED_POSITION + 0.01):
                linear_actuator.set_duty_cycle(0)  # STOP
                flag = True
            time.sleep(0.02)

        raw_input('Press ENTER to start')
        writer.writerow(
            ('Time', 'Reference Velocity', 'Measured Velocity', 'Actuator Position', 'Exec Time'))  # headers

        #  init variables
        velocity = 0.0
        time_count = 0.0
        reference_velocity = INIT_VELOCITY

        while velocity < INIT_VELOCITY:
            velocity = bike.get_velocity()
            print velocity
            time.sleep(TIME_CONSTANT)

        print 'CLOSED LOOP START'

        while reference_velocity > 0.0:
            tick = time.time()
            reference_velocity = reference_velocity + ACCELERATION * TIME_CONSTANT
            measured_velocity = bike.get_velocity()
            if measured_velocity > 10:
                measured_velocity = reference_velocity  # bad fix for the strange issue where velocity spikes to a
                # very high number
            error_velocity = reference_velocity - measured_velocity

            duty_cycle = int(calc_duty_cycle(error_velocity))

            # safety check
            pot_position = potentiometer.read_pot_value()
            if pot_position < 0.1 and duty_cycle < 0:
                linear_actuator.set_duty_cycle(0)
            else:
                linear_actuator.set_duty_cycle(duty_cycle)

            tock = time.time() - tick
            if tock < TIME_CONSTANT:
                time.sleep(TIME_CONSTANT - tock)

            time_count += TIME_CONSTANT
            #  log some data
            writer.writerow((time_count, reference_velocity, measured_velocity, pot_position, tock))
            print ('###\nReference Velocity = %f\nMeasured Velocity = %f\nExec Time = %f' % (
                reference_velocity, measured_velocity, tock))

    except KeyboardInterrupt:
        print 'KEYBOARD BREAK'
        bike.stop()
        linear_actuator.set_duty_cycle(0)  # STOP
        raw_input('Press ENTER to extend')
        extend_actuator()


    finally:
        print 'END'
        linear_actuator.set_duty_cycle(0)  # STOP
        raw_input('Press ENTER to extend')
        extend_actuator()


if __name__ == "__main__":
    main()
