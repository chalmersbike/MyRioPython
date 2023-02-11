# This test can time function calls as modules. Useful for comparing calculations. This code should probably be
# transferred across to the controller and called at will

import math
import time
from functools import wraps

import numpy as np

GRAVITY = 9.82  # gravity
WHEELBASE = 1.095  # length between wheel centers


def fn_timer(function):
    @wraps(function)
    def function_timer(*args, **kwargs):
        t0 = time.time()
        result = function(*args, **kwargs)
        t1 = time.time()
        print ("Total time running %s: %s seconds" %
               (function.func_name, str(t1 - t0))
               )
        return result

    return function_timer


@fn_timer
def zip1(ref, st, fee):
    return sum([(r - x) * k for r, x, k in zip(
        ref, st, fee
    )])


@fn_timer
def numpytest(ref, st, fee):
    return np.matmul(np.subtract(ref, st), fee)


def get_references(velocity):
    delta_ref = 0.0  # straight line test
    phi_ref = math.atan(((velocity * velocity) / (GRAVITY * WHEELBASE)) * delta_ref)
    return [phi_ref, delta_ref, 0.0]


if __name__ == "__main__":
    references = [0.09, 0.01127, 0.02]
    states = [0.09, 0.01127, 0.02]
    feedback_gains = [-18.7580, 4.1700, -4.9389]

for _ in range(10):
    zip1(references, states, feedback_gains)
    numpytest(references, states, feedback_gains)
