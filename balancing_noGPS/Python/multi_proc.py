import multiprocessing as mul
import time
import numpy
def proc1(pipe):
    tstart = time.time()
    always = 0
    x_array = numpy.random.rand(4, 1)
    counter = 0
    tlast_sample = tstart
    while time.time() - tstart < 20:
        while pipe.poll(): # Check buffer for trigger
            if pipe.recv() == 'always_ON':
                always = 1
        if always == 1:
            counter = counter + 1
            pipe.send(counter)
            tlast_sample = time.time()

        sleep_time = 0.1 - (time.time() - tlast_sample)
        if sleep_time > 0:
            time.sleep(sleep_time)


def proc2(pipe):
    tstart = time.time()
    not_always_yet = 1
    while time.time() - tstart < 20:
        if time.time() - tstart > 5 and not_always_yet:
            print('Trigger!')
            pipe.send('always_ON')
            not_always_yet = 0
        while pipe.poll():
            print(pipe.recv())
        time.sleep(0.01)

def proc3(pipe):
    tstart = time.time()
    not_always_yet = 1
    while time.time() - tstart < 20:
        if time.time() - tstart > 5 and not_always_yet:
            print('Trigger333!')
            pipe.send('always_ON')
            not_always_yet = 0
        time.sleep(0.01)


# Build a pipe
parent, child = mul.Pipe()
if __name__ == '__main__':

    # Pass an end of the pipe to process 1
    p1 = mul.Process(target=proc1, args=(parent,), daemon=True)
    # Pass the other end of the pipe to process 2
    p2 = mul.Process(target=proc2, args=(child,), daemon=True)
    p3 = mul.Process(target=proc3, args=(child,), daemon=True)

    p1.start()
    tstart = time.time()
    p2.start()
    p3.start()

    while p1.is_alive() and p2.is_alive():
        if time.time() - tstart > 10:
            print('ALL TERMINATING!')
            while p1.is_alive() or p2.is_alive():
                p1.terminate()
                p2.terminate()
                time.sleep(0.01)
            print(p1.is_alive())
            print(p2.is_alive())
            print('ALL TERMINATED!')
            break
        time.sleep(2)