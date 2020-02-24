# https://stackoverflow.com/questions/25191620/
#   creating-lowpass-filter-in-scipy-understanding-methods-and-units

import numpy as np
from scipy.signal import butter, firwin, lfilter, freqz, filtfilt,lfilter_zi


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, z, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    print b
    print a
    # zi = np.ones(max(len(a), len(b)) - 1) * data[0]
    # z = lfilter_zi(b, a)*0
    y,z = lfilter(b, a, data,zi = z)
    return y

def firwin_filter(data, numtaps, cutoff, fs, z):
    b = firwin(numtaps, cutoff,nyq=fs)
    y,z = lfilter(b, 1, data,zi = z)
    return y,z


def moving_average_filter(data, windowSize=5):
    b = (1/windowSize)*(np.ones((windowSize,)))
    a = 1
    y,_ = lfilter(b, a, data)
    return y