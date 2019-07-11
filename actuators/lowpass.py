# https://stackoverflow.com/questions/25191620/
#   creating-lowpass-filter-in-scipy-understanding-methods-and-units

import numpy as np
from scipy.signal import butter, lfilter, freqz, filtfilt,lfilter_zi


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    # zi = np.ones(max(len(a), len(b)) - 1) * data[0]
    zi = lfilter_zi(b, a) * data[0]
    y,_ = lfilter(b, a, data,zi = zi)
    return y


def moving_average_filter(data, windowSize=5):
    b = (1/windowSize)*(np.ones((windowSize,)))
    a = 1
    y,_ = lfilter(b, a, data)
    return y