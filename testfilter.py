from scipy.signal import butter, firwin, lfilter, freqz, filtfilt,lfilter_zi
from actuators import butter_lowpass,butter_lowpass_filter,moving_average_filter,firwin_filter

pot = 0
pot_previous = 0
numtaps = 5
cutoff = 5
b = firwin(numtaps, cutoff, nyq=100)
zpot = lfilter_zi(b, 1) * 0
zpotdot = lfilter_zi(b, 1) * 0
data = 0

for i in range(1,10):
    data = data + i/10.0
    pot = data
    pot = round(pot, 3)
    pot_filt, zpot = lfilter(b, 1, [pot], zi=zpot)
    potdot = (pot_filt[0] - pot_previous) / 0.01
    potdot_filt, zpotdot = lfilter(b, 1, [potdot], zi=zpotdot)
    print pot,pot_filt,potdot,potdot_filt