#!/bin/sh

# Not a test file. This file polls EQEP channel 0 for data and can be used to debug the encoder on the Beaglebone

while [ /bin/sh ]
do
	cat /sys/devices/platform/ocp/48300000.epwmss/48300180.eqep/position
	#sleep 0.5
done
