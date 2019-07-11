EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:bike_schematic_library
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Chalmers Bike Schematic"
Date "2018-01-03"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LM317_TO3 U1
U 1 1 5A4CAC0D
P 1650 4350
F 0 "U1" H 1500 4475 50  0000 C CNN
F 1 "LM317_TO3" H 1650 4475 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-3" H 1650 4550 50  0001 C CIN
F 3 "" H 1650 4350 50  0001 C CNN
	1    1650 4350
	1    0    0    -1  
$EndComp
$Comp
L LaunchPadMSP430F5529 U6
U 1 1 5A4CB685
P 6600 4200
F 0 "U6" H 6600 3850 60  0000 C CNN
F 1 "LaunchPadMSP430F5529" H 6600 4550 60  0000 C CNN
F 2 "" H 6600 4200 60  0001 C CNN
F 3 "" H 6600 4200 60  0001 C CNN
	1    6600 4200
	1    0    0    -1  
$EndComp
$Comp
L HONEYWELL_103SR13A-1 U8
U 1 1 5A4CCF14
P 8500 1350
F 0 "U8" H 8500 1100 60  0000 C CNN
F 1 "HONEYWELL_103SR13A-1" H 8500 1750 60  0000 C CNN
F 2 "" H 8500 1350 60  0001 C CNN
F 3 "" H 8500 1350 60  0001 C CNN
	1    8500 1350
	-1   0    0    -1  
$EndComp
$Comp
L Encoder_HEDS_5540 U9
U 1 1 5A4CCF43
P 8500 2100
F 0 "U9" H 8500 1800 60  0000 C CNN
F 1 "Encoder_HEDS_5540" H 8550 2450 60  0000 C CNN
F 2 "" H 8500 2200 60  0001 C CNN
F 3 "" H 8500 2200 60  0001 C CNN
	1    8500 2100
	-1   0    0    -1  
$EndComp
$Comp
L JAGUAR U7
U 1 1 5A4CD336
P 8450 3900
F 0 "U7" H 8450 3400 60  0000 C CNN
F 1 "JAGUAR" H 8450 4200 60  0000 C CNN
F 2 "" H 8450 3900 60  0001 C CNN
F 3 "" H 8450 3900 60  0001 C CNN
	1    8450 3900
	1    0    0    -1  
$EndComp
$Comp
L SHIMANO_SWITCH U5
U 1 1 5A4CD536
P 6500 950
F 0 "U5" H 6500 750 60  0000 C CNN
F 1 "SHIMANO_SWITCH" H 6500 1150 60  0000 C CNN
F 2 "" H 6650 950 60  0001 C CNN
F 3 "" H 6650 950 60  0001 C CNN
	1    6500 950 
	-1   0    0    -1  
$EndComp
$Comp
L MAXON_MOTOR U11
U 1 1 5A4CD586
P 9600 4000
F 0 "U11" H 9600 3800 60  0000 C CNN
F 1 "MAXON_MOTOR" H 9600 4200 60  0000 C CNN
F 2 "" H 9600 4000 60  0001 C CNN
F 3 "" H 9600 4000 60  0001 C CNN
	1    9600 4000
	-1   0    0    -1  
$EndComp
$Comp
L Battery BT1
U 1 1 5A4CDB6A
P 900 2850
F 0 "BT1" H 1000 2950 50  0000 L CNN
F 1 "+11.1V" H 1000 2850 50  0000 L CNN
F 2 "" V 900 2910 50  0001 C CNN
F 3 "" V 900 2910 50  0001 C CNN
	1    900  2850
	1    0    0    1   
$EndComp
$Comp
L Battery BT2
U 1 1 5A4CDBE3
P 7600 3950
F 0 "BT2" H 7700 4050 50  0000 L CNN
F 1 "+22.2V" H 7700 3950 50  0000 L CNN
F 2 "" V 7600 4010 50  0001 C CNN
F 3 "" V 7600 4010 50  0001 C CNN
	1    7600 3950
	1    0    0    -1  
$EndComp
$Comp
L TEN_8-2411WI U2
U 1 1 5A4CE3DF
P 2100 3050
F 0 "U2" H 2100 2700 60  0000 C CNN
F 1 "TEN_8-2411WI" H 2100 3700 60  0000 C CNN
F 2 "" H 2250 3000 60  0001 C CNN
F 3 "" H 2250 3000 60  0001 C CNN
	1    2100 3050
	1    0    0    -1  
$EndComp
$Comp
L BEAGLEBONE_BLACK U3
U 1 1 5A4CEEAB
P 4450 2950
F 0 "U3" H 4450 2400 60  0000 C CNN
F 1 "BEAGLEBONE_BLACK" H 4450 3650 60  0000 C CNN
F 2 "" H 4250 2950 60  0001 C CNN
F 3 "" H 4250 2950 60  0001 C CNN
	1    4450 2950
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5A4CF002
P 1450 2850
F 0 "C1" H 1475 2950 50  0000 L CNN
F 1 "1uC" H 1475 2750 50  0000 L CNN
F 2 "" H 1488 2700 50  0001 C CNN
F 3 "" H 1450 2850 50  0001 C CNN
	1    1450 2850
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5A4CF20D
P 2100 2250
F 0 "C2" H 2125 2350 50  0000 L CNN
F 1 "1nC" H 2125 2150 50  0000 L CNN
F 2 "" H 2138 2100 50  0001 C CNN
F 3 "" H 2100 2250 50  0001 C CNN
	1    2100 2250
	0    -1   -1   0   
$EndComp
$Comp
L C C3
U 1 1 5A4CF34C
P 2100 3600
F 0 "C3" H 2125 3700 50  0000 L CNN
F 1 "1nC" H 2125 3500 50  0000 L CNN
F 2 "" H 2138 3450 50  0001 C CNN
F 3 "" H 2100 3600 50  0001 C CNN
	1    2100 3600
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 5A4CFDFE
P 2100 4500
F 0 "R1" V 2180 4500 50  0000 C CNN
F 1 "240R" V 2100 4500 50  0000 C CNN
F 2 "" V 2030 4500 50  0001 C CNN
F 3 "" H 2100 4500 50  0001 C CNN
	1    2100 4500
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A4CFE8E
P 2100 4900
F 0 "R2" V 2180 4900 50  0000 C CNN
F 1 "390R" V 2100 4900 50  0000 C CNN
F 2 "" V 2030 4900 50  0001 C CNN
F 3 "" H 2100 4900 50  0001 C CNN
	1    2100 4900
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5A4D01E2
P 2350 4900
F 0 "C4" H 2375 5000 50  0000 L CNN
F 1 "15uC" H 2375 4800 50  0000 L CNN
F 2 "" H 2388 4750 50  0001 C CNN
F 3 "" H 2350 4900 50  0001 C CNN
	1    2350 4900
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5A4D02B7
P 2550 4900
F 0 "C5" H 2575 5000 50  0000 L CNN
F 1 "1uC" H 2575 4800 50  0000 L CNN
F 2 "" H 2588 4750 50  0001 C CNN
F 3 "" H 2550 4900 50  0001 C CNN
	1    2550 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  2650 1750 2650
Wire Wire Line
	1650 2650 1650 2750
Wire Wire Line
	1650 2750 1750 2750
Connection ~ 1650 2650
Wire Wire Line
	900  3050 1750 3050
Wire Wire Line
	1650 3050 1650 3150
Wire Wire Line
	1650 3150 1750 3150
Connection ~ 1650 3050
Wire Wire Line
	1450 2250 1450 2700
Connection ~ 1450 2650
Wire Wire Line
	1450 3000 1450 3600
Connection ~ 1450 3050
Wire Wire Line
	1450 2250 1950 2250
Wire Wire Line
	2250 2250 2500 2250
Wire Wire Line
	2500 2250 2500 2850
Connection ~ 2500 2850
Wire Wire Line
	1450 3600 1950 3600
Wire Wire Line
	2250 3600 2500 3600
Wire Wire Line
	2500 3600 2500 2950
Connection ~ 2500 2950
Wire Wire Line
	1950 4350 2550 4350
Wire Wire Line
	2100 4650 2100 4750
Wire Wire Line
	1650 4650 2350 4650
Wire Wire Line
	2350 4650 2350 4750
Connection ~ 2100 4650
Wire Wire Line
	2550 4350 2550 4750
Connection ~ 2100 4350
Wire Wire Line
	2500 2950 2400 2950
Wire Wire Line
	7600 3750 8150 3750
Wire Wire Line
	7600 4150 7750 4150
Wire Wire Line
	7750 4150 7750 3850
Wire Wire Line
	7750 3850 8150 3850
Wire Wire Line
	8150 4150 7850 4150
Wire Wire Line
	7850 4150 7850 4400
Wire Wire Line
	7850 4400 6950 4400
Wire Wire Line
	5050 3250 5200 3250
Wire Wire Line
	5200 3250 5200 4300
Wire Wire Line
	5200 4300 6250 4300
Wire Wire Line
	6250 4200 5350 4200
Wire Wire Line
	5350 4200 5350 3150
Wire Wire Line
	5350 3150 5050 3150
Wire Wire Line
	2500 2850 2400 2850
$Comp
L C C8
U 1 1 5A4D79D7
P 7950 1300
F 0 "C8" H 7975 1400 50  0000 L CNN
F 1 "0.1uC" H 7975 1200 50  0000 L CNN
F 2 "" H 7988 1150 50  0001 C CNN
F 3 "" H 7950 1300 50  0001 C CNN
	1    7950 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 1450 7950 1450
Wire Wire Line
	8100 1450 8100 1250
Wire Wire Line
	8100 1250 8300 1250
Wire Wire Line
	7950 1150 8300 1150
Wire Wire Line
	8300 1350 8150 1350
Wire Wire Line
	8150 1350 8150 1550
Wire Wire Line
	8150 1550 5350 1550
$Comp
L R R9
U 1 1 5A4D84A5
P 7700 1300
F 0 "R9" V 7780 1300 50  0000 C CNN
F 1 "R" V 7700 1300 50  0000 C CNN
F 2 "" V 7630 1300 50  0001 C CNN
F 3 "" H 7700 1300 50  0001 C CNN
	1    7700 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 1550 7700 1450
$Comp
L C C6
U 1 1 5A4D8C95
P 7750 3050
F 0 "C6" H 7775 3150 50  0000 L CNN
F 1 "0.1uC" H 7775 2950 50  0000 L CNN
F 2 "" H 7788 2900 50  0001 C CNN
F 3 "" H 7750 3050 50  0001 C CNN
	1    7750 3050
	-1   0    0    -1  
$EndComp
Connection ~ 7750 3200
Connection ~ 7750 2900
Wire Wire Line
	7550 3200 8150 3200
Wire Wire Line
	8150 3100 7900 3100
Wire Wire Line
	7900 3100 7900 2900
Wire Wire Line
	7900 2900 7550 2900
Wire Wire Line
	8200 2300 7800 2300
Wire Wire Line
	8200 2000 7800 2000
$Comp
L C C7
U 1 1 5A4DA2AC
P 7800 2150
F 0 "C7" H 7825 2250 50  0000 L CNN
F 1 "0.1uC" H 7825 2050 50  0000 L CNN
F 2 "" H 7838 2000 50  0001 C CNN
F 3 "" H 7800 2150 50  0001 C CNN
	1    7800 2150
	1    0    0    -1  
$EndComp
$Comp
L LTV-356T U4
U 1 1 5A4DA552
P 5450 950
F 0 "U4" H 5240 1140 50  0000 L CNN
F 1 "LTV-356T" H 5450 1150 50  0000 L CNN
F 2 "Housings_SOIC:SO-4_4.4x3.6mm_Pitch2.54mm" H 5250 750 50  0001 L CIN
F 3 "" H 5450 950 50  0001 L CNN
	1    5450 950 
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5A4DB05B
P 5700 2550
F 0 "R4" V 5780 2550 50  0000 C CNN
F 1 "4.7kR" V 5700 2550 50  0000 C CNN
F 2 "" V 5630 2550 50  0001 C CNN
F 3 "" H 5700 2550 50  0001 C CNN
	1    5700 2550
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5A4DB0AC
P 6050 2550
F 0 "R5" V 6130 2550 50  0000 C CNN
F 1 "4.7kR" V 6050 2550 50  0000 C CNN
F 2 "" V 5980 2550 50  0001 C CNN
F 3 "" H 6050 2550 50  0001 C CNN
	1    6050 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 2950 5050 2950
Wire Wire Line
	5050 3050 6050 3050
Wire Wire Line
	8150 3000 8050 3000
Wire Wire Line
	8050 3000 8050 2800
Wire Wire Line
	5700 2800 8150 2800
Connection ~ 8050 2800
Wire Wire Line
	5700 2700 5700 2950
Connection ~ 5700 2800
Wire Wire Line
	6050 3050 6050 2700
Wire Wire Line
	8150 2900 8000 2900
Wire Wire Line
	8000 2900 8000 2700
Wire Wire Line
	6050 2700 8150 2700
Connection ~ 8000 2700
Connection ~ 6050 2700
Wire Wire Line
	5200 1900 8200 1900
Wire Wire Line
	5250 2100 8200 2100
Wire Wire Line
	5300 2200 8200 2200
$Comp
L R R8
U 1 1 5A4DBC57
P 7250 1750
F 0 "R8" V 7330 1750 50  0000 C CNN
F 1 "R" V 7250 1750 50  0000 C CNN
F 2 "" V 7180 1750 50  0001 C CNN
F 3 "" H 7250 1750 50  0001 C CNN
	1    7250 1750
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5A4DBCC1
P 7000 1950
F 0 "R7" V 7080 1950 50  0000 C CNN
F 1 "R" V 7000 1950 50  0000 C CNN
F 2 "" V 6930 1950 50  0001 C CNN
F 3 "" H 7000 1950 50  0001 C CNN
	1    7000 1950
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5A4DBD3B
P 6750 2050
F 0 "R6" V 6830 2050 50  0000 C CNN
F 1 "R" V 6750 2050 50  0000 C CNN
F 2 "" V 6680 2050 50  0001 C CNN
F 3 "" H 6750 2050 50  0001 C CNN
	1    6750 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1900 5200 2550
Wire Wire Line
	5200 2550 5050 2550
Connection ~ 7250 1900
Wire Wire Line
	5250 2100 5250 2650
Wire Wire Line
	5250 2650 5050 2650
Connection ~ 7000 2100
Wire Wire Line
	5050 2750 5300 2750
Wire Wire Line
	5300 2750 5300 2200
Connection ~ 6750 2200
Wire Wire Line
	5350 1550 5350 2850
Wire Wire Line
	5350 2850 5050 2850
Connection ~ 7700 1550
Wire Wire Line
	5750 1050 6150 1050
Wire Wire Line
	6150 1050 6150 950 
Wire Wire Line
	6150 950  6250 950 
Wire Wire Line
	5750 850  6250 850 
Wire Wire Line
	5150 850  4900 850 
$Comp
L R R3
U 1 1 5A4DD05A
P 4750 850
F 0 "R3" V 4830 850 50  0000 C CNN
F 1 "560R" V 4750 850 50  0000 C CNN
F 2 "" V 4680 850 50  0001 C CNN
F 3 "" H 4750 850 50  0001 C CNN
	1    4750 850 
	0    1    1    0   
$EndComp
Wire Wire Line
	4600 850  4600 1400
Wire Wire Line
	4600 1400 5100 1400
Wire Wire Line
	5100 1400 5100 2450
Wire Wire Line
	5100 2450 5050 2450
Text GLabel 2500 2850 2    60   Output ~ 0
GND
Text GLabel 2500 2950 2    60   Output ~ 0
+5V
Connection ~ 2350 4750
Connection ~ 2550 4750
Text GLabel 3900 2550 0    60   Input ~ 0
GND
Text GLabel 3900 2450 0    60   Input ~ 0
+5V
Text GLabel 2550 4350 2    60   Input ~ 0
+3.3V
Text GLabel 2100 5050 3    60   Input ~ 0
GND
Text GLabel 2350 5050 3    60   Input ~ 0
GND
Text GLabel 2550 5050 3    60   Input ~ 0
GND
Text GLabel 1350 4350 0    60   Input ~ 0
+5V
Text GLabel 8150 4250 0    60   Input ~ 0
GND
Text GLabel 7550 2900 0    60   Input ~ 0
GND
Text GLabel 7550 3200 0    60   Input ~ 0
+3.3V
Text GLabel 7800 2300 3    60   Input ~ 0
GND
Text GLabel 7800 2000 0    60   Input ~ 0
+5V
Text GLabel 7700 1150 1    60   Input ~ 0
+3.3V
Text GLabel 7950 1150 1    60   Input ~ 0
+5V
Text GLabel 5150 1050 0    60   Input ~ 0
GND
Text GLabel 6950 4000 2    60   Input ~ 0
+5V
Text GLabel 6950 4100 2    60   Input ~ 0
GND
Text GLabel 7250 1600 1    60   Input ~ 0
+3.3V
Text GLabel 7000 1800 1    60   Input ~ 0
+3.3V
Text GLabel 6750 1900 1    60   Input ~ 0
+3.3V
$Comp
L IMU_FRDM-STBC-AGM01 U10
U 1 1 5A4CCFA8
P 8500 2850
F 0 "U10" H 8500 2300 60  0000 C CNN
F 1 "IMU_FRDM-STBC-AGM01" H 8500 3200 60  0000 C CNN
F 2 "" H 8500 2850 60  0001 C CNN
F 3 "" H 8500 2850 60  0001 C CNN
	1    8500 2850
	-1   0    0    -1  
$EndComp
Text GLabel 6050 2400 1    60   Input ~ 0
+3.3V
Text GLabel 5700 2400 1    60   Input ~ 0
+3.3V
Wire Wire Line
	8750 3950 9400 3950
Wire Wire Line
	9400 4050 8750 4050
$EndSCHEMATC
