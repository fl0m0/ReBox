EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	9650 4050 9650 4150
Wire Wire Line
	9850 4150 9650 4150
Connection ~ 9650 4150
Wire Wire Line
	9650 4150 9650 4250
Text Label 8850 4450 0    50   ~ 0
SCL
Text Label 8850 4550 0    50   ~ 0
SDA
Text Label 8850 4750 0    50   ~ 0
~IO_INT
Wire Wire Line
	9150 6250 9150 6050
Connection ~ 9150 5950
Wire Wire Line
	9150 5950 9150 5850
Connection ~ 9150 6050
Wire Wire Line
	9150 6050 9150 5950
Text HLabel 10700 4450 2    50   Input ~ 0
BTN1
Wire Wire Line
	10700 4450 10150 4450
Text HLabel 10700 4550 2    50   Input ~ 0
BTN2
Wire Wire Line
	10700 4550 10150 4550
Text HLabel 10700 4650 2    50   Input ~ 0
BTN3
Wire Wire Line
	10700 4650 10150 4650
Text HLabel 10700 4750 2    50   Input ~ 0
BTN4
Wire Wire Line
	10700 4750 10150 4750
Text HLabel 10700 4850 2    50   Input ~ 0
BTN5
Wire Wire Line
	10700 4850 10150 4850
Text HLabel 10700 4950 2    50   Input ~ 0
BTN6
Wire Wire Line
	10700 4950 10150 4950
Text HLabel 10700 5050 2    50   Input ~ 0
BTN7
Wire Wire Line
	10700 5050 10150 5050
Text Notes 8550 5950 0    50   ~ 0
Address 0x32
Text HLabel 10700 5350 2    50   Output ~ 0
RLY1_CTRL
Text HLabel 10700 5450 2    50   Output ~ 0
RLY2_CTRL
Text HLabel 10700 5550 2    50   Output ~ 0
RLY3_CTRL
Text HLabel 10700 5650 2    50   Output ~ 0
RLY4_CTRL
Text HLabel 10700 5750 2    50   Output ~ 0
RLY5_CTRL
Text HLabel 10700 5850 2    50   Output ~ 0
RLY6_CTRL
Wire Wire Line
	10150 5350 10700 5350
Wire Wire Line
	10700 5450 10150 5450
Wire Wire Line
	10150 5550 10700 5550
Wire Wire Line
	10700 5650 10150 5650
Wire Wire Line
	10150 5750 10700 5750
Wire Wire Line
	10700 5850 10150 5850
NoConn ~ 10150 5950
NoConn ~ 10150 6050
NoConn ~ 10150 5150
$Comp
L Regulator_Linear:XC6210B332MR U203
U 1 1 61C87D83
P 4850 5200
F 0 "U203" H 4850 5567 50  0000 C CNN
F 1 "XC6210B332MR" H 4850 5476 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 4850 5200 50  0001 C CNN
F 3 "https://www.torexsemi.com/file/xc6210/XC6210.pdf" H 5600 4200 50  0001 C CNN
	1    4850 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0230
U 1 1 621C3374
P 9150 6250
F 0 "#PWR0230" H 9150 6000 50  0001 C CNN
F 1 "GND" H 9155 6077 50  0000 C CNN
F 2 "" H 9150 6250 50  0001 C CNN
F 3 "" H 9150 6250 50  0001 C CNN
	1    9150 6250
	1    0    0    -1  
$EndComp
$Comp
L interface_User:PCA9535DWR U205
U 1 1 61C48254
P 9650 5250
F 0 "U205" H 9350 6200 50  0000 C CNN
F 1 "PCA9535DWR" H 9950 6200 50  0000 C CNN
F 2 "Package_SO:SOIC-24W_7.5x15.4mm_P1.27mm" H 9650 3800 50  0001 C CNN
F 3 "https://www.ti.com/lit/ds/symlink/pca9535.pdf" H 9620 3830 50  0001 C CNN
F 4 "Texas Instruments" H 9300 3650 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 9650 3650 50  0001 C CNN "Supplier"
F 6 "296-20950-1-ND" H 10250 3650 50  0001 C CNN "SPN"
	1    9650 5250
	1    0    0    -1  
$EndComp
$Comp
L Resistor_User:2.2k,0603 R215
U 1 1 61C56E48
P 7250 1950
F 0 "R215" V 7250 1850 50  0000 L CNN
F 1 "2.2k,0603" H 7100 1450 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7350 1550 50  0001 C CNN
F 3 "~" H 7250 1950 50  0001 C CNN
F 4 "2.2k, 0603" V 7350 1700 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 7309 1859 50  0001 L CNN "Info"
	1    7250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0232
U 1 1 61C534B6
P 9650 6250
F 0 "#PWR0232" H 9650 6000 50  0001 C CNN
F 1 "GND" H 9655 6077 50  0000 C CNN
F 2 "" H 9650 6250 50  0001 C CNN
F 3 "" H 9650 6250 50  0001 C CNN
	1    9650 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0236
U 1 1 61C483F5
P 10400 4150
F 0 "#PWR0236" H 10400 3900 50  0001 C CNN
F 1 "GND" V 10405 4022 50  0000 R CNN
F 2 "" H 10400 4150 50  0001 C CNN
F 3 "" H 10400 4150 50  0001 C CNN
	1    10400 4150
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0231
U 1 1 61C4614E
P 9650 4050
F 0 "#PWR0231" H 9650 3900 50  0001 C CNN
F 1 "+3.3V" H 9665 4223 50  0000 C CNN
F 2 "" H 9650 4050 50  0001 C CNN
F 3 "" H 9650 4050 50  0001 C CNN
	1    9650 4050
	1    0    0    -1  
$EndComp
$Comp
L Capacitor_User:100nF,50V,X7R,0603 C205
U 1 1 61C40E54
P 9950 4150
F 0 "C205" V 9900 3900 50  0000 L CNN
F 1 "100nF,50V,X7R,0603" H 10050 3950 50  0001 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 10000 3750 50  0001 C CNN
F 3 "~" H 9950 4150 50  0001 C CNN
F 4 "100nF, 50V" V 9900 4200 50  0000 L CNN "Pretty Value 1"
F 5 "X7R, 0603" V 10000 4200 50  0000 L CNN "Pretty Value 2"
	1    9950 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	10400 4150 10050 4150
$Comp
L Capacitor_User:1uF,16V,X7R,0603 C201
U 1 1 61CB3C6F
P 3700 5300
F 0 "C201" H 3792 5391 50  0000 L CNN
F 1 "1uF,16V,X7R,0603" H 3850 5050 50  0001 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3750 4850 50  0001 C CNN
F 3 "~" H 3700 5300 50  0001 C CNN
F 4 "1μF, 16V" H 3792 5300 50  0000 L CNN "Pretty Value 1"
F 5 "X7R, 0603" H 3792 5209 50  0000 L CNN "Pretty Value 2"
	1    3700 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 5100 3700 5100
Wire Wire Line
	3700 5100 3700 5200
Wire Wire Line
	4350 5300 4350 5100
Connection ~ 4350 5100
$Comp
L Capacitor_User:1uF,16V,X7R,0603 C203
U 1 1 61CB5B2B
P 5350 5300
F 0 "C203" H 5442 5391 50  0000 L CNN
F 1 "1uF,16V,X7R,0603" H 5500 5050 50  0001 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5400 4850 50  0001 C CNN
F 3 "~" H 5350 5300 50  0001 C CNN
F 4 "1μF, 16V" H 5442 5300 50  0000 L CNN "Pretty Value 1"
F 5 "X7R, 0603" H 5442 5209 50  0000 L CNN "Pretty Value 2"
	1    5350 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 5100 5350 5200
Wire Wire Line
	5350 5400 5350 5600
$Comp
L power:GND #PWR0219
U 1 1 61CB89CB
P 5350 5600
F 0 "#PWR0219" H 5350 5350 50  0001 C CNN
F 1 "GND" H 5355 5427 50  0000 C CNN
F 2 "" H 5350 5600 50  0001 C CNN
F 3 "" H 5350 5600 50  0001 C CNN
	1    5350 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0217
U 1 1 61CB9868
P 4850 5600
F 0 "#PWR0217" H 4850 5350 50  0001 C CNN
F 1 "GND" H 4855 5427 50  0000 C CNN
F 2 "" H 4850 5600 50  0001 C CNN
F 3 "" H 4850 5600 50  0001 C CNN
	1    4850 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0212
U 1 1 61CB9C60
P 3700 5600
F 0 "#PWR0212" H 3700 5350 50  0001 C CNN
F 1 "GND" H 3705 5427 50  0000 C CNN
F 2 "" H 3700 5600 50  0001 C CNN
F 3 "" H 3700 5600 50  0001 C CNN
	1    3700 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5400 3700 5600
$Comp
L Fuse_User:0805L110SLxx F201
U 1 1 61CBD95A
P 2750 5100
F 0 "F201" V 2435 5100 50  0000 C CNN
F 1 "0805L110SLxx" V 2526 5100 50  0000 C CNN
F 2 "Fuse:Fuse_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2950 5650 50  0001 L CNN
F 3 "https://www.littelfuse.com/media?resourcetype=datasheets&itemid=2d55f293-08b4-4e1c-af6d-dbdeccdec69c&filename=littelfuse-ptc-low-rho-datasheet" H 2750 5100 50  0001 C CNN
F 4 "Littelfuse" H 2600 5600 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 2600 5650 50  0001 C CNN "Supplier"
F 6 "F3365CT-ND" H 2900 5850 50  0001 C CNN "SPN"
F 7 "1.1 A, 6V" V 2617 5100 50  0000 C CNN "Info"
	1    2750 5100
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0218
U 1 1 61CC0AE7
P 5350 5100
F 0 "#PWR0218" H 5350 4950 50  0001 C CNN
F 1 "+3.3V" H 5365 5273 50  0000 C CNN
F 2 "" H 5350 5100 50  0001 C CNN
F 3 "" H 5350 5100 50  0001 C CNN
	1    5350 5100
	1    0    0    -1  
$EndComp
Connection ~ 5350 5100
$Comp
L Interface_USB:MCP2221AxP U202
U 1 1 61C9EB16
P 4650 2750
F 0 "U202" H 4400 3400 50  0000 C CNN
F 1 "MCP2221AxP" H 5050 3400 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 4650 3750 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005565B.pdf" H 4650 3450 50  0001 C CNN
F 4 "Microchip" H 4650 2750 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 4650 2750 50  0001 C CNN "Supplier"
F 6 "MCP2221A-I/P-ND" H 4650 2750 50  0001 C CNN "SPN"
	1    4650 2750
	1    0    0    -1  
$EndComp
$Comp
L Resistor_User:10k,0603 R208
U 1 1 61CA0C37
P 3750 1800
F 0 "R208" H 3809 1846 50  0000 L CNN
F 1 "10k,0603" H 3600 1300 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3850 1400 50  0001 C CNN
F 3 "~" H 3750 1800 50  0001 C CNN
F 4 "10k, 0603" H 3809 1755 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 3850 1750 50  0001 L CNN "Info"
	1    3750 1800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_User:BSS138AKA Q202
U 1 1 61CA3D7A
P 3650 2250
F 0 "Q202" H 3854 2296 50  0000 L CNN
F 1 "BSS138BK" H 3854 2205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD_User:SOT-23_Handsoldering" H 3850 2175 50  0001 L CIN
F 3 "https://assets.nexperia.com/documents/data-sheet/BSS138BK.pdf" H 3650 2250 50  0001 L CNN
F 4 "Nexperia" H 3500 1750 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 3500 1650 50  0001 C CNN "Supplier"
F 6 "1727-1141-1-ND" H 4100 1650 50  0001 C CNN "SPN"
	1    3650 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2250 4250 2000
Wire Wire Line
	4250 2000 3750 2000
Wire Wire Line
	3750 2000 3750 1950
Wire Wire Line
	3750 2050 3750 2000
Connection ~ 3750 2000
$Comp
L power:+3.3V #PWR0213
U 1 1 61CB2294
P 3750 1650
F 0 "#PWR0213" H 3750 1500 50  0001 C CNN
F 1 "+3.3V" H 3765 1823 50  0000 C CNN
F 2 "" H 3750 1650 50  0001 C CNN
F 3 "" H 3750 1650 50  0001 C CNN
	1    3750 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0214
U 1 1 61CB308C
P 3750 2450
F 0 "#PWR0214" H 3750 2200 50  0001 C CNN
F 1 "GND" H 3755 2277 50  0000 C CNN
F 2 "" H 3750 2450 50  0001 C CNN
F 3 "" H 3750 2450 50  0001 C CNN
	1    3750 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 2250 3450 2050
$Comp
L Resistor_User:10k,0603 R205
U 1 1 61CBC458
P 2650 1900
F 0 "R205" V 2650 1800 50  0000 L CNN
F 1 "10k,0603" H 2500 1400 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2750 1500 50  0001 C CNN
F 3 "~" H 2650 1900 50  0001 C CNN
F 4 "10k, 0603" V 2550 1750 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 2750 1850 50  0001 L CNN "Info"
	1    2650 1900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0207
U 1 1 61CBCF4F
P 2400 1900
F 0 "#PWR0207" H 2400 1650 50  0001 C CNN
F 1 "GND" H 2405 1727 50  0000 C CNN
F 2 "" H 2400 1900 50  0001 C CNN
F 3 "" H 2400 1900 50  0001 C CNN
	1    2400 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0216
U 1 1 61CC256A
P 4650 3350
F 0 "#PWR0216" H 4650 3100 50  0001 C CNN
F 1 "GND" H 4655 3177 50  0000 C CNN
F 2 "" H 4650 3350 50  0001 C CNN
F 3 "" H 4650 3350 50  0001 C CNN
	1    4650 3350
	1    0    0    -1  
$EndComp
NoConn ~ 5050 2950
NoConn ~ 5050 3050
NoConn ~ 5050 3150
NoConn ~ 5050 2350
NoConn ~ 5050 2250
$Comp
L power:+3.3V #PWR0215
U 1 1 61CC9457
P 4650 1800
F 0 "#PWR0215" H 4650 1650 50  0001 C CNN
F 1 "+3.3V" H 4665 1973 50  0000 C CNN
F 2 "" H 4650 1800 50  0001 C CNN
F 3 "" H 4650 1800 50  0001 C CNN
	1    4650 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1800 4650 1850
Wire Wire Line
	4750 2050 4650 2050
Connection ~ 4650 2050
Wire Wire Line
	4950 1850 4650 1850
Connection ~ 4650 1850
Wire Wire Line
	4650 1850 4650 2050
$Comp
L power:GND #PWR0220
U 1 1 61CCFAC2
P 5550 1850
F 0 "#PWR0220" H 5550 1600 50  0001 C CNN
F 1 "GND" V 5555 1722 50  0000 R CNN
F 2 "" H 5550 1850 50  0001 C CNN
F 3 "" H 5550 1850 50  0001 C CNN
	1    5550 1850
	0    -1   -1   0   
$EndComp
$Comp
L Capacitor_User:470nF,25V,X7R,0603 C202
U 1 1 61CD1BBF
P 5050 1850
AR Path="/61CD1BBF" Ref="C202"  Part="1" 
AR Path="/61BDDDFC/61CD1BBF" Ref="C202"  Part="1" 
F 0 "C202" V 5100 2050 50  0000 C CNN
F 1 "470nF,25V,X7R,0603" H 5150 1650 50  0001 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5100 1450 50  0001 C CNN
F 3 "~" H 5050 1850 50  0001 C CNN
F 4 "470nF, 25V" V 5100 1600 50  0000 C CNN "Pretty Value 1"
F 5 "X7R, 0603" V 5000 1600 50  0000 C CNN "Pretty Value 2"
	1    5050 1850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5550 1850 5150 1850
$Comp
L Connector_User:2137160001 J201
U 1 1 61CA2A51
P 1050 2750
F 0 "J201" H 750 3500 50  0000 C CNN
F 1 "2137160001" H 1250 3500 50  0000 C CNN
F 2 "Connector_User:USB_C_Receptacle_Molex_2137160001" H 1200 2750 50  0001 C CNN
F 3 "https://www.molex.com/molex/products/part-detail/io_connectors/2137160001" H 1200 2750 50  0001 C CNN
F 4 "Molex" H 1050 2750 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 1157 3617 50  0001 C CNN "Supplier"
F 6 "900-2137160001CT-ND" H 1157 3526 50  0001 C CNN "SPN"
	1    1050 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2850 1650 2950
Wire Wire Line
	1650 2750 1650 2650
$Comp
L Resistor_User:5.1k,0603 R201
U 1 1 61CBF78D
P 2000 2350
F 0 "R201" V 2000 2350 50  0000 C CNN
F 1 "5.1k,0603" H 1850 1850 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2100 1950 50  0001 C CNN
F 3 "~" H 2000 2350 50  0001 C CNN
F 4 "5.1k, 0603" V 1900 2350 50  0000 C CNN "Pretty Value"
F 5 "1%, 0,1W" H 2100 2300 50  0001 L CNN "Info"
	1    2000 2350
	0    1    1    0   
$EndComp
$Comp
L Resistor_User:5.1k,0603 R202
U 1 1 61CC292F
P 2000 2450
F 0 "R202" V 2000 2450 50  0000 C CNN
F 1 "5.1k,0603" H 1850 1950 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2100 2050 50  0001 C CNN
F 3 "~" H 2000 2450 50  0001 C CNN
F 4 "5.1k, 0603" V 2100 2450 50  0000 C CNN "Pretty Value"
F 5 "1%, 0,1W" H 2100 2400 50  0001 L CNN "Info"
	1    2000 2450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0205
U 1 1 61CC94DC
P 2300 2450
F 0 "#PWR0205" H 2300 2200 50  0001 C CNN
F 1 "GND" H 2305 2277 50  0000 C CNN
F 2 "" H 2300 2450 50  0001 C CNN
F 3 "" H 2300 2450 50  0001 C CNN
	1    2300 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2350 2300 2450
NoConn ~ 1650 3250
NoConn ~ 1650 3350
$Comp
L power:GND #PWR0203
U 1 1 61CD1E1A
P 1050 3650
F 0 "#PWR0203" H 1050 3400 50  0001 C CNN
F 1 "GND" H 1055 3477 50  0000 C CNN
F 2 "" H 1050 3650 50  0001 C CNN
F 3 "" H 1050 3650 50  0001 C CNN
	1    1050 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0201
U 1 1 61CD278C
P 750 3650
F 0 "#PWR0201" H 750 3400 50  0001 C CNN
F 1 "GND" H 755 3477 50  0000 C CNN
F 2 "" H 750 3650 50  0001 C CNN
F 3 "" H 750 3650 50  0001 C CNN
	1    750  3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2750 2150 2750
Wire Wire Line
	4150 2850 4150 2650
Wire Wire Line
	4150 2650 4250 2650
Text Label 3400 2750 0    50   ~ 0
USB_D-
Text Label 3400 2850 0    50   ~ 0
USB_D+
Text Label 1800 2150 0    50   ~ 0
VUSB_UART
$Comp
L Connector_User:2137160001 J202
U 1 1 61D32A38
P 1050 5700
F 0 "J202" H 750 6450 50  0000 C CNN
F 1 "2137160001" H 1250 6450 50  0000 C CNN
F 2 "Connector_User:USB_C_Receptacle_Molex_2137160001" H 1200 5700 50  0001 C CNN
F 3 "https://www.molex.com/molex/products/part-detail/io_connectors/2137160001" H 1200 5700 50  0001 C CNN
F 4 "Molex" H 1050 5700 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 1157 6567 50  0001 C CNN "Supplier"
F 6 "900-2137160001CT-ND" H 1157 6476 50  0001 C CNN "SPN"
	1    1050 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0204
U 1 1 61D32A3E
P 1050 6600
F 0 "#PWR0204" H 1050 6350 50  0001 C CNN
F 1 "GND" H 1055 6427 50  0000 C CNN
F 2 "" H 1050 6600 50  0001 C CNN
F 3 "" H 1050 6600 50  0001 C CNN
	1    1050 6600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0202
U 1 1 61D32A44
P 750 6600
F 0 "#PWR0202" H 750 6350 50  0001 C CNN
F 1 "GND" H 755 6427 50  0000 C CNN
F 2 "" H 750 6600 50  0001 C CNN
F 3 "" H 750 6600 50  0001 C CNN
	1    750  6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 2150 2550 2150
NoConn ~ 1650 5900
NoConn ~ 1650 5800
NoConn ~ 1650 5700
NoConn ~ 1650 5600
NoConn ~ 1650 6200
NoConn ~ 1650 6300
Wire Wire Line
	2600 5100 2400 5100
Text Label 1750 5100 0    50   ~ 0
VUSB_POWER
Connection ~ 3700 5100
$Comp
L Power_Protection_User:824001 U201
U 1 1 61D92EE1
P 2550 3550
F 0 "U201" H 2300 3900 50  0000 C CNN
F 1 "824001" H 2750 3900 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6_Handsoldering" H 2550 3050 50  0001 C CNN
F 3 "https://www.we-online.com/catalog/datasheet/824001.pdf" H 2750 3900 50  0001 C CNN
F 4 "Wuerth Elektronik" H 2000 2650 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 2000 2550 50  0001 L CNN "Supplier"
F 6 "732-4464-1-ND" H 2400 2550 50  0001 L CNN "SPN"
	1    2550 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0208
U 1 1 61D9C3B1
P 2550 3950
F 0 "#PWR0208" H 2550 3700 50  0001 C CNN
F 1 "GND" H 2555 3777 50  0000 C CNN
F 2 "" H 2550 3950 50  0001 C CNN
F 3 "" H 2550 3950 50  0001 C CNN
	1    2550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3450 2150 2750
Connection ~ 2150 2750
Wire Wire Line
	2150 2750 4250 2750
Wire Wire Line
	2150 3650 2050 3650
Wire Wire Line
	2550 3150 2550 3050
Connection ~ 2550 2150
$Comp
L Power_Protection_User:824021 D201
U 1 1 61CB6C39
P 3050 5400
F 0 "D201" H 3255 5446 50  0000 L CNN
F 1 "824021" H 3255 5355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD_User:SOT-23_Handsoldering" H 3275 5350 50  0001 L CNN
F 3 "https://www.we-online.com/catalog/datasheet/824021.pdf" H 3175 5525 50  0001 C CNN
F 4 "Wurth Elektronik" H 3275 5250 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 3275 5150 50  0001 L CNN "Supplier"
F 6 "732-4468-1-ND" H 3650 5150 50  0001 L CNN "SPN"
	1    3050 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0209
U 1 1 61D85A2F
P 3050 5600
F 0 "#PWR0209" H 3050 5350 50  0001 C CNN
F 1 "GND" H 3055 5427 50  0000 C CNN
F 2 "" H 3050 5600 50  0001 C CNN
F 3 "" H 3050 5600 50  0001 C CNN
	1    3050 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 5100 2950 5100
Wire Wire Line
	2950 5200 2950 5100
Connection ~ 2950 5100
Wire Wire Line
	2950 5100 3700 5100
NoConn ~ 3150 5200
Wire Wire Line
	5050 2650 5450 2650
Text Label 6350 2550 0    50   ~ 0
MCU_TX
Text Label 6350 2650 0    50   ~ 0
MCU_RX
NoConn ~ 2950 3450
NoConn ~ 2950 3650
$Comp
L Resistor_User:10k,0603 R213
U 1 1 61DD0E4A
P 6300 1100
F 0 "R213" H 6359 1146 50  0000 L CNN
F 1 "10k,0603" H 6150 600 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6400 700 50  0001 C CNN
F 3 "~" H 6300 1100 50  0001 C CNN
F 4 "10k, 0603" H 6359 1055 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 6400 1050 50  0001 L CNN "Info"
F 6 "DNP" H 6350 950 50  0000 L CIB "DNP"
	1    6300 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 1350 6300 1250
$Comp
L power:+3.3V #PWR0222
U 1 1 61DD5066
P 6300 950
F 0 "#PWR0222" H 6300 800 50  0001 C CNN
F 1 "+3.3V" H 6315 1123 50  0000 C CNN
F 2 "" H 6300 950 50  0001 C CNN
F 3 "" H 6300 950 50  0001 C CNN
	1    6300 950 
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0221
U 1 1 61DCFE0B
P 5850 1050
F 0 "#PWR0221" H 5850 800 50  0001 C CNN
F 1 "GND" V 5855 922 50  0000 R CNN
F 2 "" H 5850 1050 50  0001 C CNN
F 3 "" H 5850 1050 50  0001 C CNN
	1    5850 1050
	0    -1   -1   0   
$EndComp
NoConn ~ 5850 1250
NoConn ~ 5850 1150
NoConn ~ 5850 1550
Text Label 5900 1350 0    50   ~ 0
MCU_RX
Text Label 5900 1450 0    50   ~ 0
MCU_TX
Wire Wire Line
	5850 1350 6300 1350
$Comp
L Connector_User:77311-118-06LF J203
U 1 1 61DA9CFE
P 5650 1250
F 0 "J203" H 5568 1667 50  0000 C CNN
F 1 "77311-118-06LF" H 5568 1576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 5650 750 50  0001 C CNN
F 3 "https://cdn.amphenol-icc.com/media/wysiwyg/files/drawing/77311.pdf" H 5650 1250 50  0001 C CNN
F 4 "Amphenol" H 5850 1250 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 5850 1150 50  0001 L CNN "Supplier"
F 6 "609-5682-ND" H 6250 1150 50  0001 L CNN "SPN"
F 7 "DNP" H 5650 850 50  0000 C CIB "DNP"
	1    5650 1250
	-1   0    0    -1  
$EndComp
$Comp
L Resistor_User:0R,0603 R209
U 1 1 61CBE6A6
P 5600 2550
F 0 "R209" V 5600 2550 50  0000 C CNN
F 1 "0R,0603" H 5700 2700 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5700 2150 50  0001 C CNN
F 3 "~" H 5700 2550 50  0001 C CNN
F 4 "0R, 0603" V 5500 2550 50  0000 C CNN "Pretty Value 1"
F 5 "1%, 0,1W" V 5485 2550 50  0001 C CNN "Pretty Value 2"
	1    5600 2550
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 2550 5050 2550
$Comp
L Resistor_User:0R,0603 R210
U 1 1 61CD49DC
P 5600 2650
F 0 "R210" V 5600 2650 50  0000 C CNN
F 1 "0R,0603" H 5700 2800 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5700 2250 50  0001 C CNN
F 3 "~" H 5700 2650 50  0001 C CNN
F 4 "0R, 0603" V 5700 2650 50  0000 C CNN "Pretty Value 1"
F 5 "1%, 0,1W" H 5700 2600 50  0001 L CNN "Pretty Value 2"
	1    5600 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	2150 2450 2300 2450
Connection ~ 2300 2450
Wire Wire Line
	2300 2350 2150 2350
Wire Wire Line
	1850 2350 1650 2350
Wire Wire Line
	1650 2450 1850 2450
$Comp
L Resistor_User:5.1k,0603 R203
U 1 1 61D01491
P 2000 5300
F 0 "R203" V 2000 5300 50  0000 C CNN
F 1 "5.1k,0603" H 1850 4800 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2100 4900 50  0001 C CNN
F 3 "~" H 2000 5300 50  0001 C CNN
F 4 "5.1k, 0603" V 1900 5300 50  0000 C CNN "Pretty Value"
F 5 "1%, 0,1W" H 2100 5250 50  0001 L CNN "Info"
	1    2000 5300
	0    1    1    0   
$EndComp
$Comp
L Resistor_User:5.1k,0603 R204
U 1 1 61D01499
P 2000 5400
F 0 "R204" V 2000 5400 50  0000 C CNN
F 1 "5.1k,0603" H 1850 4900 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2100 5000 50  0001 C CNN
F 3 "~" H 2000 5400 50  0001 C CNN
F 4 "5.1k, 0603" V 2100 5400 50  0000 C CNN "Pretty Value"
F 5 "1%, 0,1W" H 2100 5350 50  0001 L CNN "Info"
	1    2000 5400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0206
U 1 1 61D0149F
P 2300 5400
F 0 "#PWR0206" H 2300 5150 50  0001 C CNN
F 1 "GND" H 2305 5227 50  0000 C CNN
F 2 "" H 2300 5400 50  0001 C CNN
F 3 "" H 2300 5400 50  0001 C CNN
	1    2300 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5400 2300 5400
Wire Wire Line
	2300 5300 2150 5300
Wire Wire Line
	1850 5300 1650 5300
Wire Wire Line
	1650 5400 1850 5400
Wire Wire Line
	2300 5300 2300 5400
Connection ~ 2300 5400
$Comp
L Resistor_User:2.2k,0603 R214
U 1 1 61C59967
P 6900 1950
F 0 "R214" V 6900 1850 50  0000 L CNN
F 1 "2.2k,0603" H 6750 1450 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7000 1550 50  0001 C CNN
F 3 "~" H 6900 1950 50  0001 C CNN
F 4 "2.2k, 0603" V 7000 1750 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 6959 1859 50  0001 L CNN "Info"
	1    6900 1950
	1    0    0    -1  
$EndComp
Text Notes 650  1900 0    80   ~ 16
USB - UART only
Text Notes 650  4850 0    80   ~ 16
USB - Power only
$Comp
L Resistor_User:0R,0603 R207
U 1 1 620A7DAC
P 3100 3950
F 0 "R207" H 3170 4041 50  0000 L CNN
F 1 "0R,0603" H 3200 4100 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 3200 3550 50  0001 C CNN
F 3 "~" H 3200 3950 50  0001 C CNN
F 4 "0R, 0603" H 3170 3950 50  0000 L CNN "Pretty Value 1"
F 5 "1%, 0,1W" H 3200 3900 50  0001 L CNN "Pretty Value 2"
F 6 "DNP" H 3170 3859 50  0000 L CIB "DNP"
	1    3100 3950
	1    0    0    -1  
$EndComp
Text Label 1700 2750 0    50   ~ 0
USB_D-
Text Label 1700 2850 0    50   ~ 0
USB_D+
Wire Wire Line
	2400 5100 2400 4400
Wire Wire Line
	2400 4400 3100 4400
Wire Wire Line
	3100 4400 3100 4100
Connection ~ 2400 5100
Wire Wire Line
	2400 5100 1650 5100
Wire Wire Line
	3100 3800 3100 3050
Wire Wire Line
	3100 3050 2550 3050
Connection ~ 2550 3050
Wire Wire Line
	2550 3050 2550 2150
Text Label 2700 3050 0    50   ~ 0
VUSB_UART
Text Label 2550 4400 0    50   ~ 0
VUSB_POWER
Text Notes 3150 4200 0    50   ~ 0
For development purposes only!
Wire Wire Line
	2800 2150 2800 2250
$Comp
L Resistor_User:10k,0603 R206
U 1 1 61CB729B
P 3100 1800
F 0 "R206" H 3159 1846 50  0000 L CNN
F 1 "10k,0603" H 2950 1300 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3200 1400 50  0001 C CNN
F 3 "~" H 3100 1800 50  0001 C CNN
F 4 "10k, 0603" H 3159 1755 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 3200 1750 50  0001 L CNN "Info"
	1    3100 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0211
U 1 1 61CBBF60
P 3100 2450
F 0 "#PWR0211" H 3100 2200 50  0001 C CNN
F 1 "GND" H 3105 2277 50  0000 C CNN
F 2 "" H 3100 2450 50  0001 C CNN
F 3 "" H 3100 2450 50  0001 C CNN
	1    3100 2450
	1    0    0    -1  
$EndComp
Connection ~ 3100 2050
Wire Wire Line
	3450 2050 3100 2050
Wire Wire Line
	3100 1950 3100 2050
$Comp
L power:+3.3V #PWR0210
U 1 1 61CBAE94
P 3100 1650
F 0 "#PWR0210" H 3100 1500 50  0001 C CNN
F 1 "+3.3V" H 3115 1823 50  0000 C CNN
F 2 "" H 3100 1650 50  0001 C CNN
F 3 "" H 3100 1650 50  0001 C CNN
	1    3100 1650
	1    0    0    -1  
$EndComp
$Comp
L Transistor_User:BSS138AKA Q201
U 1 1 61CB559B
P 3000 2250
F 0 "Q201" H 3204 2296 50  0000 L CNN
F 1 "BSS138BK" H 3204 2205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD_User:SOT-23_Handsoldering" H 3200 2175 50  0001 L CIN
F 3 "https://assets.nexperia.com/documents/data-sheet/BSS138BK.pdf" H 3000 2250 50  0001 L CNN
F 4 "Nexperia" H 2850 1750 50  0001 C CNN "Manufacturer"
F 5 "Digi-Key" H 2850 1650 50  0001 C CNN "Supplier"
F 6 "1727-1141-1-ND" H 3450 1650 50  0001 C CNN "SPN"
	1    3000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1900 2400 1900
Wire Wire Line
	2550 2150 2800 2150
Wire Wire Line
	2800 1900 2800 2150
Connection ~ 2800 2150
Connection ~ 1650 2850
Connection ~ 1650 2750
Wire Wire Line
	1650 2850 2050 2850
Wire Wire Line
	2050 3650 2050 2850
Connection ~ 2050 2850
Wire Wire Line
	2050 2850 4150 2850
Text Notes 5450 700  0    80   ~ 16
Internal UART option
Text Label 7550 2350 0    50   ~ 0
SCL
Text Label 7550 2450 0    50   ~ 0
SDA
$Comp
L Resistor_User:10k,0603 R216
U 1 1 61C583F0
P 8650 4200
F 0 "R216" V 8650 4100 50  0000 L CNN
F 1 "10k,0603" H 8500 3700 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8750 3800 50  0001 C CNN
F 3 "~" H 8650 4200 50  0001 C CNN
F 4 "10k, 0603" V 8750 4000 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 8750 4150 50  0001 L CNN "Info"
	1    8650 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 4550 6900 2450
Wire Wire Line
	6900 4550 9150 4550
Wire Wire Line
	7250 4450 7250 2350
Wire Wire Line
	7250 4450 9150 4450
Wire Wire Line
	7250 2350 7700 2350
Wire Wire Line
	6900 2450 7700 2450
Wire Wire Line
	5750 2650 6300 2650
Wire Wire Line
	5750 2550 6200 2550
Text HLabel 10050 3500 2    50   Output ~ 0
~LED_OE
Text Notes 10350 1700 0    80   ~ 16
Programmer
Connection ~ 9750 2050
Wire Wire Line
	9750 2000 9750 2050
Wire Wire Line
	9850 3500 10050 3500
Connection ~ 9850 3500
Wire Wire Line
	9850 3450 9850 3500
Wire Wire Line
	9100 3500 9850 3500
$Comp
L Resistor_User:10k,0603 R217
U 1 1 61D7EADF
P 9750 1850
F 0 "R217" V 9750 1750 50  0000 L CNN
F 1 "10k,0603" H 9600 1350 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9850 1450 50  0001 C CNN
F 3 "~" H 9750 1850 50  0001 C CNN
F 4 "10k, 0603" V 9650 1650 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 9850 1800 50  0001 L CNN "Info"
	1    9750 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 2550 10350 2550
$Comp
L power:+3.3V #PWR0234
U 1 1 61D7DFC8
P 9950 1650
F 0 "#PWR0234" H 9950 1500 50  0001 C CNN
F 1 "+3.3V" H 9965 1823 50  0000 C CNN
F 2 "" H 9950 1650 50  0001 C CNN
F 3 "" H 9950 1650 50  0001 C CNN
	1    9950 1650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10600 2150 9950 2150
Wire Wire Line
	9750 2050 10600 2050
Wire Wire Line
	9750 2650 9750 2050
Wire Wire Line
	9100 2650 9750 2650
Wire Wire Line
	9100 2450 10600 2450
Wire Wire Line
	10600 2350 9100 2350
NoConn ~ 10350 2550
Text Label 10550 2550 2    50   ~ 0
PGM
Text Label 10550 2450 2    50   ~ 0
PGC
Text Label 10550 2350 2    50   ~ 0
PGD
Wire Wire Line
	10100 2250 10600 2250
$Comp
L power:GND #PWR0235
U 1 1 61D4C772
P 10100 2250
F 0 "#PWR0235" H 10100 2000 50  0001 C CNN
F 1 "GND" V 10105 2122 50  0000 R CNN
F 2 "" H 10100 2250 50  0001 C CNN
F 3 "" H 10100 2250 50  0001 C CNN
	1    10100 2250
	0    1    -1   0   
$EndComp
Text Label 10550 2150 2    50   ~ 0
VREF
Text Label 10550 2050 2    50   ~ 0
~MCLR
$Comp
L Connector_User:77311-118-06LF J204
U 1 1 61D1DEDE
P 10800 2250
F 0 "J204" H 10718 2667 50  0000 C CNN
F 1 "77311-118-06LF" H 10718 2576 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 10800 1750 50  0001 C CNN
F 3 "https://cdn.amphenol-icc.com/media/wysiwyg/files/drawing/77311.pdf" H 10800 2250 50  0001 C CNN
F 4 "Amphenol" H 11000 2250 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 11000 2150 50  0001 L CNN "Supplier"
F 6 "609-5682-ND" H 11400 2150 50  0001 L CNN "SPN"
	1    10800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 2050 8350 2050
Wire Wire Line
	9150 2050 8800 2050
$Comp
L power:GND #PWR0229
U 1 1 61CC58AD
P 9150 2050
F 0 "#PWR0229" H 9150 1800 50  0001 C CNN
F 1 "GND" V 9155 1922 50  0000 R CNN
F 2 "" H 9150 2050 50  0001 C CNN
F 3 "" H 9150 2050 50  0001 C CNN
	1    9150 2050
	0    -1   -1   0   
$EndComp
Connection ~ 8350 2050
$Comp
L power:+3.3V #PWR0226
U 1 1 61CC521E
P 8350 2050
F 0 "#PWR0226" H 8350 1900 50  0001 C CNN
F 1 "+3.3V" H 8365 2223 50  0000 C CNN
F 2 "" H 8350 2050 50  0001 C CNN
F 3 "" H 8350 2050 50  0001 C CNN
	1    8350 2050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8350 2050 8350 2150
$Comp
L Capacitor_User:100nF,50V,X7R,0603 C204
U 1 1 61CC1405
P 8700 2050
F 0 "C204" V 8650 1800 50  0000 L CNN
F 1 "100nF,50V,X7R,0603" H 8800 1850 50  0001 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 8750 1650 50  0001 C CNN
F 3 "~" H 8700 2050 50  0001 C CNN
F 4 "100nF, 50V" V 8650 2100 50  0000 L CNN "Pretty Value 1"
F 5 "X7R, 0603" V 8750 2100 50  0000 L CNN "Pretty Value 2"
	1    8700 2050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0227
U 1 1 61CDD206
P 8350 3050
F 0 "#PWR0227" H 8350 2800 50  0001 C CNN
F 1 "GND" H 8355 2877 50  0000 C CNN
F 2 "" H 8350 3050 50  0001 C CNN
F 3 "" H 8350 3050 50  0001 C CNN
	1    8350 3050
	-1   0    0    -1  
$EndComp
$Comp
L Resistor_User:10k,0603 R218
U 1 1 6213AFA7
P 9850 3300
F 0 "R218" H 9909 3346 50  0000 L CNN
F 1 "10k,0603" H 9700 2800 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9950 2900 50  0001 C CNN
F 3 "~" H 9850 3300 50  0001 C CNN
F 4 "10k, 0603" H 9909 3255 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 9950 3250 50  0001 L CNN "Info"
	1    9850 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0233
U 1 1 6213AFAD
P 9850 3150
F 0 "#PWR0233" H 9850 3000 50  0001 C CNN
F 1 "+3.3V" H 9865 3323 50  0000 C CNN
F 2 "" H 9850 3150 50  0001 C CNN
F 3 "" H 9850 3150 50  0001 C CNN
	1    9850 3150
	1    0    0    -1  
$EndComp
Text Label 9450 3500 0    50   ~ 0
~LED_OE
Wire Wire Line
	6900 2450 6900 2100
Connection ~ 6900 2450
Wire Wire Line
	7250 2350 7250 2100
Connection ~ 7250 2350
Wire Wire Line
	6900 1800 6900 1700
Wire Wire Line
	6900 1700 7250 1700
$Comp
L power:+3.3V #PWR0225
U 1 1 61D28741
P 7250 1700
F 0 "#PWR0225" H 7250 1550 50  0001 C CNN
F 1 "+3.3V" H 7265 1873 50  0000 C CNN
F 2 "" H 7250 1700 50  0001 C CNN
F 3 "" H 7250 1700 50  0001 C CNN
	1    7250 1700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6300 1350 6300 2650
Connection ~ 6300 1350
Connection ~ 6300 2650
Wire Wire Line
	6300 2650 7700 2650
Wire Wire Line
	6200 1450 6200 2550
Connection ~ 6200 2550
Wire Wire Line
	6200 2550 7700 2550
Text Label 7400 2650 0    50   ~ 0
MCU_RX
Text Label 7400 2550 0    50   ~ 0
MCU_TX
$Comp
L MCU_User:PIC16F18326-IP U204
U 1 1 61CA6355
P 8400 2650
F 0 "U204" H 8850 3050 50  0000 C CNN
F 1 "PIC16F18326-IP" H 8100 3050 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 8400 2350 50  0001 C CNN
F 3 "https://ww1.microchip.com/downloads/en/DeviceDoc/40001839E.pdf" H 8400 1950 50  0001 C CNN
F 4 "Digi-Key" H 8400 2650 50  0001 C CNN "Supplier"
F 5 "PIC16F18326-I/P-ND" H 8400 2650 50  0001 C CNN "SPN"
	1    8400 2650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7700 4750 8650 4750
$Comp
L power:+3.3V #PWR0228
U 1 1 61D65D56
P 8650 4050
F 0 "#PWR0228" H 8650 3900 50  0001 C CNN
F 1 "+3.3V" H 8665 4223 50  0000 C CNN
F 2 "" H 8650 4050 50  0001 C CNN
F 3 "" H 8650 4050 50  0001 C CNN
	1    8650 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7700 2850 7700 4750
Wire Wire Line
	8650 4350 8650 4750
Connection ~ 8650 4750
Wire Wire Line
	8650 4750 9150 4750
Wire Wire Line
	9950 1650 9750 1650
Wire Wire Line
	9750 1650 9750 1700
Wire Wire Line
	9950 1650 9950 2150
Connection ~ 9950 1650
$Comp
L LED_User:150060GS75000 D204
U 1 1 61CCE32D
P 10350 2750
F 0 "D204" H 10300 2650 50  0000 C CNN
F 1 "150060GS75000" H 9900 2700 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 10350 2925 50  0001 C CNN
F 3 "https://www.we-online.com/katalog/datasheet/150060GS75000.pdf" H 10300 2750 50  0001 C CNN
F 4 "Wurth Elektronik" H 9850 2450 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 9850 2350 50  0001 L CNN "Supplier"
F 6 "732-4971-1-ND" H 10250 2350 50  0001 L CNN "SPN"
F 7 "Green" H 10100 2800 50  0000 C CNN "Info"
	1    10350 2750
	-1   0    0    1   
$EndComp
Text Label 9200 2750 0    50   ~ 0
STATUS_LED
$Comp
L Resistor_User:2.2k,0603 R219
U 1 1 61CE528E
P 10000 2750
F 0 "R219" V 10000 2650 50  0000 L CNN
F 1 "2.2k,0603" H 9850 2250 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 10100 2350 50  0001 C CNN
F 3 "~" H 10000 2750 50  0001 C CNN
F 4 "2.2k, 0603" V 10100 2500 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 10059 2659 50  0001 L CNN "Info"
	1    10000 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	9100 2750 9850 2750
Wire Wire Line
	10150 2750 10250 2750
$Comp
L power:GND #PWR0237
U 1 1 61CF10D8
P 10800 2750
F 0 "#PWR0237" H 10800 2500 50  0001 C CNN
F 1 "GND" H 10805 2577 50  0000 C CNN
F 2 "" H 10800 2750 50  0001 C CNN
F 3 "" H 10800 2750 50  0001 C CNN
	1    10800 2750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	10550 2750 10800 2750
Wire Wire Line
	9100 2850 9100 3500
NoConn ~ 7700 2750
NoConn ~ 9100 2550
Wire Wire Line
	5850 1450 6200 1450
$Comp
L LED_User:150060GS75000 D202
U 1 1 61D2E093
P 5950 2850
F 0 "D202" H 5900 2750 50  0000 C CNN
F 1 "150060GS75000" H 5500 2800 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5950 3025 50  0001 C CNN
F 3 "https://www.we-online.com/katalog/datasheet/150060GS75000.pdf" H 5900 2850 50  0001 C CNN
F 4 "Wurth Elektronik" H 5450 2550 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 5450 2450 50  0001 L CNN "Supplier"
F 6 "732-4971-1-ND" H 5850 2450 50  0001 L CNN "SPN"
F 7 "Green" H 5700 2900 50  0000 C CNN "Info"
	1    5950 2850
	-1   0    0    1   
$EndComp
$Comp
L Resistor_User:2.2k,0603 R211
U 1 1 61D2E09B
P 5600 2850
F 0 "R211" V 5600 2750 50  0000 L CNN
F 1 "2.2k,0603" H 5450 2350 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5700 2450 50  0001 C CNN
F 3 "~" H 5600 2850 50  0001 C CNN
F 4 "2.2k, 0603" V 5700 2600 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 5659 2759 50  0001 L CNN "Info"
	1    5600 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 2850 5850 2850
$Comp
L power:GND #PWR0223
U 1 1 61D2E0A2
P 6400 2850
F 0 "#PWR0223" H 6400 2600 50  0001 C CNN
F 1 "GND" H 6405 2677 50  0000 C CNN
F 2 "" H 6400 2850 50  0001 C CNN
F 3 "" H 6400 2850 50  0001 C CNN
	1    6400 2850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6150 2850 6400 2850
Wire Wire Line
	5050 2850 5450 2850
$Comp
L LED_User:150060GS75000 D203
U 1 1 61D404F4
P 6500 5100
F 0 "D203" H 6450 5000 50  0000 C CNN
F 1 "150060GS75000" H 6050 5050 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6500 5275 50  0001 C CNN
F 3 "https://www.we-online.com/katalog/datasheet/150060GS75000.pdf" H 6450 5100 50  0001 C CNN
F 4 "Wurth Elektronik" H 6000 4800 50  0001 L CNN "Manufacturer"
F 5 "Digi-Key" H 6000 4700 50  0001 L CNN "Supplier"
F 6 "732-4971-1-ND" H 6400 4700 50  0001 L CNN "SPN"
F 7 "Green" H 6250 5150 50  0000 C CNN "Info"
	1    6500 5100
	-1   0    0    1   
$EndComp
$Comp
L Resistor_User:2.2k,0603 R212
U 1 1 61D404FC
P 6150 5100
F 0 "R212" V 6150 5000 50  0000 L CNN
F 1 "2.2k,0603" H 6000 4600 50  0001 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6250 4700 50  0001 C CNN
F 3 "~" H 6150 5100 50  0001 C CNN
F 4 "2.2k, 0603" V 6250 4850 50  0000 L CNN "Pretty Value"
F 5 "1%, 0,1W" H 6209 5009 50  0001 L CNN "Info"
	1    6150 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 5100 6400 5100
$Comp
L power:GND #PWR0224
U 1 1 61D40503
P 6950 5100
F 0 "#PWR0224" H 6950 4850 50  0001 C CNN
F 1 "GND" H 6955 4927 50  0000 C CNN
F 2 "" H 6950 5100 50  0001 C CNN
F 3 "" H 6950 5100 50  0001 C CNN
	1    6950 5100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6700 5100 6950 5100
Wire Wire Line
	5350 5100 6000 5100
Text HLabel 7150 2350 0    50   Output ~ 0
SCL
Text HLabel 6850 2450 0    50   BiDi ~ 0
SDA
Wire Wire Line
	6850 2450 6900 2450
Wire Wire Line
	7150 2350 7250 2350
Wire Wire Line
	7250 1700 7250 1800
Connection ~ 7250 1700
$Comp
L power:+5V #PWR0101
U 1 1 61D6AE7A
P 3700 5100
F 0 "#PWR0101" H 3700 4950 50  0001 C CNN
F 1 "+5V" H 3715 5273 50  0000 C CNN
F 2 "" H 3700 5100 50  0001 C CNN
F 3 "" H 3700 5100 50  0001 C CNN
	1    3700 5100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
