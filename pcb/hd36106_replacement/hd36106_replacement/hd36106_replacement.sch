EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR?
U 1 1 647E2303
P 5525 4225
AR Path="/63E0266A/647E2303" Ref="#PWR?"  Part="1" 
AR Path="/647E2303" Ref="#PWR0101"  Part="1" 
AR Path="/647698E0/647E2303" Ref="#PWR?"  Part="1" 
F 0 "#PWR0101" H 5525 3975 50  0001 C CNN
F 1 "GND" H 5375 4175 50  0000 C CNN
F 2 "" H 5525 4225 50  0001 C CNN
F 3 "" H 5525 4225 50  0001 C CNN
	1    5525 4225
	1    0    0    -1  
$EndComp
Text GLabel 6925 3000 2    50   Input ~ 0
6V4
Text GLabel 6800 1275 2    50   Input ~ 0
A0
Text GLabel 6800 1375 2    50   Input ~ 0
A1
Text GLabel 6800 1475 2    50   Input ~ 0
A2
Text GLabel 6800 1575 2    50   Input ~ 0
A3
Text GLabel 9700 2875 1    50   Input ~ 0
DIN
Text GLabel 6775 3200 2    50   Input ~ 0
DOUT_HD36106
Wire Wire Line
	6775 3000 6925 3000
$Comp
L Device:R_Small R11
U 1 1 647E2322
P 5100 3775
AR Path="/647E2322" Ref="R11"  Part="1" 
AR Path="/647698E0/647E2322" Ref="R?"  Part="1" 
F 0 "R11" H 5250 3825 50  0000 C CNN
F 1 "10k" H 5250 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5100 3775 50  0001 C CNN
F 3 "~" H 5100 3775 50  0001 C CNN
	1    5100 3775
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 647E2328
P 5100 4125
AR Path="/647E2328" Ref="R12"  Part="1" 
AR Path="/647698E0/647E2328" Ref="R?"  Part="1" 
F 0 "R12" H 5250 4175 50  0000 C CNN
F 1 "10k" H 5250 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 5100 4125 50  0001 C CNN
F 3 "~" H 5100 4125 50  0001 C CNN
	1    5100 4125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 647E232E
P 5100 4375
AR Path="/63E0266A/647E232E" Ref="#PWR?"  Part="1" 
AR Path="/647E232E" Ref="#PWR0102"  Part="1" 
AR Path="/647698E0/647E232E" Ref="#PWR?"  Part="1" 
F 0 "#PWR0102" H 5100 4125 50  0001 C CNN
F 1 "GND" H 4950 4325 50  0000 C CNN
F 2 "" H 5100 4375 50  0001 C CNN
F 3 "" H 5100 4375 50  0001 C CNN
	1    5100 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3875 5100 3950
Wire Wire Line
	5100 4225 5100 4375
$Comp
L Device:R_Small R9
U 1 1 647E2338
P 4750 3675
AR Path="/647E2338" Ref="R9"  Part="1" 
AR Path="/647698E0/647E2338" Ref="R?"  Part="1" 
F 0 "R9" H 4575 3800 50  0000 C CNN
F 1 "10k" H 4575 3725 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 4750 3675 50  0001 C CNN
F 3 "~" H 4750 3675 50  0001 C CNN
	1    4750 3675
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R10
U 1 1 647E233E
P 4750 4025
AR Path="/647E233E" Ref="R10"  Part="1" 
AR Path="/647698E0/647E233E" Ref="R?"  Part="1" 
F 0 "R10" H 4875 3975 50  0000 C CNN
F 1 "10k" H 4875 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 4750 4025 50  0001 C CNN
F 3 "~" H 4750 4025 50  0001 C CNN
	1    4750 4025
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 647E2344
P 4750 4375
AR Path="/63E0266A/647E2344" Ref="#PWR?"  Part="1" 
AR Path="/647E2344" Ref="#PWR0103"  Part="1" 
AR Path="/647698E0/647E2344" Ref="#PWR?"  Part="1" 
F 0 "#PWR0103" H 4750 4125 50  0001 C CNN
F 1 "GND" H 4600 4325 50  0000 C CNN
F 2 "" H 4750 4375 50  0001 C CNN
F 3 "" H 4750 4375 50  0001 C CNN
	1    4750 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3775 4750 3850
Wire Wire Line
	4750 4125 4750 4375
$Comp
L Device:R_Small R7
U 1 1 647E234E
P 3925 3575
AR Path="/647E234E" Ref="R7"  Part="1" 
AR Path="/647698E0/647E234E" Ref="R?"  Part="1" 
F 0 "R7" H 3800 3650 50  0000 C CNN
F 1 "10k" H 3825 3575 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3925 3575 50  0001 C CNN
F 3 "~" H 3925 3575 50  0001 C CNN
	1    3925 3575
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R8
U 1 1 647E2354
P 3925 3925
AR Path="/647E2354" Ref="R8"  Part="1" 
AR Path="/647698E0/647E2354" Ref="R?"  Part="1" 
F 0 "R8" V 3729 3925 50  0000 C CNN
F 1 "10k" V 3820 3925 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 3925 3925 50  0001 C CNN
F 3 "~" H 3925 3925 50  0001 C CNN
	1    3925 3925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 647E235A
P 3925 4375
AR Path="/63E0266A/647E235A" Ref="#PWR?"  Part="1" 
AR Path="/647E235A" Ref="#PWR0104"  Part="1" 
AR Path="/647698E0/647E235A" Ref="#PWR?"  Part="1" 
F 0 "#PWR0104" H 3925 4125 50  0001 C CNN
F 1 "GND" H 3775 4325 50  0000 C CNN
F 2 "" H 3925 4375 50  0001 C CNN
F 3 "" H 3925 4375 50  0001 C CNN
	1    3925 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	3925 3675 3925 3750
Wire Wire Line
	3925 4025 3925 4375
$Comp
L Device:R_Small R13
U 1 1 647E23A6
P 7400 3975
AR Path="/647E23A6" Ref="R13"  Part="1" 
AR Path="/647698E0/647E23A6" Ref="R?"  Part="1" 
F 0 "R13" V 7204 3975 50  0000 C CNN
F 1 "10k" V 7295 3975 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7400 3975 50  0001 C CNN
F 3 "~" H 7400 3975 50  0001 C CNN
	1    7400 3975
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R14
U 1 1 647E23AC
P 7400 4325
AR Path="/647E23AC" Ref="R14"  Part="1" 
AR Path="/647698E0/647E23AC" Ref="R?"  Part="1" 
F 0 "R14" V 7204 4325 50  0000 C CNN
F 1 "10k" V 7295 4325 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7400 4325 50  0001 C CNN
F 3 "~" H 7400 4325 50  0001 C CNN
	1    7400 4325
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 647E23B2
P 7400 4575
AR Path="/63E0266A/647E23B2" Ref="#PWR?"  Part="1" 
AR Path="/647E23B2" Ref="#PWR0108"  Part="1" 
AR Path="/647698E0/647E23B2" Ref="#PWR?"  Part="1" 
F 0 "#PWR0108" H 7400 4325 50  0001 C CNN
F 1 "GND" H 7250 4525 50  0000 C CNN
F 2 "" H 7400 4575 50  0001 C CNN
F 3 "" H 7400 4575 50  0001 C CNN
	1    7400 4575
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7400 3700 7400 3875
Wire Wire Line
	7400 4075 7400 4150
Wire Wire Line
	7400 4425 7400 4575
$Comp
L power:GND #PWR?
U 1 1 647E23C8
P 7775 4475
AR Path="/63E0266A/647E23C8" Ref="#PWR?"  Part="1" 
AR Path="/647E23C8" Ref="#PWR0109"  Part="1" 
AR Path="/647698E0/647E23C8" Ref="#PWR?"  Part="1" 
F 0 "#PWR0109" H 7775 4225 50  0001 C CNN
F 1 "GND" H 7625 4425 50  0000 C CNN
F 2 "" H 7775 4475 50  0001 C CNN
F 3 "" H 7775 4475 50  0001 C CNN
	1    7775 4475
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7775 3600 7775 3775
Wire Wire Line
	7775 3975 7775 4050
Wire Wire Line
	7775 4325 7775 4475
$Comp
L power:GND #PWR?
U 1 1 647E23DD
P 8150 4375
AR Path="/63E0266A/647E23DD" Ref="#PWR?"  Part="1" 
AR Path="/647E23DD" Ref="#PWR0110"  Part="1" 
AR Path="/647698E0/647E23DD" Ref="#PWR?"  Part="1" 
F 0 "#PWR0110" H 8150 4125 50  0001 C CNN
F 1 "GND" H 8000 4325 50  0000 C CNN
F 2 "" H 8150 4375 50  0001 C CNN
F 3 "" H 8150 4375 50  0001 C CNN
	1    8150 4375
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8150 3500 8150 3675
Wire Wire Line
	8150 3875 8150 3950
Wire Wire Line
	8150 4225 8150 4375
$Comp
L power:GND #PWR?
U 1 1 647E23F4
P 8525 4275
AR Path="/63E0266A/647E23F4" Ref="#PWR?"  Part="1" 
AR Path="/647E23F4" Ref="#PWR0111"  Part="1" 
AR Path="/647698E0/647E23F4" Ref="#PWR?"  Part="1" 
F 0 "#PWR0111" H 8525 4025 50  0001 C CNN
F 1 "GND" H 8375 4225 50  0000 C CNN
F 2 "" H 8525 4275 50  0001 C CNN
F 3 "" H 8525 4275 50  0001 C CNN
	1    8525 4275
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8525 3400 8525 3575
Wire Wire Line
	8525 3775 8525 3850
Wire Wire Line
	8525 4125 8525 4275
$Comp
L power:GND #PWR?
U 1 1 647E240A
P 8875 4175
AR Path="/63E0266A/647E240A" Ref="#PWR?"  Part="1" 
AR Path="/647E240A" Ref="#PWR0112"  Part="1" 
AR Path="/647698E0/647E240A" Ref="#PWR?"  Part="1" 
F 0 "#PWR0112" H 8875 3925 50  0001 C CNN
F 1 "GND" H 8725 4125 50  0000 C CNN
F 2 "" H 8875 4175 50  0001 C CNN
F 3 "" H 8875 4175 50  0001 C CNN
	1    8875 4175
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8875 3300 8875 3475
Wire Wire Line
	8875 3675 8875 3750
Wire Wire Line
	8875 4025 8875 4175
$Comp
L power:GND #PWR?
U 1 1 647E2436
P 9550 3975
AR Path="/63E0266A/647E2436" Ref="#PWR?"  Part="1" 
AR Path="/647E2436" Ref="#PWR0114"  Part="1" 
AR Path="/647698E0/647E2436" Ref="#PWR?"  Part="1" 
F 0 "#PWR0114" H 9550 3725 50  0001 C CNN
F 1 "GND" H 9400 3925 50  0000 C CNN
F 2 "" H 9550 3975 50  0001 C CNN
F 3 "" H 9550 3975 50  0001 C CNN
	1    9550 3975
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9550 3100 9550 3275
Wire Wire Line
	9550 3475 9550 3550
Wire Wire Line
	9550 3825 9550 3975
$Comp
L 4xxx_IEEE:40244 U?
U 2 1 647E2440
P 2650 5625
AR Path="/647698E0/647E2440" Ref="U?"  Part="2" 
AR Path="/647E2440" Ref="U2"  Part="2" 
F 0 "U2" H 2975 6275 50  0000 C CNN
F 1 "40244" H 3025 6200 50  0000 C CNN
F 2 "Package_SO:SO-20_12.8x7.5mm_P1.27mm" H 2650 5625 50  0001 C CNN
F 3 "" H 2650 5625 50  0001 C CNN
	2    2650 5625
	1    0    0    -1  
$EndComp
$Comp
L 4xxx_IEEE:40244 U?
U 1 1 647E2446
P 2625 6525
AR Path="/647698E0/647E2446" Ref="U?"  Part="1" 
AR Path="/647E2446" Ref="U2"  Part="1" 
F 0 "U2" H 2625 7091 50  0000 C CNN
F 1 "40244" H 2625 7000 50  0000 C CNN
F 2 "Package_SO:SO-20_12.8x7.5mm_P1.27mm" H 2625 6525 50  0001 C CNN
F 3 "" H 2625 6525 50  0001 C CNN
	1    2625 6525
	1    0    0    -1  
$EndComp
Text GLabel 3150 5525 2    50   Input ~ 0
DOUT_HD36106
Text GLabel 2900 4775 2    50   Input ~ 0
6V4
Wire Wire Line
	2900 4775 2750 4775
Wire Wire Line
	2750 4775 2750 5275
$Comp
L power:GND #PWR?
U 1 1 647E2452
P 2300 5000
AR Path="/63E0266A/647E2452" Ref="#PWR?"  Part="1" 
AR Path="/647E2452" Ref="#PWR0115"  Part="1" 
AR Path="/647698E0/647E2452" Ref="#PWR?"  Part="1" 
F 0 "#PWR0115" H 2300 4750 50  0001 C CNN
F 1 "GND" H 2150 4950 50  0000 C CNN
F 2 "" H 2300 5000 50  0001 C CNN
F 3 "" H 2300 5000 50  0001 C CNN
	1    2300 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 5275 2600 4825
Wire Wire Line
	2600 4825 2300 4825
Wire Wire Line
	2300 4825 2300 5000
$Comp
L ajm:rp2040-zero U1
U 1 1 647E76E6
P 5900 1075
F 0 "U1" H 6275 1140 50  0000 C CNN
F 1 "rp2040-zero" H 6275 1049 50  0000 C CNN
F 2 "ajm_kicad:rp2040-zero" H 5900 1075 50  0001 C CNN
F 3 "" H 5900 1075 50  0001 C CNN
	1    5900 1075
	1    0    0    -1  
$EndComp
Text GLabel 4100 2800 1    50   Input ~ 0
CLKB
Text GLabel 3725 2800 1    50   Input ~ 0
CLKA
Text GLabel 4925 2800 1    50   Input ~ 0
A3
Text GLabel 7550 2850 1    50   Input ~ 0
A2
Text GLabel 7925 2850 1    50   Input ~ 0
A1
Text GLabel 8300 2850 1    50   Input ~ 0
A0
Text GLabel 8675 2850 1    50   Input ~ 0
CE
Text GLabel 9025 2850 1    50   Input ~ 0
RW
$Comp
L Device:D_Small D3
U 1 1 6481EB35
P 5425 1275
F 0 "D3" H 5425 1068 50  0000 C CNN
F 1 "Schottky" H 5425 1159 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" V 5425 1275 50  0001 C CNN
F 3 "~" V 5425 1275 50  0001 C CNN
	1    5425 1275
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Small D2
U 1 1 6481F420
P 5050 1275
F 0 "D2" H 5050 1068 50  0000 C CNN
F 1 "Schottky" H 5050 1159 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" V 5050 1275 50  0001 C CNN
F 3 "~" V 5050 1275 50  0001 C CNN
	1    5050 1275
	-1   0    0    1   
$EndComp
Wire Wire Line
	5750 1275 5525 1275
Wire Wire Line
	5325 1275 5150 1275
$Comp
L Device:D_Small D1
U 1 1 648249D4
P 4700 1275
F 0 "D1" H 4700 1068 50  0000 C CNN
F 1 "4148" H 4700 1159 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" V 4700 1275 50  0001 C CNN
F 3 "~" V 4700 1275 50  0001 C CNN
	1    4700 1275
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 1275 4950 1275
Text GLabel 4450 1275 0    50   Input ~ 0
6V4
Wire Wire Line
	4450 1275 4600 1275
$Comp
L power:GND #PWR?
U 1 1 64831E17
P 4525 1425
AR Path="/63E0266A/64831E17" Ref="#PWR?"  Part="1" 
AR Path="/64831E17" Ref="#PWR0116"  Part="1" 
AR Path="/647698E0/64831E17" Ref="#PWR?"  Part="1" 
F 0 "#PWR0116" H 4525 1175 50  0001 C CNN
F 1 "GND" H 4375 1375 50  0000 C CNN
F 2 "" H 4525 1425 50  0001 C CNN
F 3 "" H 4525 1425 50  0001 C CNN
	1    4525 1425
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1375 4525 1375
Wire Wire Line
	4525 1375 4525 1425
$Comp
L Device:R_Small R27
U 1 1 6484CA05
P 2125 2125
AR Path="/6484CA05" Ref="R27"  Part="1" 
AR Path="/647698E0/6484CA05" Ref="R?"  Part="1" 
F 0 "R27" V 1929 2125 50  0000 C CNN
F 1 "10k" V 2020 2125 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2125 2125 50  0001 C CNN
F 3 "~" H 2125 2125 50  0001 C CNN
	1    2125 2125
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R28
U 1 1 6484CA0B
P 2125 2475
AR Path="/6484CA0B" Ref="R28"  Part="1" 
AR Path="/647698E0/6484CA0B" Ref="R?"  Part="1" 
F 0 "R28" V 1929 2475 50  0000 C CNN
F 1 "10k" V 2020 2475 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2125 2475 50  0001 C CNN
F 3 "~" H 2125 2475 50  0001 C CNN
	1    2125 2475
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 6484CA11
P 2125 2725
AR Path="/63E0266A/6484CA11" Ref="#PWR?"  Part="1" 
AR Path="/6484CA11" Ref="#PWR0117"  Part="1" 
AR Path="/647698E0/6484CA11" Ref="#PWR?"  Part="1" 
F 0 "#PWR0117" H 2125 2475 50  0001 C CNN
F 1 "GND" H 1975 2675 50  0000 C CNN
F 2 "" H 2125 2725 50  0001 C CNN
F 3 "" H 2125 2725 50  0001 C CNN
	1    2125 2725
	1    0    0    -1  
$EndComp
Wire Wire Line
	2125 1850 2125 2025
Wire Wire Line
	2125 2225 2125 2275
Wire Wire Line
	2125 2575 2125 2725
Text GLabel 5750 2075 0    50   Input ~ 0
CE2
Text GLabel 2325 2275 2    50   Input ~ 0
CE2
Text GLabel 6300 2275 3    50   Input ~ 0
DOUT_OE
Text GLabel 2150 5375 0    50   Input ~ 0
DOUT_EN
Text GLabel 2150 5525 0    50   Input ~ 0
DOUT_PICO
Wire Wire Line
	2325 2275 2125 2275
Connection ~ 2125 2275
Wire Wire Line
	2125 2275 2125 2375
Wire Wire Line
	3725 2800 3725 3750
Wire Wire Line
	3725 3750 3925 3750
Connection ~ 3925 3750
Wire Wire Line
	3925 3750 3925 3825
Wire Wire Line
	4100 2800 4100 3850
Wire Wire Line
	4100 3850 4350 3850
Connection ~ 4750 3850
Wire Wire Line
	4750 3850 4750 3925
Wire Wire Line
	4925 2800 4925 3950
Wire Wire Line
	4925 3950 5100 3950
Connection ~ 5100 3950
Wire Wire Line
	5100 3950 5100 4025
Wire Wire Line
	9700 2875 9700 3550
Wire Wire Line
	9700 3550 9550 3550
Connection ~ 9550 3550
Wire Wire Line
	9550 3550 9550 3625
Wire Wire Line
	9025 2850 9025 3750
Wire Wire Line
	9025 3750 8875 3750
Connection ~ 8875 3750
Wire Wire Line
	8875 3750 8875 3825
Wire Wire Line
	8675 2850 8675 3850
Wire Wire Line
	8675 3850 8525 3850
Connection ~ 8525 3850
Wire Wire Line
	8525 3850 8525 3925
Wire Wire Line
	8300 2850 8300 3950
Wire Wire Line
	8300 3950 8150 3950
Connection ~ 8150 3950
Wire Wire Line
	8150 3950 8150 4025
Wire Wire Line
	7925 2850 7925 4050
Wire Wire Line
	7925 4050 7775 4050
Connection ~ 7775 4050
Wire Wire Line
	7775 4050 7775 4125
Wire Wire Line
	7550 2850 7550 4150
Wire Wire Line
	7550 4150 7400 4150
Connection ~ 7400 4150
Wire Wire Line
	7400 4150 7400 4225
$Comp
L Connector:Conn_01x01_Male J1
U 1 1 648B3799
P 1925 1850
F 0 "J1" H 2033 2031 50  0000 C CNN
F 1 "Conn_01x01_Male" H 2033 1940 50  0000 C CNN
F 2 "TestPoint:TestPoint_Pad_2.0x2.0mm" H 1925 1850 50  0001 C CNN
F 3 "~" H 1925 1850 50  0001 C CNN
	1    1925 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 5625 2050 5625
Wire Wire Line
	2050 5625 2050 5725
Wire Wire Line
	2050 5725 2150 5725
Wire Wire Line
	2050 5725 2050 5825
Wire Wire Line
	2050 5825 2150 5825
Connection ~ 2050 5725
Wire Wire Line
	2050 5825 2050 6275
Wire Wire Line
	2050 6725 2125 6725
Connection ~ 2050 5825
Wire Wire Line
	2125 6625 2050 6625
Connection ~ 2050 6625
Wire Wire Line
	2050 6625 2050 6725
Wire Wire Line
	2125 6525 2050 6525
Connection ~ 2050 6525
Wire Wire Line
	2050 6525 2050 6625
Wire Wire Line
	2125 6425 2050 6425
Connection ~ 2050 6425
Wire Wire Line
	2050 6425 2050 6525
Wire Wire Line
	2125 6275 2050 6275
Connection ~ 2050 6275
Wire Wire Line
	2050 6275 2050 6425
$Comp
L power:GND #PWR?
U 1 1 648EC6DF
P 2050 6800
AR Path="/63E0266A/648EC6DF" Ref="#PWR?"  Part="1" 
AR Path="/648EC6DF" Ref="#PWR0105"  Part="1" 
AR Path="/647698E0/648EC6DF" Ref="#PWR?"  Part="1" 
F 0 "#PWR0105" H 2050 6550 50  0001 C CNN
F 1 "GND" H 1900 6750 50  0000 C CNN
F 2 "" H 2050 6800 50  0001 C CNN
F 3 "" H 2050 6800 50  0001 C CNN
	1    2050 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 6725 2050 6800
Connection ~ 2050 6725
$Comp
L ajm:HD36106 U3
U 1 1 64A16A79
P 6025 2950
F 0 "U3" H 6250 3165 50  0000 C CNN
F 1 "HD36106" H 6250 3074 50  0000 C CNN
F 2 "ajm_kicad:tc5006p" H 6025 2950 50  0001 C CNN
F 3 "" H 6025 2950 50  0001 C CNN
	1    6025 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5725 3000 5525 3000
Wire Wire Line
	5525 3000 5525 4225
Wire Wire Line
	5725 3600 5100 3600
Wire Wire Line
	5100 3600 5100 3675
Wire Wire Line
	5725 3500 4750 3500
Wire Wire Line
	4750 3500 4750 3575
Wire Wire Line
	5725 3400 3925 3400
Wire Wire Line
	3925 3400 3925 3475
$Comp
L Device:D_Schottky D5
U 1 1 64A55EA4
P 4350 4000
F 0 "D5" V 4304 4080 50  0000 L CNN
F 1 "SS36" V 4395 4080 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 4350 4000 50  0001 C CNN
F 3 "~" H 4350 4000 50  0001 C CNN
	1    4350 4000
	0    1    1    0   
$EndComp
Connection ~ 4350 3850
Wire Wire Line
	4350 3850 4750 3850
$Comp
L power:GND #PWR?
U 1 1 64A74135
P 4350 4375
AR Path="/63E0266A/64A74135" Ref="#PWR?"  Part="1" 
AR Path="/64A74135" Ref="#PWR0106"  Part="1" 
AR Path="/647698E0/64A74135" Ref="#PWR?"  Part="1" 
F 0 "#PWR0106" H 4350 4125 50  0001 C CNN
F 1 "GND" H 4200 4325 50  0000 C CNN
F 2 "" H 4350 4375 50  0001 C CNN
F 3 "" H 4350 4375 50  0001 C CNN
	1    4350 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4150 4350 4375
Wire Wire Line
	6775 3700 7400 3700
Wire Wire Line
	6775 3600 7775 3600
Wire Wire Line
	6775 3500 8150 3500
Wire Wire Line
	6775 3400 8525 3400
Wire Wire Line
	6775 3300 8875 3300
Wire Wire Line
	6775 3100 9550 3100
$Comp
L Device:D_Schottky D4
U 1 1 64AA8B30
P 3450 4000
F 0 "D4" V 3404 4080 50  0000 L CNN
F 1 "SS36" V 3495 4080 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123" H 3450 4000 50  0001 C CNN
F 3 "~" H 3450 4000 50  0001 C CNN
	1    3450 4000
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 64AA8B36
P 3450 4375
AR Path="/63E0266A/64AA8B36" Ref="#PWR?"  Part="1" 
AR Path="/64AA8B36" Ref="#PWR0107"  Part="1" 
AR Path="/647698E0/64AA8B36" Ref="#PWR?"  Part="1" 
F 0 "#PWR0107" H 3450 4125 50  0001 C CNN
F 1 "GND" H 3300 4325 50  0000 C CNN
F 2 "" H 3450 4375 50  0001 C CNN
F 3 "" H 3450 4375 50  0001 C CNN
	1    3450 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4150 3450 4375
Wire Wire Line
	3725 3750 3450 3750
Wire Wire Line
	3450 3750 3450 3850
Connection ~ 3725 3750
Text GLabel 6800 1675 2    50   Input ~ 0
DIN
Text GLabel 6800 1775 2    50   Input ~ 0
RW
Text GLabel 6800 1875 2    50   Input ~ 0
CE
Text GLabel 6800 2075 2    50   Input ~ 0
CLKB
Text GLabel 6200 2275 3    50   Input ~ 0
DOUT_PICO
Text GLabel 6500 2275 3    50   Input ~ 0
CLKA
Text Notes 4925 5425 0    50   ~ 0
CLKA and CLK B are at VB (-14V) levels, but we use VA (-6V4) as our GND for\nthe RP2040 zero. This means that VA referenced signals are 0-6V4 and we\ndivide them by two to get 0-3V2 for the RP2040. VB referenced signals are\nseen as -7V6 to +6V4, which we divide by 2 to get -3V8 to 3V2, which we clip\nwith a diode to get -0V3 to 3V2.\n
$Comp
L Device:R_Small R2
U 1 1 64AC008C
P 7775 4225
AR Path="/64AC008C" Ref="R2"  Part="1" 
AR Path="/647698E0/64AC008C" Ref="R?"  Part="1" 
F 0 "R2" V 7579 4225 50  0000 C CNN
F 1 "10k" V 7670 4225 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7775 4225 50  0001 C CNN
F 3 "~" H 7775 4225 50  0001 C CNN
	1    7775 4225
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 64AC370C
P 7775 3875
AR Path="/64AC370C" Ref="R1"  Part="1" 
AR Path="/647698E0/64AC370C" Ref="R?"  Part="1" 
F 0 "R1" V 7579 3875 50  0000 C CNN
F 1 "10k" V 7670 3875 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 7775 3875 50  0001 C CNN
F 3 "~" H 7775 3875 50  0001 C CNN
	1    7775 3875
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 64AC6D61
P 8150 3775
AR Path="/64AC6D61" Ref="R3"  Part="1" 
AR Path="/647698E0/64AC6D61" Ref="R?"  Part="1" 
F 0 "R3" V 7954 3775 50  0000 C CNN
F 1 "10k" V 8045 3775 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8150 3775 50  0001 C CNN
F 3 "~" H 8150 3775 50  0001 C CNN
	1    8150 3775
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 64ACA3AC
P 8150 4125
AR Path="/64ACA3AC" Ref="R4"  Part="1" 
AR Path="/647698E0/64ACA3AC" Ref="R?"  Part="1" 
F 0 "R4" V 7954 4125 50  0000 C CNN
F 1 "10k" V 8045 4125 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8150 4125 50  0001 C CNN
F 3 "~" H 8150 4125 50  0001 C CNN
	1    8150 4125
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R6
U 1 1 64ACD9DD
P 8525 4025
AR Path="/64ACD9DD" Ref="R6"  Part="1" 
AR Path="/647698E0/64ACD9DD" Ref="R?"  Part="1" 
F 0 "R6" V 8329 4025 50  0000 C CNN
F 1 "10k" V 8420 4025 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8525 4025 50  0001 C CNN
F 3 "~" H 8525 4025 50  0001 C CNN
	1    8525 4025
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R5
U 1 1 64AD108A
P 8525 3675
AR Path="/64AD108A" Ref="R5"  Part="1" 
AR Path="/647698E0/64AD108A" Ref="R?"  Part="1" 
F 0 "R5" V 8329 3675 50  0000 C CNN
F 1 "10k" V 8420 3675 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8525 3675 50  0001 C CNN
F 3 "~" H 8525 3675 50  0001 C CNN
	1    8525 3675
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R15
U 1 1 64AD4627
P 8875 3575
AR Path="/64AD4627" Ref="R15"  Part="1" 
AR Path="/647698E0/64AD4627" Ref="R?"  Part="1" 
F 0 "R15" V 8679 3575 50  0000 C CNN
F 1 "10k" V 8770 3575 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8875 3575 50  0001 C CNN
F 3 "~" H 8875 3575 50  0001 C CNN
	1    8875 3575
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R16
U 1 1 64AD7D8D
P 8875 3925
AR Path="/64AD7D8D" Ref="R16"  Part="1" 
AR Path="/647698E0/64AD7D8D" Ref="R?"  Part="1" 
F 0 "R16" V 8679 3925 50  0000 C CNN
F 1 "10k" V 8770 3925 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 8875 3925 50  0001 C CNN
F 3 "~" H 8875 3925 50  0001 C CNN
	1    8875 3925
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R18
U 1 1 64ADB336
P 9550 3725
AR Path="/64ADB336" Ref="R18"  Part="1" 
AR Path="/647698E0/64ADB336" Ref="R?"  Part="1" 
F 0 "R18" V 9354 3725 50  0000 C CNN
F 1 "10k" V 9445 3725 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 9550 3725 50  0001 C CNN
F 3 "~" H 9550 3725 50  0001 C CNN
	1    9550 3725
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_Small R17
U 1 1 64ADEA4E
P 9550 3375
AR Path="/64ADEA4E" Ref="R17"  Part="1" 
AR Path="/647698E0/64ADEA4E" Ref="R?"  Part="1" 
F 0 "R17" V 9354 3375 50  0000 C CNN
F 1 "10k" V 9445 3375 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 9550 3375 50  0001 C CNN
F 3 "~" H 9550 3375 50  0001 C CNN
	1    9550 3375
	-1   0    0    -1  
$EndComp
$EndSCHEMATC
