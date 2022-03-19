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
L Regulator_Linear:XC6206PxxxMR U1
U 1 1 616EB372
P 4550 1300
F 0 "U1" H 4550 1542 50  0000 C CNN
F 1 "XC6206P332MR" H 4550 1451 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 1525 50  0001 C CIN
F 3 "https://www.torexsemi.com/file/xc6206/XC6206.pdf" H 4550 1300 50  0001 C CNN
F 4 "XC6206P332MR-G-ND" H 4550 1300 50  0001 C CNN "Digikey SKU"
	1    4550 1300
	1    0    0    -1  
$EndComp
$Comp
L OLED_Components:OLED-Display U3
U 1 1 616ED016
P 7400 1300
F 0 "U3" H 7950 1255 50  0000 C CNN
F 1 "OLED-Display" H 7950 1164 50  0000 C CNN
F 2 "LidPCB:FPC_Soldering_P0.65x25Pin_OLED" H 7400 1300 50  0001 C CNN
F 3 "https://www.buydisplay.com/download/manual/ER-OLED015-3_Datasheet.pdf" H 7400 1300 50  0001 C CNN
F 4 "ER-OLED015-3W" H 7950 1073 50  0000 C CNN "Feld4"
	1    7400 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR01
U 1 1 616EF85C
P 4050 1300
F 0 "#PWR01" H 4050 1150 50  0001 C CNN
F 1 "+5V" H 4065 1473 50  0000 C CNN
F 2 "" H 4050 1300 50  0001 C CNN
F 3 "" H 4050 1300 50  0001 C CNN
	1    4050 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 616F11CF
P 4550 1750
F 0 "#PWR02" H 4550 1500 50  0001 C CNN
F 1 "GND" H 4555 1577 50  0000 C CNN
F 2 "" H 4550 1750 50  0001 C CNN
F 3 "" H 4550 1750 50  0001 C CNN
	1    4550 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1750 4550 1700
$Comp
L Device:R R2
U 1 1 61702B28
P 5850 2800
F 0 "R2" H 5920 2846 50  0000 L CNN
F 1 "10k" H 5920 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5780 2800 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 5850 2800 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 5850 2800 50  0001 C CNN "Digikey SKU"
	1    5850 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 61704EDB
P 6100 2800
F 0 "R3" H 6170 2846 50  0000 L CNN
F 1 "10k" H 6170 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6030 2800 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 6100 2800 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 6100 2800 50  0001 C CNN "Digikey SKU"
	1    6100 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 617051EB
P 6350 2800
F 0 "R4" H 6420 2846 50  0000 L CNN
F 1 "10k" H 6420 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6280 2800 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 6350 2800 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 6350 2800 50  0001 C CNN "Digikey SKU"
	1    6350 2800
	1    0    0    -1  
$EndComp
Text GLabel 5200 1300 2    50   Output ~ 0
+3.3V
Text GLabel 5350 2650 0    50   Input ~ 0
+3.3V
Connection ~ 5850 2650
Wire Wire Line
	5850 2650 6100 2650
Connection ~ 6100 2650
Wire Wire Line
	6100 2650 6350 2650
Wire Wire Line
	5850 3400 5850 2950
Wire Wire Line
	6100 2950 6100 3200
Wire Wire Line
	7350 3600 7350 3350
Wire Wire Line
	7350 3350 7550 3350
Wire Wire Line
	6350 3300 6350 2950
Text GLabel 8500 2000 2    50   Input ~ 0
SCK
Text GLabel 5100 3400 0    50   Input ~ 0
SDA
Text GLabel 8500 2150 2    50   Input ~ 0
SDA
Wire Wire Line
	8500 2150 8350 2150
Wire Wire Line
	8350 2450 8350 2600
Connection ~ 8350 2600
Wire Wire Line
	8350 2600 8350 2750
Connection ~ 8350 2750
Wire Wire Line
	8350 2750 8350 2900
Connection ~ 8350 2900
Wire Wire Line
	8350 2900 8350 3050
Wire Wire Line
	8350 3050 8450 3050
Wire Wire Line
	8450 3050 8450 3350
Wire Wire Line
	8450 3350 8350 3350
Connection ~ 8350 3050
$Comp
L power:GND #PWR09
U 1 1 617101ED
P 8450 3550
F 0 "#PWR09" H 8450 3300 50  0001 C CNN
F 1 "GND" H 8455 3377 50  0000 C CNN
F 2 "" H 8450 3550 50  0001 C CNN
F 3 "" H 8450 3550 50  0001 C CNN
	1    8450 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 3350 8450 3550
Connection ~ 8450 3350
Wire Wire Line
	8350 2000 8500 2000
Wire Wire Line
	8350 1700 8350 1850
$Comp
L power:GND #PWR010
U 1 1 61712487
P 8600 1650
F 0 "#PWR010" H 8600 1400 50  0001 C CNN
F 1 "GND" H 8605 1477 50  0000 C CNN
F 2 "" H 8600 1650 50  0001 C CNN
F 3 "" H 8600 1650 50  0001 C CNN
	1    8600 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 1700 8350 1650
Wire Wire Line
	8350 1650 8600 1650
Connection ~ 8350 1700
$Comp
L power:GND #PWR04
U 1 1 6171389F
P 6500 2350
F 0 "#PWR04" H 6500 2100 50  0001 C CNN
F 1 "GND" H 6505 2177 50  0000 C CNN
F 2 "" H 6500 2350 50  0001 C CNN
F 3 "" H 6500 2350 50  0001 C CNN
	1    6500 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 2150 7500 2150
Wire Wire Line
	7500 2150 7500 2750
Wire Wire Line
	7500 2750 7550 2750
Wire Wire Line
	7500 2750 7500 2900
Wire Wire Line
	7500 2900 7550 2900
Connection ~ 7500 2750
$Comp
L power:GND #PWR06
U 1 1 6171698D
P 6850 3750
F 0 "#PWR06" H 6850 3500 50  0001 C CNN
F 1 "GND" H 6855 3577 50  0000 C CNN
F 2 "" H 6850 3750 50  0001 C CNN
F 3 "" H 6850 3750 50  0001 C CNN
	1    6850 3750
	1    0    0    -1  
$EndComp
Text GLabel 6650 5300 0    50   Input ~ 0
+3.3V
$Comp
L OLED_Components:R1200N002A-TR-FE U2
U 1 1 617282EA
P 7250 4700
F 0 "U2" H 7550 4765 50  0000 C CNN
F 1 "R1200N002A-TR-FE" H 7550 4674 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6_Handsoldering" H 7250 4700 50  0001 C CNN
F 3 "https://www.n-redc.co.jp/en/pdf/datasheet/r1200-ea.pdf" H 7250 4700 50  0001 C CNN
F 4 "2129-R1200N002A-TR-FETR-ND" H 7250 4700 50  0001 C CNN "Digikey SKU"
	1    7250 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4900 7000 4900
Wire Wire Line
	7000 4900 7000 5300
Connection ~ 7000 5300
Wire Wire Line
	7000 5300 7200 5300
$Comp
L Device:C C5
U 1 1 6172AA5B
P 6750 5450
F 0 "C5" H 6550 5500 50  0000 L CNN
F 1 "1u" H 6550 5400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6788 5300 50  0001 C CNN
F 3 "~" H 6750 5450 50  0001 C CNN
F 4 "587-6338-1-ND" H 6750 5450 50  0001 C CNN "Digikey SKU"
	1    6750 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 5300 6750 5300
Connection ~ 6750 5300
$Comp
L Device:L L1
U 1 1 6172EF01
P 7550 5600
F 0 "L1" V 7650 5600 50  0000 C CNN
F 1 "22uH" V 7750 5600 50  0000 C CNN
F 2 "LidPCB:IND_LQH2MCN220K02L" H 7550 5600 50  0001 C CNN
F 3 "https://search.murata.co.jp/Ceramy/image/img/P02/JELF243A-0053.pdf" H 7550 5600 50  0001 C CNN
F 4 "490-4048-2-ND" V 7550 5600 50  0001 C CNN "Digikey SKU"
	1    7550 5600
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 5300 7000 5300
Wire Wire Line
	7000 5300 7000 5600
Wire Wire Line
	7000 5600 7400 5600
Wire Wire Line
	7700 5600 7950 5600
Wire Wire Line
	7950 5600 7950 5300
Wire Wire Line
	7950 5300 7900 5300
$Comp
L power:GND #PWR05
U 1 1 61738B32
P 6750 5750
F 0 "#PWR05" H 6750 5500 50  0001 C CNN
F 1 "GND" H 6755 5577 50  0000 C CNN
F 2 "" H 6750 5750 50  0001 C CNN
F 3 "" H 6750 5750 50  0001 C CNN
	1    6750 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 5600 6750 5750
$Comp
L power:GND #PWR08
U 1 1 61739FD4
P 8350 5300
F 0 "#PWR08" H 8350 5050 50  0001 C CNN
F 1 "GND" H 8355 5127 50  0000 C CNN
F 2 "" H 8350 5300 50  0001 C CNN
F 3 "" H 8350 5300 50  0001 C CNN
	1    8350 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5100 8350 5100
Wire Wire Line
	8350 5100 8350 5250
$Comp
L Device:R R6
U 1 1 6173B9DA
P 8650 4700
F 0 "R6" H 8720 4746 50  0000 L CNN
F 1 "140k" H 8720 4655 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8580 4700 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 8650 4700 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 8650 4700 50  0001 C CNN "Digikey SKU"
	1    8650 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 6173CA35
P 8650 5100
F 0 "R7" H 8720 5146 50  0000 L CNN
F 1 "10k" H 8720 5055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 8580 5100 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rmcf_rmcp.pdf" H 8650 5100 50  0001 C CNN
F 4 "RMCF0603FT140KCT-ND" H 8650 5100 50  0001 C CNN "Digikey SKU"
	1    8650 5100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 6173D056
P 9050 4700
F 0 "R8" H 9120 4746 50  0000 L CNN
F 1 "2k" H 9120 4655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8980 4700 50  0001 C CNN
F 3 "~" H 9050 4700 50  0001 C CNN
F 4 "A126358CT-ND" H 9050 4700 50  0001 C CNN "Digikey SKU"
	1    9050 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 6173D684
P 9050 5100
F 0 "C8" H 8850 5150 50  0000 L CNN
F 1 "22pF" H 8750 5050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9088 4950 50  0001 C CNN
F 3 "https://datasheets.avx.com/LD-Series.pdf" H 9050 5100 50  0001 C CNN
F 4 "478-12912-2-ND" H 9050 5100 50  0001 C CNN "Digikey SKU"
	1    9050 5100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7200 5100 6900 5100
Wire Wire Line
	6900 5100 6900 4550
Wire Wire Line
	6900 4550 8650 4550
Wire Wire Line
	8650 4550 9050 4550
Connection ~ 8650 4550
Wire Wire Line
	8650 4850 8650 4900
Wire Wire Line
	8650 5250 8350 5250
Connection ~ 8350 5250
Wire Wire Line
	8350 5250 8350 5300
Wire Wire Line
	8650 4900 7900 4900
Connection ~ 8650 4900
Wire Wire Line
	8650 4900 8650 4950
Wire Wire Line
	9050 4850 9050 4950
Text GLabel 9450 4550 2    50   Output ~ 0
Vcc
Wire Wire Line
	9050 4550 9450 4550
Connection ~ 9050 4550
Wire Wire Line
	9050 5250 8650 5250
Connection ~ 8650 5250
Text GLabel 6100 1850 0    50   Input ~ 0
Vcc
Wire Wire Line
	4050 1300 4250 1300
Text GLabel 9000 3200 2    50   Input ~ 0
Vcc
$Comp
L Device:C C3
U 1 1 61763C24
P 6300 2000
F 0 "C3" H 6050 1950 50  0000 L CNN
F 1 "4.7u" H 6000 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6338 1850 50  0001 C CNN
F 3 "https://datasheets.avx.com/LD-Series.pdf" H 6300 2000 50  0001 C CNN
F 4 "478-12912-2-ND" H 6300 2000 50  0001 C CNN "Digikey SKU"
	1    6300 2000
	1    0    0    1   
$EndComp
Wire Wire Line
	7550 1450 7400 1450
Wire Wire Line
	7400 1450 7400 1500
Wire Wire Line
	7550 1450 7550 1700
$Comp
L Device:C C4
U 1 1 61771D47
P 6600 2150
F 0 "C4" H 6700 2100 50  0000 L CNN
F 1 "4.7u" H 6700 2200 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6638 2000 50  0001 C CNN
F 3 "https://datasheets.avx.com/LD-Series.pdf" H 6600 2150 50  0001 C CNN
F 4 "478-12912-2-ND" H 6600 2150 50  0001 C CNN "Digikey SKU"
	1    6600 2150
	1    0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 61772503
P 7200 2300
F 0 "C7" V 7450 2250 50  0000 L CNN
F 1 "100n" V 7350 2200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7238 2150 50  0001 C CNN
F 3 "https://datasheets.avx.com/U-Series.pdf" H 7200 2300 50  0001 C CNN
F 4 "478-10836-1-ND" H 7200 2300 50  0001 C CNN "Digikey SKU"
	1    7200 2300
	0    1    -1   0   
$EndComp
Wire Wire Line
	6100 1850 6300 1850
Connection ~ 6300 1850
Wire Wire Line
	6600 2300 6500 2300
Wire Wire Line
	6500 2300 6500 2350
Wire Wire Line
	6500 2300 6300 2300
Wire Wire Line
	6300 2300 6300 2150
Connection ~ 6500 2300
Wire Wire Line
	6300 1850 7550 1850
Wire Wire Line
	6600 2000 7550 2000
$Comp
L Device:C C6
U 1 1 6178DE97
P 7000 2450
F 0 "C6" V 6850 2400 50  0000 L CNN
F 1 "100n" V 6750 2350 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7038 2300 50  0001 C CNN
F 3 "https://datasheets.avx.com/U-Series.pdf" H 7000 2450 50  0001 C CNN
F 4 "478-10836-1-ND" H 7000 2450 50  0001 C CNN "Digikey SKU"
	1    7000 2450
	0    1    -1   0   
$EndComp
Wire Wire Line
	7350 2300 7550 2300
Wire Wire Line
	7150 2450 7550 2450
Text GLabel 6900 1650 0    50   Input ~ 0
+3.3V
Wire Wire Line
	6900 1650 7050 1650
Wire Wire Line
	7050 1650 7050 2300
$Comp
L power:GND #PWR07
U 1 1 61798A7F
P 7400 1500
F 0 "#PWR07" H 7400 1250 50  0001 C CNN
F 1 "GND" H 7405 1327 50  0000 C CNN
F 2 "" H 7400 1500 50  0001 C CNN
F 3 "" H 7400 1500 50  0001 C CNN
	1    7400 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 2450 6850 2750
Wire Wire Line
	6850 2750 7500 2750
Connection ~ 6850 2750
Wire Wire Line
	6850 2750 6850 3050
$Comp
L Device:R R5
U 1 1 6179EA76
P 7250 3050
F 0 "R5" V 7043 3050 50  0000 C CNN
F 1 "1meg" V 7134 3050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7180 3050 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204&DocType=DS&DocLang=English" H 7250 3050 50  0001 C CNN
F 4 "A129785CT-ND" H 7250 3050 50  0001 C CNN "Digikey SKU"
	1    7250 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 3050 7550 3050
Wire Wire Line
	7100 3050 6850 3050
Connection ~ 6850 3050
Wire Wire Line
	6850 3050 6850 3750
Wire Wire Line
	9000 3200 8350 3200
$Comp
L Device:R R1
U 1 1 617A6D69
P 5550 2800
F 0 "R1" H 5620 2846 50  0000 L CNN
F 1 "10k" H 5620 2755 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5480 2800 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 5550 2800 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 5550 2800 50  0001 C CNN "Digikey SKU"
	1    5550 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3200 6100 3200
Wire Wire Line
	5100 3300 6350 3300
Wire Wire Line
	5100 3400 5850 3400
Wire Wire Line
	5100 3600 5550 3600
Wire Wire Line
	5550 2950 5550 3600
Connection ~ 5550 3600
Wire Wire Line
	5550 3600 7350 3600
Wire Wire Line
	5350 2650 5550 2650
Connection ~ 5550 2650
Wire Wire Line
	5550 2650 5850 2650
$Comp
L power:GND #PWR03
U 1 1 617B7B1D
P 5550 3900
F 0 "#PWR03" H 5550 3650 50  0001 C CNN
F 1 "GND" H 5555 3727 50  0000 C CNN
F 2 "" H 5550 3900 50  0001 C CNN
F 3 "" H 5550 3900 50  0001 C CNN
	1    5550 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 617B8246
P 5550 3750
F 0 "C2" H 5700 3750 50  0000 L CNN
F 1 "100n" H 5700 3850 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5588 3600 50  0001 C CNN
F 3 "https://datasheets.avx.com/U-Series.pdf" H 5550 3750 50  0001 C CNN
F 4 "478-10836-1-ND" H 5550 3750 50  0001 C CNN "Digikey SKU"
	1    5550 3750
	1    0    0    1   
$EndComp
$Comp
L Device:C C1
U 1 1 617B9210
P 5000 1550
F 0 "C1" H 5150 1500 50  0000 L CNN
F 1 "100n" H 5150 1600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5038 1400 50  0001 C CNN
F 3 "https://datasheets.avx.com/U-Series.pdf" H 5000 1550 50  0001 C CNN
F 4 "478-10836-1-ND" H 5000 1550 50  0001 C CNN "Digikey SKU"
	1    5000 1550
	1    0    0    1   
$EndComp
Wire Wire Line
	4850 1300 5000 1300
Wire Wire Line
	5000 1300 5000 1400
Connection ~ 5000 1300
Wire Wire Line
	5000 1300 5200 1300
Wire Wire Line
	5000 1700 4550 1700
Connection ~ 4550 1700
Wire Wire Line
	4550 1700 4550 1600
$Comp
L CAP1293:CAP1293-1-SN U4
U 1 1 6171A5BA
P 2500 5950
F 0 "U4" H 3500 6337 60  0000 C CNN
F 1 "CAP1293-1-SN" H 3500 6231 60  0000 C CNN
F 2 "LidPCB:CAP1293-1-SN" H 3500 6190 60  0001 C CNN
F 3 "https://ww1.microchip.com/downloads/en/DeviceDoc/00001566B.pdf" H 2500 5950 60  0001 C CNN
F 4 "CAP1293-1-SN-ND" H 2500 5950 50  0001 C CNN "Digikey SKU"
	1    2500 5950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 6171C5EE
P 4650 6250
F 0 "#PWR012" H 4650 6000 50  0001 C CNN
F 1 "GND" H 4655 6077 50  0000 C CNN
F 2 "" H 4650 6250 50  0001 C CNN
F 3 "" H 4650 6250 50  0001 C CNN
	1    4650 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 6250 4650 6250
Text GLabel 2300 6150 0    50   Input ~ 0
SCK
Text GLabel 2300 6050 0    50   Input ~ 0
SDA
Wire Wire Line
	2300 6050 2500 6050
Wire Wire Line
	2300 6150 2500 6150
Text GLabel 2300 6250 0    50   Input ~ 0
+3.3V
Wire Wire Line
	2300 6250 2500 6250
Text Notes 900  2000 0    79   ~ 16
Input-Connector
Text GLabel 1750 2200 2    50   Output ~ 0
+5V
$Comp
L power:GND #PWR011
U 1 1 61739E4E
P 2150 3250
F 0 "#PWR011" H 2150 3000 50  0001 C CNN
F 1 "GND" H 2155 3077 50  0000 C CNN
F 2 "" H 2150 3250 50  0001 C CNN
F 3 "" H 2150 3250 50  0001 C CNN
	1    2150 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2200 1750 2200
Text GLabel 5100 3200 0    50   Input ~ 0
SCK
Text GLabel 1750 2700 2    50   Output ~ 0
SCK
Wire Wire Line
	1500 2400 1750 2400
Wire Wire Line
	1500 2300 2150 2300
Text GLabel 5100 3300 0    50   Input ~ 0
DC
Text GLabel 1750 2600 2    50   Output ~ 0
DC
Text GLabel 1750 2800 2    50   Output ~ 0
SDA
Wire Wire Line
	1500 2600 1750 2600
NoConn ~ 7550 3200
Text GLabel 5100 3600 0    50   Input ~ 0
RST
Text GLabel 1750 2400 2    50   Output ~ 0
RST
Wire Wire Line
	1500 2700 1750 2700
Wire Wire Line
	6350 3300 7300 3300
Wire Wire Line
	7300 3300 7300 3500
Wire Wire Line
	7300 3500 7550 3500
Connection ~ 6350 3300
Text GLabel 2250 5950 0    50   Output ~ 0
IRQ
Wire Wire Line
	2250 5950 2450 5950
$Comp
L Device:R R9
U 1 1 617614C9
P 2450 5550
F 0 "R9" H 2520 5596 50  0000 L CNN
F 1 "10k" H 2520 5505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 2380 5550 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncp.pdf" H 2450 5550 50  0001 C CNN
F 4 "RNCP0603FTD10K0CT-ND" H 2450 5550 50  0001 C CNN "Digikey SKU"
	1    2450 5550
	1    0    0    -1  
$EndComp
Text GLabel 2200 5400 0    50   Input ~ 0
+3.3V
Wire Wire Line
	2200 5400 2450 5400
Wire Wire Line
	2450 5700 2450 5950
Connection ~ 2450 5950
Wire Wire Line
	2450 5950 2500 5950
Text GLabel 4950 5950 2    50   Input ~ 0
TS1
Wire Wire Line
	4500 5950 4950 5950
Text GLabel 4950 6150 2    50   Input ~ 0
TS2
Wire Wire Line
	4500 6150 4950 6150
Text GLabel 4950 6050 2    50   Input ~ 0
SG
Wire Wire Line
	4500 6050 4950 6050
Text Label 7300 2450 2    50   ~ 0
VDD
Text Label 7350 2300 0    50   ~ 0
VCI
Text Label 7400 3050 0    50   ~ 0
Iref
Text Label 7300 2000 0    50   ~ 0
VCOMH
Text Label 7950 4900 0    50   ~ 0
Vfb
Text Label 7950 5300 0    50   ~ 0
Lx
Text Notes 9300 4750 0    50   ~ 0
can be 1-5k to reduce noise
Wire Wire Line
	8350 2300 8350 2150
Connection ~ 8350 2150
Wire Wire Line
	7350 2300 7350 2600
Wire Wire Line
	7350 2600 7550 2600
Connection ~ 7350 2300
$Comp
L Connector:Conn_01x08_Female J1
U 1 1 6172C833
P 1300 2500
F 0 "J1" H 1373 2430 50  0000 C CNN
F 1 "Conn_01x8_Female" V 1464 2430 50  0000 C CNN
F 2 "LidPCB:FPC_8-pin_horizontal_686108188622" H 1300 2500 50  0001 C CNN
F 3 "~" H 1300 2500 50  0001 C CNN
F 4 "732-686108188622CT-ND" H 1300 2500 50  0001 C CNN "Digikey SKU"
	1    1300 2500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1500 2900 2150 2900
Wire Wire Line
	1500 2800 1750 2800
Text GLabel 1750 2500 2    50   Input ~ 0
IRQ
Wire Wire Line
	1750 2500 1500 2500
Wire Wire Line
	2150 2300 2150 2900
Wire Wire Line
	2150 2900 2150 3250
Connection ~ 2150 2900
$EndSCHEMATC
