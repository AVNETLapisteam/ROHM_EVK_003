2021/6/9 - Update 17.0.2

*ROHM Sensor Shield

- Applicable Part Number: SensorShield-EVK-003
    ROHM Sensor Shield include sensor
1. KX224-I2C
2. BM1383AGLV-I2C
3. BM1422AGMV-I2C
4. RPR0521RS-I2C
5. BH1749NUC-I2C
6. BH1790GLC-I2C
7. BD7411G
8. BD1020HFV


- Bluetooth Low Energy Chip
    nRF52832
Test Board
1. nRF52DK
2. Lapis - MK71521EK1A


- Change circuit
1. nRF52DK

Change VDD circuit on SensorShield-EVK-003

    remove Pull up resister(R10 and R11).
    
    Connect VDD on nRF52DK and I2C power of the removed pull up resisiter(R10 and R11) SensorShield-EVK-003

2. MK71521EK1A

    Cut SB_P26 and SB_P27.


- Other
1. Download SDK17.0.2 in nordicsemi.com 

    After updating all sensors, I plan to update in the next version.

    https://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF52832/Download#infotabs

2. Create a ROHM_EVK_003 folder and git pull it in the created folder.

    location : SDK17_directory\examples\ROHM_EVK_003\

3. Lapis - MK71521EK1A

    Please perpare nRF52DK(Need SWD cable for connection) or J-Link debugger. MK71521EK1A is no built-in debugger.

4. Demo

        iOS : Download nRFToolbox in Appstore 

        Android : Download nRFToolbox in google play store or custom nRFToolbox in git

@KX224-I2C : Accelerometer - connect accel application in custom nRFToolbox.

@BM1383AGLV : Air Press sensor - connect press application in custom nRFToolbox. If want to connect iOS, you can test HRS in nRFToolbox.

@BM1422AGMV : Magnetometer - connect magneto application in custom nRFToolbox.

@RPR0521RS : Ambient light sensor and Proximity sensor - connect alsps application in custom nRFToolbox.

@BH1749NUC : Color sensor - connect color application in custom nRFToolbox.

@BH1790GLC : Heart Rate sensor - connect HRS application in anything nRFToolbox.

@BD1020HFV : Temperature sensor - connect HTS application in anything nRFToolbox.

@BD7411G : Hall sensor - need not application. When the magnet is brought close to the sensor, LED4 turn on or off.
