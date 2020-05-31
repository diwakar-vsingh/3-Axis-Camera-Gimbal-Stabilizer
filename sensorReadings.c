/* DriverLib Defines */
#include "driverlib.h"
#include <math.h>

/* Standard Defines */
#include <string.h>
#include <stdio.h>

/* Slave Address for I2C Slave */
#define SLAVE_ADDRESS       0x68
#define pi                  3.14159

/* Variables */
int8_t accXReg[] = {0x3B, 0x3C};
int8_t accYReg[] = {0x3D, 0x3E};
int8_t accZReg[] = {0x3F, 0x40};
uint8_t tempReg[] = {0x41, 0x42};
int8_t gyrXReg[] = {0x43, 0x44};
int8_t gyrYReg[] = {0x45, 0x46};
int8_t gyrZReg[] = {0x47, 0x48};
uint8_t configdata;
uint8_t WhoAmI;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
double accAngleX, accAngleY;
double gyroAngleX = 0, gyroAngleY = 0, gyroAngleZ = 0;
float roll, pitch, yaw = 0;
float AccErrorX = 0.0, AccErrorY = 0.0;
float GyroErrorX = 0.0, GyroErrorY = 0.0, GyroErrorZ = 0.0;
float elapsedTime, currentTime, previousTime;
float Temp_degC;

/* Declare functions */
void initializeI2C(void);
int configRegRead(uint8_t *Register);
void ConfigRegWrite(uint8_t Register, uint8_t Data);
int configSingleByteRegRead(uint8_t Register);
void calculate_IMU_error(void);

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
 EUSCI_B_I2C_CLOCKSOURCE_SMCLK,         // SMCLK Clock Source
 3000000,                               // SMCLK = 3MHz
 EUSCI_B_I2C_SET_DATA_RATE_400KBPS,     // Desired I2C Clock of 400kHz
 0,                                     // No byte counter threshold
 EUSCI_B_I2C_NO_AUTO_STOP               // No Autostop
};

void main(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    /* Configure clock */
    uint32_t dcoFrequency = 3E+6;           // 3MHz
    MAP_CS_setDCOFrequency(dcoFrequency);   // Set clock source to 3MHz
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initialize I2C communication protocol */
    initializeI2C();

    /* verify sensor communication */
    WhoAmI = configSingleByteRegRead(0x75); // Used to verify the identity of sensor, this should read 0x71
    if (WhoAmI==0x71){
        printf("IMU sensor communication established. \r\n");
    }
    else {
        printf("IMU sensor communication failed. \r\n");    
    }

    /* Starting calibration */
    printf("Initializing sensor calibration... \r\n");
    calculate_IMU_error();
    printf("Calibration completed. \r\n");
    
    while(1){
        /* Read time */
        previousTime = currentTime;        // Previous time is stored before the actual time read
        currentTime = counter;            // Current time actual time read
        elapsedTime = (currentTime - previousTime); // in ms

        /*****************************************************
                         Gyroscope Readings
         *****************************************************/
        GyroX = configRegRead(gyrXReg)/131.0 - GyroErrorX;
        GyroY = configRegRead(gyrYReg)/131.0 - GyroErrorY;
        GyroZ = configRegRead(gyrZReg)/131.0 - GyroErrorZ;

        /*****************************************************
                       Accelerometer Readings
         *****************************************************/
        AccX = configRegRead(accXReg)/16384.0;
        AccY = configRegRead(accYReg)/16384.0;
        AccZ = configRegRead(accZReg)/16384.0;

        Temp_degC = (configRegRead(tempReg)/333.87)+21.0;

        accAngleX = (atan(AccY / sqrt(pow(AccX,2) + pow(AccZ,2))) * 180/pi) - AccErrorX;  // in degrees
        accAngleY = (atan(AccX / sqrt(pow(AccY,2) + pow(AccZ,2))) * 180/pi) - AccErrorY;  // in degrees

        /* Integrating the angular velocity to get angles*/
        // gyroAngleX = gyroAngleX + GyroX * elapsedTime;
        // gyroAngleY = gyroAngleY + GyroY * elapsedTime;
        // yaw =  yaw + GyroZ * elapsedTime;

        /* Complementary filter - combine acceleromter and gyro angle values */
        // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
        // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    }
}

void initializeI2C(void)
{
    /* Select I2C function for I2C_SCL & I2C_SDA */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Initializing I2C Master to SMCLK at 400kbs with no autostop */
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);

    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B1_BASE);

    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SLAVE_ADDRESS);

    /* Make reset, place a 0 into the 0x6B register */
    ConfigRegWrite(0x1B, 0x00);
    configdata = configSingleByteRegRead(0x1C);
}

int configRegRead(uint8_t *Register)
{
    int i = 0;
    uint8_t RXData[2];
    int16_t Data;

    for (i=0;i<2;i++){

        /* Set write mode. */
        MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

        // Write START + address + register to the sensor.
        MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE, Register[i]);

        /* Making sure that the last transaction has been completely sent */
        while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)){}   // Indicates whether STOP got sent.

        // Set read mode.
        MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);

        // RESTART.
        MAP_I2C_masterReceiveStart(EUSCI_B1_BASE);

        // Read two bytes from the sensor, STOP.
        RXData[i] = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_B1_BASE);
    }

    /* Parse the sensor's data. */
    Data = (int16_t)(RXData[0] << 8 | RXData[1]);
    return Data;
}

int configSingleByteRegRead(uint8_t Register)
{
    uint8_t RxData;

    /* Set write mode. */
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    // Write START + address + register to the sensor.
    MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE, Register);

    /* Making sure that the last transaction has been completely sent */
    while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)){}   // Indicates whether STOP got sent.

    // Set read mode.
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);

    // RESTART.
    MAP_I2C_masterReceiveStart(EUSCI_B1_BASE);

    // Read one bytes from the sensor, STOP.
    RxData = MAP_I2C_masterReceiveMultiByteFinish(EUSCI_B1_BASE);

    return RxData;
}

void ConfigRegWrite(uint8_t Register, uint8_t Data)
{
    int i = 0 ;

    // Set write mode.
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    /* Send start bit and register */
    MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE, Register);

    // Delay
    for(i = 0; i < 2000; i++){}

    /* Now write the data byte */
    MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, Data);

    while (MAP_I2C_masterIsStopSent(EUSCI_B1_BASE)){}   // Indicates whether STOP got sent.
}

void calculate_IMU_error(void)
{
    /* Call this function in the main function to calculate the accelerometer and gyro data error.  *
     * Note: Place the IMU flat in order to get the proper values                                   */

    int c = 0;
    // Read accelerometer and gyroscope values 200 times
    while (c < 200){
        
        i++;

        GyroX = configRegRead(gyrXReg)/131.0;
        GyroY = configRegRead(gyrYReg)/131.0;
        GyroZ = configRegRead(gyrZReg)/131.0;

        AccX = configRegRead(accXReg)/16384.0;
        AccY = configRegRead(accYReg)/16384.0;
        AccZ = configRegRead(accZReg)/16384.0;

        AccErrorX = AccErrorX + ((atan(AccY / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / pi));
        AccErrorY = AccErrorY + ((atan(AccX / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / pi));

        GyroErrorX = GyroErrorX + (GyroX / 131.0);
        GyroErrorY = GyroErrorY + (GyroY / 131.0);
        GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
        c++;
    }

    /* Average 200 values to get the error value */
    AccErrorX = AccErrorX / 200.0;
    AccErrorY = AccErrorY / 200.0;
    GyroErrorX = GyroErrorX / 200.0;
    GyroErrorY = GyroErrorY / 200.0;
    GyroErrorZ = GyroErrorZ / 200.0;
}

void TA0_0_IRQHandler(void){
    /*  Clear the interrupt flag */
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    counter++;
}
