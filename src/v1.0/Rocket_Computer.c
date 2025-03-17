/* 

    Model Rocket Flight Computer V1.0 - GNU GPLv3
    Active steering control for vertical-directed flight

    Working code, WIP. No guarantees or warranty implied.
    Rocketry has inherent danger, use at own risk.

    Hardware Implement:
    [X] Servo PIO Control
    [X] BNO055 IMU Data
    [X] SPI SD Card Record
    [ ] PA1616D GPS Data - UART speed interferes with steering
    [ ] BMP390 Baro Data - init code incomplete

    Next Feature Improvements:
    [ ] Interrupt-driven IMU and servos
    [ ] PIO SDIO SD Card + Polling LED

    Note: carlk3's github library is used to interface to the SD card:
    https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico

*/

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "blink.pio.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

// initialize and poll modules on/off
#define SET_SERVOS true
#define GET_IMU true
#define RECORD_IMU true
#define GET_GPS false
#define GET_BARO false

#define PIO_LED // comment in/out to turn on/off PIO LED state machine

// turn on/off debugging code 
//#define DEBUG_MODE   // comment in to compile/display cycles per second
//#define DEBUG_PRINTS // comment in to compile and send print statements
#define PRINT_IMU false  // t/f to allow IMU serial printout
//#define PRINT_GPS true etc...

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define BN0_ADDY 0x28  // IMU
#define BMP_ADDY 0x77  // barometer

// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define GPS_UART_ID uart1
//#define GPS_BAUD_RATE 115200
#define GPS_BAUD_RATE 9600
//#define GPS_BAUD_RATE 38400
#define UART_TX_PIN 8
#define UART_RX_PIN 9


// recording pin and filename defines
#define RECORDING_PIN 1
// SD Card Filename builders
const char filenameIMUbegin[] = "IMU_";
const char filenameIMUextension[] = ".csv";

#define RECORD_LED_PIN 6
#define STATUS_LED_PIN 7

#define SERVO_X1_PIN 2
#define SERVO_X2_PIN 3
#define SERVO_Y1_PIN 4
#define SERVO_Y2_PIN 5
// servo midpoint and steps (times gravity vectors 0-9.8) in 125MHz clock ticks
#define SERVO_MIDPOINT 187500 
//#define SERVO_STEP 4000 // first two flights at this value
#define SERVO_STEP 7000   // higher response than previous
#define SERVO_X1_OFFSET 0
#define SERVO_X2_OFFSET 0
#define SERVO_Y1_OFFSET 0
#define SERVO_Y2_OFFSET 0

// polling timer delays (in ms)
#define SERVO_DELAY 10
#define IMU_DELAY 10
#define RECORD_DELAY 100 // nested inside IMU delay, can't be faster
#define GPS_DELAY 5
#define BARO_DELAY 10


// init IMU function
void bno055_init (void) {

    // wait for IMU to initialize
    sleep_ms(1000); 
    // pull chip ID and compare with expected ID
    uint8_t IDreg = 0x00;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, BN0_ADDY, &IDreg, 1, true);
    i2c_read_blocking(I2C_PORT, BN0_ADDY, chipID, 1, false);

    if (chipID[0] != 0xA0) {
        while(1){
            #ifdef DEBUG_PRINTS 
                printf("BNO055 Chip ID Not Correct - Check Connection!\n");
            #endif
            sleep_ms(5000);
        }
    }    
    
    // use external oscillator and reset
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);

    // Reset all interrupt status bits
    data[0] = 0x3F;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);

    /*
    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);
    sleep_ms(50);
    */

    // Default Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);

    // set units to windows, fahrenheit, and m/s^2
    data[0] = 0x3B;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);
    sleep_ms(30);

    // set operation to NDOF
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, BN0_ADDY, data, 2, true);
    sleep_ms(100);

    #ifdef DEBUG_PRINTS 
        printf("BNO055 Initialized.\n");
    #endif

}



// init barometer function
void bmp390_init (void) {

    // wait for barometer to initialize
    sleep_ms(1000); 
    // pull chip ID and compare with expected ID
            
    uint8_t IDreg = 0x00;
    uint8_t chipID[1];

    i2c_write_blocking(I2C_PORT, BMP_ADDY, &IDreg, 1, true);
    i2c_read_blocking(I2C_PORT, BMP_ADDY, chipID, 1, false);

    if (chipID[0] != 0x60) {
        while(1){
            #ifdef DEBUG_PRINTS 
                printf("BMP390 Chip ID Not Correct - Check Connection!\n");
            #endif
            sleep_ms(5000);
        }
    }    

    uint8_t data[2];
    
    // Configure FIFO
    data[0] = 0x17;
    data[1] = 0x00; // disable
    i2c_write_blocking(I2C_PORT, BMP_ADDY, data, 2, true);
    sleep_ms(50);

    // Configure Power Mode (normal, on pres, on temp)
    data[0] = 0x1B;
    data[1] = 0x33;
    i2c_write_blocking(I2C_PORT, BMP_ADDY, data, 2, true);
    sleep_ms(50);

    // Configure Oversampling
    data[0] = 0x1C;
    data[1] = 0x00; //  no oversample
    //data[1] = 0x0C; //  16x baro, 2x temp
    //data[1] = 0x0D; //  32x baro, 2x temp
    i2c_write_blocking(I2C_PORT, BMP_ADDY, data, 2, true);
    sleep_ms(50);

    // Configure Data Rate (200Hz)
    data[0] = 0x1D;
    //data[1] = 0x00; // 200Hz
    data[1] = 0x01; // 100Hz
    //data[1] = 0x02; // 50Hz
    //data[1] = 0x07; // 640ms
    i2c_write_blocking(I2C_PORT, BMP_ADDY, data, 2, true);
    sleep_ms(50);

    // Configure IIR Filter
    data[0] = 0x1F;
    data[1] = 0x00; // bypass
    //data[1] = 0x03; // coef_7
    //data[1] = 0x05;
    //data[1] = 0x07; // coef_127
    i2c_write_blocking(I2C_PORT, BMP_ADDY, data, 2, true);
    sleep_ms(50);
    #ifdef DEBUG_PRINTS 
        printf("BMP390 Initialized.\n");
    #endif

}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    // modified blink example
    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}


void servo(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    pio->txf[sm] = SERVO_MIDPOINT; // mid
}

int main()
{
    stdio_init_all();

    //SD Card vars
    FRESULT fr;
    FATFS fs;
    FIL fil;
    int ret;
    char buf[100];
    bool recording = false;
    bool recordingWasOn = false;
    uint8_t fileIncrement = 1;
    
    // Recording switch init
    gpio_init(RECORDING_PIN);
    gpio_set_dir(RECORDING_PIN, GPIO_IN);
    gpio_pull_up(RECORDING_PIN);
    
    uint offset;

    #ifdef PIO_LED
        // PIO LED Blinking (status and recording LEDs)
        PIO pioLED = pio0;
        offset = pio_add_program(pioLED, &blink_program);
        //printf("Loaded program at %d\n", offset);
        blink_pin_forever(pioLED, 0, offset, STATUS_LED_PIN, 3);
        blink_pin_forever(pioLED, 1, offset, RECORD_LED_PIN, 3);
    #endif
    
    // servo control PIO init
    PIO pioServo = pio1;
    offset = pio_add_program(pioServo, &blink_program);
    //printf("Loaded program at %d\n", offset);
    servo(pioServo, 0, offset, SERVO_X1_PIN, 3); // X1
    servo(pioServo, 1, offset, SERVO_X2_PIN, 3); // X2
    servo(pioServo, 2, offset, SERVO_Y1_PIN, 3); // Y1
    servo(pioServo, 3, offset, SERVO_Y2_PIN, 3); // Y2

    // I2C initialization. Using it at 400Khz (BNO055 max).
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // initialize BNO055
    if (GET_IMU) bno055_init();
    // initialize BMP390
    if (GET_BARO) bmp390_init();

    // initialize UART (GPS)
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);
    //uart_init(UART_ID, 9600);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    //uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    sleep_ms(500);

    // OPTIONAL GPS SETTINGS (baud rate settings)
    // In a default system, printf will also output via the default UART
    // uart_puts(GPS_UART_ID, "$PMTK251,38400*27");
    // uart_puts(UART_ID, "$PMTK251,115200*1F\r\n");
    // uart_puts(UART_ID, "$PMTK251,0*28\r\n"); // default setting
    // uart_putc(UART_ID, 0x0D);
    // uart_putc(UART_ID, 0x0A);

    //SD TEST
    if (RECORD_IMU){
        // Initialize SD card
        if (!sd_init_driver()) {
            #ifdef DEBUG_PRINTS 
                printf("ERROR: Could not initialize SD card\r\n");
            #endif
            while (true);
        }

        // Mount drive
        fr = f_mount(&fs, "0:", 1);
        if (fr != FR_OK) {
            #ifdef DEBUG_PRINTS 
                printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
            #endif
            while (true);
        }
    }

    #ifdef PIO_LED
        // INIT COMPLETE - change LED blink speed
        pio0->txf[0] = (1250000000 / (2 * 3)) - 3; // status slow blink
        pio0->txf[1] = 0; // recording led off
    #endif
    
    // init global variables
    // IMU
    uint8_t accel[6]; // pull location for 6 acceleration registers from IMU
    uint8_t grav[6]; // pull location for 6 gravity registers from IMU
    uint8_t head[2]; // pull location for 2 heading registers from IMU
    uint8_t mag[2]; // pull location for 2 heading registers from IMU
    int8_t temperatureIMU; // pull location for IMU temperature register from IMU
    int16_t accelX, accelY, accelZ, heading, gravX, gravY, gravZ, magnetoZ; // combined 3-axis data
    float f_accelX, f_accelY, f_accelZ, f_gravX, f_gravY, f_gravZ, f_heading, f_temperatureIMU;
    uint8_t accelRegStart = 0x08; //start address of accel registers
    uint8_t magRegStart = 0x12; //start address of magnetometer Z registersa
    uint8_t headRegStart = 0x1A; //start address of heading register
    uint8_t gravRegStart = 0x2E; //start address of gravity vector register
    uint8_t tempRegStart = 0x34; //start address of IMU temperature register
    // baro
    uint8_t pressure[3]; // pull location for 3 pressure registers from baro
    uint8_t temperature[3]; // pull location for 3 pressure registers from baro
    uint32_t pressureData;
    uint32_t temperatureData;
    float f_altitudeData, f_pressureData, f_temperatureData;
    uint8_t  baroRegStart = 0x04; //start address of barometer registers (pressure)
    uint8_t  baroRegStart2 = 0x07; //start address of barometer registers (temperature)


    #ifdef DEBUG_MODE
        //debug helpers
        uint32_t CPS = 0;
        uint32_t CPS_timer = to_ms_since_boot(get_absolute_time());
    #endif

    // global loop ms timers (slightly staggered)
    uint32_t currentTime = 0;
    uint32_t uartTimer = 1;
    uint32_t accelTimer = 2;
    uint32_t recordTimer = 2;
    uint32_t baroTimer = 3;
    uint32_t servoTimer = 4;


    // MAIN LOOP
    while (true) {

        currentTime = to_ms_since_boot(get_absolute_time());

        // read gps uart
        if (GET_GPS && (uartTimer + GPS_DELAY <= currentTime)) {
            if (uart_is_readable(GPS_UART_ID)){
                char holding[2];
                uart_read_blocking(GPS_UART_ID, holding, 2);
                #ifdef DEBUG_PRINTS 
                    printf(holding);
                #endif
            }
            uartTimer = currentTime;
            //printf("\n");
        }

        #ifdef DEBUG_MODE
            // debug Cycles Per Second
            CPS += 1;
            if (CPS_timer + 1000 <= currentTime) {
                printf("\n");
                printf("Cycles Per Second: %d\n", CPS);
                CPS = 0;
                CPS_timer = currentTime;
            }
        #endif

        // pull IMU data
        if (GET_IMU && (accelTimer + IMU_DELAY <= currentTime)) {
            
            i2c_write_blocking(I2C_PORT, BN0_ADDY, &accelRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BN0_ADDY, accel, 6, false);
            accelX = ((accel[1]<<8) | accel[0]);
            accelY = ((accel[3]<<8) | accel[2]);
            accelZ = ((accel[5]<<8) | accel[4]);
            f_accelX = accelX / 100.00;
            f_accelY = accelY / 100.00;
            f_accelZ = (accelZ / 100.00); 

            i2c_write_blocking(I2C_PORT, BN0_ADDY, &gravRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BN0_ADDY, grav, 6, false);
            gravX = ((grav[1]<<8) | grav[0]);
            gravY = ((grav[3]<<8) | grav[2]);
            gravZ = ((grav[5]<<8) | grav[4]);
            f_gravX = gravX / 100.00;
            f_gravY = gravY / 100.00;
            f_gravZ = gravZ / 100.00;

            i2c_write_blocking(I2C_PORT, BN0_ADDY, &headRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BN0_ADDY, head, 2, false);
            heading = ((head[1]<<8) | head[0]);
            f_heading = heading / 16.00; 
            
            i2c_write_blocking(I2C_PORT, BN0_ADDY, &tempRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BN0_ADDY, &temperatureIMU, 1, false);
            f_temperatureIMU = (temperatureIMU * (9.0/5.0)) + 32; 

           
            i2c_write_blocking(I2C_PORT, BN0_ADDY, &magRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BN0_ADDY, mag, 2, false); 
            magnetoZ = ((mag[1]<<8) | mag[0]);

            #ifdef DEBUG_PRINTS 
                if (PRINT_IMU) {
                    //printf("%d %d %d %d %d %d\n", accel[0], accel[1], accel[2], accel[3], accel[4], accel[5]);
                    //printf("X: %d    Y: %d    Z: %d\n", accelX, accelY, accelZ);
                    printf("X: %6.2f    Y: %6.2f    Z: %6.2f\n", f_accelX, f_accelY, f_accelZ);
                    printf("GrX: %6.2f    GrY: %6.2f    GrZ: %6.2f\n", f_gravX, f_gravY, f_gravZ);
                    printf("Heading: %6.2f    Temp: %d*C / %3.0f*F  Mag: %d\n", f_heading, temperatureIMU, f_temperatureIMU, magnetoZ);
                }
            #endif

            // manual debugging code
            // printf("Heading: %6.2f  Mag: %d\n", f_heading,  magnetoZ);

            recording = !gpio_get(RECORDING_PIN);

            // set recording led/filename based on switch position
            if (RECORD_IMU && (recording != recordingWasOn)) {
                if (recording) {

                    // PREPARE FILE
                    char num[10];
                    bool fileReady = false;

                    while (!fileReady){

                        // build filename from variable
                        sprintf(num, "%d", fileIncrement);
                        strcpy(buf, filenameIMUbegin);
                        strcat(buf, num);
                        strcat(buf, filenameIMUextension);

                        // debug print
                        //printf("Constructing filename: ");
                        //printf(buf);
                        //printf("\n");

                        // check if file + csv header needs created
                        fr = f_open(&fil, buf, FA_OPEN_EXISTING);
                        if (fr == FR_NO_FILE){
                            f_open(&fil, buf,  (FA_WRITE | FA_CREATE_NEW));
                            f_printf(&fil, "Time(ms), AccelX, AccelY, AccelZ, GravX, GravY, GravZ, Heading, Temp (*C)\r\n");
                            fileReady = true;
                            // debug print
                            //printf("File doesn't exist, new file created...\n");

                        // continuous record or increment filename if making a new record
                        } else if ((fr == FR_OK) && recordingWasOn){
                            fileReady = true;
                            // debug print
                            //printf("Already recording, Continue Record...\n");
                        } else if ((fr == FR_OK) && !recordingWasOn){
                            f_close(&fil);
                            fileIncrement += 1;
                            // debug print
                            //printf("Filename used by another recording, increment filename...\n");
                        }
                        
                    }
                    // Close file
                    f_close(&fil);



                    #ifdef PIO_LED
                        pio0->txf[1] = (1250000000 / (2 * 3)) - 3; // record status LED slow blink
                    #endif
                }


                #ifdef PIO_LED
                    else pio0->txf[1] = 0; // record status LED off
                #endif
            } 


            // actively record data
            if (RECORD_IMU && recording && (recordTimer + RECORD_DELAY <= currentTime)) {

                // debug print
                //printf("Begin Data Logging...\n");

                // RECORD DATA
                // Open file for appending data
                f_open(&fil, buf, FA_WRITE);
                // Seek to end
                f_lseek(&fil, f_size(&fil));
                // Write something to file
                f_printf(&fil, "%lu, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %d\r\n", 
                    currentTime, f_accelX, f_accelY, f_accelZ, f_gravX, f_gravY, f_gravZ, f_heading, temperatureIMU);
                //f_printf(&fil, "%6.2f, %6.2f, %6.2f, ", f_accelX, f_accelY, f_accelZ); // accelerations
                //f_printf(&fil, "%6.2f, %6.2f, %6.2f, ", f_gravX, f_gravY, f_gravZ); //gravities
                //f_printf(&fil, "%6.2f, %d\r\n", f_heading, temperatureIMU); // heading, temperature
                // Close file
                f_close(&fil);
                
                // debug print 
                //printf("Record Saved.\n");

                recordTimer = currentTime;
            }
            
            recordingWasOn = recording;
            accelTimer = currentTime;
        }

        
        // pull barometer data
        if (GET_BARO && (baroTimer + BARO_DELAY <= currentTime)) {
            
            i2c_write_blocking(I2C_PORT, BMP_ADDY, &baroRegStart, 1, true);
            i2c_read_blocking(I2C_PORT, BMP_ADDY, &pressure[0], 3, false);
            i2c_write_blocking(I2C_PORT, BMP_ADDY, &baroRegStart2, 1, true);
            i2c_read_blocking(I2C_PORT, BMP_ADDY, &temperature[0], 3, false);

            //pressureData = ((pressure[0] & 0x1F)<<16 | pressure[1]<<8 | pressure[2]); // 21 bits @ 32x oversample
            //f_pressureData = pressureData * 0.085; // 21bit = 0.085 Pa per bit
            pressureData = (pressure[2]<<8 | pressure[1]); // 16 bits @ no oversample
            f_pressureData = pressureData * 2.64; // 16bit = 2.64 Pa per bit
            //temperatureData = ((temperature[0] & 0x01)<<16 | temperature[1]<<8 | temperature[2]); // 17 bits @ 2x oversample
            //f_temperatureData = temperatureData / 400.0; // 17bit = 0.0025 C per bit
            temperatureData = (temperature[2]<<8 | temperature[1]); // 16 bits @ no oversample
            f_temperatureData = temperatureData * 0.0007; // 16bit = 0.005 C per bit
            f_altitudeData = 44330.0 * (1.0 - pow(((f_pressureData / 100.0) / 1013.25), 0.1903));  // 1013.25 hPa at sea level
            
            #ifdef DEBUG_PRINTS 
                //printf("Baro pressure: %d, %d, %d  temperature: %d, %d, %d \n", pressure[0], pressure[1], pressure[2], temperature[0], temperature[1], temperature[2]);
                printf("Baro pressure: %6.2f  temperature: %6.2f altitude: %6.2f\n", f_pressureData, f_temperatureData, f_altitudeData);
            #endif

            baroTimer = currentTime;

        }

        
        // update servo positioning
        if (SET_SERVOS && (servoTimer + SERVO_DELAY <= currentTime)) {
            
            int32_t finPos[4];

            if(f_gravX >= -9.8 && f_gravX <= 9.8) {
                //X1
                finPos[0] = (f_gravX * SERVO_STEP) + SERVO_MIDPOINT + SERVO_X1_OFFSET;
                pio1->txf[0] = finPos[0];
                //X2
                finPos[1] = (f_gravX * SERVO_STEP * -1) + SERVO_MIDPOINT + SERVO_X2_OFFSET;
                pio1->txf[1] = finPos[1];
            }
            if(f_gravY >= -9.8 && f_gravY <= 9.8) {
                //Y1
                finPos[2] = (f_gravY * SERVO_STEP) + SERVO_MIDPOINT + SERVO_Y1_OFFSET;
                pio1->txf[2] = finPos[2];
                //Y2
                finPos[3] = (f_gravY * SERVO_STEP * -1) + SERVO_MIDPOINT + SERVO_Y2_OFFSET;
                pio1->txf[3] = finPos[3];
            }

            servoTimer = currentTime;
        }

    }
}
