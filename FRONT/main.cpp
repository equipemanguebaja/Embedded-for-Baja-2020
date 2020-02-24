#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "frontdefs.h"
#include "CANMsg.h"
#include "LSM6DS3.h"
#include "Kalman.h"

//#define MB1
#define MB2

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
LSM6DS3 LSM6DS3(PB_7, PB_6);

/* I/O pins */
InterruptIn freq_sensor(PB_1, PullNone);
InterruptIn choke_switch(PB_10, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PB_11, PullUp);       // servomotor RUN mode

/* Debug pins */
DigitalOut led(PC_13);
PwmOut signal(PA_7);
//DigitalOut dbg1(PC_14);
//DigitalOut dbg2(PC_15);
//DigitalOut dbg3(PA_4);

/* Interrupt services routine */
void canISR();
void servoSwitchISR();
void ticker2HzISR();
void ticker5HzISR();
void ticker20HzISR();
void tickerTrottleISR();
void frequencyCounterISR();

/* Interrupt handlers */
void canHandler();
void throttleDebounceHandler();

/* General functions*/
void setupInterrupts();
void filterMessage(CANMsg msg);
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt);
void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, bool comb, int16_t gp, int16_t gr);

/* Debug variables */
Timer t;
bool buffer_full = false;
unsigned int t0, t1;
/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker2Hz;
Ticker ticker5Hz;
Ticker ticker20Hz;
Ticker tickerTrottle;
Timeout debounce_throttle;
//Timeout debounce_horn;
//Timeout debounce_headlight;
// Timeout horn_limiter;                       // stop sound of horn after a determined period
CircularBuffer <state_t, BUFFER_SIZE> state_buffer;
/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00, flags = 0x00, pulse_counter = 0, temp_motor = 0;
state_t current_state = IDLE_ST;
uint64_t current_period = 0, last_count = 0, last_acq = 0;
uint8_t imu_failed = 0;                         // number of times before a new connection attempt with imu 
float speed_hz = 0;
uint16_t rpm_hz = 0, speed_display = 0, speed_radio = 0, dt = 0;
int16_t angle_roll = 0, angle_pitch = 0; 
packet_t data;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    //horn = horn_button.read();                               // horn OFF
    //headlight = headlight_switch.read();                          // headlight OFF
    led = 1;                                // led OFF
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    t0 = t.read_us();
    uint16_t lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26); 
    t1 = t.read_us();
//    serial.printf("%d\r\n", (t1 - t0));
    setupInterrupts();          

    while (true) {
        if (state_buffer.full())
        {
            buffer_full = true;
            led = 0;
            state_buffer.pop(current_state);
        }
        else
        {
            led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }
        
        switch (current_state)
        {
            case IDLE_ST:
//                Thread::wait(2);
                break;
            case SLOWACQ_ST:
                break;
            case IMU_ST:
                t0 = t.read_us();
                //dbg1 = !dbg1;
                if (lsm_addr)
                {
                    bool nack = LSM6DS3.readAccel();                        // read accelerometer data into LSM6DS3.aN_raw
                    if (!nack)
                        nack = LSM6DS3.readGyro();                         //  "   gyroscope data into LSM6DS3.gN_raw
                    
                    if (nack)
                    {
                        lsm_addr = 0;
                        LSM6DS3.ax_raw = 0;
                        LSM6DS3.ay_raw = 0;
                        LSM6DS3.az_raw = 0;
                        LSM6DS3.gx_raw = 0;
                        LSM6DS3.gy_raw = 0;
                        LSM6DS3.gz_raw = 0;
                    }
                }
                else if (imu_failed == IMU_TRIES)
                {
                    lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26);                                    
                    t1 = t.read_us();
                    imu_failed = 0;
//                    serial.printf("%d\r\n", (t1 - t0));
                }
                else
                {
                    imu_failed++;
                }
                
                last_acq = t.read_ms();
//                serial.printf("accz = %d\r\n", LSM6DS3.gz_raw);
                calcAngles(LSM6DS3.ax_raw, LSM6DS3.ay_raw, LSM6DS3.az_raw, LSM6DS3.gx_raw, LSM6DS3.gy_raw, LSM6DS3.gz_raw, dt);
                 /* Send accelerometer data */
                txMsg.clear(IMU_ACC_ID);
                txMsg << LSM6DS3.ax_raw << LSM6DS3.ay_raw << LSM6DS3.az_raw;
                if(can.write(txMsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    txMsg.clear(IMU_DPS_ID);
                    txMsg << LSM6DS3.gx_raw << LSM6DS3.gy_raw << LSM6DS3.gz_raw << dt;
                    can.write(txMsg);
                }
                break;
            case SPEED_ST:
//            serial.printf("speed ok\r\n");
                //dbg2 = !dbg2;
                freq_sensor.fall(NULL);         // disable interrupt
                if (current_period != 0)
                {
                    speed_hz = 1000000*((float)(2*pulse_counter)/current_period);    //calculates frequency in Hz
                }
                else
                {
                    speed_hz = 0;
                }
                speed_display = ((float)(3.6*PI*WHEEL_DIAMETER*speed_hz)/WHEEL_HOLES_NUMBER);    // make conversion hz to km/h
                speed_radio = ((float)((speed_display)/60.0)*65535);
                pulse_counter = 0;                          
                current_period = 0;                         //|-> reset pulses related variables
                last_count = t.read_us();        
                freq_sensor.fall(&frequencyCounterISR);     // enable interrupt
                /* Send speed data */
                txMsg.clear(SPEED_ID);
                txMsg << speed_radio;
                can.write(txMsg);
//                state_buffer.push(DEBUG_ST);                // debug
                break;
            case THROTTLE_ST:
//                serial.printf("throttle ok\r\n");
                //dbg3 = !dbg3;
                if (switch_clicked)
                {                    
                    switch_state = !choke_switch.read() << 1 | !run_switch.read() << 0;
                    //serial.printf("switch_state = %d\r\n", switch_state);
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    txMsg << switch_state;
//                    serial.printf("can ok\r/n");                  // append data (8 bytes max)
                    can.write(txMsg);
                    
                    switch_clicked = false;
                }
                break;
            case DISPLAY_ST:
//                serial.printf("rpm_hz=%d\r\n", rpm_hz);
                displayData(speed_display, rpm_hz, temp_motor, (flags & 0x08), angle_pitch, angle_roll);
//                state_buffer.push(DEBUG_ST);
                break;
            case DEBUG_ST:
//                serial.printf("r= %d, p=%d\r\n", angle_roll, angle_pitch);
//                serial.printf("imu acc x =%d\r\n", LSM6DS3.ax_raw);
//                serial.printf("imu acc y =%d\r\n", LSM6DS3.ay_raw);
//                serial.printf("imu acc z =%d\r\n", LSM6DS3.az_raw);
//                serial.printf("imu dps x =%d\r\n", LSM6DS3.gx_raw);
//                serial.printf("imu dps y =%d\r\n", LSM6DS3.gy_raw);
//                serial.printf("imu dps z =%d\r\n", LSM6DS3.gz_raw);
                break;
            default:
                break;
        }
    }
}

/* Interrupt services routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void servoSwitchISR()
{
    choke_switch.rise(NULL);     //  throttle interrupt in both edges dettach
    run_switch.rise(NULL);       //  throttle interrupt in both edges dettach
    choke_switch.fall(NULL);     //  throttle interrupt in both edges dettach
    run_switch.fall(NULL);       //  throttle interrupt in both edges dettach
    switch_clicked = true;
    debounce_throttle.attach(&throttleDebounceHandler, 0.1);
}

void ticker2HzISR()
{
    state_buffer.push(DISPLAY_ST);
}

void ticker5HzISR()
{
    state_buffer.push(SPEED_ST);
    state_buffer.push(DISPLAY_ST);
}

void ticker20HzISR()
{
    state_buffer.push(IMU_ST);
}

void frequencyCounterISR()
{
    pulse_counter++;
    current_period += t.read_us() - last_count;
    last_count = t.read_us();        
}

/* Interrupt handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

void throttleDebounceHandler()
{
    state_buffer.push(THROTTLE_ST);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
}

/* General functions */
void setupInterrupts()
{
    can.attach(&canISR, CAN::RxIrq);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
    ticker2Hz.attach(&ticker2HzISR, 0.4);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    ticker20Hz.attach(&ticker20HzISR, 0.05);
    signal.period_ms(2);
    signal.write(0.5f);
}

void filterMessage(CANMsg msg)
{
  led = !led;

    if(msg.id == RPM_ID)
 {
        msg >> rpm_hz;
    }
    
    else if(msg.id == TEMPERATURE_ID)
    {
        msg >> temp_motor;
    }
    
    else if (msg.id == FLAGS_ID)
    {
        msg >> flags;
    }
}

/* Function adapted from Kristian Lauszus library example, source: https://github.com/TKJElectronics/KalmanFilter*/
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt){
    static Kalman kalmanX, kalmanY;
    float kalAngleX, kalAngleY;
    float pitch, roll;
    float gyroXrate, gyroYrate;
    float ax, ay, az;
    static bool first_execution = true;
    
    ax = (float) accx * TO_G;
    ay = (float) accy * TO_G;
    az = (float) accz * TO_G;
    pitch = atan2(ay, az) * RAD_TO_DEGREE;
    roll = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEGREE;
    gyroXrate = grx / TO_DPS;                            // Convert to deg/s
    gyroYrate = gry / TO_DPS;                            // Convert to deg/s

    if (first_execution)
    {
        // set starting angle if first execution
        first_execution = false;
        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);
    }
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    }
    else
    {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if(abs(kalAngleX) > 90)
    {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    angle_roll = roll;
    angle_pitch = pitch;
}

void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, bool comb, int16_t gp, int16_t gr)
{
    static uint16_t vel2 = 0, Hz2 = 0, temp2 = 0;
    static int16_t gr2 = 0, gp2 = 0;
    static bool comb2 = true;

    char str[512];
    int aux=0;
    strcpy(str,"");
    
    if(vel!=vel2)
        sprintf(str + strlen(str), "page0.v.val=%d%c%c%c",vel,0xff,0xff,0xff);
    if(Hz!=Hz2)
    {
        #ifdef MB1
            int r = (Hz*6000)/(5000.0);
        #endif
        #ifdef MB2
            int r = (Hz*7200)/(5000.0);
        #endif
        sprintf(str + strlen(str), "page0.r.val=%d%c%c%c",r,0xff,0xff,0xff);
    }
    if(comb!=comb2)
    {
        if (!comb)
            sprintf(str + strlen(str),"page0.c.val=%d%c%c%c",20,0xff,0xff,0xff);
        else
            sprintf(str + strlen(str),"page0.c.val=%d%c%c%c",100,0xff,0xff,0xff);
    }
    if(temp!=temp2)
    {
        sprintf(str + strlen(str),"page0.tp.val=%d%c%c%c",temp,0xff,0xff,0xff);
    }        
    if(gp!=gp2)
    {
        int gpn = 3;
        if(gp<=-30){gpn=0;}
        if(gp>-30&&gp<=-15){gpn=1;}
        if(gp>-15&&gp<=-5){gpn=2;}
        if(gp>-5&&gp<5){gpn=3;}
        if(gp>=5&&gp<15){gpn=4;}
        if(gp>=15&&gp<30){gpn=5;}
        if(gp>=30){gpn=6;}
        
        sprintf(str + strlen(str),"page1.gp.val=%d%c%c%c",gpn,0xff,0xff,0xff);
        sprintf(str + strlen(str),"page1.gp_n.val=%d%c%c%c",gp,0xff,0xff,0xff);
    }
    if(gr!=gr2)
    {    
        int grn = 3;
        if(gr<=-30){grn=0;}
        if(gr>-30&&gr<=-15){grn=1;}
        if(gr>-15&&gr<=-5){grn=2;}
        if(gr>-5&&gr<5){grn=3;}
        if(gr>=5&&gr<15){grn=4;}
        if(gr>=15&&gr<30){grn=5;}
        if(gr>=30){grn=6;}
        
        sprintf(str + strlen(str),"page1.gr.val=%d%c%c%c",grn,0xff,0xff,0xff);
        sprintf(str + strlen(str),"page1.gr_n.val=%d%c%c%c",gr,0xff,0xff,0xff);
    }
    
    while(str[aux]!='\0')
    {
        serial.putc(str[aux]);
        aux++;
    }
    vel2 = vel;
    Hz2 = Hz;
    temp2 = temp;
    gr2=gr;
    gp2=gp;
    comb2 = comb;
}