#include "mbed.h"
#include "stats_report.h"
/* User libraries */
#include "definitions.h"
#include "reardefs.h"
#include "CANMsg.h"
#include "RFM69.h"

#define MB1                   // uncomment this line if MB1
//#define MB2                 // uncomment this line if MB2

#ifdef MB1
#define NODE_ID MB1_ID
#endif
#ifdef MB2
#define NODE_ID MB2_ID
#endif

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
RFM69 radio(PB_15, PB_14, PB_13, PB_12, PA_8); // RFM69::RFM69(PinName  PinName mosi, PinName miso, PinName sclk,slaveSelectPin, PinName int)
/* I/O pins */
AnalogIn analog(PA_0);
#ifdef MB1
DigitalIn fuel_sensor(PB_0, PullNone);                                          // MB1
InterruptIn freq_sensor(PB_4, PullNone);                                        // MB1
#endif
#ifdef MB2
DigitalIn fuel_sensor(PB_0, PullNone);                                          // MB2
InterruptIn freq_sensor(PB_4, PullNone);                                        // MB2
#endif
PwmOut servo(PA_6);
/* Debug pins */
PwmOut signal(PA_7);
DigitalOut led(PC_13);
//DigitalOut dbg1(PC_14);
//DigitalOut dbg2(PC_15);
//DigitalOut dbg3(PA_1);
//DigitalOut dbg4(PA_4);
/* Interrupt services routine */
void canISR();
void servoSwitchISR();
void ticker1HzISR();
void ticker5HzISR();
void ticker100HzISR();
void frequencyCounterISR();
/* Interrupt handlers */
void canHandler();
/* General functions*/
void initPWM();
void initRadio();
void setupInterrupts();
void filterMessage(CANMsg msg);
void writeServo(uint8_t state);

/* Debug variables */
Timer t;
Timer engine_counter;
bool buffer_full = false;
uint32_t tim1, tim2, imu_last_acq = 0;

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
Ticker ticker1Hz;
Ticker ticker5Hz;
Ticker ticker100Hz;
Timeout fuel_timeout;
CircularBuffer <state_t, 2*BUFFER_SIZE> state_buffer;
CircularBuffer <imu_t*, 20> imu_buffer;

/* Global variables */
bool switch_clicked = false;
uint8_t switch_state = 0x00;
state_t current_state = IDLE_ST;
uint8_t pulse_counter = 0;
uint64_t current_period = 0, last_count = 0;
float rpm_hz, V_termistor = 0;
uint8_t fuel_timer = 0;
uint8_t fuel_counter = 0;
packet_t data;                                // Create package for radio comunication
uint8_t imu_counter;

int main()
{
    /* Main variables */
    CANMsg txMsg;
    /* Initialization */
    t.start();
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    initPWM();
    initRadio();
    setupInterrupts();

    while (true) {
        if (state_buffer.full()) {
            buffer_full = true;
            led = 0;
        } else {
            led = 1;
            buffer_full = false;
            if (!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

        switch (current_state) {
            case IDLE_ST:
//              Thread::wait(1);
                break;
            case TEMP_ST:
                //serial.printf("t");
                //dbg1 = 0;
                V_termistor = VCC*analog.read();
                data.temperature = ((float) (1.0/0.032)*log((1842.8*(VCC - V_termistor)/(V_termistor*R_TERM))));
                /* Send temperature data */
                txMsg.clear(TEMPERATURE_ID);
                txMsg << data.temperature;
                can.write(txMsg);
                break;
            case FUEL_ST:
                //serial.printf("f");
                data.flags &= ~(0x8);                             // clear fuel flag
                data.flags |= (fuel_counter > NORMAL_THRESHOLD) ? (0x01 << 3) : 0;

                txMsg.clear(FLAGS_ID);
                txMsg << data.flags;
                can.write(txMsg);

                fuel_timer = 0;
                fuel_counter = 0;
                ticker100Hz.attach(&ticker100HzISR, 0.01);
                //dbg1 = 0;
                break;
            case RPM_ST:
                //serial.printf("r");
                //dbg2 = 1;
                freq_sensor.fall(NULL);         // disable interrupt
                if (current_period != 0) {
                    rpm_hz = ((float)pulse_counter/(current_period/1000000.0));    //calculates frequency in Hz
                    if (switch_state != RUN_MODE)
                        writeServo(RUN_MODE);
//                    engine_counter.start();
                } else {
                    rpm_hz = 0;
                    writeServo(switch_state);
//                    engine_counter.stop();
                }
                /* Send rpm data */
                txMsg.clear(RPM_ID);
                txMsg << ((uint16_t) rpm_hz);
                can.write(txMsg);
                /* prepare to re-init rpm counter */
                pulse_counter = 0;
                current_period = 0;                                   // reset pulses related variables
                last_count = t.read_us();
                freq_sensor.fall(&frequencyCounterISR);               // enable interrupt

                //dbg2 = 0;
                break;
            case THROTTLE_ST:
                if (switch_clicked) {
                    writeServo(switch_state);
                    switch_clicked = false;
                }

                break;
            case RADIO_ST:
                imu_t* temp_imu;
                //dbg4 = 1;
                //if(radio.receiveDone())
//                {
//                    if (radio.ACKRequested())
//                        radio.sendACK();
//                    led = 0;
//                }

//                serial.printf("%d,%d,%d\r\n", (!imu_buffer.empty()), (!d10hz_buffer.empty()), (!temp_buffer.empty()));
//                if((!imu_buffer.empty()) && (!d10hz_buffer.empty()) && (!temp_buffer.empty()))
//                {
                if (!imu_buffer.empty()) {
                    imu_buffer.pop(temp_imu);
                    memcpy(&data.imu, temp_imu, 4*sizeof(imu_t));
                    data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                    radio.send((uint8_t)BOXRADIO_ID, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                } else if (t.read_ms() - imu_last_acq > 500) {
                    memset(&data.imu, 0, 4*sizeof(imu_t));
                    data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                    radio.send((uint8_t)BOXRADIO_ID, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                }
//                }
//                radio.receiveDone();
                //dbg4 = 0;
//                tim2 = t.read_us() - tim1;
//                serial.printf("%d\r\n",tim2);
                break;
            case DEBUG_ST:
                //state_buffer.push(RADIO_ST);
//                serial.printf("radio state pushed");
//                serial.printf("bf=%d, cr=%d\r\n", buffer_full, switch_state);
//                serial.printf("speed=%d\r\n", data.data_10hz[packet_counter[N_SPEED]].speed);
//                serial.printf("rpm=%d\r\n", data.data_10hz[packet_counter[N_RPM]].rpm);
//                serial.printf("imu acc x =%d\r\n", data.imu[imu_counter].acc_x);
//                serial.printf("imu acc y =%d\r\n", data.imu[imu_counter].acc_y);
//                serial.printf("imu acc z =%d\r\n", data.imu[imu_counter].acc_z);
//                serial.printf("imu dps x =%d\r\n", data.imu[imu_counter].dps_x);
//                serial.printf("imu dps y =%d\r\n", data.imu[imu_counter].dps_y);
//                serial.printf("imu dps z =%d\r\n", data.imu[imu_counter].dps_z);
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

void ticker1HzISR()
{
    state_buffer.push(TEMP_ST);
}

void ticker5HzISR()
{
    state_buffer.push(RPM_ST);
    state_buffer.push(RADIO_ST);
}

void ticker100HzISR()
{
    if (fuel_timer < 100) {
        fuel_timer++;
        fuel_counter += fuel_sensor.read();
    } else {
        state_buffer.push(FUEL_ST);
        ticker100Hz.detach();
    }
}

void frequencyCounterISR()
{
    led = !led;
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

/* General functions */
void initPWM()
{
    servo.period_ms(20);                        // set signal frequency to 50Hz
    servo.write(0);                             // disables servo
    signal.period_ms(32);                       // set signal frequency to 1/0.032Hz
    signal.write(0.5f);                         // dutycycle 50%
}

void initRadio()
{
    radio.initialize(FREQUENCY_915MHZ, NODE_ID, NETWORK_ID);
    radio.encrypt(0);
    radio.setPowerLevel(31);
    radio.setHighPower();
}

void setupInterrupts()
{
    can.attach(&canISR, CAN::RxIrq);
    ticker1Hz.attach(&ticker1HzISR, 1.0);
    ticker5Hz.attach(&ticker5HzISR, 0.2);
    ticker100Hz.attach(&ticker100HzISR, 0.01);
    freq_sensor.fall(&frequencyCounterISR);
}

void filterMessage(CANMsg msg)
{
    led = !led;
    //serial.printf("alalala1");
    if (msg.id == THROTTLE_ID) {
        switch_clicked = true;
        state_buffer.push(THROTTLE_ST);
        msg >> switch_state;
    } else if (msg.id == IMU_ACC_ID) {
        imu_last_acq = t.read_ms();
        msg >> data.imu[imu_counter].acc_x >> data.imu[imu_counter].acc_y >> data.imu[imu_counter].acc_z;
    } else if (msg.id == IMU_DPS_ID) {
        msg >> data.imu[imu_counter].dps_x >> data.imu[imu_counter].dps_y
            >> data.imu[imu_counter].dps_z;
        if (imu_counter < 3) {
            imu_counter++;
        } else if (imu_counter == 3) {
            imu_buffer.push(data.imu);
            imu_counter = 0;
        }
    } else if (msg.id == SPEED_ID) {
        msg >> data.speed;
//        serial.printf("alalala2");
//      serial.printf("\r\nspeed = %d\r\n",data.data_10hz[packet_counter[N_SPEED]].speed);
//      d10hz_buffer.push(data.data_10hz);
    }
}

void writeServo(uint8_t state)
{
    data.flags &= ~(0x07);         // reset servo-related flags

    switch (state) {
        case MID_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_MID);
            data.flags &= ~(0x03); // reset run and choke flags
            break;
        case RUN_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_RUN);
            data.flags |= RUN_MODE;    // set run flag
            break;
        case CHOKE_MODE:
            //dbg3 = !dbg3;
            servo.pulsewidth_us(SERVO_CHOKE);
            data.flags |= CHOKE_MODE;    // set choke flag
            break;
        default:
//            serial.printf("Choke/run error\r\n");
            data.flags |= 0x04;    // set servo error flag
            break;
    }
}
