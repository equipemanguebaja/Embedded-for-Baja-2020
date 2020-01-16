#ifndef REARDEFS_H
#define REARDEFS_H

#define SERVO_RUN           866
#define RUN_MODE            0x01
#define SERVO_MID           1320
#define MID_MODE            0x00
#define SERVO_CHOKE         1780
#define CHOKE_MODE          0x02
#define VCC                 3.3
#define R_TERM              1000

/* Radio definitions */
#define NETWORK_ID          101
#define BOXRADIO_ID         69
#define MB1_ID              11
#define MB2_ID              22
#define FREQUENCY_915MHZ    91
#define NORMAL_THRESHOLD    68

typedef enum
{
    IDLE_ST,        // wait
    TEMP_ST,        // measure temperatures
    FUEL_ST,        // proccess fuel data sampling
    RPM_ST,         // calculate speed
    THROTTLE_ST,    // write throttle position (PWM)
    RADIO_ST,       // send data for box via radio (SPI)
    DEBUG_ST        // send data for debug
} state_t;

#endif
