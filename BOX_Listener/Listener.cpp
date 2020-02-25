// Listener - print sender messages
// From Tnode/Tsender @ anarduino.com
// 2014 - anarduino.com
//
#include <RFM69.h>
#include <SPI.h>
#include "definitions.h"

#define MB1
//#define MB2 

#ifdef MB1 
#define GATEWAY_ID    69     // this is ME, TGateway
#endif
#ifdef MB2 
#define GATEWAY_ID    70     // this is ME, TGateway
#endif
#define NETWORKID     101   //the same on all nodes that talk to each other
 
// Uncomment only one of the following three to match radio frequency
//#define FREQUENCY     RF69_433MHZ    
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
 
#define IS_RFM69HW   //NOTE: uncomment this ONLY for RFM69HW or RFM69HCW
#define ENCRYPT_KEY    "EncryptKey123456"  // use same 16byte encryption key for all devices on net
#define ACK_TIME       50                  // max msec for ACK wait
#define SERIAL_BAUD    115200
#define VERSION  "1.0"
 
#define MSGBUFSIZE 64   // message buffersize, but for this demo we only use: 
                        // 1-byte NODEID + 4-bytes for time + 1-byte for temp in C + 2-bytes for vcc(mV)
 
Serial pc(PA_9, PA_10);
DigitalOut myled(PC_13);
DigitalOut but(PA_1, PullUp);
//RFM69::RFM69(PinName  PinName mosi, PinName miso, PinName sclk,slaveSelectPin, PinName int)
RFM69 radio(PB_15, PB_14, PB_13, PB_12, PA_8);

bool promiscuousMode = false; // set 'true' to sniff all packets on the same network
bool requestACK=false;
uint8_t data[sizeof(packet_t)];
packet_t packet;
Timer tmr;

typedef struct
{  
    
    uint8_t car;
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
    uint16_t dps_x;
    uint16_t dps_y;
    uint16_t dps_z;
    uint16_t rpm;
    uint16_t speed;
    uint8_t temperature;
    uint8_t flags;      // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    uint32_t timestamp;
} gui_t;

gui_t gui;
 
main() {
  memset(data,0,sizeof(packet_t));
  uint8_t theNodeID;
  tmr.start();
 
   pc.baud(SERIAL_BAUD);
   //pc.printf("\r\nListener %s startup at %d Mhz...\r\n",VERSION,(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915));
   wait(1);
   radio.initialize(FREQUENCY, GATEWAY_ID, NETWORKID);
   radio.encrypt(0);
   radio.promiscuous(promiscuousMode);
   radio.setPowerLevel(31);
#ifdef IS_RFM69HW
  radio.setHighPower(); //uncomment #define ONLY if radio is of type: RFM69HW or RFM69HCW 
#endif

while(1) {
  if (radio.receiveDone()) {
     
     //pc.printf("Received from TNODE: %d ",radio.SENDERID);
     memcpy(&data, (uint8_t *)radio.DATA, sizeof(packet_t));
     //memcpy(&packet, (packet_t *)data, sizeof(data));
     theNodeID = radio.SENDERID;
     
   /*
     gui.car = theNodeID;
     gui.acc_x = packet.imu[0].acc_x;
     gui.acc_y = packet.imu[0].acc_y;
     gui.acc_z = packet.imu[0].acc_z;
     gui.dps_x = packet.imu[0].dps_x;
     gui.dps_y = packet.imu[0].dps_y;
     gui.dps_z = packet.imu[0].dps_z;
     gui.rpm = packet.rpm;
     gui.speed = packet.speed;
     gui.temperature = packet.temperature;
     gui.flags = packet.flags;
     gui.timestamp = tmr.read_ms();
     */
     //memcpy(&data, &gui, sizeof(gui_t));
     
     pc.putc(theNodeID);
     for(int i = 0; i < sizeof(data); i++)
     {
        pc.putc(data[i]);
     }
     pc.putc(0xff);
//     pc.putc(0xff);
//     pc.putc(0xff);
        
/*    pc.printf("\r\ntemperature = %d\r\n",packet.temperature);
    pc.printf("\r\nspeed=%d\r\n", packet.speed);
    pc.printf("\r\nrpm=%d\r\n", packet.rpm);
    pc.printf("\r\nimu acc x =%d\r\n", packet.imu[0].acc_x);
    pc.printf("\r\nimu acc y =%d\r\n", packet.imu[0].acc_y);
    pc.printf("\r\nimu acc z =%d\r\n", packet.imu[0].acc_z);
    pc.printf("\r\nimu dps x =%d\r\n", packet.imu[0].dps_x);
    pc.printf("\r\nimu dps y =%d\r\n", packet.imu[0].dps_y);
    pc.printf("\r\nimu dps z =%d\r\n", packet.imu[0].dps_z);
*/     
     if (radio.ACKRequested()){
        theNodeID = radio.SENDERID;
        radio.sendACK();
     }
     myled = !myled;
     //pc.printf(" - ACK sent. Receive RSSI: %d\r\n",radio.RSSI);
  } 
 }
}