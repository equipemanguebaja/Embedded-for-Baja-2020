#include "mbed.h"
#include <stdio.h>
#include <errno.h>
#include "SDBlockDevice.h"
#include "FATFileSystem.h"

/* User libraries */
#include "definitions.h"
#include "CANMsg.h"

/* State type declaration */
typedef enum
{
    IDLE,       // Do nothing
    OPEN,       // Open archive
    SAVE,       // Save data
    CLOSE,      // Close archive
    CAN_STATE   // Enter CAN handler
} state_t;


/* Mbed OS Tools */
Timer t;

/* Communication protocols */
//Serial pc(PA_2, PA_3);
Serial bluetooth(PA_9, PA_10, 9600);
CAN can(PB_8, PB_9, 1000000);

/* Interrupt services routine */
void canISR();

/* Interrupt handlers */
void canHandler();

/* General functions*/
void setupInterrupts();
void filterMessage(CANMsg msg);
uint32_t count_files_in_sd(const char *fsrc);

/* I/O */
SDBlockDevice   sd(PB_15, PB_14, PB_13, PB_12);     // Mosi, miso, sck, cs
FATFileSystem   fileSystem("sd");                   // File system declaration
DigitalOut led(PC_13);                              // Led for debug
DigitalOut logger_on(PB_0);                              // Led for debug
DigitalOut debugging(PB_1);                              // Led for debug

/* Variables */
int err, svd_pck = 0;
packet_t data;
uint8_t bluet[sizeof(packet_t)];
state_t state = OPEN,
        last_state = IDLE;

int main() 
{
    logger_on = 1;                              // Device is on, so led is on
    memset(bluet,0,sizeof(packet_t));           // Prepare serial to send packets of our type
    int num_parts = 0,                          // Number of parts already saved
        num_files = 0,                          // Number of files in SD
        svd_pck = 0;                            // Number of saved packets (in current data part)
    char name_dir[12];                          // Name of current folder (new LOG)
    char name_file[20];                         // Name of current file (dataX)
    FILE *fp;
    bool first_open = true;                     // Sinalizes the first open to create a new directory
    setupInterrupts();
    CAN_IER &= ~CAN_IER_FMPIE0;                 // Disable RX interrupt
    
 /* Wait for SD mount */
    do
    {
        debugging = 0;                          // Debugging led is off untill SD mounted
        
        /* Try to mount the filesystem */
        //pc.printf("Mounting the filesystem... ");
        fflush(stdout);

        err = fileSystem.mount(&sd);
        //pc.printf("%s\n", (err ? "Fail" : "OK"));
        if (err)
        {
            /* Reformat if we can't mount the filesystem
            this should only happen on the first boot */
            //pc.printf("No filesystem found, formatting... ");
            fflush(stdout);
            err = fileSystem.reformat(&sd);
            //pc.printf("%s\n", (err ? "Fail" : "OK"));
            if (err) 
            {
                error("error: %s (%d)\n", strerror(-err), err);
            }
        }
    }while(err);   
    
    debugging = 1;      // Debugging led is on because SD is already mounted, device ready
    t.start();
    
    while(1)
    {
        switch (state)
        {
            case IDLE:
                if(last_state == CAN_STATE)
                {
                    CAN_IER |= CAN_IER_FMPIE0;          // Enable RX interrupt
                }
                last_state = IDLE;
                
                break;
            
            case OPEN:
                if(first_open)
                {
                    num_files = count_files_in_sd("/sd");
                    sprintf(name_dir, "%s%d", "/sd/LOG", num_files + 1);
                    /* Create RUN directory */
                    mkdir(name_dir, 0777);
                    first_open = false;
                }
            
                sprintf(name_file, "%s%s%d", name_dir, "/data", num_parts);
                num_parts++;
                fp = fopen(name_file, "a");
            
                if (fp == NULL)             // If it can't open the file then print error message
                {
                    led = 1;
                    //pc.printf("/r/nfp = null/r/n");
                }
             
                state = IDLE;
                last_state = OPEN;
                CAN_IER |= CAN_IER_FMPIE0;  // Enable RX interrupt
                
                break;
                
            case SAVE:
                debugging = !debugging;
                fwrite((void *)&data, sizeof(packet_t), 1, fp);         // Write a packet to the file
                
                memcpy(&bluet, (uint8_t *)&data, sizeof(packet_t)); // Makes narrow conversion to send uint8_t format packet by bluetooh
                
                
                for(int i = 0; i < sizeof(bluet); i++)
                {   
                    bluetooth.putc('e');                                // This char represents the start of a packet
                    bluetooth.putc(bluet[i]);                           // Send packet char by char
                    bluetooth.putc('d');                                // This char represents the end of a packet
               
                } 
                svd_pck++;
                if(svd_pck == 20)                                         // If 20 packets were wroten, close file
                {
                    svd_pck = 0;
                    last_state = SAVE;
                    state = CLOSE;
                }else
                {
                    last_state = SAVE;
                    state = IDLE;
                    CAN_IER |= CAN_IER_FMPIE0;                          // Enable RX interrupt
                }
                
                break;
                
            case CLOSE:
                CAN_IER &= ~CAN_IER_FMPIE0;                 // Disable RX interrupt
                fclose(fp);                                 // Close file
                //NVIC_SystemReset();                       // Reset the system
                last_state = CLOSE;
                state = OPEN;
                
                break;
                
            case CAN_STATE:
                
                //led =!led;
                state = IDLE;
                canHandler();
                last_state = CAN_STATE;
                
                break;
                
            default:
                break;
        }
    } 
}

void setupInterrupts() 
{
    can.attach(&canISR, CAN::RxIrq);
}

void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                   // Disable RX interrupt
    state = CAN_STATE;
}

void canHandler()
{
      CANMsg rxMsg;
      can.read(rxMsg);
      filterMessage(rxMsg);
      //CAN_IER |= CAN_IER_FMPIE0;                  // Enable RX interrupt
}

void filterMessage(CANMsg msg)
{
   
  if(msg.id == RPM_ID)
  {
      msg >> data.rpm;
      state = SAVE;
  }
  
  else if(msg.id == TEMPERATURE_ID)
  {
      msg >> data.temperature;
  }
  
  else if (msg.id == FLAGS_ID)
  {
      msg >> data.flags;
  }
  else if (msg.id == IMU_ACC_ID)
  {
      msg >> data.imu.acc_x >> data.imu.acc_y >> data.imu.acc_z;
  }

  else if (msg.id == IMU_DPS_ID)
  {
      msg >> data.imu.dps_x >> data.imu.dps_y >> data.imu.dps_z;
  }
  else if (msg.id == SPEED_ID)
  {
      msg >> data.speed;
  }
}

uint32_t count_files_in_sd(const char *fsrc)
{   
    DIR *d = opendir(fsrc);
    struct dirent *p;
    uint32_t counter = 0;
    
    while ((p = readdir(d)) != NULL)   
    {
        if(strcmp(p->d_name, ".Trash-1000"))
            counter++;
    }
    closedir(d);
    return counter;
}
