#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define NUM_PACKETS 20
// #define LOG 28

typedef struct
{
    int16_t acclsmx;
    int16_t acclsmy;
    int16_t acclsmz;
    int16_t anglsmx;
    int16_t anglsmy;
    int16_t anglsmz;
    uint16_t rpm;
    uint16_t speed;
    uint16_t temperature;
    uint16_t flags;
    uint32_t timestamp;
    } packet;

int main()
{
    char error, first = 0;
    int LOG, part = 0;
    char filename[50];
    char foldername[30];
    FILE *f, *fp;

    printf("Insira o nome da pasta em que se encontram os dados: ");
    scanf(" %s", foldername);
    while(1)
    {
        error = 0;
        part = 0;
        printf("Insira o número da corrida a ser lida (negativo para sair): ");
        scanf(" %d", &LOG);

        if(LOG < 0)
            break;

        sprintf(filename, "%s/LOG%d.csv", foldername, LOG);
        f = fopen(filename, "wt");
        // printf("file = %ld\r\n", f);
        
        fprintf(f, "lsmaccx,lsmaccy,lsmaccz,lsmangx,lsmangy,lsmangz,rpm,speed,temperature,flags,timestamp\n");
        while(!error)
        {
            char name[70];
            sprintf(name, "%s/%s%d/%s%d", foldername,"LOG", LOG, "data", part++);
            printf("filename = %s\n", name);
            fp = fopen(name, "r");
            packet x[NUM_PACKETS];
            
            if (fp == NULL)
            {
                error = 1;
                break;
            }   
        
            printf("~~~~~~~~data %d ~~~~~~~~", part);
            
            fread((void *)x, sizeof(packet), NUM_PACKETS, fp);
            for(int i = 0; i<(ftell(fp)/sizeof(packet)); i++)
            {
                fprintf(f, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", x[i].acclsmx, x[i].acclsmy, x[i].acclsmz, x[i].anglsmx, x[i].anglsmy, x[i].anglsmz, x[i].rpm, x[i].speed, x[i].temperature, x[i].flags, x[i].timestamp);
            }
            fclose(fp);
            fp = NULL;
        }
        fclose(f);
        f = NULL;
    }
    
    return 0;
}
