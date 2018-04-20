#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../Inc/parser.h"  

struct GPSObj parser(char data[])
{
    const char s1[2] = "\r";
    const char s2[2] = ",";
    char *token1, *token2;
    char *saveptr1, *saveptr2;
    token1 = strtok_r(data, s1, &saveptr1);
    char des[80];
    struct GPSObj r;
    while (token1 != NULL)
    {
        strcpy(des, token1);
        token2 = strtok_r(des, s2, &saveptr2);
        const char *t[15];
        int i = 0;
        if (strncmp(token2, "$GNRMC", 7) == 0)
        {
            while (token2)
            {
                t[i] = token2;
                token2 = strtok_r(NULL, s2, &saveptr2);
                i++;
            }
            i = 0;
            if (strncmp(t[2], "A", 2) == 0)
            {
                r._lat = atof(t[3]);
                r._long = atof(t[5]);
            }
            else {
                r._lat = 0.0;
                r._long = 0.0;
            }
        }
        token1 = strtok_r(NULL, s1, &saveptr1);
    }
    return r;
}

int main(void)
{
    char data[] =
        "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r"
        "$GNVTG,0.00,T,,M,0.00,N,0.00,K,N*2C\r"
        "$GNGGA,000443.100,,,,,0,0,,,M,,M,,*54\r"
        "$GPGSA,A,1,,,,,,,,,,,,,,,*1E\r"
        "$GLGSA,A,1,,,,,,,,,,,,,,,*02\r"
        "$GAGSA,A,1,,,,,,,,,,,,,,,*0F\r"
        "$GPGSV,1,1,00*79\r"
        "$GLGSV,1,1,00*65\r"
        "$GAGSV,1,1,00*68\r"
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r";
    struct GPSObj r = parser(data);
    printf("(%.6f,%.6f)\n", r._lat, r._long);
    return 0;
}
