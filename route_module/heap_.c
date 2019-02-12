

#include <stdint.h>
#include <stdlib.h>

/* Application Header files */
#include "RFQueue.h"
#include <ti/devices/DeviceFamily.h>
#define PAYLOAD_LEN   254
#define SIZE   5
#define UNUSED 0
#define USED 1
typedef struct MsgObj {
     uint8_t packetData[PAYLOAD_LEN];
     uint8_t used;
} MsgObjd;
MsgObjd msgs[SIZE];


void * get_malloc(void)
{
    uint32_t i;
    for (i = 0; i < SIZE; i++)
    {
        /* Find the first available entry in the command pool */
        if (!(msgs[i].used & USED))
        {
            msgs[i].used = USED;
            return &msgs[i];
        }
    }
    return NULL;
}

void  free_malloc(void * ptr)
{
    uint32_t i;
    for (i = 0; i < SIZE; i++)
        {
            /* Find the first available entry in the command pool */
            if ((void *)&msgs[i]==ptr)
            {
                msgs[i].used = UNUSED;
            }
        }
}
