/*
 * common.c
 *
 *  Created on: 2018年10月26日
 *      Author: Administrator
 */
/* BIOS Header files */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
/* TI-RTOS Header files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/uart.h>

#include <ti/drivers/NVS.h>
#include <ti/display/Display.h>
#include <ti/devices/cc13x0/driverlib/crypto.h>
#include <ti/devices/cc13x0/driverlib/trng.h>
#include <ti/drivers/power/PowerCC26XX.h>
/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <common_route.h>
#include <crc.h>
#include "Board.h"
/*
 * 声明变量
 */
extern Semaphore_Handle auxSemaphoreHandle;
extern PIN_Handle ledPinHandle;
uint8_t send_buff[PAYLOAD_LENGTH +1];
extern uint64_t macAddress;
uint8_t logicMac[6];
static uint8_t wir_status = WIR_STATUS_NOMAL;
static const uint8_t aesKey[16] = {0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39};
static locks locks_all;
uint8_t send_msg = 0;
uint32_t random_number;
enum
{
    Encrypt             = 0x01,
    Decrypt             = 0x00,
    InterruptsEnabled   = 0x01,
    InterruptsDisabled  = 0x00,
};



/*
 *  ======== void wirToLock( uint8_t* buff, size_t size) ========
 *  处理485数据包
 *  para:
 *  1.uint8_t* buff：485数据包起始指针
 *  2.size_t size ：485数据包长度
 *
 */
void wirToLock( uint8_t* buff, size_t size)
{
    if(checkPack(buff,size)==true)
         {
            uint8_t to;
            to = isPaired(&buff[2]);
            if(DEBUG||to!=0xff)//判断是否已配对
                 {
                    cmd_response(CMD_OK);
                     switch(buff[10])
                     {
                     case 0x01://开锁命令  调用专用开锁指令
                         sendCrypedPack(to ,buff[11]);
                         break;
                     default:
                            send_buff[0] = size;
                             uint8_t i;
                            for (i = 1; i < send_buff[0] +1; i++)
                            {
                               send_buff[i] = buff[i-1];
                            }
                            /* Send packet */
                            send_to_rf(send_buff,send_buff[0]+1);
                            cmd_response(CMD_OK);
                           break;
                     }
                 }
            else{
                cmd_response(CMD_NOTPAIR);
            }
         }
        else
        {
            cmd_response(CMD_CHKERR);
        }
}
void cmd_response(uint8_t status)
{
//    uint8_t buff[14];
//    buff[0]=FRAME_HEADER_H;
//    buff[1]=FRAME_HEADER_L;
//    memset(& buff[2] , 0 , 6);
//    buff[7]=0;
//    buff[8]=0x0d;
//    buff[9]=0x20;
//    buff[10]=status;
//    buff[11]=getCheckSum(&buff[0],11);
//    buff[12]=FRAME_END;
//    uart_485_send(uart,buff, 13);
}

/*·
 *  ======== void lockToWir(char * buff,size_t size) ========
 *  处理锁端数据包
 *  para:
 *  1.uint8_t* buffin：锁端数据包起始指针
 *  2.size_t size_in ：锁端数据包长度
 */
void lockToWir(uint8_t * buff,size_t size)
{
    uint8_t from;
    /* Create packet with incrementing sequence number and random payload */
    if(checkPack(buff,size)==true)
    {

        switch(buff[10])
            {
                case 0x18://中继收到配对请求包  判断是否处于配对接收状态， 如果是 返回配对结果
                    //CPUdelay((uint32_t)((48000000/3)*1.010f));//需要延时  否则失败
                    // 任务休眠 1 秒，  1000000us, 下面函数的单位是10us
                     Task_sleep(100000);
                    if(wir_status == WIR_STATUS_PAIR)
                    {

                       addPairedLock(&buff[2]);
                       send_pair_response(&buff[2],0x01);
                       wir_status = WIR_STATUS_NOMAL;
                    }
                    else
                    {
                        send_pair_response(&buff[2],0x00);
                    }
                    break;
                case 0x1a://中继收到同步请求包  判断是否处于配对接收状态，
                    from = isPaired(&buff[2]);
                    if(from!=0xff)
                        send_syc_response(from,&buff[11]);
                    break;

                case 0x14:
                    //节点复位
                    from = isPaired(&buff[2]);
                    if(from!=0xff)//判断是否已配对
                        {
                        removePairedLock(from);
                        }
                    break;
                case 0x04://开锁包  指示灯
                    if(isPaired(&buff[2])!=0xff)//判断是否已配对
                        {
                        PIN_setOutputValue(ledPinHandle, Board_OPEN_LOCK_OUT_PIN, 1);
                        // 任务休眠 0.1 秒，  100000us, 下面函数的单位是10us
                           Task_sleep(10000);
                        PIN_setOutputValue(ledPinHandle, Board_OPEN_LOCK_OUT_PIN, 0);
                       }

                default:
                    if(DEBUG||isPaired(&buff[2])!=0xff)//判断是否已配对
                    {

                        uart_485_send(buff, size);

                    }
                    break;
            }
    }
}

/*
 *  ======== uint8_t send_pair_request(void) ========
 *  调用射频发送配对响应
 *  para:
 *  1 uint8_t * t0  ：目标MAC
 *  2 uint8_t status ：配对状态
 */
uint8_t send_pair_response(uint8_t * to,uint8_t status)
{
    send_buff[0] = 19;
    send_buff[1] = FRAME_HEADER_H;
    send_buff[2] = FRAME_HEADER_L;
    memcpy(&send_buff[3],to,6);
    send_buff[9] = 0;send_buff[10] = 19;send_buff[11] = 0x17;
    if(status == 0x00)
    {
      memset(&send_buff[12],0,6);
    }
    else
    {

      memcpy(&send_buff[12],logicMac,6);
    }

    send_buff[18] = getCheckSum(&send_buff[1],17);
    send_buff[19] = FRAME_END;
    /* Send packet */
    send_to_rf(send_buff,send_buff[0]+1);
    return 1;
}


 /*  ========uint8_t checkPack( uint8_t* buff, size_t size) ========
 *  根据数据包长度  桢头和CRC校验数据包完整
 *  para:
 *  1 uint8_t* buff：数据包地址
 *  2 size_t size ：数据包长度
 */
uint8_t checkPack( uint8_t* buff, size_t size)
{


    if((buff[9]+buff[8]*256)!=size)
    {
        return 0;
    }
    if(buff[0]==FRAME_HEADER_H&&buff[1]==FRAME_HEADER_L)//校验头部
    {
        if(DEBUG||(getCheckSum(  buff,  size-2)==buff[size-2]))//校验和
        {
             return  1;
        }
        else
        {
            return  0;
        }

    }
    else
    {
        return 0;
    }
}

/*
 *  ========uint8_t getCheckSum( uint8_t* buff, size_t size) ========
 *  获取校验和
 *    para:
 *   1 uint8_t* buff：数据包地址
 *  2 size_t size ：数据包长度
 */
uint8_t getCheckSum( uint8_t* buff, size_t size)
{
    uint8_t sum=0,i;
    for(i=0;i<size;i++)
    {
        sum += buff[i];
    }
    return sum;
}
/*
 *  ========void getLogicMac( uint64_t PHY_Mac,uint8_t *macBuff) ========
 *  从8字节MAC 获取6字节逻辑地址
 *    para:
 *   1 uint64_t PHY_Mac：6字节逻辑地址
 */
void setLogicMac( uint64_t PHY_Mac)
{
    CRC48( logicMac,(uint8_t *)&PHY_Mac, 8);

}
/*
 *  ========void pairKeyPressed( void) ========
 *  配对
 */
void pairKeyPressed(void)
{
   wir_status = WIR_STATUS_PAIR;
   send_msg = SEND_PAIR;
   Semaphore_post(auxSemaphoreHandle);// sendCrypedPack(0,0x01);
}
/*
 *  ========void resetKeyPressed( void) ========
 *  复位
 */
void resetKeyPressed(void)
{

    PIN_setOutputValue(ledPinHandle, Board_STATUS_PIN_LED0, 0);
    memset(&locks_all,0,sizeof(locks_all));
    saveLockData();
}
/*
 *  ========void openDoorKeyPressed( void) ========
 *   开锁输入
 */
void openDoorKeyPressed(void)
{
    if(DEBUG||locks_all.locks_number>0)
    {
       send_msg = SEND_OPEN_DOOR;
       Semaphore_post(auxSemaphoreHandle);// sendCrypedPack(0,0x01);
    }
}
/*
 *  ========void read_rolling_code( void) ========
 *  从flash读出滚动码
 */
void readLockData(void)
{
        NVS_Handle rHandle;
        NVS_Attrs regionAttrs;
        uint_fast16_t status;
            // Initialize the NVS driver
            NVS_init();
            rHandle = NVS_open(Board_NVSINTERNAL, NULL);
            // confirm that the NVS region opened properly
           if (rHandle == NULL) {
                // Error opening NVS driver
                while (1);
            }
            // fetch the generic NVS region attributes
            NVS_getAttrs(rHandle, &regionAttrs);
            status = NVS_read(rHandle, 0, (void *)&locks_all, sizeof(locks_all));
            if (status != NVS_STATUS_SUCCESS) {
                // Error handling code
            }
            // close the region
            NVS_close(rHandle);
            if(locks_all.locks_number == 0xff)
            {
                memset((void *)&locks_all,0 ,sizeof(locks_all));
                saveLockData();
            }
            checkPair();

}

/*
 *  ========void save_rolling_code( void) ========
 *  保存滚动码
 */
void saveLockData(void)
{
    NVS_Handle rHandle;
    NVS_Attrs regionAttrs;
    uint_fast16_t status;
   // Initialize the NVS driver
   NVS_init();
   rHandle = NVS_open(Board_NVSINTERNAL, NULL);
    // confirm that the NVS region opened properly
   if (rHandle == NULL) {
        // Error opening NVS driver
        while (1);
    }
    // fetch the generic NVS region attributes
    NVS_getAttrs(rHandle, &regionAttrs);
    // erase the first sector of the NVS region
    status = NVS_erase(rHandle, 0, regionAttrs.sectorSize);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }
    // write "Hello" to the base address of region 0, verify after write
    status = NVS_write(rHandle, 0, (void *)&locks_all, sizeof(locks_all), NVS_WRITE_POST_VERIFY);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }
    // close the region
    NVS_close(rHandle);
}
/*
 *  ========uint8_t isPaired(uint8_t* logicMac) ========
 *  判断是否有配对
 */
void checkPair(void)
{
    if(locks_all.locks_number>0)
                            PIN_setOutputValue(ledPinHandle, Board_STATUS_PIN_LED0, 1);
                        else
                            PIN_setOutputValue(ledPinHandle, Board_STATUS_PIN_LED0, 0);
}
/*
 *  ========uint8_t isPaired(uint8_t* logicMac) ========
 *  判断是否配对
 *  para:
 *  uint8_t* mac ：
 *  return : 已经配对返回序号  未配对返回0xff
 */
uint8_t isPaired(uint8_t* mac)
{
    uint8_t i=0;
   for(;i<locks_all.locks_number;i++)
   {
       if(memcmp(&locks_all.locks_data[i],mac,6)==0)
       {
           break;
       }
   }
   if (i==locks_all.locks_number)
   {
       return 0xff;
   }
   else
   {
       return i;
   }
}
/*
 *  ========uint8_t addPairedLock(uint8_t* logicMac) ========
 *  添加配对
 *  para:
 *  uint8_t* mac：要添加配对的MAC
 *  return : 成功配对返回序号  失败返回错误代码
 */
uint8_t addPairedLock(uint8_t* mac)
{
    if(isPaired(mac)==0xff)//判断是否已配对
      {
        if(locks_all.locks_number<MAX_PAIR_LOCK)
        {
            memcpy(locks_all.locks_data[locks_all.locks_number].lock_mac,mac,6);
            memcpy(locks_all.locks_data[locks_all.locks_number].saved,mac,6);
            locks_all.locks_data[locks_all.locks_number].lock_rolling_code = 0;
            locks_all.locks_number++;
            saveLockData();
        }
        else
        {
            return 0xf2;//满了
        }
      }
    else
    {
        return 0xf1;//已经配对
    }
    return 0xf1;//已经配对
}
/*
 *  ========uint8_t removePairedLock(uint8_t lockid) ========
 *  解除配对
 *  para:
 *  uint8_t lockid ：要删除的索引
 *  return : 成功配对返回序号  失败返回错误代码
 */
uint8_t removePairedLock(uint8_t lockid)
{
    uint8_t i;
            //循环前移数据
            for(i = lockid;i<locks_all.locks_number-1;i++)
            {
                memcpy(locks_all.locks_data[i].lock_mac,locks_all.locks_data[i+1].lock_mac,sizeof(locks_type));
            }

            locks_all.locks_number--;
            saveLockData();
            checkPair();
        return 0xf1;//已经配对

}
/*
 *  ======== uint8_t get_fleep_flag(void) ========
 *  无线休眠状态获取
 */
uint8_t get_wir_status(void)
{
return wir_status;
}
/*
 *  ======== void set_fleep_flag(uint8_t flag) ========
 *   无线休眠状态设置
 */
void set_wir_status(uint8_t sta)
{
  wir_status=sta;
}
/*
 *  ========void sendCrypedPack(uint8_t* to) ========
 *  调用射频发送加密开锁包
 *  para:
 *   uint8_t to   ：目标地址
 *   uint8_t param：开锁参数
 */
void sendCrypedPack(uint8_t to,uint8_t param)
{

    Power_setDependency(PowerCC26XX_PERIPH_TRNG);
    TRNGEnable();
    while (!(TRNGStatusGet() & TRNG_NUMBER_READY))
    {
        //wait for random number generator
    }
    random_number = TRNGNumberGet(TRNG_LOW_WORD);
    TRNGDisable();
    Power_releaseDependency(PowerCC26XX_PERIPH_TRNG);

    send_buff[0] = 30;
    send_buff[1] = FRAME_HEADER_H;
    send_buff[2] = FRAME_HEADER_L;
    memcpy(&send_buff[3],locks_all.locks_data[to].lock_mac,6);
    send_buff[9] = 0;send_buff[10] = 30;send_buff[11] = 0x01;send_buff[12] = param;
    locks_all.locks_data[to].lock_rolling_code += 1;
    saveLockData();
    get_crypt_pack( &send_buff[13],
                        locks_all.locks_data[to].lock_mac,
                        locks_all.locks_data[to].lock_rolling_code,
                        random_number
                        );
    send_buff[29] = getCheckSum(&send_buff[1],28);
    send_buff[30] = FRAME_END;

    /* Send packet */
    send_to_rf(send_buff,send_buff[0]+1);
}
/*
 *  ======== uint8_t send_syc_response(uint8_t from,uint8_t * buff) ========
 *  接收到同步请求后 解析 同步 然后发送开锁
 *  para:
 *  uint8_t from    源主机MAC
 *  uint8_t * buff   加密数据包
 */
uint8_t send_syc_response(uint8_t from,uint8_t * buff)
{
    uint32_t rolling_temp = 0;
    uint32_t random = 0;
    uint8_t mac_temp[6];
    get_crypt_data(buff, mac_temp,&rolling_temp,&random);
    if(random == random_number)
    {
        locks_all.locks_data[from].lock_rolling_code = rolling_temp;
        saveLockData();
        sendCrypedPack(from,0x01);
    }
    return 1;
}


/*
 *  ======== uint8_t get_crypt_pack( uint8_t *crypt_pack, uint64_t macAddress,uint32_t rolling_code) ========
 *  调用加密模块获取加密包
 *  para:
 *  1uint8_t *crypt_pack  :加密数据包
 *  2 uint8_t * logicMacAddress ：逻辑MAC地址
 *  3 uint32_t rolling_code：滚动码
 *  4 uint32_t random：随机数
 */
uint8_t get_crypt_pack( uint8_t *crypt_pack, uint8_t * logicMacAddress,uint32_t rolling_code, uint32_t random)
{
    uint8_t AEC_input[16];
    uint8_t keyIndex = CRYPTO_KEY_AREA_3;
    memcpy(&AEC_input[0],logicMacAddress,6);
    memcpy(&AEC_input[6],(uint8_t *)&rolling_code,4);
    memcpy(&AEC_input[10],(uint8_t *)&random,4);
    memset(&AEC_input[14],0,2);
    Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);
          // 禁止待机 , 加密的时候才需要这个代码，加密结束之后，释放，允许进入待机就可以了
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    // Load the key to the crypto unit
    CRYPTOAesLoadKey((uint32_t*)aesKey, keyIndex);
    CRYPTOAesEcb((uint32_t*)AEC_input, (uint32_t*)crypt_pack, keyIndex, Encrypt, InterruptsDisabled);
    while (CRYPTOAesEcbStatus() != AES_SUCCESS);
    CRYPTOAesEcbFinish();
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
    Power_releaseDependency(PowerCC26XX_PERIPH_CRYPTO);
    return 1;
}

/*
 *  ========uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random) ========
 *  调用解密模块获取加密前数据
 *   para:
 *  1uint8_t *crypt_pack  :加密数据包
 *  2 uint8_t * logicMacAddress ：逻辑MAC地址
 *  3 uint32_t *rolling_code：滚动码地址
 *  4 uint32_t *random：随机数地址
 */
uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random)
{
    uint8_t AEC_output[16];
    uint8_t keyIndex = CRYPTO_KEY_AREA_3;
    Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);
          // 禁止待机 , 加密的时候才需要这个代码，加密结束之后，释放，允许进入待机就可以了
    Power_setConstraint(PowerCC26XX_SB_DISALLOW);
    // Load the key to the crypto unit
    CRYPTOAesLoadKey((uint32_t*)aesKey, keyIndex);
    CRYPTOAesEcb((uint32_t*)crypted_pack, (uint32_t*)AEC_output, keyIndex, Decrypt, InterruptsDisabled);
    while (CRYPTOAesEcbStatus() != AES_SUCCESS);
    CRYPTOAesEcbFinish();
    Power_releaseConstraint(PowerCC26XX_SB_DISALLOW);
    Power_releaseDependency(PowerCC26XX_PERIPH_CRYPTO);
    memcpy(logicMacAddress,&AEC_output[0],6);
    memcpy((uint8_t *)rolling_code,&AEC_output[6],4);
    memcpy((uint8_t *)random,&AEC_output[10],4);
    return 1;
}
