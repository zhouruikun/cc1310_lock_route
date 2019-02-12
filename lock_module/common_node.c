/*
 * common.c
 *
 *  Created on: 2018��10��26��
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
#include <ti/drivers/power/PowerCC26XX.h>
/* Standard C Libraries */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <common_node.h>
#include <crc.h>
#include "Board.h"
/*
 * ��������
 */
extern UART_Handle uart;
uint8_t send_buff[PAYLOAD_LENGTH +1];
extern rfc_CMD_PROP_TX_ADV_t RF_cmdPropTxAdv;
extern RF_Handle rfHandle;
extern Semaphore_Handle txSemaphoreHandle;
extern uint64_t macAddress;
extern UART_Params uartParams;
uint8_t logicMac[6];
locks_status_type locks_status;
uint32_t random_number;
uint8_t sleep_flag = STATUS_WORKING;
extern PIN_Handle outPinHandle;
static const uint8_t aesKey[16] = {0x5a, 0x69, 0x67, 0x42, 0x65, 0x65, 0x41, 0x6c, 0x6c, 0x69, 0x61, 0x6e, 0x63, 0x65, 0x30, 0x39};
enum
{
    Encrypt             = 0x01,
    Decrypt             = 0x00,
    InterruptsEnabled   = 0x01,
    InterruptsDisabled  = 0x00,
};

/*
 *  ======== void wirToLock( uint8_t* buff, size_t size) ========
 *  ����������ݰ�
 *  para:
 *  1.uint8_t* buff���������ݰ���ʼָ��
 *  2.size_t size ���������ݰ�����
 *
 */
void wirToLock( uint8_t* buff, size_t size)
{
    uint8_t temp[6];
    if(checkPack(buff,size,TO_LOCK)==true)
    {
        Semaphore_post(txSemaphoreHandle);
         set_pin_mode(PIN_OUT);
         PIN_setOutputValue(outPinHandle, Board_PIN_D_CE, 0);
         Task_sleep(5000);
         PIN_setOutputValue(outPinHandle, Board_PIN_D_CE, 1);
         set_pin_mode(PIN_IN);
        switch(buff[10])
            {
                case 0x01://����ͬ���벢���͸�������
                     parse_cryped_pack(&buff[11]);
                    break;
                case 0x17:

                    memset(temp,0,6);
                    if(memcmp(&buff[11],temp,6)==0)
                    {
                        get_pair_response(0x00,0x00ff);//���ʧ��
                    }
                    else
                    {
                        get_pair_response(0x00,0x0000);//��Գɹ�
                    }
                    break;
                default:
                    buff[0] = FRAME_HEADER_LOCK_H;
                    buff[1] = FRAME_HEADER_LOCK_L;
                    memset(&buff[2],0,6);
                    //insert crc
                    buff[size-2] = getCheckSum( buff,size-2);
                    UART_write(uart,buff, size);
                    break;
            }
    }
}



/*��
 *  ======== void lockToWir(char * buff,size_t size) ========
 *  �����������ݰ�
 *  para:
 *  1.uint8_t* buffin���������ݰ���ʼָ��
 *  2.size_t size_in ���������ݰ�����
 */
void lockToWir(uint8_t * buffin,size_t size_in)
{
    uint16_t index=0;
    int16_t size,size_remine;
    uint8_t * buff;
    size_remine = size_in;
    /* Create packet with incrementing sequence number and random payload */

    while((buff = findCMD(buffin+index,size_remine))!=0)
    {
        size = (buff[9]+buff[8]*256);
        if(size <= 0 ||size >500)
        {
            break;
        }
        index += (size-1);
        if(size_remine>=size)
            size_remine -= size;
        else
            size_remine = 0;
        if(checkPack(buff,size,TO_WIR)==true)
           {
               switch(buff[10])
               {
               case 0x0d://wifi��������    ���̷���wifi����  �������ñ��ĺ��˳�
                   get_pair_response(0x01,0x0000);
                   send_pair_request();
                   break;
               case 0x15://���̷���wifi״̬
                   wifi_status_response();
                   break;
               case 0x06://�ж�����
                   if(buff[11]==0x02)
                       sleep_flag = STATUS_SLEEPING;
               case 0x0c:

               default://�޸�֡ͷ�� ͸����������
                   send_buff[0] = size;
                    uint8_t i;
                   for (i = 1; i < send_buff[0] +1; i++)
                   {
                      send_buff[i] = buff[i-1];
                   }
                   send_buff[1] = FRAME_HEADER_H;
                   send_buff[2] = FRAME_HEADER_L;
                   memcpy(&send_buff[3],logicMac,6);
                   send_buff[size-1] = getCheckSum(  &send_buff[1] ,size-2);
                   send_buff[size] = FRAME_END;
                   send_to_rf(send_buff,send_buff[0]+1);

                  break;
               }


       }
    }

}
/*
 *  ======== char * findCMD(char * buff,size_t size) ========
 *  para:�ڲ������� �������ݰ�������ͷ�����ص�ַ��
 *         para1:buff
 *         para2:size
 */
uint8_t * findCMD(uint8_t * buff,size_t size)
{
    uint8_t index =0;
    for(;index<size;index++)
    {
        if(buff[index]==FRAME_HEADER_LOCK_H&&buff[index+1]==FRAME_HEADER_LOCK_L)
        {
            return buff+index;
        }
    }
    return 0;
}
/*
 *  ======== uint8_t wifi_status_response(void) ========
 *  para:���ô��ڷ���wifi״̬
 *
 */
uint8_t wifi_status_response(void)
{
    uint8_t buff[20];
    buff[0] = FRAME_HEADER_LOCK_H;
    buff[1] = FRAME_HEADER_LOCK_L;
    memset(&buff[2],0,6);
    buff[8]=0x00; buff[9]=0x10;
    buff[10]=0x16; buff[11]=0x0; buff[12]=0x00,buff[13]=0x00;buff[14]=getCheckSum(buff,14);buff[15]=0x16;
    UART_write(uart,buff, 16);
    return 1;
}
/*
 *  ======== uint8_t get_pair_response(void) ========
 *  para:���ô��ڷ������״̬
 *         para1:para1��״̬�ֽ�1
 *         para2:para2��״̬�ֽ�2
 */
uint8_t get_pair_response(uint8_t para1,uint16_t para2)
{
    uint8_t buff[20];
    buff[0] = FRAME_HEADER_LOCK_H;
    buff[1] = FRAME_HEADER_LOCK_L;
    memset(&buff[2],0,6);
    buff[8]=0x00; buff[9]=0x0e;
    buff[10]=0x0e;
    buff[11]=para1,buff[12]=para2/256;buff[13]=para2%256;buff[14]=getCheckSum(buff,14);buff[15]=0x16;
    UART_write(uart,buff,16);
    return 1;
}
/*
 *  ======== uint8_t send_pair_request(void) ========
 *  ������Ƶ �����������
 */
uint8_t send_pair_request( void)
{

    send_buff[0] = 19;
    send_buff[1] = FRAME_HEADER_H;
    send_buff[2] = FRAME_HEADER_L;
    memcpy(&send_buff[3],logicMac,6);
    send_buff[9] = 0;send_buff[10] = 19;send_buff[11] = 0x18;
    memcpy(&send_buff[12],logicMac,6);
    send_buff[18] = getCheckSum(&send_buff[1],17);
    send_buff[19] = FRAME_END;
    send_to_rf(send_buff,send_buff[0]+1);
    return 1;
}
/*
 *  ======== uint8_t send_syc_request(void) ========
 *  ������Ƶ ����ͬ������
 */
uint8_t send_syc_request(void)
{
    send_buff[0] = 29;
    send_buff[1] = FRAME_HEADER_H;
    send_buff[2] = FRAME_HEADER_L;
    memcpy(&send_buff[3],&logicMac,6);
    send_buff[9] = 0;send_buff[10] = 29;send_buff[11] = 0x1a;
    get_crypt_pack( &send_buff[12],
                    logicMac,
                    locks_status.rolling_code,
                    random_number);
    send_buff[28] = getCheckSum(&send_buff[1],27);
    send_buff[29] = FRAME_END;
    send_to_rf(send_buff,send_buff[0]+1);
    return 1;
}

/*
 *  ========uint8_t checkPack( uint8_t* buff, size_t size) ========
 *  �������ݰ����򣬳���  ��ͷ��CRCУ�����ݰ�����
 *  para:
 *   1 uint8_t* buff�����ݰ���ַ
 *  2 size_t size �����ݰ�����
 */
uint8_t checkPack( uint8_t* buff, size_t size,uint8_t dir)
{


    if((buff[9]+buff[8]*256)!=size)
    {
        return 0;
    }
    if(dir==TO_LOCK)
    {
        if(buff[0]==FRAME_HEADER_H&&buff[1]==FRAME_HEADER_L)//У��ͷ��
        {
            if(DEBUG||getCheckSum(  buff,  size-2)==buff[size-2])//У���
            {
                //���յ����������ݻ���ҪУ��MAC�Ƿ�Ϊ�Լ�
                if(DEBUG||memcmp(&buff[2],logicMac,6)==0)
                    return  1;
                else
                    return 0;
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
    else
    {
        if(buff[0]==FRAME_HEADER_LOCK_H&&buff[1]==FRAME_HEADER_LOCK_L)//У��ͷ��
          {
              if(DEBUG||getCheckSum(  buff,  size-2)==buff[size-2])//У���
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

}

/*
 *  ========uint8_t getCheckSum( uint8_t* buff, size_t size) ========
 *  ��ȡУ���
 *    para:
 *   1 uint8_t* buff�����ݰ���ַ
 *  2 size_t size �����ݰ�����
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
 *  ��8�ֽ�MAC ��ȡ6�ֽ��߼���ַ
 *    para:
 *   1 uint64_t PHY_Mac��6�ֽ��߼���ַ
 */
void setLogicMac( uint64_t PHY_Mac)
{
    CRC48( logicMac,(uint8_t *)&PHY_Mac, 8);


}
/*
 *  ========void send_open_lock( uint8_t param) ========
 *  ���ô��ڷ��Ϳ���ָ��
 *   para:
 *   1 uint8_t param����������
 */
void send_open_lock(uint8_t param)
{
    uint8_t buff[20];
    buff[0] = FRAME_HEADER_LOCK_H;
    buff[1] = FRAME_HEADER_LOCK_L;
    memset(&buff[2],0,6);
    buff[8]=0x00; buff[9]=0x0e;
    buff[10]=0x01;
    buff[11]=param;buff[12]=getCheckSum(buff,12);buff[13]=0x16;
    UART_write(uart,buff,14);

}
/*
 *  ========void parse_cryped_pack( uint8_t *buff) ========
 *  ���������������ݰ�
 *  para:
 *  uint8_t *buff �����ݰ�ָ��
 */
void parse_cryped_pack(uint8_t *buff)
{
    uint32_t rolling_temp = 0;
    uint8_t mac_temp[6];
    get_crypt_data(&buff[1], mac_temp,&rolling_temp,&random_number);
    if(rolling_temp<=locks_status.rolling_code||(rolling_temp-locks_status.rolling_code)>5)//ͬ�������  ����ͬ������
    {
        send_syc_request();
        return ;
    }
    locks_status.rolling_code = rolling_temp ;
    save_rolling_code();
    //ͬ������ȷ  �������˿���ָ��
    send_open_lock(buff[0]);
}

/*
 *  ========void read_rolling_code( void) ========
 *  ��flash����������
 */
void read_rolling_code(void)
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
            status = NVS_read(rHandle, 0, (void *)&locks_status, sizeof(locks_status));
            if (status != NVS_STATUS_SUCCESS) {
                // Error handling code
            }
            // close the region
            NVS_close(rHandle);
            if(locks_status.rolling_code == 0xffffffff)
            {
                memset((void *)&locks_status,0 ,sizeof(locks_status));
                save_rolling_code();
            }
}
/*
 *  ========void save_rolling_code( void) ========
 *  ���������
 */
void save_rolling_code(void)
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
    status = NVS_write(rHandle, 0, (void *)&locks_status, sizeof(locks_status), NVS_WRITE_POST_VERIFY);
    if (status != NVS_STATUS_SUCCESS) {
        // Error handling code
    }
    // close the region
    NVS_close(rHandle);
}
/*
 *  ======== uint8_t get_fleep_flag(void) ========
 *  ��������״̬��ȡ
 */
uint8_t get_sleep_flag(void)
{
    return sleep_flag;
}
/*
 *  ======== void set_fleep_flag(uint8_t flag) ========
 *   ��������״̬����
 */
void set_sleep_flag(uint8_t flag)
{
    sleep_flag = flag;
}
/*
 *  ======== uint8_t get_crypt_pack( uint8_t *crypt_pack, uint64_t macAddress,uint32_t rolling_code) ========
 *  ���ü���ģ���ȡ���ܰ�
 *  para:
 *  1uint8_t *crypt_pack  :�������ݰ�
 *  2 uint8_t * logicMacAddress ���߼�MAC��ַ
 *  3 uint32_t rolling_code��������
 *  4 uint32_t random�������
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
          // ��ֹ���� , ���ܵ�ʱ�����Ҫ������룬���ܽ���֮���ͷţ������������Ϳ�����
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
 *  ======== uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random) ========
 *  ���ý���ģ���ȡ����ǰ����
 *   para:
 *  1uint8_t *crypt_pack  :�������ݰ�
 *  2 uint8_t * logicMacAddress ���߼�MAC��ַ
 *  3 uint32_t *rolling_code���������ַ
 *  4 uint32_t *random���������ַ
 */
uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random)
{
    uint8_t AEC_output[16];
    uint8_t keyIndex = CRYPTO_KEY_AREA_3;
    Power_setDependency(PowerCC26XX_PERIPH_CRYPTO);
          // ��ֹ���� , ���ܵ�ʱ�����Ҫ������룬���ܽ���֮���ͷţ������������Ϳ�����
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
