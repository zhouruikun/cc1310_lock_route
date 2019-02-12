/*
 * common.h
 *
 *  Created on: 2018年10月26日
 *      Author: Administrator
 */
#define DEBUG     0   //调试模式  忽略校验 忽略配对状态  不休眠
#ifndef COMMON_ROUTE_H_
#define COMMON_ROUTE_H_
#define FRAME_HEADER_L 0x0E
#define FRAME_HEADER_H 0xFA
#define FRAME_END 0x16
#define PAYLOAD_LENGTH 255
#define WIR_STATUS_PAIR   1
#define WIR_STATUS_NOMAL   2
#define SEND_OPEN_DOOR 1
#define SEND_PAIR 2
#define MAX_PAIR_LOCK  10
#define CMD_OK  0
#define CMD_CHKERR  1
#define CMD_NOTPAIR  2
typedef struct locksStruct{
    uint8_t lock_mac[6];
    uint32_t lock_rolling_code;
    uint8_t saved[6];
}locks_type;
typedef struct locks{
   locks_type locks_data[MAX_PAIR_LOCK];
   uint8_t locks_number;
}locks;
extern rfc_CMD_ABORT_t RF_cmdABORT;
extern uint8_t send_msg ;
uint8_t get_crypt_pack( uint8_t *crypt_pack, uint8_t * logicMacAddress,uint32_t rolling_code, uint32_t random);
uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random);
uint8_t checkPack( uint8_t* buff, size_t size);
uint8_t getCheckSum( uint8_t* buff, size_t size);
void setLogicMac( uint64_t PHY_Mac);
uint8_t send_pair_response(uint8_t * to,uint8_t status);
void pairKeyPressed(void);
void resetKeyPressed(void);
void openDoorKeyPressed(void);
void wirToLock( uint8_t* buff, size_t size);
void lockToWir(uint8_t * buffin,size_t size_in);
void readLockData(void);
void saveLockData(void);
uint8_t isPaired(uint8_t* mac);
void sendCrypedPack(uint8_t to,uint8_t param);
uint8_t addPairedLock(uint8_t* mac);
uint8_t send_syc_response(uint8_t from,uint8_t * buff);
uint8_t get_wir_status(void);
void set_wir_status(uint8_t sta);
void checkPair(void);
void cmd_response(uint8_t status);
void uart_485_send( const void *buffer, size_t size);
uint8_t removePairedLock(uint8_t lockid);
void send_to_rf(uint8_t *data,uint8_t len);
void protocol_rf_TaskInit();
void protocol_uart_TaskInit();
void uartTaskInit();
void * get_malloc(void);
void  free_malloc(void * ptr);
#endif /* COMMON_ROUTE_H_ */
