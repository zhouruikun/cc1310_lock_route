/*
 * common.h
 *
 *  Created on: 2018年10月26日
 *      Author: Administrator
 */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#define DEBUG     0   //调试模式  忽略校验 忽略配对状态  不休眠
#ifndef COMMON_NODE_H_
#define COMMON_NODE_H_
#define FRAME_HEADER_L 0x0E     //外部协议
#define FRAME_HEADER_H 0xFA
#define FRAME_HEADER_LOCK_L 0x01 //内部协议
#define FRAME_HEADER_LOCK_H 0xFA
#define TO_LOCK   1
#define TO_WIR    0
#define PIN_IN 0
#define PIN_OUT 1
#define FRAME_END 0x16
#define PAYLOAD_LENGTH 255
#define STATUS_WORKING 2
#define STATUS_SLEEPING 3
#define SLEEP_TIMES         2
#define WAKE_TIMES          2
typedef struct {
    uint32_t rolling_code;
   uint8_t locks_status;
   uint8_t save[11];
}locks_status_type;
uint8_t checkPack( uint8_t* buff, size_t size,uint8_t dir);
uint8_t getCheckSum( uint8_t* buff, size_t size);
void setLogicMac( uint64_t PHY_Mac);
uint8_t send_pair_request( void);
uint8_t get_pair_response(uint8_t para1,uint16_t para2);
uint8_t wifi_status_response(void);
uint8_t get_crypt_data( uint8_t *crypted_pack, uint8_t *logicMacAddress,uint32_t *rolling_code,uint32_t *random);
void read_rolling_code(void);
uint8_t send_syc_request(void);
uint8_t get_sleep_flag(void);
void save_rolling_code(void);
void parse_cryped_pack(uint8_t *buff);
uint8_t * findCMD(uint8_t * buff,size_t size);
void set_sleep_flag(uint8_t flag);
void set_pin_mode(uint8_t mode);
void lockToWir(uint8_t * buffin,size_t size_in);
void wirToLock( uint8_t* buff, size_t size);
void send_to_rf(uint8_t *data,uint8_t len);
void rxTaskInit();
uint8_t get_crypt_pack( uint8_t *crypt_pack, uint8_t * logicMacAddress,uint32_t rolling_code, uint32_t random);
void protocol_rf_TaskInit();
void protocol_uart_TaskInit();


void * get_malloc(void);
void  free_malloc(void * ptr);
#endif /* COMMON_NODE_H_ */
