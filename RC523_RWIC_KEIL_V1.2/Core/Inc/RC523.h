#ifndef __RC523_H__
#define __RC523_H__
#include "main.h"
#include "stdio.h"

/////////////////////////////////////////////////////////////////////
// MF523命令字
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE 0x00       // 取消当前命令
#define PCD_AUTHENT 0x0E    // 验证密钥
#define PCD_RECEIVE 0x08    // 接收数据
#define PCD_TRANSMIT 0x04   // 发送数据
#define PCD_TRANSCEIVE 0x0C // 发送并接收数据
#define PCD_RESETPHASE 0x0F // 复位
#define PCD_CALCCRC 0x03    // CRC计算

/////////////////////////////////////////////////////////////////////
// Mifare_One卡片命令字
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL 0x26    // 寻天线区内未进入休眠状态
#define PICC_REQALL 0x52    // 寻天线区内全部卡
#define PICC_ANTICOLL1 0x93 // 防冲撞
#define PICC_ANTICOLL2 0x95 // 防冲撞
#define PICC_AUTHENT1A 0x60 // 验证A密钥
#define PICC_AUTHENT1B 0x61 // 验证B密钥
#define PICC_READ 0x30      // 读块
#define PICC_WRITE 0xA0     // 写块
#define PICC_DECREMENT 0xC0 // 扣款
#define PICC_INCREMENT 0xC1 // 充值
#define PICC_RESTORE 0xC2   // 调块数据到缓冲区
#define PICC_TRANSFER 0xB0  // 保存缓冲区中数据
#define PICC_HALT 0x50      // 休眠

/////////////////////////////////////////////////////////////////////
// MF523 FIFO长度定义
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH 64 // FIFO size=64byte
#define MAXRLEN 18

/////////////////////////////////////////////////////////////////////
// MF523寄存器定义
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define RFU00 0x00
#define CommandReg 0x01
#define ComIEnReg 0x02
#define DivlEnReg 0x03
#define ComIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define RFU0F 0x0F
// PAGE 1
#define RFU10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define RFU1A 0x1A
#define RFU1B 0x1B
#define MifareReg 0x1C
#define RFU1D 0x1D
#define RFU1E 0x1E
#define SerialSpeedReg 0x1F
// PAGE 2
#define RFU20 0x20
#define CRCResultRegM 0x21
#define CRCResultRegL 0x22
#define RFU23 0x23
#define ModWidthReg 0x24
#define RFU25 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsCfgReg 0x28
#define ModGsCfgReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F
// PAGE 3
#define RFU30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define RFU3C 0x3C
#define RFU3D 0x3D
#define RFU3E 0x3E
#define RFU3F 0x3F

#define REQ_ALL 0x52
#define KEYA 0x60
#define KEYB 0x61

/////////////////////////////////////////////////////////////////////
// 和MF532通讯时返回的错误代码
/////////////////////////////////////////////////////////////////////
#define MI_OK 0
#define MI_NOTAGERR (1)
#define MI_ERR (2)

#define SHAQU1 0X01
#define KUAI4 0X04
#define KUAI7 0X07
#define REGCARD 0xa1
#define CONSUME 0xa2
#define READCARD 0xa3
#define ADDMONEY 0xa4
#define RC_ISO14443_A 'A'

#define SET_SPI_CS (GPIOF->BSRR = 0X01)
#define CLR_SPI_CS (GPIOF->BRR = 0X01)

#define SET_RC522RST GPIOF->BSRR = 0X02
#define CLR_RC522RST GPIOF->BRR = 0X02

/***********************RC523 函数宏定义**********************/

#define RC523_NSS_Enable() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
#define RC523_NSS_Disable() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

#define RC523_Reset_Enable() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
#define RC523_Reset_Disable() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

#define RC523_SCK_0() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
#define RC523_SCK_1() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

#define RC523_MOSI_0() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
#define RC523_MOSI_1() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

#define RC523_MISO_GET() HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);

#define DEBUG

void RC522_Handle(void);     // 测试程序0，完成addr读写读
void RC522_Handle1(void);    // 测试程序1，完成0x0F块 验证KEY_A、KEY_B 读 写RFID1 验证KEY_A1、KEY_B1 读 写RFID2
void RC522_data_break(void); // 测试用数据爆破程序，仅供学习参考，请勿非法使用

void RC523_Init(void);                                    // 初始化
void PcdReset(void);                                      // 复位
void PcdConfigISOType(uint8_t iType);                     // 工作方式
uint8_t ReadReg(uint8_t Address);
void WriteReg(uint8_t Address, uint8_t Value);
uint8_t PcdRequest(uint8_t iReq_code, uint8_t *pTagType); // 寻卡
uint8_t PcdAnticoll(uint8_t *pSnr);                       // 读卡号
uint8_t PcdSelect(uint8_t *pSnr);
uint8_t PcdAuthState(uint8_t iAuth_mode, uint8_t iAddr, uint8_t *pKey, uint8_t *pSnr);
uint8_t PcdWrite(uint8_t iAddr, uint8_t *pData);
uint8_t PcdRead(uint8_t iAddr, uint8_t *pData);
void ShowID(uint8_t *p);                                                                                // 显示卡的卡号，以十六进制显示
void WaitCardOff(void);                                                                                 // 等待卡离开
void IC_RW(uint8_t *UID, uint8_t key_type, uint8_t *KEY, uint8_t RW, uint8_t data_addr, uint8_t *data); // UID为你要修改的卡的UID key_type：0为KEYA，非0为KEYB KEY为密钥 RW:1是读，0是写 data_addr为修改的地址 data为数据内容
void SelectAntennaOn(GPIO_TypeDef* GPIOx_OA, uint16_t GPIO_Pin_OA,GPIO_TypeDef* GPIOx_OB, uint16_t GPIO_Pin_OB);
void SelectAntennaOff(GPIO_TypeDef* GPIOx_OA, uint16_t GPIO_Pin_OA,GPIO_TypeDef* GPIOx_OB, uint16_t GPIO_Pin_OB);
void SelectAllAntennaOff(void);
void WDataInit(void);
void RC523Task(void);
void RC523SelectChannlxRWTask(uint8_t channlx, uint8_t *ReadData, uint8_t* WriteData);
void RC523SelectChannlxReadTask(uint8_t channlx, uint8_t *ReadData);
void RC523SelectChannlxWriteTask(uint8_t channlx, uint8_t *WriteData);
void ScanChannlx(void);
extern char *POINT_LNG;
extern char *POINT_LAT;
extern char *POINT_LNG_ON;
extern char *POINT_LAT_ON;
extern char *POINT_LNG_OFF;
extern char *POINT_LAT_OFF;

#endif /*__RC523_H*/
