#include "RC523.h"
#include "spi.h"
#include "string.h"

uint8_t icType[2];
uint8_t UID[4];
uint8_t KEY_A[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t addr = 0x01;
uint8_t RData[16];
uint8_t WData[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
extern uint8_t RAllData[96];
extern uint8_t WAllData[96];

enum CHANNLX
{
	O1 = 0,
	O2,
	O3,
	O4,
	O5,
	O6,
	O7,
	O8,
	CHANNL_NUM
};

GPIO_TypeDef* Channlx_Port[CHANNL_NUM][2] = 
{	
	{O1A_GPIO_Port,O1B_GPIO_Port},
	{O2A_GPIO_Port,O2B_GPIO_Port},
	{O3A_GPIO_Port,O3B_GPIO_Port},
	{O4A_GPIO_Port,O4B_GPIO_Port},
	{O5A_GPIO_Port,O5B_GPIO_Port},
	{O6A_GPIO_Port,O6B_GPIO_Port},
	{O7A_GPIO_Port,O7B_GPIO_Port},
	{O8A_GPIO_Port,O8B_GPIO_Port}
};

uint16_t Channlx_Pin[CHANNL_NUM][2] =
{	
	{O1A_Pin,O1B_Pin},
	{O2A_Pin,O2B_Pin},
	{O3A_Pin,O3B_Pin},
	{O4A_Pin,O4B_Pin},
	{O5A_Pin,O5B_Pin},
	{O6A_Pin,O6B_Pin},
	{O7A_Pin,O7B_Pin},
	{O8A_Pin,O8B_Pin}
};


void WDataInit(void)
{
	uint8_t i = 0;
	for(i = 0; i < 96; i++)
	{
		WAllData[i] = i;
	}
}
	

/**
 * @brief 初始化RC523模块
 */
void RC523_Init(void) //
{
    RC523_Reset_Disable();           // 禁用RC523模块的复位功能
    RC523_NSS_Disable();             // 禁用NSS引脚，控制模块的通信状态
    PcdReset();                      // 对模块进行复位操作
    PcdConfigISOType(RC_ISO14443_A); // 设置工作方式
    SelectAllAntennaOff();
    SelectAntennaOn(O8A_GPIO_Port, O8A_Pin, O8B_GPIO_Port, O8B_Pin); // 默认选择天线1开启							 // 选择天线
#ifdef Debug
    printf("\r\n RC523_Init Success \r\n");
#endif
}

/**
 * @brief 通过SPI发送一个字节到RC523芯片
 * @param byte 要发送的字节数据
 */
static void SPI_RC523_SendByte(uint8_t byte) //
{
    // 使用SPI1发送一个字节的数据，等待时间最长为100毫秒
    HAL_SPI_Transmit(&hspi1, &byte, 1, 1000);
}

/**
 * @brief 从SPI_RC523中读取一个字节的数据。
 * @return 从SPI_RC523设备接收到的一个字节数据。
 */
static uint8_t SPI_RC523_ReadByte(void) //
{
    uint8_t SPI_Data;
    HAL_SPI_Receive(&hspi1, &SPI_Data, 1, 1000);
    return SPI_Data;
}

/**
 * @brief 通过SPI接口与RC523芯片通信，以读取指定地址的寄存器值
 * @param Address 寄存器地址，用于指定需要读取的寄存器位置
 * @return 返回读取到的寄存器值，作为函数的返回值
 */
uint8_t ReadReg(uint8_t Address) //
{
    // 准备寄存器地址，通过左移操作和按位或操作，以符合SPI通信协议的要求
    uint8_t Addr, Ret;
    Addr = ((Address << 1) & 0x7E) | 0x80;

    // 启动SPI通信，通过NSS引脚使能RC523芯片
    RC523_NSS_Enable();
    // 发送准备好的寄存器地址，以启动读取操作
    SPI_RC523_SendByte(Addr);
    //    // 接收并存储从RC523芯片读取到的数据
    Ret = SPI_RC523_ReadByte();
    // 结束SPI通信，通过NSS引脚禁能RC523芯片
    RC523_NSS_Disable();
    //    printf("\r\n read :%02x \r\n", Ret);
    // HAL_Delay(1000);
    // 返回读取到的寄存器值
    return Ret;
}

/**
 * @brief 向指定寄存器写入一个值。
 * @param Address 要写入的寄存器地址，内部会处理以匹配SPI通信格式。
 * @param Value 要写入寄存器的值。
 */
void WriteReg(uint8_t Address, uint8_t Value) //
{
    uint8_t Addr;
    Addr = (Address << 1) & 0x7E;

    RC523_NSS_Enable();
    SPI_RC523_SendByte(Addr);
    SPI_RC523_SendByte(Value);
    RC523_NSS_Disable();
    //    printf("\r\n write addr: %02x ", Address);
    //    printf(" value : %02x \r\n", Value);
}

/**
 * @brief 对指定寄存器相应位置1
 * @param Reg: 寄存器地址
 * @param Mask: 要设置的位掩码，指明哪些位需要被设置为1
 */
static void SetBitMask(uint8_t Reg, uint8_t Mask)
{
    uint8_t Temp;
    // 读取当前寄存器的值
    Temp = ReadReg(Reg);
    // 将寄存器的值与掩码进行按位或操作，以设置相应的位为1
    WriteReg(Reg, Temp | Mask); // set bit mask
}

/**
 * @brief 对指定寄存器相应位置0
 * @param Reg: 寄存器地址
 * @param Mask: 需要清除的位掩码，与运算后将清除这些位
 */
static void ClearBitMask(uint8_t Reg, uint8_t Mask)
{
    uint8_t Temp;
    // 读取寄存器的当前值
    Temp = ReadReg(Reg);
    // 清除位掩码并写回寄存器
    WriteReg(Reg, Temp & (~Mask)); // clear bit mask
}

/**
 * @brief 打开天线
 */
static void PcdAntennaOn(void) //
{
    uint8_t i;
    i = ReadReg(TxControlReg);

    if (!(i & 0x03))
        SetBitMask(TxControlReg, 0x03);
}

/**
 * @brief 打开天线
 */
static void PcdAntennaOff(void) //
{
    ClearBitMask(TxControlReg, 0x03);
}

/**
 * @brief 配置PCD（读卡器）的ISO类型 A OR B。
 * @param iType ISO类型代码。RC_ISO14443_A---'A'
 */
void PcdConfigISOType(uint8_t iType) //
{
    if (iType == 'A') // ISO14443_A
    {
        ClearBitMask(Status2Reg, 0x08);

        WriteReg(ModeReg, 0x3D);
        WriteReg(RxSelReg, 0x86);
        WriteReg(RFCfgReg, 0x7F);
        WriteReg(TReloadRegL, 30);
        WriteReg(TReloadRegH, 0);
        WriteReg(TModeReg, 0x8D);
        WriteReg(TPrescalerReg, 0x3E);
        HAL_Delay(2);

        PcdAntennaOn(); // 开天线
#ifdef DEBUG
        printf("\r\n antenna ON !!! \r\n");
#endif
    }
}

/**
 * @brief 重置PCD（读卡器）
 */
void PcdReset(void) //
{
    RC523_Reset_Disable();
    HAL_Delay(1);
    RC523_Reset_Enable();
    HAL_Delay(1);
    RC523_Reset_Disable();
    HAL_Delay(1);

    WriteReg(CommandReg, PCD_RESETPHASE);
    while (ReadReg(CommandReg) & 0x10);
    HAL_Delay(1);

    WriteReg(ModeReg, 0x3D);       // 定义发送和接收常用模式 和Mifare卡通讯，CRC初始值0x6363
    WriteReg(TReloadRegL, 30);     // 16位定时器低位
    WriteReg(TReloadRegH, 0);      // 16位定时器高位
    WriteReg(TModeReg, 0x8D);      // 定义内部定时器的设置
    WriteReg(TPrescalerReg, 0x3E); // 设置定时器分频系数
    WriteReg(TxAutoReg, 0x40);     // 调制发送信号为100%ASK
#ifdef DEBUG
    printf("\r\n PcdReset(void) !!! \r\n");
#endif
}

/**
 * @brief 通过RC523芯片与ISO14443卡通讯
 * @param Command 要发送的RC523命令
 * @param InData 通过RC523发送到卡片的数据
 * @param InlenByte 发送数据的字节长度
 * @param OutData 接收到的卡片返回数据
 * @param pOutlenBit 返回数据的位长度
 * @return uint8_t  状态值 = MI_OK，成功
 */
static uint8_t PcdCmdRC523(uint8_t Command, uint8_t *InData, uint8_t InlenByte, uint8_t *OutData, uint32_t *pOutlenBit) //
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitFor = 0x00;
    uint8_t lastBits;
    uint8_t N;
    uint32_t i;

    switch (Command)
    {
    case PCD_AUTHENT:   // Mifare认证
        irqEn = 0x12;   // 允许错误中断请求ErrIEn,允许空闲中断IdleIEn
        waitFor = 0x10; // 认证寻卡等待时候,查询空闲中断标志位
        break;

    case PCD_TRANSCEIVE: // 接收发送,发送接收
        irqEn = 0x77;    // 允许TxIEn RxIEn IdleIEn LoAlertIEn ErrIEn TimerIEn
        waitFor = 0x30;  // 寻卡等待时候 查询接收中断标志位与 空闲中断标志位
        break;

    default:
        break;
    }

    WriteReg(ComIEnReg, irqEn | 0x80); // IRqInv置位管脚IRQ与status1Reg的IRq位的值相反
    ClearBitMask(ComIrqReg, 0x80);     // Set1该位清零时，CommIRqReg的屏蔽位清零
    WriteReg(CommandReg, PCD_IDLE);    // 写空闲命令
    SetBitMask(FIFOLevelReg, 0x80);    // 置位FlushBuffer清除内部FIFO的读和写指针以及ErrReg的BufferOvfl标志位被清除

    for (i = 0; i < InlenByte; i++)
    {
        WriteReg(FIFODataReg, InData[i]); // 写数据进FIFOdata
    }

    WriteReg(CommandReg, Command); // 写命令

    if (Command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80); // StartSend置位启动数据发送 该位与收发命令使用时才有效
    }

    i = 2000; // 根据时钟频率调整，操作M1卡最大等待时间25ms

    do // 认证 与寻卡等待时间
    {
        N = ReadReg(ComIrqReg); // 查询事件中断
        i--;
    } while ((i != 0) && (!(N & 0x01)) && (!(N & waitFor))); // 退出条件i=0,定时器中断，与写空闲命令

    ClearBitMask(BitFramingReg, 0x80); // 清理允许StartSend位

    if (i != 0)
    {
        if (!((ReadReg(ErrorReg) & 0x1B))) // 读错误标志寄存器BufferOfI CollErr ParityErr ProtocolErr
        {
            status = MI_OK;

            if (N & irqEn & 0x01) // 是否发生定时器中断
            {
                status = MI_NOTAGERR;
            }

            if (Command == PCD_TRANSCEIVE)
            {
                N = ReadReg(FIFOLevelReg);             // 读FIFO中保存的字节数
                lastBits = ReadReg(ControlReg) & 0x07; // 最后接收到得字节的有效位数

                if (lastBits)
                {
                    *pOutlenBit = (N - 1) * 8 + lastBits; // N个字节数减去1（最后一个字节）+最后一位的位数 读取到的数据总位数
                }
                else
                {
                    *pOutlenBit = N * 8; // 最后接收到的字节整个字节有效
                }

                if (N == 0)
                {
                    N = 1;
                }

                if (N > MAXRLEN)
                {
                    N = MAXRLEN;
                }

                for (i = 0; i < N; i++)
                {
                    OutData[i] = ReadReg(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    SetBitMask(ControlReg, 0x80);   // stop timer now
    WriteReg(CommandReg, PCD_IDLE); // 清除 TRANSCEIVE 命令

    return status;
}

/**
 * @brief 寻卡操作，若寻卡成功返回卡的类型
 * @param Req_code 寻卡方式
 *                     = 0x52，寻感应区内所有符合14443A标准的卡PICC_REQALL
 *                     = 0x26，寻未进入休眠状态的卡PICC_REQIDL
 * @param TagType  卡片类型代码
 *                   = 0x4400，Mifare_UltraLight
 *                   = 0x0400，Mifare_One(S50)
 *                   = 0x0200，Mifare_One(S70)
 *                   = 0x0800，Mifare_Pro(X))
 *                   = 0x4403，Mifare_DESFire
 * @return uint8_t  函数执行状态，MI_OK表示成功，其他值表示错误
 */
uint8_t PcdRequest(uint8_t Req_code, uint8_t *TagType) //
{
    uint8_t status;
    uint8_t ComMF523Buf[MAXRLEN];
    uint32_t len;

    ClearBitMask(Status2Reg, 0x08); // 清理指示MIFARECyptol单元接通以及所有卡的数据通信被加密的情况
    WriteReg(BitFramingReg, 0x07);  //	发送的最后一个字节的 七位
    SetBitMask(TxControlReg, 0x03); // TX1,TX2管脚的输出信号传递经发送调制的13.56的能量载波信号

    ComMF523Buf[0] = Req_code; // 存入 卡片命令字

    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 1, ComMF523Buf, &len); // 寻卡

    if ((status == MI_OK) && (len == 0x10)) // 寻卡成功返回卡类型
    {
        *TagType = ComMF523Buf[0];
        *(TagType + 1) = ComMF523Buf[1];
#ifdef DEBUG
        printf("PcdRequest Success IC Type is:0x%02x 0x%02x\r\n", ComMF523Buf[0], ComMF523Buf[1]);
#endif
    }
    else
    {
        status = MI_ERR;
#ifdef DEBUG
        printf("PcdRequest Failed\r\n");
#endif
    }

    return status;
}

/**
 * @brief  执行卡片防冲突操作，确保只选中一张卡片，并获取其UID。
 * @param  pSnr 卡片序列号，4字节
 * @return 返回操作状态，MI_OK表示成功，其他值表示错误
 */
uint8_t PcdAnticoll(uint8_t *pSnr) //
{
    uint8_t status;
    uint8_t i, Snr_check = 0;
    uint8_t ComMF523Buf[MAXRLEN];
    uint32_t len;

    ClearBitMask(Status2Reg, 0x08); // 清MFCryptol On位 只有成功执行MFAuthent命令后，该位才能置位
    WriteReg(BitFramingReg, 0x00);  // 清理寄存器 停止收发
    ClearBitMask(CollReg, 0x80);    // 清ValuesAfterColl所有接收的位在冲突后被清除

    /*
    PCD 发送 SEL = ‘93’，NVB = ‘20’两个字节
    迫使所有的在场的PICC发回完整的UID CLn作为应答。
    */
    ComMF523Buf[0] = PICC_ANTICOLL1; // 卡片防冲突命令
    ComMF523Buf[1] = 0x20;           // 发送两字节

    // 发送并接收数据 接收的数据存储于ComMF523Buf
    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 2, ComMF523Buf, &len); // 与卡片通信

    if (status == MI_OK) // 通信成功
    {
        // 收到的UID 存入pSnr
        for (i = 0; i < 4; i++)
        {
            *(pSnr + i) = ComMF523Buf[i]; // 0-3 读出UID
            Snr_check ^= ComMF523Buf[i];  // 校验id
        }

        if (Snr_check != ComMF523Buf[i]) // 4 校验值
        {
            status = MI_ERR;
        }
#ifdef DEBUG
        printf("PcdAnticoll Success, IC Card UID is: 0x%02x%02x%02x%02x\r\n", ComMF523Buf[0], ComMF523Buf[1], ComMF523Buf[2], ComMF523Buf[3]);
#endif
    }
    else
    {
#ifdef DEBUG
        printf("PcdAnticoll Failed\r\n");
#endif
    }

    SetBitMask(CollReg, 0x80); // 在 106kbps良好的防冲突情况下该位置1
    return status;
}

/**
 * @brief 计算给定数据的CRC校验值。
 * @param InData 计算CRC16的数组。
 * @param len 输入数据的长度。
 * @param OutData 指向存储计算结果的缓冲区。
 */
static void CalulateCRC(uint8_t *InData, uint8_t len, uint8_t *OutData) //
{
    uint8_t i, iN;

    ClearBitMask(DivIrqReg, 0x04);
    WriteReg(CommandReg, PCD_IDLE); // 取消当前命令
    SetBitMask(FIFOLevelReg, 0x80);

    for (i = 0; i < len; i++)
    {
        WriteReg(FIFODataReg, *(InData + i));
    }

    WriteReg(CommandReg, PCD_CALCCRC);
    i = 0xFF;

    do
    {
        iN = ReadReg(DivIrqReg);
        i--;
    } while ((i != 0) && !(iN & 0x04)); // 当CRCIRq 所有数据被处理完毕该位置位

    OutData[0] = ReadReg(CRCResultRegL);
    OutData[1] = ReadReg(CRCResultRegM);
}

/**
 * @brief 通过防冲撞机制来选择一个特定的PICC，并验证其UID（唯一标识符）。
 * @param *pSnr 指向存储PICC序列号的指针。
 * @return 返回选择操作的状态，MI_OK表示成功，MI_ERR表示失败。
 */
uint8_t PcdSelect(uint8_t *pSnr) //
{
    uint8_t status;
    uint8_t i;
    uint8_t ComMF523Buf[MAXRLEN];
    uint32_t len;

    // 防冲撞 0x93
    ComMF523Buf[0] = PICC_ANTICOLL1;
    // 假设没有冲突，PCD 指定NVB为70，此值表示PCD将发送完整的UID CLn，与40位UID CLn 匹配的PICC，以SAK作为应答
    ComMF523Buf[1] = 0x70;
    ComMF523Buf[6] = 0; // id校验清零

    // 3 4 5 6位存放UID，第7位一直异或。。。
    for (i = 0; i < 4; i++)
    {
        ComMF523Buf[i + 2] = *(pSnr + i);
        ComMF523Buf[6] ^= *(pSnr + i);
    }

    // CRC(循环冗余校验)
    CalulateCRC(ComMF523Buf, 7, &ComMF523Buf[7]);
    ClearBitMask(Status2Reg, 0x08); // 清空校验成功

    // 发送并接收数据
    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 9, ComMF523Buf, &len);

    if ((status == MI_OK) && (len == 0x18))
    {
        status = MI_OK;
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/**
 * @brief 验证卡片密码，认证通过后才能对卡片进行读写操作
 * @param Auth_mode A（0x60）或 B（0x61）
 * @param Addr 卡片的块地址
 * @param pKey 认证密钥的指针，密钥长度为6字节
 * @param pSnr 卡片序列号的指针，序列号长度为4字节
 * @return 返回认证状态，MI_OK表示认证成功，其他值表示认证失败
 */
uint8_t PcdAuthState(uint8_t Auth_mode, uint8_t Addr, uint8_t *pKey, uint8_t *pSnr) //
{
    uint8_t status;
    uint8_t i, ComMF523Buf[MAXRLEN];
    uint32_t len;

    ComMF523Buf[0] = Auth_mode; // 验证A密钥
    ComMF523Buf[1] = Addr;      // addr[n] :块地址

    for (i = 0; i < 6; i++)
    {
        ComMF523Buf[i + 2] = *(pKey + i);
    }

    for (i = 0; i < 6; i++) // 可能需要修改
    {
        ComMF523Buf[i + 8] = *(pSnr + i);
    }

    // 验证密钥命令
    status = PcdCmdRC523(PCD_AUTHENT, ComMF523Buf, 12, ComMF523Buf, &len);

    if ((status != MI_OK) || (!(ReadReg(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}

/**
 * @brief 向MF523卡写入数据。写数据到M1卡一块
 *
 * @param Addr 要写入的块地址。
 * @param pData 指向要写入的数据， 16字节。
 * @return 返回操作状态，MI_OK表示成功，MI_ERR表示失败。
 */
uint8_t PcdWrite(uint8_t Addr, uint8_t *pData) //
{
    uint8_t status;
    uint8_t i, ComMF523Buf[MAXRLEN];
    uint32_t len;

    ComMF523Buf[0] = PICC_WRITE;
    ComMF523Buf[1] = Addr;

    CalulateCRC(ComMF523Buf, 2, &ComMF523Buf[2]);

    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 4, ComMF523Buf, &len);

    if ((status != MI_OK) || (len != 4) || ((ComMF523Buf[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        // memcpy(ComMF523Buf, pData, 16);
        for (i = 0; i < 16; i++)
        {
            ComMF523Buf[i] = *(pData + i);
        }

        CalulateCRC(ComMF523Buf, 16, &ComMF523Buf[16]);
        status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 18, ComMF523Buf, &len);

        if ((status != MI_OK) || (len != 4) || ((ComMF523Buf[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

/**
 * @brief 从指定块地址读取数据
 * @param Addr 读取数据的块地址
 * @param pData 读出的数据，16字节
 * @return 返回操作状态，MI_OK表示成功，MI_ERR表示失败
 */
uint8_t PcdRead(uint8_t Addr, uint8_t *pData) //
{
    uint8_t status;
    uint8_t i, ComMF523Buf[MAXRLEN];
    uint32_t len;

    ComMF523Buf[0] = PICC_READ;
    ComMF523Buf[1] = Addr;

    CalulateCRC(ComMF523Buf, 2, &ComMF523Buf[2]);
    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 4, ComMF523Buf, &len);

    if ((status == MI_OK) && (len == 0x90))
    {
        for (i = 0; i < 16; i++)
        {
            *(pData + i) = ComMF523Buf[i];
        }
    }
    else
    {
        status = MI_ERR;
    }

    return status;
}

/**
 * @brief 命令卡片进入休眠状态。
 * @return 返回状态码，表示函数执行是否成功。= MI_OK，成功
 */
uint8_t PcdHalt(void) //
{
    uint8_t status = MI_ERR;
    uint8_t ComMF523Buf[MAXRLEN];
    uint32_t len;

    ComMF523Buf[0] = PICC_HALT;
    ComMF523Buf[1] = 0;

    CalulateCRC(ComMF523Buf, 2, &ComMF523Buf[2]);
    status = PcdCmdRC523(PCD_TRANSCEIVE, ComMF523Buf, 4, ComMF523Buf, &len);

    return status;
}

/**
 * @brief IC卡读写
 * @param UID IC卡的唯一标识符
 * @param key_type 密钥类型，0表示KEYA，非0表示KEYB
 * @param KEY 密钥数组
 * @param RW 读写选择，1表示读，0表示写
 * @param data_addr 数据地址，指定要读写的数据块地址
 * @param data 数据数组，用于存储读取的数据或准备写入的数据
 */
void IC_RW(uint8_t *UID, uint8_t key_type, uint8_t *KEY, uint8_t RW, uint8_t data_addr, uint8_t *data)
{
    uint8_t status;
    uint8_t i = 0;
    uint8_t Array_ID[4] = {0}; // 先后存放IC卡的类型和UID(IC卡序列号)

    status = PcdRequest(0x52, Array_ID); // 寻卡
    if (status == MI_OK)
    {
        ShowID(Array_ID);
    }
    else
    {
        return;
    }

    status = PcdAnticoll(Array_ID); // 防冲撞
    if (status != MI_OK)
        return;

    status = PcdSelect(UID); // 选定卡
    if (status != MI_OK)
    {
        printf("UID don't match\r\n");
        return;
    }

    if (0 == key_type)
    {
        status = PcdAuthState(KEYA, data_addr, KEY, UID); // 校验
    }
    else
    {
        status = PcdAuthState(KEYB, data_addr, KEY, UID); // 校验
    }

    if (status != MI_OK)
    {
        printf("KEY don't match\r\n");
        return;
    }

    if (RW) // 读写选择，1是读，0是写
    {
        status = PcdRead(data_addr, data);
        if (status == MI_OK)
        {
            printf("data:");
            for (i = 0; i < 16; i++)
            {
                printf("%02x", data[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("PcdRead() failed\r\n");
            return;
        }
    }
    else
    {
        status = PcdWrite(data_addr, data);
        if (status == MI_OK)
        {
            printf("PcdWrite() finished\r\n");
        }
        else
        {
            printf("PcdWrite() failed\r\n");
            return;
        }
    }

    status = PcdHalt();
    if (status == MI_OK)
    {
        printf("PcdHalt() finished\r\n");
    }
    else
    {
        printf("PcdHalt() failed\r\n");
        return;
    }
}
/**
 * @brief 显示卡的卡号，以十六进制显示
 * @param p 指向ID信息的指针，ID信息是4字节长。
 */
void ShowID(uint8_t *p)
{
    uint8_t arr[9];
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        arr[i * 2] = p[i] / 16;
        arr[i * 2] > 9 ? (arr[i * 2] += '7') : (arr[i * 2] += '0');
        arr[i * 2 + 1] = p[i] % 16;
        arr[i * 2 + 1] > 9 ? (arr[i * 2 + 1] += '7') : (arr[i * 2 + 1] += '0');
    }
    arr[8] = 0;
    printf("ID>>>%s\r\n", arr);
}
/**
 * @brief 等待卡离开
 * 这种方法用于确保卡片确实已经不在读卡器上，避免因读卡器灵敏度或环境因素导致的误判
 */

void WaitCardOff(void)
{
    uint8_t status;
    uint8_t TagType[2];

    while (1)
    {
        status = PcdRequest(REQ_ALL, TagType);
        if (status)
        {
            status = PcdRequest(REQ_ALL, TagType);
            if (status)
            {
                status = PcdRequest(REQ_ALL, TagType);
                if (status)
                {
                    return;
                }
            }
        }
        HAL_Delay(1000);
    }
}

/**
 * @brief 指定开启天线通道
 */
void SelectAntennaOn(GPIO_TypeDef *GPIOx_OA, uint16_t GPIO_Pin_OA, GPIO_TypeDef *GPIOx_OB, uint16_t GPIO_Pin_OB)
{
    // 关闭天线
    PcdAntennaOff();
    HAL_GPIO_WritePin(GPIOx_OA, GPIO_Pin_OA, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOx_OB, GPIO_Pin_OB, GPIO_PIN_RESET);
    PcdAntennaOn();
#ifdef DEBUG
	  printf("SelectAntennaOn !!!! \r\n");
#endif

}

void SelectAntennaOff(GPIO_TypeDef *GPIOx_OA, uint16_t GPIO_Pin_OA, GPIO_TypeDef *GPIOx_OB, uint16_t GPIO_Pin_OB)
{
    PcdAntennaOff();
    HAL_GPIO_WritePin(GPIOx_OA, GPIO_Pin_OA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOx_OB, GPIO_Pin_OB, GPIO_PIN_SET);
    PcdAntennaOn();
}
/**
 * @brief 关闭所有天线
 */
void SelectAllAntennaOff(void)
{

    HAL_GPIO_WritePin(O1A_GPIO_Port, O1A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O1B_GPIO_Port, O1B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O2A_GPIO_Port, O2A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O2B_GPIO_Port, O2B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O3A_GPIO_Port, O3A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O3B_GPIO_Port, O3B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O4A_GPIO_Port, O4A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O4B_GPIO_Port, O4B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O5A_GPIO_Port, O5A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O5B_GPIO_Port, O5B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O6A_GPIO_Port, O6A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O6B_GPIO_Port, O6B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O7A_GPIO_Port, O7A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O7B_GPIO_Port, O7B_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(O8A_GPIO_Port, O8A_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(O8B_GPIO_Port, O8B_Pin, GPIO_PIN_SET);
    PcdAntennaOff();
#ifdef DEBUG
    printf("SelectAllAntennaOff !!!! \r\n");
#endif

}

void RC523Task(void)
{
    uint8_t i = 0;
    uint8_t status = MI_ERR;
    // PcdReset(); // 每次操作卡的时候对卡进行复位防止出现偶数次寻卡失败的问题
    status = PcdRequest(PICC_REQALL, icType); // 寻卡

    // 防冲突 获取UID
    if (status == MI_OK)
    {
        status = MI_ERR;
        status = PcdAnticoll(UID);
    }

    // 选择卡片

    if (status == MI_OK)
    {
        status = MI_ERR;
        ShowID(UID);

        status = PcdSelect(UID);
    }

    if (status == MI_OK)
    {
        status = MI_ERR;

        // 验证密码

        status = PcdAuthState(KEYA, addr, KEY_A, UID);

        if (status == MI_OK)
        {
            printf("PcdAuthState(A) success\r\n");
        }
        else
        {
            printf("PcdAuthState(A) failed\r\n");
        }
    }

    // 先读一下数据
    if (status == MI_OK)
    {
        status = MI_ERR;

        status = PcdRead(addr, RData);
        if (status == MI_OK)
        {
            printf("wirte Before Data:");
            for (i = 0; i < 16; i++)
            {
                printf("%02x", RData[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("PcdRead() failed\r\n");
        }
        // HAL_Delay(1000);
    }

    // 写卡
    if (status == MI_OK)
    {
        status = MI_ERR;

        status = PcdWrite(addr, WData);

        if (status == MI_OK)
        {
            printf("PcdWrite() success\r\n");
        }
        else
        {
            printf("PcdWrite() failed\r\n");
        }
        // HAL_Delay(1000);
    }

    if (status == MI_OK)
    {
        status = MI_ERR;

        status = PcdRead(addr, RData);
        if (status == MI_OK)
        {
            printf("wirte after Data:");
            for (i = 0; i < 16; i++)
            {
                printf("%02x", RData[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("PcdRead() failed\r\n");
        }
    }

    if (status == MI_OK)
    {
        status = MI_ERR;

        status = PcdWrite(0x02, WData);

        if (status == MI_OK)
        {
            printf("PcdWrite() success\r\n");
        }
        else
        {
            printf("PcdWrite() failed\r\n");
        }
        // HAL_Delay(1000);
    }

    if (status == MI_OK)
    {
        status = MI_ERR;

        status = PcdRead(0x02, RData);
        if (status == MI_OK)
        {
            printf("wirte after Data:");
            for (i = 0; i < 16; i++)
            {
                printf("%02x", RData[i]);
            }
            printf("\r\n");
        }
        else
        {
            printf("PcdRead() failed\r\n");
        }
    }
    PcdHalt();
}

// 按照存储规则，读取相应块数据

uint8_t RC523ChannlxReadData(uint8_t* AllData)
{
	uint8_t i;
	uint8_t status = MI_ERR;
	
	for(i = 0; i < 3; i++)
	{
		status = PcdAuthState(KEYA, (i*4) + 1, KEY_A, UID);
#ifdef DEBUG
		if (status == MI_OK)
		{
				printf("PcdAuthState(A) success\r\n");
		}
		else
		{
				printf("PcdAuthState(A) failed\r\n");
		}
#endif		
		if(status == MI_OK)
		{
				status = PcdRead((i*4) + 1, AllData + 16 * i * 2 ); // 0  32  64    1  5  9
		}
		else
		{
				return status;
		}

		if(status == MI_OK)
		{
				status = PcdRead((i*4) + 2, AllData + 16 * (2 * i + 1) ); // 16 48  80 2 6 10
		}
		else
		{
				return status;
		}
 	}
	
	return status;
}

// 按照存储规则，写相应块数据
uint8_t RC523ChannlxWriteData(uint8_t* AllData)
{
	uint8_t i;
	uint8_t status = MI_OK;
	
	for(i = 0; i < 3; i++)
	{
		status = PcdAuthState(KEYA, (i*4) + 1, KEY_A, UID);
		
#ifdef DEBUG
		if (status == MI_OK)
		{
				printf("PcdAuthState(A) success\r\n");
		}
		else
		{
				printf("PcdAuthState(A) failed\r\n");
		}
#endif
		
		if(status == MI_OK)
		{
				status = PcdWrite((i*4) + 1, AllData + 16 * i * 2 ); // 0  32  64    1  5  9
		}
		else
		{
				return status;
		}
		
		if(status == MI_OK)
		{
				status = PcdWrite((i*4) + 2, AllData + 16 * (2 * i + 1) ); // 16 48  80 2 6 10
		}
		else
		{
				return status;
		}
 	}
	
	if(status == MI_OK)
	{
//			printf("wtite channl multi block data success");
	}
	
	return status;
}

void RC523SelectChannlxRWTask(uint8_t channlx, uint8_t *ReadData, uint8_t* WriteData)
{

    uint8_t status = MI_ERR;
	
		SelectAllAntennaOff();
		SelectAntennaOn(Channlx_Port[channlx][0], Channlx_Pin[channlx][0], Channlx_Port[channlx][1], Channlx_Pin[channlx][1]);
    // PcdReset(); // 每次操作卡的时候对卡进行复位防止出现偶数次寻卡失败的问题
    status = PcdRequest(PICC_REQALL, icType); // 寻卡

    // 防冲突 获取UID
    if (status == MI_OK)
    {
        status = MI_ERR;
        status = PcdAnticoll(UID);
    }

    // 选择卡片

    if (status == MI_OK)
    {
        status = MI_ERR;
        ShowID(UID);

        status = PcdSelect(UID);
    }

	WDataInit();
		
	if (status == MI_OK)
    {
        status = MI_ERR;

        // 验证密码

        status = RC523ChannlxReadData(RAllData);
#ifdef DEBUG
				uint8_t i = 0;
        if (status == MI_OK)
        {
					for(i = 0; i < 96; i++)
					{
							printf("%02x ",RAllData[i]);
							if((i+ 1) % 16 == 0)
							{
								printf("\n");
							}
					}
					printf("\n");
        }
        else
        {
            printf("RC523ChannlxReadData failed\r\n");
        }
#endif
    }
		
		if (status == MI_OK)
    {
        status = MI_ERR;

        // 验证密码

        status = RC523ChannlxWriteData(WAllData);
#ifdef DEBUG
        if (status == MI_OK)
        {
						printf("RC523ChannlxWriteData Success\r\n");
        }
        else
        {
            printf("RC523ChannlxWriteData failed\r\n");
        }
#endif
    }
		
		if (status == MI_OK)
    {
        status = MI_ERR;

        // 验证密码

        status = RC523ChannlxReadData(RAllData);
#ifdef DEBUG
			uint8_t i = 0;
        if (status == MI_OK)
        {
					for(i = 0; i < 96; i++)
					{
							printf("%02x ",RAllData[i]);
							if((i+ 1) % 16 == 0)
							{
								printf("\n");
							}
					}
        }
        else
        {
            printf("RC523ChannlxReadData failed\r\n");
        }
#endif
    }
		
}

void RC523SelectChannlxReadTask(uint8_t channlx, uint8_t *ReadData)
{

    uint8_t status = MI_ERR;
	
		SelectAllAntennaOff();
		SelectAntennaOn(Channlx_Port[channlx][0], Channlx_Pin[channlx][0], Channlx_Port[channlx][1], Channlx_Pin[channlx][1]);
    
    status = PcdRequest(PICC_REQALL, icType); // 寻卡

    // 防冲突 获取UID
    if (status == MI_OK)
    {
        status = MI_ERR;
        status = PcdAnticoll(UID);
    }

    // 选择卡片

    if (status == MI_OK)
    {
        status = MI_ERR;
#ifdef DEBUG	
        ShowID(UID);
#endif

        status = PcdSelect(UID);
    }
	
	if (status == MI_OK)
    {
        status = MI_ERR;

        // 验证密码

        status = RC523ChannlxReadData(RAllData);
#ifdef DEBUG
			uint8_t i = 0;
      if (status == MI_OK)
      {
				for(i = 0; i < 96; i++)
				{
					printf("%02x ",RAllData[i]);
					if((i+ 1) % 16 == 0)
					{
						printf("\n");
					}
				}
					printf("\n");
      }
      else
      {
          printf("RC523ChannlxReadData failed\r\n");
      }
#endif
    }
		PcdHalt();
}

void RC523SelectChannlxWriteTask(uint8_t channlx, uint8_t *WriteData)
{

    uint8_t status = MI_ERR;
	
		SelectAllAntennaOff();
		SelectAntennaOn(Channlx_Port[channlx][0], Channlx_Pin[channlx][0], Channlx_Port[channlx][1], Channlx_Pin[channlx][1]);
    
    status = PcdRequest(PICC_REQALL, icType); // 寻卡

    // 防冲突 获取UID
    if (status == MI_OK)
    {
        status = MI_ERR;
        status = PcdAnticoll(UID);
    }

    // 选择卡片

    if (status == MI_OK)
    {
        status = MI_ERR;
#ifdef DEBUG	
        ShowID(UID);
#endif
        status = PcdSelect(UID);
    }
	
		if (status == MI_OK)
    {
        status = MI_ERR;
        status = RC523ChannlxWriteData(WriteData);
#ifdef DEBUG
        if (status == MI_OK)
        {
						printf("RC523ChannlxWriteData Success\r\n");
        }
        else
        {
            printf("RC523ChannlxWriteData failed\r\n");
        }
#endif				
    }
		PcdHalt();
}

// 轮询每个通道
void ScanChannlx(void)
{
	uint8_t i = 0;
	for(i = 0 ; i < 8; i++)
	{
		printf("CHANNL %d \r\n",i + 1);
		SelectAllAntennaOff();
		SelectAntennaOn(Channlx_Port[i][0], Channlx_Pin[i][0], Channlx_Port[i][1], Channlx_Pin[i][1]);
		RC523SelectChannlxReadTask(i,RAllData);
		//HAL_Delay(1000);
	}
}

