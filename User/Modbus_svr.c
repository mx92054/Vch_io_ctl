#include <string.h>
#include "Modbus_svr.h"
#include "stm32f4xx_conf.h"

#define ADDRESS wReg[100] //��ǰվ��ַ
#define BAUDRATE wReg[101]
#define FRAMETIMER 5 //֡ʱ����

short wReg[REG_LEN];
short coils[COIL_LEN];
u8 bSaved = 0;
u8 bChanged = 0;

u8 buffer[512]; //�����
u8 *tsk_buf;	//������򻺳�
u8 *isr_buf;	//�жϳ��򻺳�

u8 pos_msg = 0;		//����ָ��
u8 frame_len;		//����֡����
u8 trans_len;		//��Ӧ֡����
u8 bFrameStart = 0; //��ʼ֡��Ӧ

__IO u16 nMBInterval = 0; // interval timer
//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void MODBUS_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	/* �������ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* �����ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	/* ʹ���ж� */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ��ʼ������NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void MODBUS_Config(u32 baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK | DEBUG_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);

	/* ���ô�DEBUG_USART ģʽ */
	/* ���������ã�DEBUG_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	/* �ֳ�(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	MODBUS_NVIC_Configuration();

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	Э��ջ��ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void Modbus_init(void)
{
	char buf[100];
	tsk_buf = buffer;
	isr_buf = buffer + 256;

	ADDRESS = 1;

	if (BAUDRATE != 96 && BAUDRATE != 192 && BAUDRATE != 384 && BAUDRATE != 1152)
	{
		BAUDRATE = 1152;
		bSaved = 1;
	}

	if (ADDRESS <= 1 || ADDRESS >= 255)
	{
		ADDRESS = 1;
		bSaved = 1;
	}

	MODBUS_NVIC_Configuration();
	MODBUS_Config(BAUDRATE * 100);

	sprintf(buf, " Program Initialize... Adr:%d, Baud:%d", ADDRESS, BAUDRATE);
	Usart_SendString(DEBUG_USARTx, buf);
}

/*-------------------------------------------------------------------------------
	@brief:		Э���������
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void Modbus_task(void)
{
	u8 *ptr;
	u8 err;

	if (nMBInterval > FRAMETIMER) //ͨ��֡���ʱ���ѵ������������л�
	{
		ptr = tsk_buf;
		tsk_buf = isr_buf;
		isr_buf = ptr;
		frame_len = pos_msg;
		pos_msg = 0;
		bFrameStart = 1;

		if (frame_len >= 8 && tsk_buf[0] == ADDRESS)
		{
			err = Modbus_Procotol_Chain(); //Э�����
			if (err)					   //Э��Э�����
				Error_response(err);
			else //Э�������ȷ
				Normal_response();
		}

		nMBInterval = 0;
	}
}

//-------------------------------------------------------------------------------
//	@brief	MODBUSЭ���������ʱ����
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void Normal_response(void)
{
	u16 uCrc1;
	int i;

	//-----------Product CRC byte------------------------------------
	uCrc1 = CRC16(tsk_buf, trans_len - 2);
	tsk_buf[trans_len - 2] = uCrc1 & 0x00FF;
	tsk_buf[trans_len - 1] = uCrc1 >> 8;

	//-------------Transmitter frame---------------------------------
	for (i = 0; i < trans_len; i++)
	{
		USART_SendData(DEBUG_USARTx, tsk_buf[i]);
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TC) == RESET)
			;
	}
}

//-------------------------------------------------------------------------------
//	@brief	MODBUSЭ���������ʱ����
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void Error_response(u8 errno)
{
	u16 uCrc1;
	int i;

	tsk_buf[1] |= 0x80;
	tsk_buf[2] = errno;

	//-----------Product CRC byte------------------------------------
	uCrc1 = CRC16(tsk_buf, 3);
	tsk_buf[3] = uCrc1 & 0x00FF;
	tsk_buf[4] = uCrc1 >> 8;

	//-------------Transmitter frame---------------------------------
	for (i = 0; i < 5; i++)
	{
		USART_SendData(DEBUG_USARTx, tsk_buf[i]);
		while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TC) == RESET)
			;
	}
}

//-------------------------------------------------------------------------------
//	@brief	MODBUSЭ������ͻظ�
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
u8 Modbus_Procotol_Chain(void)
{
	u16 reg_adr, uCrc1, uCrc2;
	u16 i, data_len, cur_adr;
	u8 cur_bit;
	u8 *ptr;
	short *ptrReg;

	if (tsk_buf[1] == 0 || (tsk_buf[1] > 6 && tsk_buf[1] < 15) || tsk_buf[1] > 16)
		return 1; //ILLEGAL FUNCTION

	if (frame_len > 250 || frame_len < 8)
		return 3; // ILLEGAL DATA VALUE

	uCrc1 = CRC16(tsk_buf, frame_len - 2);
	uCrc2 = tsk_buf[frame_len - 1] << 8 | tsk_buf[frame_len - 2];

	if (uCrc1 != uCrc2)
		return 3; // ILLEGAL DATA VALUE

	reg_adr = tsk_buf[2] << 8 | tsk_buf[3];
	data_len = tsk_buf[4] << 8 | tsk_buf[5];

	//----------Read Coils & Discrete inputs----------------------
	if (tsk_buf[1] == 1 || tsk_buf[1] == 2)
	{
		if (data_len > 960)
			return 3; // ILLEGAL DATA VALUE
		if ((reg_adr + data_len) >= COIL_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		ptr = &tsk_buf[2];
		*ptr++ = BIT2BYTE(data_len);
		cur_adr = reg_adr;
		cur_bit = 0;
		for (i = 0; i < data_len; i++)
		{
			if (coils[cur_adr])
				*ptr = SETBIT_BYTE(*ptr, cur_bit);
			else
				*ptr = RESETBIT_BYTE(*ptr, cur_bit);

			if (++cur_bit >= 8)
			{
				cur_bit = 0;
				ptr++;
			}
			cur_adr++;
		}
		trans_len = 5 + tsk_buf[2];
	}

	//----------Write single coil---------------------------------
	if (tsk_buf[1] == 5)
	{
		if (reg_adr >= COIL_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		if (data_len == 0xFF00)
			coils[reg_adr] = 1;
		else if (data_len == 0)
			coils[reg_adr] = 0;
		else
			return 3; //ILLEGAL DATA VALUE
		trans_len = 8;
	}

	//----------Write multiple coils---------------------------------
	if (tsk_buf[1] == 15)
	{
		if ((reg_adr + data_len) >= COIL_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		cur_bit = 0;
		ptr = &tsk_buf[7];
		ptrReg = &coils[reg_adr];
		for (i = 0; i < data_len; i++)
		{
			*ptrReg++ = ((*ptr & (0x01 << cur_bit)) == 0) ? 0 : 1;
			if (++cur_bit >= 8)
			{
				cur_bit = 0;
				ptr++;
			}
		}
		trans_len = 8;
	}

	//----------Read Input & Holding Register----------------------
	if (tsk_buf[1] == 3 || tsk_buf[1] == 4)
	{
		if (data_len > 125)
			return 3; // ILLEGAL DATA VALUE

		if ((reg_adr + data_len) >= REG_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		ptr = &tsk_buf[2];
		ptrReg = &wReg[reg_adr];
		*ptr++ = data_len << 1; //  Byte count
		for (i = reg_adr; i < reg_adr + data_len; i++)
		{
			*ptr++ = *ptrReg >> 8;
			*ptr++ = *ptrReg & 0x00FF;
			ptrReg++;
		}
		trans_len = 5 + tsk_buf[2];
	}

	//----------Write single holding register----------------------
	if (tsk_buf[1] == 6)
	{
		if (reg_adr >= REG_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		wReg[reg_adr] = data_len;
		if ( reg_adr >= 100 )
			bSaved = 1;

		// ��д������Ĵ�������Ҫ����λ����������
		if (reg_adr >= 100 && reg_adr <= 120)
			bChanged = 1;

		trans_len = 8;
	}

	//----------Write multiple holding register--------------------
	if (tsk_buf[1] == 16)
	{

		if ((reg_adr + data_len) >= REG_LEN)
			return 2; //ILLEGAL DATA ADDRESS

		ptr = &tsk_buf[7];
		ptrReg = &wReg[reg_adr];
		for (i = reg_adr; i < reg_adr + data_len; i++)
		{
			*ptrReg = ((*ptr++) << 8);
			*ptrReg |= (*ptr++);
			ptrReg++;
		}
		if ( reg_adr >= 100 )
			bSaved = 1;

		// ��д������Ĵ�������Ҫ����λ����������
		if (reg_adr >= 100 && reg_adr < 120)
			bChanged = 1;

		trans_len = 8;
	}

	return 0;
}
//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ModbusTimer(void)
{
	nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	�Ĵ������ݱ��ֵ�BKP
//	@param	nAddr: �Ĵ�����ַ
//  				val:	 �Ĵ�����ַ��Ӧ�ļĴ���ֵ
//	@retval	None
//-------------------------------------------------------------------------------
void SaveToBKP(u16 nAddr, u16 val)
{
	//	if ( nAddr >= 100 )
	//	{
	//		if ( nAddr < 110 )
	//				BKP_WriteBackupRegister(BKP_DR1 + (nAddr-100)*4, val) ;
	//		else
	//		{
	//			if ( nAddr < 120 )
	//				BKP_WriteBackupRegister(BKP_DR11 + (nAddr-110)*4, val) ;
	//		}
	//	}
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DEBUG_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(DEBUG_USARTx); //�����Ĵ��������ݻ��浽���ջ�������
		if (bFrameStart)
		{
			if (ch != ADDRESS && pos_msg == 0)
				bFrameStart = 0;

			isr_buf[pos_msg++] = ch;
		}
		nMBInterval = 0;
	}

	if (USART_GetITStatus(DEBUG_USARTx, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(DEBUG_USARTx, USART_IT_TXE, DISABLE);
	}
}

//-------------------------------------------------------------------------------
//	@brief	MODBUS CRC16�������
//	@param	nData:��Ҫ���������֡��ַ
//					wLength: ��Ҫ���������֡����
//	@retval	������
//-------------------------------------------------------------------------------
static const uint16_t wCRCTable[] = {
	0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
	0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
	0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
	0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
	0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
	0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
	0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
	0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
	0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
	0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
	0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
	0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
	0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
	0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
	0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
	0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
	0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
	0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
	0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
	0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
	0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
	0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
	0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
	0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
	0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
	0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
	0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
	0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
	0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
	0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
	0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
	0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

u16 CRC16(const uint8_t *nData, uint8_t wLength)
{
	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;

	while (wLength--)
	{
		nTemp = *nData++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}
	return wCRCWord;
}

//-------------------------------------------------------------------------------
//	@brief	����һ���ֽ�
//	@param	pUSARTx:���Ͷ˿ں�
//					ch: �����͵��ֽ�
//	@retval	None
//-------------------------------------------------------------------------------
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
	USART_SendData(pUSARTx, ch);

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;
}

//-------------------------------------------------------------------------------
//	@brief	����һ���ֽ���
//	@param	pUSARTx:���Ͷ˿ں�
//					ch: �����͵��ֽ�
//	@retval	None
//-------------------------------------------------------------------------------
void Usart_SendBytes(USART_TypeDef *pUSARTx, uint8_t *buf, uint8_t len)
{
	int i;
	for (i = 0; i < len; i++)
	{
		USART_SendData(pUSARTx, *buf++);
		while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
			;
	}
}

//-------------------------------------------------------------------------------
//	@brief	����һ���ַ���
//	@param	pUSARTx:���Ͷ˿ں�
//					str: �����͵��ַ���
//	@retval	None
//-------------------------------------------------------------------------------
void Usart_SendString(USART_TypeDef *pUSARTx, char *str)
{
	unsigned int k = 0;
	do
	{
		Usart_SendByte(pUSARTx, *(str + k));
		k++;
	} while (*(str + k) != '\0');

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
	{
	}
}

//-------------------------------------------------------------------------------
//	@brief	����һ��16λ����
//	@param	pUSARTx:���Ͷ˿ں�
//					ch: �����͵���
//	@retval	None
//-------------------------------------------------------------------------------
void Usart_SendHalfWord(USART_TypeDef *pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;

	temp_h = (ch & 0XFF00) >> 8;
	temp_l = ch & 0XFF;

	USART_SendData(pUSARTx, temp_h);
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;

	USART_SendData(pUSARTx, temp_l);
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET)
		;
}

//----------------------------------------------------------------------------
int fputc(int ch, FILE *f)
{
	USART_SendData(DEBUG_USARTx, (uint8_t)ch);

	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
		;

	return (ch);
}

//----------------------------------------------------------------------------
int fgetc(FILE *f)
{
	while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET)
		;

	return (int)USART_ReceiveData(DEBUG_USARTx);
}

//-----------------end of file---------------------------------------------
