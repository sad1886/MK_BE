//
#include "MDR32F9x.h"

#define _CAN_
#include "Can.h"

//==========================================================================
void CAN1_DeInit(void)
//==========================================================================
{  uint32_t i;
  MDR_CAN1->CONTROL = 0;
  MDR_CAN1->STATUS = 0;
  MDR_CAN1->BITTMNG = 0;
  MDR_CAN1->INT_EN = 0;
  MDR_CAN1->OVER = 0;
  MDR_CAN1->INT_RX = 0;
  MDR_CAN1->INT_TX = 0;
  for (i = 0; i < 32; i++)  {    MDR_CAN1->BUF_CON[i] = 0;  }
}

//==========================================================================
void CAN2_DeInit(void)
//==========================================================================
{  uint32_t i;
	
	for(i = 0; i < 10000; i++) { MDR_CAN2->STATUS = 0; }

	for (i = 0; i < 31; i++)  {    MDR_CAN2->BUF_CON[i] = 0;  }
	// Очистим буфера
	for(i = 0; i < 31; i++)	{
		MDR_CAN2->CAN_BUF[i].ID 				=0;
		MDR_CAN2->CAN_BUF[i].DATAL 			=0;		MDR_CAN2->CAN_BUF[i].DATAH 				 =0;
		MDR_CAN2->CAN_BUF_FILTER[i].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[i].FILTER =0;
	}
	
  //MDR_CAN2->CONTROL = 0;
  MDR_CAN2->STATUS = 0;
  MDR_CAN2->BITTMNG = 0;
  MDR_CAN2->INT_EN = 0;
  MDR_CAN2->OVER = 0;
  MDR_CAN2->INT_RX = 0;
  MDR_CAN2->INT_TX = 0;
	
	MDR_CAN2->CONTROL = 0;
	MDR_RST_CLK->CAN_CLOCK &= ~(1 << CAN2_CLC_EN);
	MDR_RST_CLK->PER_CLOCK &= ~(1 << CAN2_PER_CLK);
}

//==========================================================================
void CAN1_Init(void)
//==========================================================================
{
	// Настройка контроллера тактовой частоты
	MDR_RST_CLK->PER_CLOCK |= (1 << CAN1_PER_CLK);	// Разр тактир периф блоков. Регистр управления тактовой частотой периферийных блоков
	MDR_RST_CLK->CAN_CLOCK |= (1 << CAN1_CLC_EN);		// Разрешение тактирования CAN1,2. Регистр управления тактовой частотой CAN.
	
	// BitTime = (1 + (PSEG+1) + (SEG1+1) + (SEG2+1) )* Tq = 25,   Tq = (BRP + 1)/HCLC(MГц) = 4/80 = 50 нс (1000 кбит/с)
	// *************** CAN1
	//         			       	SB				 SJW				 SEG2				SEG1				PSEG			BRP
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 9;		// CAN1 speed 1000 kbit/s	от 01.07.16
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 19;		// CAN1 speed 500 kbit/s	от 01.07.16
	//MDR_CAN1->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 39;		// CAN1 speed 250 kbit/s	РѕС‚ 28.12.17
	
	//................ 40Мг ..........................................................
	//MDR_CAN1->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 9;	// CAN1 speed 250 kbit/s	РѕС‚ 24.10.18 40Mg

	//................ 24Мг ..........................................................
	MDR_CAN1->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 5;	// CAN1 speed 250 kbit/s	РѕС‚ 24.10.18 24Mg

//	MDR_CAN1->BITTMNG =	(0 << 27) | (0 << 25) | (6 << 22) | (5 << 19) | (5 << 16) | 3;		// CAN1 speed 1000 kbit/s	старое
//	MDR_CAN1->BITTMNG =	(0 << 27) | (0 << 25) | (6 << 22) | (5 << 19) | (5 << 16) | 7;			//CAN1 speed 500 kbit/s	старое
	MDR_CAN1->OVER	 = 127;																																// Регистр границы счета ошибок	
	
	MDR_CAN1->BUF_CON[nbuf_RX] = (1 << CAN_BUF_EN) | (1 << CAN_RX_ON);										// Подготовка буфера для приёма. Режим приём
//	MDR_CAN1->INT_EN = (1<<ERR_OVER_INT)|(1<<ERR_INT_EN)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN1->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN1->INT_RX = 1<<nbuf_RX;																												// Разрешить прерывание от 30-го приемного буфера

	MDR_CAN1->CAN_BUF_FILTER[nbuf_RX].MASK =	0;																					// 0x7FF << 18;
	MDR_CAN1->CAN_BUF_FILTER[nbuf_RX].FILTER = 0;																					// ((nbuf_RX+1) << 18);
	
	// Режим нормальной передачи (регистр CAN_STATUS : ROM = 0, STM = 0)
	MDR_CAN1->CONTROL = (1 << CAN_EN);																										// Регистр управления контроллером //  |  | (1 << CAN_ROM) |  | (1 << CAN_ROP)
}

//==========================================================================
void CAN2_Init(void)
//==========================================================================
{
	volatile uint32_t status;
	
	// Настройка контроллера тактовой частоты
	MDR_RST_CLK->PER_CLOCK |= (1 << CAN2_PER_CLK);	// Разр тактир периф блоков. Регистр управления тактовой частотой периферийных блоков
	MDR_RST_CLK->CAN_CLOCK |= (1 << CAN2_CLC_EN);		// Разрешение тактирования CAN1,2. Регистр управления тактовой частотой CAN.

	//         			       	SB				 SJW				 SEG2				SEG1				PSEG			BRP
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 9;		// CAN2 speed 1000 kbit/s	от 01.07.16
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 19;		// CAN2 speed 500 kbit/s	от 01.07.16
	//MDR_CAN2->BITTMNG =	(0 << 27) | (1 << 25) | (2 << 22) | (2 << 19) | (0 << 16) | 39;		// CAN2 speed 250 kbit/s	РѕС‚ 28.12.17

	//................ 40Мг ..........................................................
	MDR_CAN2->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 9;	// CAN1 speed 250 kbit/s	РѕС‚ 24.10.18 40Mg

	//................ 24Мг ..........................................................
	MDR_CAN2->BITTMNG	 =	(0 << 27) | (0 << 25) | (3 << 22) | (7 << 19) | (2 << 16) | 5;	// CAN1 speed 250 kbit/s	РѕС‚ 24.10.18 24Mg

	//	MDR_CAN2->BITTMNG =	(0 << 27) | (0 << 25) | (6 << 22) | (5 << 19) | (5 << 16) | 3;		// CAN2 speed 1000 kbit/s
//	MDR_CAN2->BITTMNG =	(0 << 27) | (0 << 25) | (6 << 22) | (5 << 19) | (5 << 16) | 7;			//CAN1 speed 500 kbit/s
	MDR_CAN2->OVER	 = maxOVER;																																// Регистр границы счета ошибок	
	
	MDR_CAN2->BUF_CON[nbuf_RX] = (1 << CAN_BUF_EN) | (1 << CAN_RX_ON);										// Подготовка буфера для приёма. Режим приём, nbuf_RX = 30
//	MDR_CAN2->INT_EN = (1<<ERR_OVER_INT)|(1<<ERR_INT_EN)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN2->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN2->INT_RX = 1<<nbuf_RX;																												// Разрешить прерывание от 30-го приемного буфера

	MDR_CAN2->CAN_BUF_FILTER[nbuf_RX].MASK =	0;																					// 0x7FF << 18;
	MDR_CAN2->CAN_BUF_FILTER[nbuf_RX].FILTER = 0;																					// ((nbuf_RX+1) << 18);
	
	// Режим нормальной передачи (регистр CAN_STATUS : ROM = 0, STM = 0)
	MDR_CAN2->CONTROL = (1 << CAN_EN);																										//  |  | (1 << CAN_ROM) |  | (1 << CAN_ROP)
	status = MDR_CAN2->STATUS;
}

//==========================================================================
void CAN1_MakeMSG(unsigned char lenData, volatile unsigned char * volatile pnt)
//==========================================================================
{	int j;
unsigned char buf_num, i, MaxInd, ost;
	
	// Очистим буфера
	for(buf_num = 0; buf_num < nbuf_RX; ++buf_num)	{
		MDR_CAN1->CAN_BUF[buf_num].ID 				=0;
		MDR_CAN1->CAN_BUF[buf_num].DATAL 			=0;		MDR_CAN1->CAN_BUF[buf_num].DATAH 				 =0;
		MDR_CAN1->CAN_BUF_FILTER[buf_num].MASK=0;		MDR_CAN1->CAN_BUF_FILTER[buf_num].FILTER =0;
	}
	// Подготовка данных для передачи
	i=0;	ost=0;
	MaxInd = lenData/8;	 	ost = lenData%8;	if (ost) MaxInd++;
																																													// Число сообщений в пакете
	for(buf_num = 0; buf_num < MaxInd; buf_num++)		{
		MDR_CAN1->CAN_BUF[buf_num].ID		= nPrior_BE << 24 | AdrMUK_BE << 20 | nMUKs_ZRU << 16  |  (byteCMD << 12) |MaxInd << 6 | (buf_num+1);

		if ((buf_num == MaxInd-1)&&(ost))	
			MDR_CAN1->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | ost;		// Длина передаваемых данных в пакете (в байтах)
		else
			MDR_CAN1->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | 8;			// Длина передаваемых данных в пакете (в байтах)

		if (i<lenData)	{
			MDR_CAN1->CAN_BUF[buf_num].DATAL = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Четвёртый..первый байт в пакете
		if (i<lenData)	{
			MDR_CAN1->CAN_BUF[buf_num].DATAH = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Восьмой..пятый байт в пакете

		//MDR_CAN1->INT_TX |= 1<<buf_num;																														// Регистр разрешения прерываний от передающих буферов TX_INT_EN
		MDR_CAN1->BUF_CON[buf_num]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);													// Запрос на отправку сообщения, установить бит TX_REQ
		for(j=0; j < 4000; j++)	{	i=i;	}
		for(j=0; j < 2500; j++)	{	i=i;	}																														// Дополнительная задержка для скорости обмена 250К
	}
	
	//MDR_CAN1->INT_EN = (1<<ERR_OVER_INT)|(1<<ERR_INT_EN)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN1->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN1->STATUS = 0;

}

//==========================================================================
void CAN2_MakeMSG(unsigned char lenData, volatile unsigned char * volatile pnt)
//==========================================================================
{	int j;
unsigned char buf_num, i, MaxInd, ost;
	
	// Очистим буфера
	for(buf_num = 0; buf_num < nbuf_RX; ++buf_num)	{
		MDR_CAN2->CAN_BUF[buf_num].ID 				=0;
		MDR_CAN2->CAN_BUF[buf_num].DATAL 			=0;		MDR_CAN2->CAN_BUF[buf_num].DATAH 				 =0;
		MDR_CAN2->CAN_BUF_FILTER[buf_num].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[buf_num].FILTER =0;
	}
	// Подготовка данных для передачи
	i=0;	ost=0;
	MaxInd = lenData/8;	 	ost = lenData%8;	if (ost) MaxInd++;
																																													// Число сообщений в пакете
//	for(buf_num = 0; buf_num < 2; buf_num++)		{
	for(buf_num = 0; buf_num < MaxInd; buf_num++)		{
		MDR_CAN2->CAN_BUF[buf_num].ID		= nPrior_BE << 24 | (AdrMUK_BE+1) << 20 | nMUKs_ZRU << 16  |  (byteCMD << 12) |MaxInd << 6 | (buf_num+1);

		if ((buf_num == MaxInd-1)&&(ost))	
			MDR_CAN2->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | ost;		// Длина передаваемых данных в пакете (в байтах)
		else
			MDR_CAN2->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | 8;			// Длина передаваемых данных в пакете (в байтах)

		if (i<lenData)	{
			MDR_CAN2->CAN_BUF[buf_num].DATAL = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Четвёртый..первый байт в пакете
		if (i<lenData)	{
			MDR_CAN2->CAN_BUF[buf_num].DATAH = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Восьмой..пятый байт в пакете

		//MDR_CAN2->INT_TX |= 1<<buf_num;																														// Регистр разрешения прерываний от передающих буферов TX_INT_EN
		MDR_CAN2->BUF_CON[buf_num]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);													// Запрос на отправку сообщения, установить бит TX_REQ
		for(j=0; j < 4000; j++)	{	i=i;	}
		for(j=0; j < 2500; j++)	{	i=i;	}																														// Дополнительная задержка для скорости обмена 250К
	}
	
	//MDR_CAN2->INT_EN = (1<<ERR_OVER_INT)|(1<<ERR_INT_EN)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN2->INT_EN = (1<<ERR_OVER_INT)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
	MDR_CAN2->STATUS = 0;

}

//==========================================================================
//void _CAN2_MakeMSG(unsigned char lenData, volatile unsigned char * volatile pnt)
//==========================================================================
//{	//int j, cntFrms, cyckl;
//unsigned char buf_num, i, MaxInd, ost;
  
	
	// Очистим буфера
//	for(buf_num = 0; buf_num < nbuf_RX; ++buf_num)	{
//		MDR_CAN2->CAN_BUF[buf_num].ID 				=0;
//		MDR_CAN2->CAN_BUF[buf_num].DATAL 			=0;		MDR_CAN2->CAN_BUF[buf_num].DATAH 				 =0;
//		MDR_CAN2->CAN_BUF_FILTER[buf_num].MASK=0;		MDR_CAN2->CAN_BUF_FILTER[buf_num].FILTER =0;
//	}
//	// Подготовка данных для передачи
//	i=0;	ost=0;
//	MaxInd = lenData/8;	 	ost = lenData%8;	if (ost) MaxInd++;
//																																													// Число сообщений в пакете
//	buf_num = 0;	cntFrms = 0;	cyckl = 0;
//	while (cntFrms < MaxInd)		{
//		
//		while ((MDR_CAN2->BUF_CON[buf_num] & 0x00000020))	{
//			if (buf_num < Maxbuf_TX) buf_num++;	else {	buf_num=0;	cyckl++;	}
//			if (cyckl>=2) {	bCAN2_ERROR_OVER=1;	}
//		}	

//		if (bCAN2_ERROR_OVER)	{	cntFrms = MaxInd; }
//		else	{
//		MDR_CAN2->CAN_BUF[buf_num].ID		= nPrior_BE << 24 | nMUK_BE << 20 | nMUKs_ZRU << 16  |  (byteCMD << 12) |MaxInd << 6 | (cntFrms+1);

//		if ((cntFrms == MaxInd-1)&&(ost))	
//			MDR_CAN2->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | ost;		// Длина передаваемых данных в пакете (в байтах)
//		else
//			MDR_CAN2->CAN_BUF[buf_num].DLC	= (1 << CAN_IDE) | (1 << CAN_SSR) | (1 << CAN_R1) | 8;			// Длина передаваемых данных в пакете (в байтах)

//		if (i<lenData)	{
//			MDR_CAN2->CAN_BUF[buf_num].DATAL = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Четвёртый..первый байт в пакете
//		if (i<lenData)	{
//			MDR_CAN2->CAN_BUF[buf_num].DATAH = ((*(pnt+i+3))<<24)|((*(pnt+i+2))<<16)|((*(pnt+i+1))<<8)|(*(pnt+i));	i+=4;		}					// Восьмой..пятый байт в пакете

//		//MDR_CAN2->INT_TX |= 1<<buf_num;																														// Регистр разрешения прерываний от передающих буферов TX_INT_EN
//		MDR_CAN2->BUF_CON[buf_num]	= (1 << CAN_BUF_EN)|(1 << CAN_TX_REQ);													// Запрос на отправку сообщения, установить бит TX_REQ
//		for(j=0; j < 4000; j++)	{	i=i;	}
//		cntFrms++;
//		}
//	}
//	
//	//MDR_CAN2->INT_EN = (1<<ERR_OVER_INT)|(1<<ERR_INT_EN)|(1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
//	MDR_CAN2->INT_EN = (1<<TX_INT_EN)|(1<<RX_INT_EN)|(1<<GLB_INT_EN);
//	MDR_CAN2->STATUS = 0;

//}

//end of can.c
