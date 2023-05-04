// Инициализация всего
#include <stdint.h>

#ifndef INIT_H
#define INIT_H

void Clock_Init (void);								// Инициализация тактового генератора
void WWDT_Init (void);								// Инициализация оконного следящего таймера
void Ports_Init (void);								// Инициализация всех портов 
void Ports_Init_Tst (void);
void ADC_Init (void);									// Инициализация АЦП и включение его прерываний
void UART_Init (void); 								// Инициализация UART 
//void Set_Adr_AE(void);								// Установка текущего адреса выбора АЭ
void Set_Adr_Datch(void);							// Установка текущего адреса выбора пары датчиков
//void Calendar_Init(void);
void SysTickInit(uint32_t nMks);			// Значение в микросекундах
#endif
