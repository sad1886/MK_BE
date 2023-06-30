/****************************************************************************************************************************
 * main.c
 * ЗАО ОРБИТА
 * V1.0.1
 * 27/03/2015
 * Карташов Ю.Д.
 ****************************************************************************************************************************
		ПО МУК БЭ предназначен для:
	Измерение давления и температуры в АБ с помощью двух групп (по 5штук) соответствующих датчиков.
	Измерение напряжений 72 аккумуляторных элементов (АЭ) АБ.
	Контроль заряда/разряда при отказе связи по CAN.
	Обеспечения пошагового режима при отладке с АКПА.
	
		По интерфейсу CAN от ЗРУ или АКПА принимает команды:
	включить шаговый режим;
	выдать напряжение пары АК;
	отключить шаговый режим;
	выдачи полного массива измерений аккумуляторов;
	выдачи полного массива информации ДД, ДТ;
	включения/отключения разрядного сопротивления;

		Передача информации в ЗРУ или АКПА.
	По интерфейсу передается порядка 100 непосредственных значений измерений параметров с пересчетом фактических напряжений 
	по датчикам давлений и температур и 15 интегральных (расчетных) параметров.

	К непосредственно измеренным параметрам относятся:
	72 значения  напряжений АЭ;
	напряжение АБ;
	5 значений напряжений давлений;
	5 значений напряжений температур;
	напряжения контрольных значений проверки АЦП.

	К интегральным параметрам относятся:
	напряжение АБ (измеряется в цикле измерения датчиков, в ЗРУ);
	средние значения напряжений по всем 72 АЭ, давлений по 5 датчикам и температур по 5 датчикам;
	максимальные и минимальные значения давлений с указанием порядковых номеров указанных ДД;
	максимальные и минимальные значения температур с указанием порядковых номеров указанных ДТ.
	Средние значения давлений, температур и напряжений АЭ вычисляются из массива корректных данных (без отказавших).
	
	4.8. ЧТЗ
	При выходе из строя обоих CAN активируется процесс управления запретами заряда и разряда внутренней подпрограммой
	контроля давления. При превышении давления Рв (50атм), выдается сигнал “запрет заряда” (PE2=1). При снижении давления
	ниже Рн (10атм) выдается сигнал “запрет разряда” (PE3=1). С гистерезисом 10%.
****************************************************************************************************************************/

#include <stdlib.h>		
#include "MDR32F9x.h"
#include "init.h"														// Файл с описанием процедур инициализации и аппаратных настроек
#include "init_bkp.h"												// Файл с описанием процедур инициализации часов реального времени
#include "init_IWDG.h"											// Файл с описанием процедур инициализации IWDG
#include "Can.h"
#include "parameters.h"

// Команды оконного следящего таймера
#define WWDT_ENABLE		MDR_WWDG->CR |= WWDG_CR_WDGA		// Запуск сторожевого таймера
#define WWDT_RESET		MDR_WWDG->CR |= WWDG_CR_T_Msk		// Сброс сторожевого таймера, максимально допуcтимый интервал срабатывания

//--------------------------- Работа CAN-----------------------------------------------------------------------
// Коды пакетов CAN
#define CAN_PI_Datch		0x01								// Отправить телеметрию ДД, ДТ 											*** МУК БЭ 
#define CAN_PI_AK				0x02								// Отправить телеметрию аккумуляторов 							*** МУК БЭ 
#define CAN_Pasport_Put	0x03								// Отправить Паспортные данные ДД, ДТ								*** МУК БЭ 
#define CAN_ResCmd			0x04								// Результат выполнения команды 										*** МУК БЭ 

#define CAN_Vkl_RS			0x05								// Включить разрядные сопротивления в БЭ 						*** АРК, КПА !
#define CAN_Otkl_RS			0x06								// Отключить разрядные сопротивления в БЭ 					*** АРК, КПА !
#define CAN_Step_On			0x07								// Включить шаговый режим 													*** КПА !
#define CAN_Step_Off		0x08								// Отключить шаговый режим 													*** КПА !
#define CAN_U_AK				0x09								// Выдать напряжение пары АК 												*** КПА !
#define CAN_SetPeriod		0x0A								// Установить период выдачи телеметрии датчиков			***	АРК, КПА !

#define CAN_MSG_Ok			0x0B								// Подтверждение в получении пакета телеметрии 			*** АРК, МУК БЭ, КПА !
#define CAN_StatErr			0x0C								// Ошибки аппаратуры и отказов ДД, ДТ, АЭ						*** МУК БЭ 
#define CAN_NumBadAk		0x0D								// Номера отказавших АЭ АБ													***	АРК, КПА 
#define CAN_Pasport_Get	0x0E								// Запрос на паспортные данные ДД, ДТ								*** КПА 

//--------------------------- Общие режимы работы -------------------------------------------------------------
#define Init_Run				0x10								// Инициализация всех процессов
#define AK_START				0x20								// Начальный запуск
#define NORMAL					0x30								// Режим опроса каналов АЦП
#define AK_AR_READY			0x40								// Обработка готового массива измерений аккумуляторов / датчиков
#define ADC_ERR					0x50								// Ошибка АЦП (появление флага OVERWRITE), повторный запуск опроса текущей пары каналов
//#define PACK_from_CAN		0x60								// Разбор пакета, полученного по CAN и выполнение принятой команды
#define RRazr_On				0x70								// Команда включение разрядного резистора
#define RRazr_Off				0x80								// Команда отключение разрядного резистора
#define MODE_STEP				0x90								// Пошаговый режим
#define HARD_ERR				0xA0								// Ошибка аппаратуры модуля
#define Read_U_AK				0xB0								// Чтение напряжений выбранных АК(i) и АК(i+1)
#define SEND_U_AK				0xC0								// Отправка напряжений выбранных АК(i) и АК(i+1)

#define ReadAE						1									// Режим чтения АЭ
#define ReadDatch					2									// Режим чтения напряжения на датчиках

//............................................................................................................
#define nPA7							7									// Номер вывода порта РА для увеличения нагрузки
#define nPA6							6									// Номер вывода порта РА для увеличения нагрузки
#define nPA5							5									// Номер вывода порта РА для увеличения нагрузки

//............................................................................................................
#define nChannels					2									// Число рабочих каналов АЦП (ADC2,ADC3) или (ADC4,ADC5)
#define Nadc							2									// Размерность рабочего массива измерения каналов АЦП
//
//const float	Uref[3]	= {2.985, 3.031, 3.019};		// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0
//
// ************************** Параметры для модуля МИБ ************************
//
#define nAllAE					72									// Число АЭ
#define MinValAdc				0x1									// Минимальное допустимое значение напряжение на АЭ (0,7В), АЦП=1
#define MaxValAdc				0x910								// Максимальное допустимое значение напряжение на АЭ (1,7В), АЦП=2320
#define npki						3										// Размерность массива контрольной ирформации pki

#define deltaUAB				1										// Пульсация напряжения АБ в рабочем режиме (Вольты)
#define ctstTime				10									// Период проверки измнения напряжения АБ (Секунды)

//............................................................................................................
unsigned char status_AB=0;									// Статус АБ: 0 - разряд, 1 - заряд

float Uref_wrk;															// Напряжение коррекции зачений измеренных каналов
float TtMUK;																// Температура МК для отладки
union uBytesFloat16 tMUK;										// Температура МК

union uBytesFloat16 AE[2];									// Массив двух АЭ для чтения в шаговом режиме
union uBytesFloat16 akkum[nAllAE];					// Значения напряжений на АЦП, заполняемые в процессе опроса АЭ

float AccumVoltage[nAllAE];
union uBytesFloat16 akkCn[nAllAE];					// Реальные (новые) значения напряжений (В) на АЭ, пересчитанные по формуле в ЧТЗ
union uBytesFloat16 akkCnCopy[nAllAE];			// Реальные (новые) значения напряжений (В) на АЭ, пересчитанные по формуле в ЧТЗ
union uBytesFloat16 akkCnOld[nAllAE];				// Реальные (старые) значения напряжений (В) на АЭ, пересчитанные по формуле в ЧТЗ

union uBytesFloat16 aIrazr, aIzar;					// Получены по CAN из ЗРУ для коррекции U AKi

unsigned char cBadAE[nAllAE]	=	{ 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
																	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
																	2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
																	2,2,2,2,2,2,2,2,2,2,2,2	};								// Флаг недостоверного значения U аккумуляторного элемента
unsigned char cNulU_AE;											// Счётчик значений U аккумуляторов равных НУЛЮ

unsigned char nBadAE_BCU[nMaxBadAE];				// Номера отказавших АЭ, полученные по CAN от БЦУ
float deltaUAE;															// 

volatile unsigned char nAE;									// Число рабочих аккумуляторных элементов

float Takkum[nAllAE];												// Значения напряжений на АЦП, заполняемые в процессе опроса АЭ

union uBytesFloat16 pki[npki];							// pki[0]-сред знач, pki[1]-макс отклон, pki[2]- напряжение АБ
volatile unsigned char ibias, imaxU, iminU;	// Номера АЭ макс отклон от среднего, с максимальным и минимальным значением напряжения
union uBytesFloat16 bias, maxU, minU;				// Макс отклон от среднего, макс и мин значение напряжения
union uBytesFloat16 dU_AE_IsmAE;						// Разница напряжений между измерительными и неизмерительными НВА

volatile int iBadAE;												// Указатель на свободный элемент массива nBadAE, число отказавших АЭ
volatile unsigned char nBadAE[nMaxBadAE];		// Номера вышедших из строя элементов аккумуляторов

volatile unsigned char str;									// Строка для формирования адреса выбора АЭ
volatile int c;							// Добавка к индексу "a", счётчик циклов измерений 1...72
//volatile int endcycl;												// Значение завершения цикла
volatile unsigned char col, count_cycl;			// Столбец для формирования адреса выбора АЭ, счётчик циклов (1..6) чтения значения напряжения АЭ

volatile unsigned char bDataABOk;						// Данные по АБ готовы

// 
// ************************** Параметры для модуля ПСД ************************
// 
#define ndatch	6														// Размерность массива datch
#define ndatCn	26													// Число значений данных по датчикам для передачи по CAN

float PressureSensorVoltage[nDatchD];
float TemperatureSensorVoltage[nDatchD];
union uBytesFloat16 UdatchD[nDatchD];				// Значения напряжений на ДД
union uBytesFloat16 UdatchT[nDatchT];				// Значения напряжений на ДТ

float tUrealOld[nDatchT], tUreal[nDatchT];
float Old_sumAE;														// Старое (предыдущее) заначение напряжения АБ

union uBytesFloat16 UdatchDold[nDatchD];		// Значения напряжений на ДД предыдущее
union uBytesFloat16 UdatchTold[nDatchT];		// Значения напряжений на ДТ предыдущее
unsigned char cBadUD[nDatchD];							// Счётчик Флаг недостоверного значения U
unsigned char cBadUT[nDatchD];							// Счётчик Флаг недостоверного значения U

float VdatchDD[nDatchD];										// Значения на ДД

union uBytesFloat16 VdatchD[nDatchD];				// Значения на ДД
union uBytesFloat16 VdatchT[nDatchT];				// Значения на ДТ

//float datchT[ndatch];					// Для отладки Значения U_АБ1, U_АБ2, U_опДД, U_опДТ, Общ_ДД, Общ_ДТ, заполняемые в процессе опроса АЦП //Ненужно???

//float BatteryVoltageCh1, BatteryVoltageCh2;
float AB1, AB2;
union uBytesFloat16 oldAB[2];								// Значения U_АБ1, U_АБ2
unsigned char cBadUAB[2];										// Счётчик Флаг недостоверного значения U

union uBytesFloat16 datch[ndatch];					// Значения U_АБ1, U_АБ2, U_опДД, U_опДТ, Общ_ДД, Общ_ДТ, заполняемые в процессе опроса АЦП
float fdatch[ndatch];												// Значения U_АБ1, U_АБ2, U_опДД, U_опДТ, Общ_ДД, Общ_ДТ, заполняемые в процессе опроса АЦП

union uBytesFloat16 datCn[ndatCn];					// Значения на 2*(ДД+ДТ)+6, U_АБ, U_опДД, U_опДТ, Общ_ДД, Общ_ДТ предназначенные для передачи по CAN
float fdatCn[ndatCn];												// Значения на 2*(ДД+ДТ)+6, U_АБ, U_опДД, U_опДТ, Общ_ДД, Общ_ДТ предназначенные для передачи по CAN

union uBytesFloat16 pkiDD[2]={0,0};					// Массив контрольных значений. среднего значения на ДД, наибольшего отклонения
union uBytesFloat16 pkiDT[2]={0,0};					// Массив контрольных значений. среднего значения на ДТ, наибольшего отклонения
volatile unsigned char ibiasDD, ibiasDT;		// Номера с наибольшим отклонением значением давления и температуры

volatile unsigned char imaxPDD, iminPDD,		// Номера с максимальным и минимальным значением давления
											 imaxCDT, iminCDT;		// Номера с максимальным и минимальным значением температуры
union uBytesFloat16 biasDD, maxPDD, minPDD,
										biasDT, maxCDT, minCDT;	// Отклонение от среднего, макс и мин значение давления и температуры
//union uBytesFloat16 W_tek;									// Текущее значение энергии разряда
volatile unsigned char nDD;									// Число рабочих ДД
volatile unsigned char nDT;									// Число рабочих ДТ
unsigned char iDT, iDD;											// Индексы датчиков 0..4

volatile unsigned char bReadDatch=0;
volatile unsigned char bDataDatchOk;				// Данные по датчикам готовы

volatile unsigned char nBitBadDatchD;				// Маска вышедших из строя датчиков давления
volatile unsigned char nBitBadDatchT;				// Маска вышедших из строя датчиков температуры
volatile int iBadDatchD, iBadDatchT;				// Указатель на свободный элемент массива nBadDatch, число отказавших датчиков

//--------------------------- Общие ---------------------------------------------------------------------------------------
	float fp, ft;
int cntst;
volatile unsigned char mode, HardErr;				// Текущий режим работы контроллера, опорное напряжение < 2.3В
volatile unsigned char new_mode;						// Для смены режима
volatile unsigned char save_mode;						// Для сохранения текущего режима при обслуживании прерыв по CAN
volatile unsigned char StatRdADC, OkDataADC;// Статус чтения АЦП. Значение канала АЦП готово

volatile unsigned char coun=0;							// Счётчик принятых пакетов от ЗРУ
volatile unsigned char i;										// "Бегунок" по массивам общий
volatile unsigned char full_send=0;					// Отправка полной информации или только ПКИ

volatile int iadc, j;												// Указатель на текущий канал измерения АЦП
volatile int add_chanl;											// Смещение для вычисления реального номера канала АЦП

volatile int nReadADC;										  // Количество чтений АЦП для одного канала

volatile int iReadAdc, ii;									// Указатель на текущий канал измерения АЦП
volatile int NumReadAdc;										// Число достоверных значений измерения АЦП текущего канала
uint32_t Result;														// Результат чтения АЦП
unsigned char OkResult;											// Результат получен

uint16_t TempCod;														// Результат чтения АЦП код температуры МК
uint32_t summa;															// Сумма значений кодов АЦП

volatile int a;															// "Бегунок" по номерам аккумуляторов
volatile uint32_t ADC1_REG_GO=2;						// #define ADC1_CFG_REG_GO  ((uint32_t)0x00000002)

volatile float Uadc;												// Значения, прочитанные АЦП в вольтах
volatile float tUadcDT[5];									// Значения, прочитанные АЦП в вольтах
volatile float tUadcDD[5];									// Значения, прочитанные АЦП в вольтах

volatile float OldUadc[18];									// Последние Значения АЦП по шагам a=0..17 при чтении заначений датчиков
unsigned char cBadUadc[18];									// Счётчики значений напряжений = 0 каналов измерения ДД и ДТ

volatile float adc[Nadc];										// Значения, прочитанные АЦП в вольтах
volatile float * volatile pr;								// Промежуточный указатель для работы с массивами
volatile unsigned char * volatile p_CANdata;// Указатель на массив значений датчиков, передаваемый по CAN

volatile unsigned char LendataCAN;					// длина массива передаваемых данных по CAN
volatile unsigned char bOkCAN, bMagorOk;		// Флаг CAN работает
//volatile unsigned char bControlP;						// Флаг работы процедуры контроля давления
volatile unsigned char bStatPC;							// Состояние разрядных резисторов: 0 - выкл., 1 - вкл.
volatile unsigned char dataCAN[200];				// данные, предназначенные для передачи по CAN.

volatile unsigned char bTrnsAB;							// Флаг разрешения передачи телеметрии АБ
volatile unsigned char bTrnsDats;						// Флаг разрешения передачи телеметрии датчиков

float minUAE = -1.0;												// (В) Нижняя граница значения рабочего напряжения АЭ
float maxUAE =  2.0;												// (В) Верхняя граница значения рабочего напряжения АЭ

//--------------------------- Входная информация по CAN -------------------------------------------------------------------
																						// 4, 5, 6	Номер/адрес МУК1..3 ЗРУ, (КПА использует те же адреса)
volatile unsigned char nMUK_ZRU[3]	= {0,0,0};	// Флаг активности узла-передатчика пакета
volatile unsigned char codMUK_ZRU[3]= {0,0,0};	// Коды пакетов

uint32_t	RecievCanID;
uint32_t	RecievCanDLC;
uint32_t 	RecievCanDATAL;										// байт4-параметр1, байт3-параметр1, байт2-пошаговый режим, байт1-команда
uint32_t 	RecievCanDATAH;										// байт8, байт7, байт6, байт5

volatile unsigned char byteCMD, bStep;			// Команда, принятая по CAN. Флаг пошагового режима дл АКПА
volatile unsigned char lastCMD;							// Последняя команда, принятая по CAN
volatile unsigned char PeriodSndTelem;			// Период выдачи телеметрии

//--------------------------- Коды дешифратора для чтения АЭ1, АЭ1, ... -------------------------------------------------------------
#define  MaxTwoAEs		39										// Максим номер пары АЭ, полуенный от КПА (для измерения напряжения)
//																1			2			3			4			5			6			7			8			9			10		11		12
int KodDeshifr[MaxTwoAEs] =	 {	0x9, 	0xA, 	0xB, 	0xC,	0xD, 	0xE, 	0x17, 0x11, 0x12, 0x13, 0x14, 0x15,
																0x1E,	0x1F, 0x19, 0x1A, 0x1B, 0x1C, 0x25, 0x26, 0x27, 0x21, 0x22, 0x23,
																0x2C,	0x2D, 0x2E, 0x2F, 0x29, 0x2A, 0x33, 0x34, 0x35, 0x36, 0x37, 0x31,
																0x3A, 0x3B, 0x0};


uint32_t nTwoAEs;														// Номер пары АЭ, полуенный от КПА (для измерения напряжения)
																
//--------------------------- Данные для работы с системным таймером ------------------------------------------------------
volatile unsigned char	bPeriod=0, 					// Флаг разрешения передачи телеметрии датчиков
												timerST=0,					// Счётчик прерываний телеметрии датчиков
												secST;							// Счётчик секунд
volatile int deleyST=10;										// 10 - 1 секунда, 5 - 0.5 секунды через deleyST передача телеметрии.
volatile int bWeitTime=1;										// Флаг времени ожидания прихода команд от МУК1..3 ЗРУ
volatile int bWeitCycl;											// Задержка для МУК3 в цикле измерения датчиков
volatile int WeitTime=0;										// Время ожидания прихода команд от МУК1..3 ЗРУ
volatile int WeitTimeC=0;										// Время ожидания прихода команд от МУК1..3 ЗРУ

volatile unsigned char bStrtGetCmd=0;				// Стартовал процесс прихода команд от МУК1..3 ЗРУ
volatile unsigned char bPauseSetAdr;				// Флаг - пауза 
volatile unsigned char cPauseSetAdr;				// Счётчик для паузы
volatile int vtstTime;											// Время проверить измнение напряжения АБ

//--------------------------- Данные для работы с датой и временем --------------------------------------------------------
extern tTime	sTime;
extern int NewDay;

volatile unsigned char mCount5Main,
											 mCountSecMain;				// Счётчик 5 мин для измерения температуры АБ, счётчик секунд
volatile unsigned char mCount5,
											 mCountSec;						// Счётчик 5 мин для паузы
volatile unsigned char bOneSec;							// Флаг 1 секунда
volatile unsigned char sCount5;							// Счётчик 5 сек для расчёта C и W
volatile unsigned char sCount20;						// Счётчик 20 сек для задержки повтора 3 раза алгоритма заряда
volatile unsigned char bPauza5, bPauza20,
											 bPauza5m;						// Флаги включения пауз 5 сек, 20 сек, 5 мин
volatile unsigned char nSutok;							// Число суток с начала счёта часов в ТВЦ при разряде РС

unsigned char bFl=1;

int cnt_tst30=0;

//--------------------------- Датчик температуры МК -----------------------------------------------------------------------
#define FACTORY_TEMP25      		25     							// ADC value = 1700 @ 25C = 1.36996V - from milandr demo project
#define FACTORY_ADC_TEMP25      1700     						// ADC value = 1700 @ 25C = 1.36996V - from milandr demo project
//#define FACTORY_VTEMP25         1.36996  				 	// 1.38393 @ 26C. 1.34-1.52, 1.43 V typical @ factory delta_calib
#define FACTORY_ADC_AVG_SLOPE   5.83    	   				// ADC delta value @ 1C, from milandr demo project
//#define FACTORY_AVG_SLOPE      0.004835				   // 4.0-4.6, 4.3 mV/C typical @ factory delta_calib

// расчёт в int
//temperature_C = (adc_value - FACTORY_ADC_TEMP25)/FACTORY_ADC_AVG_SLOPE + FACTORY_TEMP25;
// я предпочитаю вот так (расчёты во float):
//temperature_C = ((Vtemp - Vtemp25) / Avg_Slope) + FACTORY_TEMP25;

//..............................................................................................................................
float tmp[10]	=	{1.47, 1.46, 1.52, 1.47, 1.47, 1.41, 1.41, 0.47, 1.47, 1.47};
float tmO[10];
float tmpV, akkCnOldV, akkCnV;

unsigned char  bVklZRU;															// Состояние ЗРУ: 1 - подана команда ВклЗРУ
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
__fp16 Fdata;

float abs_f (float vf)
{	if (vf<0)	return -vf;	else 	return vf;
}	

//__fp16 abs_f16 (__fp16 vf16)
//{	if (vf16<0)	return -vf16;	else 	return vf16;
//}	

// Выбор канала и включение АЦП
// ch - номер канала				1f0 | 4<<4 | 1
void ADC_Start(int ch)
{
	MDR_ADC->ADC1_CFG &= (~ADC1_CFG_REG_CHS_Msk);																	// Сброс номера канала
	MDR_ADC->ADC1_CFG |= ((ch+add_chanl)<<ADC1_CFG_REG_CHS_Pos);									// Установить номер канала
	MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_ADON;																				// АЦП включено
}
//-------------------------------------------------------------------------------------------------------------------------------------------------
// Задержка
void Wait_ (int wait)	{	ii=0;	while (ii<wait) {ii++;}	}													// max int 2 147 483 647

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Зауск процесса преобразования
void ADC_GO (void)
{	
	MDR_ADC->ADC1_STATUS |= ADC_STATUS_ECOIF_IE;																	// Разрешение прерываний от АЦП
	ADC_Start(iadc);																															// Опрос начинается с iadc+2-го канала
	Wait_(100);
	
	//MDR_PORTA->RXTX &= (~0xe0);																										// PA5,PA6,PA7=0 для уменьш потребл тока

	MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;																					// Запуск преобразования
}


//=================================================================================================================================================
void ADC_IRQHandler (void)																													// Обработчик прерывания от АЦП
//=================================================================================================================================================
{	//uint32_t Result;
	
	if (!(MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_OVERWRITE))	{											// Проверка на переписывание значения в регистре
		if ((MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_EOCIF)==ADC_STATUS_FLG_REG_EOCIF)		// Проверка готовности данных в регистре
		{	
			Result = (MDR_ADC->ADC1_RESULT & ADC_RESULT_Msk);															// Чтение (nReadADC раз) результата преобразования во временный массив
			OkResult = 1;
		}
		else mode=ADC_ERR;																									  					// Перезапуск опроса текущей четвёрки
	}
	else	{
		new_mode=ADC_ERR;																																// Перезапуск опроса текущей четвёрки
		MDR_ADC->ADC1_STATUS &=(!ADC_STATUS_FLG_REG_OVERWRITE);													// Сброс флага перезаписи
	}

	if (MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_AWOIFEN)
	{
		MDR_ADC->ADC1_STATUS &= ~(ADC_STATUS_FLG_REG_AWOIFEN);
	}
	
}

//=================================================================================================================================================
void ADC_IRQHandler_Old (void)																													// Обработчик прерывания от АЦП
//=================================================================================================================================================
{	//uint32_t Result;
	
	if (!(MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_OVERWRITE))	{											// Проверка на переписывание значения в регистре
		if ((MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_EOCIF)==ADC_STATUS_FLG_REG_EOCIF)		// Проверка готовности данных в регистре
		{	
			Result = (MDR_ADC->ADC1_RESULT & ADC_RESULT_Msk);															// Чтение (nReadADC раз) результата преобразования во временный массив
			//Result = 0xff;																															// Тестовое значение

			iReadAdc++;																																		// 
			if (iReadAdc < nReadADC)	{																										// Промежуточный массив значений текущего канала заполнен
				if (Result>0)	{	
					summa+=Result;	NumReadAdc++;		}																					// Допустимый результат
					MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO & ADC1_REG_GO;												// Повторить измерение
			}
			else	{
				
				if ((NumReadAdc)&&(summa)) {
					Result = summa/NumReadAdc;
					if (add_chanl)	{	Uadc = ((Result)*(AUcc[nMUK_BE-1]/0xfff));					}		// Пересчёт значения АЦП в вольты
					else						{	
							AUcc[nMUK_BE-1] = (Uref[nMUK_BE-1]*0xfff)/(Result);										// Коррекция AUcc
					}
				}	
				else	{	Uadc=0;	}																														// АЭ вышел из строя
				OkDataADC=1;	//summa=0;	NumReadAdc=0;
			}	
		}
		else mode=ADC_ERR;																									  					// Перезапуск опроса текущей четвёрки
	}
	else	{
		new_mode=ADC_ERR;																																// Перезапуск опроса текущей четвёрки
		MDR_ADC->ADC1_STATUS &=(!ADC_STATUS_FLG_REG_OVERWRITE);													// Сброс флага перезаписи
	}

	if (MDR_ADC->ADC1_STATUS & ADC_STATUS_FLG_REG_AWOIFEN)
	{
		MDR_ADC->ADC1_STATUS &= ~(ADC_STATUS_FLG_REG_AWOIFEN);
	}
	
}

//=================================================================================================================================================
void CAN1_IRQHandler()
//=================================================================================================================================================
{	uint32_t can1_status;
	unsigned char  cod, adrTx;	//, adrRx, msg;
		
	can1_status = MDR_CAN1->STATUS;
	
	if(can1_status & (1<<CAN_TX_READY))	{																						// Передача
		MDR_CAN1->INT_EN &= ~(1<<TX_INT_EN);
	}

	if((can1_status & (1<<CAN_RX_READY)))		{																				// Приём

		RecievCanDATAH = MDR_CAN1->CAN_BUF[nbuf_RX].DATAH;
		RecievCanDATAL = MDR_CAN1->CAN_BUF[nbuf_RX].DATAL;
		
		if(CAN1->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))	{
			MDR_CAN1->BUF_CON[nbuf_RX] &= ~(1<<CAN_RX_FULL);														//Флаг готовности приема: 0 – нет принятого сообщения;1 – принятое сообщение в буфере
		}

		//....................................................................................................................
		cod		= 0x0000000f & MDR_CAN1->CAN_BUF[nbuf_RX].ID >> 12;											// Код пакета
		adrTx = 0x0000000f & MDR_CAN1->CAN_BUF[nbuf_RX].ID >> 20;											// Адрес узла-передатчика пакета
	
		if ((adrTx>6)&&(cod>0x04)&&(cod<0x0F))	{																			// обслужить запросы adrTx = 7..12 от МУК1..3 ЗРУ, КПА
			if (bWeitTime)	{																														// Флаг времени ожидания прихода всех команд от МУК1..3 ЗРУ (1 секунда)
				bStrtGetCmd = 1;																													// Стартовал процесс прихода команд от МУК1..3 ЗРУ
				bWeitTime = 0;	WeitTime=0;																								// Сброс при получении первой команды
			}	
			// ...............................................................................................
			switch (adrTx)	{
			case 7:
			case 8:		adrTx=0;	break;																									// Пакет от МУК1
			case 9:
			case 10:	adrTx=1;	break;																									// Пакет от МУК2
			case 11:
			case 12:	adrTx=2;	break;																									// Пакет от МУК3
			default:	;
			}
			nMUK_ZRU[adrTx] = 1;																												// Флаг активности узла-передатчика пакета
			codMUK_ZRU[adrTx] = cod;																										// Код (команда) пакета
			lastCMD = cod;
		}																																							//  4, 5, 6, 7	адрес МУК1..3 ЗРУ, КПА 
//
	}

}

//=================================================================================================================================================
void CAN2_IRQHandler()
//=================================================================================================================================================
{	uint32_t can2_status;
	unsigned char  cod, adrTx;	//, adrRx, msg;
		
	can2_status = MDR_CAN2->STATUS;
	
	if(can2_status & (1<<CAN_TX_READY))	{																						// Передача
		MDR_CAN2->INT_EN &= ~(1<<TX_INT_EN);
	}

	if((can2_status & (1<<CAN_RX_READY)))		{																				// Приём

		RecievCanDATAH = MDR_CAN2->CAN_BUF[nbuf_RX].DATAH;
		RecievCanDATAL = MDR_CAN2->CAN_BUF[nbuf_RX].DATAL;
		
		if(CAN2->CAN_BUF_CON[nbuf_RX] & (1<<CAN_RX_FULL))	{
			MDR_CAN2->BUF_CON[nbuf_RX] &= ~(1<<CAN_RX_FULL);														//Флаг готовности приема: 0 – нет принятого сообщения;1 – принятое сообщение в буфере
		}

		//....................................................................................................................
		cod		= 0x0000000f & MDR_CAN2->CAN_BUF[nbuf_RX].ID >> 12;											// Код пакета
		adrTx = 0x0000000f & MDR_CAN2->CAN_BUF[nbuf_RX].ID >> 20;											// Адрес узла-передатчика пакета
	
		if ((adrTx>6)&&(cod>0x04)&&(cod<0x0F))	{																			// обслужить запросы adrTx = 7..12 от МУК1..3 ЗРУ, КПА
			if (bWeitTime)	{																														// Флаг времени ожидания прихода всех команд от МУК1..3 ЗРУ (1 секунда)
				bStrtGetCmd = 1;																													// Стартовал процесс прихода команд от МУК1..3 ЗРУ
				bWeitTime = 0;	WeitTime=0;																								// Сброс при получении первой команды
			}	
			// ...............................................................................................
			switch (adrTx)	{
			case 7:
			case 8:		adrTx=0;	break;																									// Пакет от МУК1
			case 9:
			case 10:	adrTx=1;	break;																									// Пакет от МУК2
			case 11:
			case 12:	adrTx=2;	break;																									// Пакет от МУК3
			default:	;
			}
			nMUK_ZRU[adrTx] = 1;																												// Флаг активности узла-передатчика пакета
			codMUK_ZRU[adrTx] = cod;																										// Код (команда) пакета
			lastCMD = cod;
		}																																							//  4, 5, 6, 7	адрес МУК1..3 ЗРУ, КПА 
//
	}

}


//=================================================================================================================================================
void SysTick_Handler()
//=================================================================================================================================================
{
	if (timerST < deleyST){	timerST++;		}																					// Флаг разрешения передачи телеметрии датчиков через deleyST сек.
	else				{	timerST=0;		bPeriod=1;	}

	if ((WeitTimeC < 12)&&(!bWeitCycl))	{	WeitTimeC++; 		}													// Задержка для МУК3 в цикле измерения датчиков
	else								{	WeitTimeC=0;		bWeitCycl=1;	}
	
	if (WeitTime < 10)	{	WeitTime++;		}																						// Время ожидания прихода команд от МУК1..3 ЗРУ	 1 сек
	else				{	WeitTime=0;		bWeitTime = 1;																			// Флаг времени ожидания прихода команд от МУК1..3 ЗРУ
	}

	if (secST < 10)	{secST++;
		if ((bPauseSetAdr)&&(cPauseSetAdr<1)) cPauseSetAdr++; 
		else	{	cPauseSetAdr=0;	bPauseSetAdr=0;	}	
	}	
	else 	{	secST = 0;																															// Счётчик секунд
		sTime.sec++;	bOneSec=1;
		mCountSecMain++;
		
		if (mCountSecMain==59)	{	mCountSecMain=0;	mCount5Main++;	}									// Счётчик мин в main
//		if (bPauza5)	{																																// Обслуживание паузы 5 сек
//			sCount5++;
//			if (sCount5 >= 5)		{	
//				bPauza5 =0;	sCount5 =0;	}	
//		}
		if (bPauza20)	{	sCount20++;	if (sCount20==20)	{bPauza20=0;	sCount20=0;}	}		// Обслуживание паузы 20 сек 
		if (bPauza5m)	{	mCountSec++;																									// Обслуживание паузы 5 мин 
			if (mCountSec==59)	{	mCountSec=0;	mCount5++;}				
			if (mCount5	 == 5)	{	bPauza5m =0;	mCount5=0;}
		}
		if (sTime.sec == 59)	{		sTime.sec=0;																				// Прошло 60 сек
			if (sTime.min < 59)		{	sTime.min++;	mCount5Main++;}
			else	{									sTime.min=0;	
				if (sTime.hour < 23)	{	sTime.hour++;	}
				else	{									sTime.hour = 0;		NewDay=1;												// Смена суток
				}
			}
		}
	}		
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Установить выходной код на портах РA2…РA0 для коммутации ДДi и ДТi на входы ADC2, ADC3 
void Set_Adr_Datch(void)											//(unsigned char str_, unsigned char col_)
{
	//MDR_PORTA->RXTX &= 0xfffffff8;																						// Сброс адреса. РA4,РA3 - E,D. Вкл/выкл РС
	MDR_PORTA->RXTX &= 0xf8;																									// Сброс адреса. РA4,РA3 - E,D. Вкл/выкл РС
	MDR_PORTA->RXTX |= (0x7 & str);																						// Адрес коммутации пары датчиков РA2…РA0 - C,B,A
	summa=0;	NumReadAdc=0;																										// Очистка перемен. суммы и числа корректных значений для получения усреднённого значения АЦП
	iReadAdc=0;																																// Индекс временного массива для получения усреднённого значения АЦП
	if (str)	str--;																													// Следующий адрес
	else str=7;
	for (i=0;i<koefZadergDat;i++)	{	Wait_(10000);	}														// Ожидание коммутации ключей дешифратора
	
	//MDR_PORTA->RXTX &= (~0xe0);																								// PA5,PA6,PA7=0 для уменьш потребл тока
}	

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Установить выходной код на портах PB10,PB9,PB8,PB7,PB6,PB5 для коммутации i-го и i+1-го АЭ на входы ADC4, ADC5 
// 	С 17.02.2015											0		0		1		0		0		1
void Set_Adr_AE_Step(int kod)	
{																																						// Начальные значения str=1;	col=1; count_cycl=1; 
 	MDR_PORTB->RXTX  = 0x00000000;																						// Сброс 
 	MDR_PORTB->RXTX  = kod<<5;																								// Установить выходной код на порте
	
	summa=0;	NumReadAdc=0;		iReadAdc=0;
	for (i=0;i<koefZadergAE;i++)	{	Wait_(10000);	}														// Ожидание коммутации ключей дешифратора
	
	//MDR_PORTA->RXTX &= (~0xe0);																								// PA5,PA6,PA7=0 для уменьш потребл тока
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
int MagorOk (void)
{
	if ((nMUK_ZRU[0] && nMUK_ZRU[1] && (codMUK_ZRU[0] == codMUK_ZRU[1]))||
			(nMUK_ZRU[0] && nMUK_ZRU[2] && (codMUK_ZRU[0] == codMUK_ZRU[2]))||
			(nMUK_ZRU[1] && nMUK_ZRU[2] && (codMUK_ZRU[1] == codMUK_ZRU[2])))
	{				return 1;	}
	else		return 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Подключить разрядные резисторы. PA3, PA4 замыкает ключи, соединяющие разрядный резистор с общим проводом
void RRazrOn()
{
	MDR_PORTA->RXTX &= ~0x8;																									
	MDR_PORTC->RXTX &= ~0x1;																									
	MDR_PORTA->RXTX |= 0x10;																									// PA3=0, PA4=1
	MDR_PORTA->RXTX |= (1 << 6);
	MDR_PORTC->RXTX |= 0x02;																									// PC0=0, PC1=1
	Wait_(50);
	bStatPC = 1;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Отключить разрядные резисторы. PA3, PA4 размыкает ключи, соединяющие разрядный резистор с общим проводом
void RRazrOff()		
{
	MDR_PORTA->RXTX &= ~0x10;																									// 
	MDR_PORTA->RXTX &= ~(1 << 6);
	MDR_PORTC->RXTX &= ~0x02;																									// 
	MDR_PORTA->RXTX |= 0x8;																										// PA3=1, PA4=0
	MDR_PORTC->RXTX |= 0x1;																										// PC0=1, PC1=0
	Wait_(50);
	bStatPC = 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Инициализация переменных
void Var_init()
{	ADC1_REG_GO=2;
	OkResult=0;
	OkDataADC=0;	summa=0;	NumReadAdc=0;
	iReadAdc=0;																																// Индекс временного массива для получения усреднённого значения АЦП
	str=7;																																		// Начальные значения для формирования адреса коммутатации выводов ДД и ДТ 
	iBadDatchD=0;	iBadDatchT=0;																								// iBadAE=0;		Счётчик неисправных АЭ и индекс в массиве номеров неисправных ДД и ДТ
	a=0;	c=0;																																// Индекс адреса. Добавка к индексу "a", счётчик циклов измерений 1...72
	nReadADC = nWrkReadADC;																										// Число чтений канала АЦП для получения усредненного значения
	for (i=0;i<Nadc;i++)	{	adc[i]=0;	}																				// Инициализация массива измеряемых значений каналов АЦП
	iadc=0;																																		// 
	bReadDatch=0;
	cNulU_AE=0;
	p_CANdata = dataCAN;
	//bPauza5 = 1;	sCount5 =0;																									// Флаг отсчёта паузы 5 сек
	bOneSec = 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Обработка массива напряжений ДД и ДТ. Нормализация напряжения
void RaschotArrayDatch (void)
{	float Rcur, Ureal;
	float sumDD, sumDT;																												// Суммы для вычисления среднего значения
	float proc=0.66;																													// Допустимый разброс значений МУКов БЭ 66%

	//************* Температура ТФ = (RДТ – R0)/α*R0 *************************************
	for (iDT=0;iDT<5;iDT++)	{																									//	tUadcDT[iDT] = Uadc;	Значения, прочитанные АЦП в вольтах
					
		Ureal = tUadcDT[iDT] * kDT[nMUK_BE-1][iDT] + dDT[nMUK_BE-1][iDT] + kTemp[nMUK_BE-1][iDT] * (TempCod-kodADC_t25[nMUK_BE-1]);
		
		if (tUadcDT[iDT] < LimUadc[nMUK_BE-1][iDT]) {														// коррекция первой уставки
			//Ureal = Ureal - kLimUadc[nMUK_BE-1][iDT]*((tUadcDT[iDT] - LimUadc[nMUK_BE-1][iDT])*(tUadcDT[iDT] - LimUadc[nMUK_BE-1][iDT]));
			
			Ureal = Ureal - kLimUadc[nMUK_BE-1][iDT]*((tUadcDT[iDT] - LimUadc[nMUK_BE-1][iDT])*(tUadcDT[iDT] - LimUadc[nMUK_BE-1][iDT])) - kLimUadcT[nMUK_BE-1][iDT] * (TempCod-kodADC_t25[nMUK_BE-1]) ;
			
		}	
		
		if (tUadcDT[iDT] > LimUadc_5[nMUK_BE-1][iDT]) {													// коррекция пятой уставки
			//Ureal = Ureal + kLimUadc_5[nMUK_BE-1][iDT]*((tUadcDT[iDT] - LimUadc_5[nMUK_BE-1][iDT])*(tUadcDT[iDT] - LimUadc_5[nMUK_BE-1][iDT]));
			
			Ureal = Ureal + kLimUadc_5[nMUK_BE-1][iDT]*((tUadcDT[iDT] - LimUadc_5[nMUK_BE-1][iDT])*(tUadcDT[iDT] - LimUadc_5[nMUK_BE-1][iDT])) - kLimUadc_5_T[nMUK_BE-1][iDT] * (TempCod-kodADC_t25[nMUK_BE-1]) ;;
		}	
		
		tUreal[iDT] = Ureal;
		TemperatureSensorVoltage[iDT] = Ureal;
		
	 if ((tUadcDT[iDT] >= MinUdatchT)&&(tUadcDT[iDT] <= MaxUdatchT))	{
		//............. коррекция значений напряжения .....................................
		deltaUAE = tUreal[iDT] - tUrealOld[iDT];
		if (abs_f(deltaUAE) >= 0.001)	{
			cBadUT[iDT]++;
			if (cBadUT[iDT] <= nCorrectDT)	{		tUreal[iDT]		 = tUrealOld[iDT];	}
			else								{		tUrealOld[iDT] = tUreal[iDT];		cBadUT[iDT]=0;	}
		}
		else	{	
			if (abs_f(deltaUAE) > 0.0001)	{	
				if (deltaUAE > 0)	{		tUrealOld[iDT] += 0.0001;	}	
				else							{		tUrealOld[iDT] -= 0.0001;	}	
				tUreal[iDT] = tUrealOld[iDT];
			}
			tUrealOld[iDT] = tUreal[iDT];
		}
		//................................................................................
		UdatchT[iDT].Fdata = tUreal[iDT];
		
			Rcur = tUreal[iDT]/ItarT[iDT];
			VdatchT[iDT].Fdata = (Rcur-R0[iDT])/(alpha[iDT]*R0[iDT]);							// Фактическая температура в грд Цельсия
		
		}
		else			{	
			VdatchT[iDT].Fdata = 99;
		}	
	}						
				
	//************* Давление P = {a1(UДД) + a2(UДД2) + a3} *************************************
	for (iDD=0;iDD<5;iDD++)	{
					
		Ureal = kDD[nMUK_BE-1][iDD] * tUadcDD[iDD] + dDD[nMUK_BE-1][iDD];
		
		UdatchD[iDD].Fdata = Ureal;
		PressureSensorVoltage[iDD] = Ureal;
		
	 if ((tUadcDD[iDD] >= MinUdatchD)&&(tUadcDD[iDD] <= MaxUdatchD))	{	
			
		//............. коррекция значений напряжения .....................................
		deltaUAE = UdatchD[iDD].Fdata - UdatchDold[iDD].Fdata;
		if (abs_f(deltaUAE) >= 0.001)	{
			cBadUD[iDD]++;
			if (cBadUD[iDD] <= nCorrectDD)	{	
				UdatchD[iDD].Fdata 		= UdatchDold[iDD].Fdata;	}
			else							{	
				UdatchDold[iDD].Fdata = UdatchD[iDD].Fdata;
				cBadUD[iDD]=0;	}
		}
		else	{	
			if (abs_f(deltaUAE) > 0.0001)	{	
				if (deltaUAE > 0)	{		UdatchDold[iDD].Fdata += 0.0001;	}	
				else							{		UdatchDold[iDD].Fdata -= 0.0001;	}	
				UdatchD[iDD].Fdata = UdatchDold[iDD].Fdata;
			}
			UdatchDold[iDD].Fdata = UdatchD[iDD].Fdata;
		}
		//................................................................................
		Ureal = UdatchD[iDD].Fdata*1000;
					
			VdatchD[iDD].Fdata =  (a2[nMUK_BE-1][iDD]*Ureal*Ureal + a1[nMUK_BE-1][iDD]*Ureal + a0[nMUK_BE-1][iDD]);
		}
		else	{
			VdatchD[iDD].Fdata = 99;	
		}
	}	
	
	//************* Корркция значений AB1, AB2 ********************************************
	for (iDD=0;iDD<2;iDD++)	{
		
		deltaUAE = datch[iDD].Fdata - oldAB[iDD].Fdata;													// Значения U_АБ1, U_АБ2
		
		if (abs_f(deltaUAE) >= 0.9)	{
			cBadUAB[iDD]++;
			if (cBadUAB[iDD] <= nCorrectDAB)	{	
				datch[iDD].Fdata 		= oldAB[iDD].Fdata;	
			}
			else							{	
				oldAB[iDD].Fdata = datch[iDD].Fdata;
				cBadUAB[iDD]=0;	
			}
		}
		else	{	
			if (abs_f(deltaUAE) > 0.2)	{	
				if (deltaUAE > 0)	{		oldAB[iDD].Fdata += 0.1;	}	
				else							{		oldAB[iDD].Fdata -= 0.1;	}	
				datch[iDD].Fdata = oldAB[iDD].Fdata;
			}
			oldAB[iDD].Fdata = datch[iDD].Fdata;
		}
	}
	
	//************* Расчёт интегральных параметров ****************************************
	nDD = nDatchD;		nDT = nDatchT;																					// Число рабочих ДД, ДТ
	sumDD = 0;	sumDT = 0;		nBitBadDatchD = 0;	nBitBadDatchT = 0;
	HardErr &= ~0x10;		HardErr &= ~0x20;
	
	//.....................................................................................
	for (i=0; i<nDatchD; i++)	{ 																							// По всем значениям датчиков
		if ((VdatchD[i].Fdata>=(PMinDatchD*(1-kdelta)))&&
				(VdatchD[i].Fdata<=(PMaxDatchD*(1+kdelta))))	{											// Высичление сумм значений ДД
				sumDD += VdatchD[i].Fdata;	}			
		else 	{																																	// Исключение недопустимых значений
			nDD--;																																// Число рабочих датчиков стало меньше
			nBitBadDatchD |= 1<<i;	iBadDatchD++;																	// Маска вышедших из строя датчиков Посчиали "плохой" датчик
			if (iBadDatchD > nMaxBadDatchD)	{	HardErr |= 0x20;	}									// Число вышедших из строя датчиков давления превысило лимит
		}
		if ((VdatchT[i].Fdata>=(TMinDatchT*(1-kdelta)))&&
				(VdatchT[i].Fdata<=(TMaxDatchT*(1+kdelta))))	{											// Высичление сумм значений ДТ
				sumDT += VdatchT[i].Fdata;	}			
		else 	{																																	// Исключение недопустимых значений
			nDT--;																																// Число рабочих датчиков стало меньше
			nBitBadDatchT |= 1<<i;	iBadDatchT++;																	// Маска вышедших из строя датчиков температуры
			if (iBadDatchT > nMaxBadDatchT)	{	HardErr |= 0x10;	}									// Число вышедших из строя датчиков температуры превысило лимит
		}
	}			

	//.....................................................................................
	if (nDD) 	{ pkiDD[0].Fdata=(sumDD/nDD);																		// Вычисление среднего значения ДД	
	 if (pkiDD[0].Fdata>10)	{																		// Вычисление среднего значения ДД	
		nDD=0;	sumDD=0;
		for (i=0; i < 5; i++)	{																									// Коррекция среднего значения ДД	
			if 	((!(nBitBadDatchD & 1<<i))&&
					(abs_f(VdatchD[i].Fdata-pkiDD[0].Fdata)) <= pkiDD[0].Fdata*proc)	{
				sumDD += VdatchD[i].Fdata;		nDD++;	
			}	
			else	{
				nBitBadDatchD |= 1<<i;	iBadDatchD++;																// Маска вышедших из строя датчиков
				if (iBadDatchD > nMaxBadDatchD)	{	HardErr |= 0x20;	}								// Число вышедших из строя датчиков давления превысило лимит
			}
		}	
		if ((nDD)&&(!(HardErr&0x20))) 	pkiDD[0].Fdata=(sumDD/nDD);	
		//else			//pkiDD[0].Fdata=0;
	 }
	}
	else				pkiDD[0].Fdata=0;

	fp = pkiDD[0].Fdata;
	
	//.....................................................................................
	if (nDT) 	{	pkiDT[0].Fdata=(sumDT/nDT);																		// Вычисление среднего значения ДТ
		nDT=0;	sumDT=0;
		for (i=0; i < 5; i++)	{																									// Коррекция среднего значения ДТ
			if 	((!(nBitBadDatchT & 1<<i))&&
					(abs_f(VdatchT[i].Fdata-pkiDT[0].Fdata)) <= pkiDT[0].Fdata*proc)	{
				sumDT += VdatchT[i].Fdata;		nDT++;	
			}	
			else	{
				nBitBadDatchT |= 1<<i;	iBadDatchT++;																// Маска вышедших из строя датчиков
				if (iBadDatchT > nMaxBadDatchT)	{	HardErr |= 0x10;	}								// Число вышедших из строя датчиков температуры превысило лимит
			}
		}	
		if (nDT) 	pkiDT[0].Fdata=(sumDT/nDT);	
		else			pkiDT[0].Fdata=0;
	}
	else 				pkiDT[0].Fdata=0;

	//.....................................................................................
	pkiDD[1].Fdata=0;		maxPDD.Fdata=0;	 minPDD.Fdata=PMaxDatchD;
	pkiDT[1].Fdata=0;		maxCDT.Fdata=0;	 minCDT.Fdata=TMaxDatchT;		

	for (i=0; i<nDatchD; i++)	{																								// Поиск наибольшего отклонения
		datCn[i].Fdata 		= VdatchD[i].Fdata;																		// Значение давления
		datCn[i+5].Fdata 	= VdatchT[i].Fdata;																		// Значение температуры
		datCn[i+10].Fdata = UdatchD[i].Fdata;																		// Значение напряжения на датч давления
		datCn[i+15].Fdata = UdatchT[i].Fdata;																		// Значение напряжения на датч температуры
		datCn[i+20].Fdata = datch[i].Fdata;																			// Значения U_АБ1, U_АБ2, U_опДД, U_опДТ, Общ_ДД,	Общ_ДТ
		
		if ((!(nBitBadDatchT & 1<<i))&&(pkiDT[0].Fdata))	{																						// Фильтр допустимых значений
			//biasDT.Fdata = abs_f(pkiDT[0].Fdata-VdatchT[i].Fdata);
			if (maxCDT.Fdata < VdatchT[i].Fdata) {	maxCDT.Fdata = VdatchT[i].Fdata;	imaxCDT=i+1;	}		// Наибольшее значение
			if (minCDT.Fdata > VdatchT[i].Fdata) {	minCDT.Fdata = VdatchT[i].Fdata;	iminCDT=i+1;	}		// Наименьшее значение
			//if (biasDT.Fdata > pkiDT[1].Fdata)	 {	pkiDT[1].Fdata = biasDT.Fdata;		ibiasDT=i+1;	}		// Наибольшее отклонение
		}
		else	{
			maxCDT.Fdata=0;	imaxCDT=0;	minCDT.Fdata=0;	iminCDT=0;																			//pkiDT[1].Fdata=0;	ibiasDT=0;biasDT.Fdata=0;	
		}	
		if ((!(nBitBadDatchD & 1<<i))&&(pkiDD[0].Fdata))	{																						// Фильтр допустимых значений  
			//biasDD.Fdata = abs_f(pkiDD[0].Fdata-VdatchD[i].Fdata);
			if (maxPDD.Fdata < VdatchD[i].Fdata) {	maxPDD.Fdata = VdatchD[i].Fdata;	imaxPDD=i+1;	}		// Наибольшее значение
			if (minPDD.Fdata > VdatchD[i].Fdata) {	minPDD.Fdata = VdatchD[i].Fdata;	iminPDD=i+1;	}		// Наименьшее значение
			//if (biasDD.Fdata > pkiDD[1].Fdata)	 {	pkiDD[1].Fdata = biasDD.Fdata;		ibiasDD=i+1;	}		// Наибольшее отклонение
		}	
		else	{
			maxPDD.Fdata=0;	imaxPDD=0;	minPDD.Fdata=0;	iminPDD=0;																			//pkiDD[1].Fdata=0;	ibiasDD=0;biasDD.Fdata=0;	
		}	
	}	
	pkiDT[1].Fdata = maxCDT.Fdata-minCDT.Fdata;
	pkiDD[1].Fdata = maxPDD.Fdata-minPDD.Fdata;																											// Разница между max min DD
	datCn[ndatCn-1].Fdata = datch[ndatch-1].Fdata;																									// Значение Общ_ДТ
	//	fdatCn[ndatCn-1] = datch[ndatch-1].Fdata;																											// Значение Общ_ДТ для отладки
	bDataDatchOk=1;																																									// Данные по датчикам готовы
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
int NumOk (int ind)	{
	int k=0;
	while ((k<nDatchD)&&(NumIzmAE[k]!=ind))	{	k++;	};
	if (k==nDatchD)	return 0;
	else						return 1;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Обработка (нормализация) массива напряжений аккумуляторов
void RaschotArrayAE (void)
{	float AEsr, sumAE, dAE;																													// Сумма для вычисления среднего значения
	float sumIzmAE=0, nsumIzmAE=0;
	int ind, iBad, bBadAE, cind;

	HardErr &= ~0x40;																																			// 6-ой бит - число вышедших из строя АЭ превысило лимит
		
	sumAE = 0;	ind = 0; cind = 1;																												//iBadAE = 0;		
	nAE = nAllAE;																																					// Число рабочих аккумуляторных элементов

	//....................................................................................................	
	// Вычисление реальных значений напряжений на АЭ и АБ
	for (i=0; i<nAllAE; i++)	{

		if (Takkum[i]>0)	{		
		 
			if (bVklZRU)	dAE = dAE_w[nMUK_BE-1][i];			else	dAE = dAE_0[nMUK_BE-1][i];
			
			if (i%2) 		{
				//AccumVoltage[i] = (cUsm[nMUK_BE-1][ind] - Takkum[i]) * kAE_[nMUK_BE-1][i] - dAE_[nMUK_BE-1][i];
				AccumVoltage[i] = ((cUsm[nMUK_BE-1][ind]) - Takkum[i]) * kAE_[nMUK_BE-1][i] - dAE + kTempAE[nMUK_BE-1] * (TempCod-kodADC_t25[nMUK_BE-1]);
				AccumVoltage[i] = -AccumVoltage[i];
			}
			else		{
				AccumVoltage[i] = ((cUsm[nMUK_BE-1][ind]) - Takkum[i]) * kAE_[nMUK_BE-1][i] + dAE + kTempAE[nMUK_BE-1] * (TempCod-kodADC_t25[nMUK_BE-1]);
				//AccumVoltage[i] = (cUsm[nMUK_BE-1][ind] - Takkum[i]) * kAE_[nMUK_BE-1][i] + dAE_[nMUK_BE-1][i];
			}
			//....................................................................................
			akkCn[i].Fdata = AccumVoltage[i];

			//............. коррекция значений напряжения АЭ .....................................
			deltaUAE = akkCn[i].Fdata - akkCnOld[i].Fdata;
			if (abs_f(deltaUAE) >= 0.1)	{
				cBadAE[i]++;
				if (cBadAE[i] <= nCorrectAE)	{	akkCn[i].Fdata 	= akkCnOld[i].Fdata;		}
				else													{	
					akkCnOld[i].Fdata = akkCn[i].Fdata;		akkCnCopy[i].Fdata = akkCn[i].Fdata;	cBadAE[i]=0;
				}
			}
			else	{	
				if (abs_f(deltaUAE) > 0.01)	{	
					if (deltaUAE > 0)	{		akkCnOld[i].Fdata += 0.01;	}	
					else							{		akkCnOld[i].Fdata -= 0.01;	}	
					akkCn[i].Fdata = akkCnOld[i].Fdata;
				}
				akkCnOld[i].Fdata = akkCn[i].Fdata;	cBadAE[i]=0;
			}
		}
		else	{
			if (cBadAE[i]<10)	{akkCn[i].Fdata = akkCnOld[i].Fdata;	cBadAE[i]++;}
			else							 akkCn[i].Fdata = 0;
		}
				
		//....................................................................................
		bBadAE=0;
		for (iBad=0; iBad < nMaxBadAE; iBad++)	{
			if ((i+1) == nBadAE_BCU[iBad])	{	bBadAE=1;		nAE--;	}														// Номера отказавших АЭ, полученные по CAN от БЦУ
		}
		//  Высичление суммарного напряжения аккумуляторов (АБ)
//		if ((akkCn[i].Fdata >= minUAE) && (akkCn[i].Fdata <= maxUAE) && (!bBadAE))	{
			if (NumOk(i)) {	sumIzmAE += akkCn[i].Fdata;		nsumIzmAE++;	}											// Сумма напряжений на измерительных АЭ
			sumAE += akkCn[i].Fdata;																													// Высичление суммарного напряжения аккумуляторов
//		}	
//		else {																																							// Исключение недопустимых значений (<700мВ)
//			nAE--;																																						// Число рабочих АЭ стало меньше на 1
//			if (iBadAE < nMaxBadAE)	{	nBadAE[iBadAE]=i+1;		iBadAE++;		}											// Запомнили номер "плохого" АЭ
//			else										{	HardErr |= 0x40;	}																			// Число вышедших из строя АЭ превысило лимит
//		}
		//....................................................................................
		//union uBytesFloat16 aIrazr, aIzar;																									// Получены по CAN из ЗРУ для коррекции U AKi
	//Коррекция напряжения 72-го АК АБ
//	if (stat1[iMUK_ZRU]	& bRazryad) {	fV_AB[76].Fdata += aI_razr*0.002;	}
//	if (stat1[iMUK_ZRU]	& bZaryad)  {	fV_AB[76].Fdata -= aI_zar *0.002;	}
//	if (stat1[iMUK_ZRU] & bPC)		  {	fV_AB[76].Fdata += 0.1;	}
/*
		if (i == 71)  {
			if (aIzar.Fdata)  akkCn[i].Fdata -= aIzar.Fdata*0.002;
			if (aIrazr.Fdata) akkCn[i].Fdata += aIrazr.Fdata*0.002;
			if (bStatPC) 			akkCn[i].Fdata += 0.1;
		}
*/		
		cind++;
		if (cind==2)	{	ind = !ind;	 cind = 0;	}
		
	}	//end of for (i=0; i<nAllAE; i++)
	
	//....................................................................................................	
	// Вычисление интегральных параметров
	if (nAE>0)		{
		if (vtstTime >= ctstTime)	{
			if ((sumAE-Old_sumAE+deltaUAB) < 0)	status_AB = 0;																// Статус АБ: 0 - разряд, 1 - заряд
			else 																status_AB = 1;
			Old_sumAE = sumAE;
			vtstTime = 0;
		}
		
		pki[2].Fdata = sumAE;																																// Напряжение АБ
		pki[0].Fdata = (sumAE/nAE);																													// Вычисление среднего значения напряжения на всех АЭ
		AEsr   = ((sumAE-sumIzmAE)/(nAE-nsumIzmAE));																				// Вычисление среднего значения напряжения на АЭ без измерит АЭ
		dU_AE_IsmAE.Fdata = abs_f(AEsr-sumIzmAE/nsumIzmAE);																	// Разница напряжений между измерительными и неизмерительными АЭ НВА

		maxU.Fdata=0;	 minU.Fdata=10;	pki[1].Fdata=0;
		for (i=0; i<nAllAE; i++)	{												
//			if ((akkCn[i].Fdata >= minUAE) && (akkCn[i].Fdata <= maxUAE))	{										// Пропуск недопустимых значений
			bBadAE=0;
			for (iBad=0; iBad < nMaxBadAE; iBad++)	{
				if ((i+1) == nBadAE_BCU[iBad])	{	bBadAE=1;		}																	// Номера отказавших АЭ, полученные по CAN от БЦУ
			}
			if (!bBadAE)	{
				bias.Fdata=abs_f(pki[0].Fdata-akkCn[i].Fdata);
				if (maxU.Fdata < akkCn[i].Fdata) {	maxU.Fdata = akkCn[i].Fdata;	imaxU=i+1;	}	// Поиск наибольшего значения
				if (minU.Fdata > akkCn[i].Fdata) {	minU.Fdata = akkCn[i].Fdata;	iminU=i+1;	}	// Поиск наименьшего значения
				if (bias.Fdata > pki[1].Fdata)	 {	pki[1].Fdata = bias.Fdata;		ibias=i+1;	}	// Поиск наибольшего отклонения АЭ и его номера
				akkCnCopy[i].Fdata = akkCn[i].Fdata;
			}	
//			}
//			else		{	
//				if (abs_f(akkCn[i].Fdata) > maxUAE)	{	akkCn[i].Fdata = akkCnCopy[i].Fdata;	}	
//			}
		}		
	}											
	else	{
		pki[2].Fdata=0;	pki[0].Fdata=0;	dU_AE_IsmAE.Fdata=0;	maxU.Fdata=0;	 minU.Fdata=0;	pki[1].Fdata=0;	ibias=0;	imaxU=0;	iminU=0;
	}
	
	if (nMUK_BE	== nMUK3_BE)	HardErr &= ~0x40;	
	bDataABOk=1;																																					// Данные по АБ готовы
	HardErr &= ~0x80;																																			// Флаг обновления данных телеметрии (=0 обновлены)
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Пуск процесса чтения напряжений датчиков
void ReadDatch_GO (void)
{	Var_init();	
	add_chanl = 2;
	StatRdADC = ReadDatch;																										// Чтение значений напряжений начинаем с датчиков
	iDT=0;	iDD=0;
	Set_Adr_Datch();																													// Выбор (коммутация) первой пары датчиков для измерения напряжений
	HardErr |= 0x80;																													// Флаг обновления данных телеметрии
	ADC_GO();																																	// Запуск преобразования cо 2-го канала
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Пуск процесса чтения напряжений АЭ
void ReadAE_GO (void)
{	Var_init();
	iadc=1;
	add_chanl = 4;																														// Добавка для вычисления канала измерения АЦП
	StatRdADC = ReadAE;																												// Чтение значений напряжений начинаем с датчиков
	nTwoAEs = 1;																															// Стартовый номер пары АЭ
	Set_Adr_AE_Step(KodDeshifr[nTwoAEs-1]);																		// Выбор (коммутация) первой пары датчиков для измерения напряжений
	ADC_GO();																																	// Запуск преобразования c 4-го канала
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Начальный запуск или переход в нормальный режим
void Start (void)
{
	p_CANdata = dataCAN;
	
//	bOkCAN = 0;																																// Сброс флага CAN-сеть исправна
	bMagorOk = 0;																															// Сброс флага мажоритар собрался

	RRazrOff();
	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);
	NVIC_EnableIRQ(CAN2_IRQn);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Подготовка массива телеметрии по датчикам (Для отладки)
//void InitTEL_Dt(void)
//{	int j=0;
//	for (i=0; i < 2; i++)		pkiDD[i].Fdata = i+1;		i++;											// Контрольные значения ДД 3 по 2 байта
//	ibiasDD = i++;																														// Номер с макс отклонением 3
//	maxPDD.Fdata = i++;		imaxPDD = i++;
//	minPDD.Fdata = i++;		iminPDD = i++;																			// 7
//	
//	for (j=0; j < 2; j++)		pkiDT[j].Fdata = i++;															// Контрольные значения ДД 3 по 2 байта
//	ibiasDT = i++;																														// Номер с макс отклонением 10
//	maxCDT.Fdata = i++;		imaxCDT = i++;
//	minCDT.Fdata = i++;		iminCDT = i++;																			// 14
//	
//	for (j=0; j < ndatCn; j++)	datCn[j].Fdata = i++;													// Массив 25 значений ДД и ДТ, напряжений ДД и ДТ, опорных и общих напряжений
////	bStatPC = i;																														// Состояние разрядных резисторов
//	bStatPC = bStatPC;																												// Состояние разрядных резисторов
//}
		
//-------------------------------------------------------------------------------------------------------------------------------------------------
// Отправка массива по CAN
void TransmCAN_Dt(void)
{	int j=0;
	//InitTEL_Dt();																															// Заполнение тестовыми значениями 
	// Подготовка массива телеметрии по датчикам
		// 1 .................................................................
//	pkiDD[0].Fdata=1.5;
	for (i=0; i < 2; i++)		{																									// Контрольные значения ДД 2 по 2 байта
		dataCAN[j] = pkiDD[i].b[0];	j++;	dataCAN[j] = pkiDD[i].b[1]; j++;	}
	dataCAN[j] = ibiasDD;			j++;																						// Номер с макс отклонением	
	//dataCAN[j] = HardErr;			j++;																						// Номер с макс отклонением	HardErr
	dataCAN[j] = maxPDD.b[0];	j++;	dataCAN[j] = maxPDD.b[1];	j++;
	dataCAN[j] = imaxPDD;			j++;																						// 8 - 1 
	//dataCAN[j] = iReadAdc>>8;			j++;																						// 8 - 1 

		// 2 .................................................................
	dataCAN[j] = minPDD.b[0];	j++;	dataCAN[j] = minPDD.b[1];	j++;	
	//dataCAN[j] = iReadAdc;			j++;																						// 11 байт iReadAdc
	dataCAN[j] = iminPDD;			j++;																						// 11 байт iReadAdc
	for (i=0; i < 2; i++)	{																										// Контрольные значения ДТ
		dataCAN[j] = pkiDT[i].b[0];	j++;	dataCAN[j] = pkiDT[i].b[1];	j++;	}
	dataCAN[j] = ibiasDT;			j++;																						// 16 - 2

		// 3 .................................................................
	dataCAN[j] = maxCDT.b[0];	j++;	dataCAN[j] = maxCDT.b[1];	j++;
	dataCAN[j] = imaxCDT;			j++;
	dataCAN[j] = minCDT.b[0];	j++;	dataCAN[j] = minCDT.b[1];	j++;
	dataCAN[j] = iminCDT;			j++;																						// 22

	for (i=0; i < ndatCn; i++)		{																						// Массив[26] значений ДД и ДТ, напряжений ДД и ДТ, опорных и общих напряжений
		// 4 .................................................................
		dataCAN[j] = datCn[i].b[0];	j++;																				// 
		dataCAN[j] = datCn[i].b[1]; j++;	 }																		// 
			
	dataCAN[j] = bStatPC;		j++;																							// Состояние разрядных резисторов	str
	//tMUK.Fdata = str;																											// Состояние разрядных резисторов	str
	dataCAN[j] = tMUK.b[0];		j++;		dataCAN[j] = tMUK.b[1];		j++;					// Температура МУК
//	dataCAN[j] = 	 TempCod;		j++;		dataCAN[j] = TempCod>>8;	j++;					// Код текущего значения температуры МУК
			
	cntst=j;	// ДЛЯ ОТЛАДКИ																									// 48
	LendataCAN = j;																														// Число байт данных SIZE_PI_Dt or SIZE_PKI_Dt
	byteCMD = CAN_PI_Datch;																										// Код пакета
	CAN1_MakeMSG(LendataCAN, p_CANdata);
	CAN2_MakeMSG(LendataCAN, p_CANdata);
	bDataDatchOk=0;																														// Данные по датчикам отправлены
}	

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Подготовка массива телеметрии по АБ (Для отладки)
//void InitTEL_AB(void)
//{	int j=0;
//		// Тест-заполнение массива значений АЭi тестовыми значениями.
////		if (!(i%7))	{	akkCn[i].Fdata = 1.6;}
////		else				{ akkCn[i].Fdata = 1.5;}
////		if (!(i%3))		akkCn[i].Fdata = 1.4;																		// Тест-заполнение массива значений АЭi тестовые значения.

//	i=++cntst;
//	for (j=0; j < npki; j++)		pki[j].Fdata = i;		i++;											// Контрольные значения
//	ibias = i++;																															// Номер с макс отклонением
//	imaxU = i++;
//	maxU.Fdata = i++;		minU.Fdata = i++;
//	dU_AE_IsmAE.Fdata = i++;	
//	
//	for (j=0; j < nAllAE; j++)	akkCn[j].Fdata = i++;													// Контрольные значения ДД 3 по 2 байта
//	iminU = i;																																// Состояние разрядных резисторов
//}
		
//-------------------------------------------------------------------------------------------------------------------------------------------------
// Отправка массива по CAN телеметрии АБ
void TransmCAN_AB(void)							
{	int j=0;
	//InitTEL_AB();
	// pki[0]-сред знач, pki[1]-макс отклон, pki[2]- напряжение АБ
	for (i=0; i < npki-1; i++)	{																							// Послать контрольные значения
		dataCAN[j] = pki[i].b[0];		j++;		
		dataCAN[j] = pki[i].b[1];		j++;	}																			// 4
	dataCAN[j] = ibias;			j++;																							// номер АЭ с максимальным отклонением

	dataCAN[j] = imaxU;			j++;																							// номер АЭ с максим. значением напряжения
	dataCAN[j] = maxU.b[0];	j++;		dataCAN[j] = maxU.b[1];	j++;
	dataCAN[j] = minU.b[0];	j++;		dataCAN[j] = minU.b[1];	j++;
	dataCAN[j] = dU_AE_IsmAE.b[0];j++;	dataCAN[j] = dU_AE_IsmAE.b[1];j++;		// Разница напряжений между измерительными и неизмерительными АЭ НВА
																																						// 12 байт
	// Подготовка информации по АЭ
	for (i=0; i < nAllAE; i++)	{																							// Массив значений напряжений АЭ, 72
		dataCAN[j] = akkCn[i].b[0];	j++;
		dataCAN[j] = akkCn[i].b[1];	j++;																				// 
	}																																					// 144+14=158
	dataCAN[j] = iminU;			j++;

	LendataCAN = j;																														// Число байт данных SIZE_PI_AB or SIZE_PKI_AB
	byteCMD = CAN_PI_AK;																											// Код пакета
  CAN1_MakeMSG(LendataCAN, p_CANdata);
  CAN2_MakeMSG(LendataCAN, p_CANdata);
	bDataABOk=0;																															// Данные по АБ отправлены
}				

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Отправка по CAN паспортных данных датчиков
void TransmCAN_PasportDach(void)							
{	int j=0;
	union uBytesFloat16 tmp;

// ------------------------------- Датчики давления:
	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = ItarD[i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = ItarD[i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = a0[0][i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = a1[0][i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = a2[0][i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

// ------------------------------- Датчики температуры:
	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = ItarT[i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = ItarT[i];			//IpasT[i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = R0[i];							
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	for (i=0; i < nDatchD; i++)	{		tmp.Fdata = alpha[i];						
		dataCAN[j] = tmp.b[0];		j++;	dataCAN[j] = tmp.b[1];		j++;	}	

	LendataCAN = j;																														// Число байт данных
	byteCMD = CAN_Pasport_Put;																								// Код пакета
  CAN1_MakeMSG(LendataCAN, p_CANdata);
  CAN2_MakeMSG(LendataCAN, p_CANdata);
}				

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Результат выполнения команды
void TransmCAN_Msg(int cmd, int result, int U_AE)
{	int j=0;

	dataCAN[j] = result;		j++;
	dataCAN[j] = cmd;				j++;
	
	if ((U_AE)&&(result==1))	{																								// Подготовка пакета информации по U АЭ(i) и АЭ(i+1)
		for (i=0; i < 2; i++)	{	
			dataCAN[j] = akkCn[i].b[0];	j++;	dataCAN[j] = akkCn[i].b[1];	j++;	}	
	}
	LendataCAN = j;																														// Число байт данных
	byteCMD = CAN_ResCmd;

	CAN1_MakeMSG(LendataCAN, p_CANdata); 
  CAN2_MakeMSG(LendataCAN, p_CANdata); 
//	bDataABOk=0;																															// Данные отправлены
	for (i=0; i<3; i++)	{ nMUK_ZRU[i]=0; codMUK_ZRU[i]=0;}										// Сброс признака активности узла-передатчика и каманды
}				

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Подготовка тестовой телеметрии ошибок (Для отладки)
//void InitTEL_Err(void)
//{	
//	HardErr 			= 0x5;																											// Ошибки аппаратуры
//	nBitBadDatchD = 0xA;																											// Байт маски вышедших из строя датчиков Д
//	nBitBadDatchT = 0x15;																											// Байт маски вышедших из строя датчиков Т
//	nBadAE[0]=0xf;	nBadAE[1]=0x40;																						// Номера отказавших АЭ
//	iBadAE=2;		
//}
		
//-------------------------------------------------------------------------------------------------------------------------------------------------
// Состояние ошибок БЭ и байта ошибок
void TransmCAN_Err(void)
{	int j;
	//	HardErr - байт ошибок по битам:
	//	0 -	Опорное напряжение ПСД вне допустимого значения.
	//	1 -	Общий ПСД вне допустимого значения (>0.1)
	//	2 -	Опорное напряжение МИБ вне допуска, Uop < 2*0.95
	//	3 -	В МИБ напряжение смещения вне допуска (>0.1)
	//	4 - Число вышедших из строя датчиков температуры превысило лимит
	//	5 - Число вышедших из строя датчиков давления превысило лимит
	//	6 - Число вышедших из строя АЭ превысило лимит
	//	7 - Нет обновления данных телеметрии (АЦП не работает)
//	InitTEL_Err();
	j=0;
	// Подготовка пакета информации 
	dataCAN[j] = HardErr;					j++;																				// Ошибки аппаратуры
	dataCAN[j] = nBitBadDatchD;		j++;																				// Младший байт маски вышедших из строя датчиков
	dataCAN[j] = nBitBadDatchT;		j++;																				// Старший байт маски вышедших из строя датчиков

	for (i=0; i < nMaxBadAE; i++)	{																						// Номера отказавших АЭ. nMaxBadAE = 3
		dataCAN[j] = nBadAE_BCU[i];			j++;	}

	byteCMD = CAN_StatErr;
	LendataCAN = j;																														// Число байт данных
  CAN1_MakeMSG(LendataCAN, dataCAN);
  CAN2_MakeMSG(LendataCAN, dataCAN);
//  CAN1_MakeMSG(LendataCAN, p_CANdata);
//  CAN2_MakeMSG(LendataCAN, p_CANdata);
}				

//-------------------------------------------------------------------------------------------------------------------------------------------------
// ************ Получение напряжений АЭ(i) и АЭ(i+1) *************
void PutParamADC_AE (void)																									// С 20.01.2020
{	
		switch (nTwoAEs)	{																											// Измерение АЭ, каналы 4,5	iadc=1,0
		case 37:	akkCn[iadc].Fdata = kUop[iadc]*Uadc;													// Uop[iadc] = kUop[iadc]*Uadc;
							//Set_Adr_AE_Step(KodDeshifr[0]);
							break;
		case 38:	akkCn[iadc].Fdata = Uadc;																			// Usm[iadc] = Uadc;
							TUsm[iadc] = Uadc;																					//
							Set_Adr_AE_Step(KodDeshifr[0]);
							break;
		default:			
				if (nTwoAEs%2)	{		Takkum[((nTwoAEs-1)*2)+(!iadc)] = Uadc;		}
				else						{		Takkum[((nTwoAEs-1)*2)+( iadc)] = Uadc;		}
		}	

		if (iadc)		{	iadc=0;																										// Переход к другому каналу измерения
			OkDataADC=0;	summa=0;	NumReadAdc=0;		iReadAdc=0;
			ADC_Start(iadc);																											// Выбор канала для чтения
			MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO & ADC1_REG_GO;										// Запуск преобразования
		}			
		else				{	
			if (nTwoAEs<37)	{
				akkCn[0].Fdata = (cUsm[nMUK_BE-1][0] - Takkum[((nTwoAEs-1)*2)  ]) * kAE_[nMUK_BE-1][((nTwoAEs-1)*2)  ] + dAE_w[nMUK_BE-1][((nTwoAEs-1)*2)  ] + kTempAE[nMUK_BE-1] * (TempCod-kodADC_t25[nMUK_BE-1]);
				akkCn[1].Fdata = (cUsm[nMUK_BE-1][1] - Takkum[((nTwoAEs-1)*2)+1]) * kAE_[nMUK_BE-1][((nTwoAEs-1)*2)+1] - dAE_w[nMUK_BE-1][((nTwoAEs-1)*2)+1] + kTempAE[nMUK_BE-1] * (TempCod-kodADC_t25[nMUK_BE-1]);
				akkCn[1].Fdata = -akkCn[1].Fdata;
			}
			new_mode=NORMAL;																											// Возврат в нормальный рабочий режим
			TransmCAN_Msg(CAN_U_AK,1,1);																					// Отправить значения U  AЭ(i) и AЭ(i+1)
			ReadDatch_GO();																												// Продолжить чтение напряжений датчиков
		}	
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
// ************ Чтение датчика температуры *************
void InitReadADC_DT (void)																											// Чтение датчика температуры
{	
//	MDR_ADC->ADC1_CFG = ADC1_CFG_REG_ADON |										// Включение АЦП
//											(3 << ADC1_CFG_REG_DIVCLK_Pos) |			// Выбор коэффициента деления частоты процессора 0011 – CPU_CLK = HCLK/8
//											ADC1_CFG_TS_EN | 											// Включения датчика температуры и источника опорного напряжения
//											ADC1_CFG_TS_BUF_EN |									//•	установить биты TS_BUF_EN и SEL_TS   ((uint32_t)0x00040000)
//											ADC1_CFG_SEL_TS |  										//                     ((uint32_t)0x00080000)
//											ADC1_CFG_REG_RNGC;										// Разрешение автоматического контроля уровней
																														//
	MDR_ADC->ADC1_CFG |= ADC1_CFG_TS_BUF_EN |									//•	установить биты TS_BUF_EN и SEL_TS   ((uint32_t)0x00040000)
											 ADC1_CFG_SEL_TS;
																														//
	MDR_ADC->ADC1_CHSEL	=0x80000000;													//•	в регистре ADC1_CHSEL выбрать только 31 канал
																														//•	установить бит Cfg_REG_CHCH в 1, либо
																														//•	установить номер 31-го канала в битах Cfg_REG_CHS[4:0]
																														//•	и сбросить бит Cfg_REG_CHCH в 0.
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DeInitReadADC_DT (void)																											// Отмена Чтения датчика температуры
{	
	MDR_ADC->ADC1_CFG &= ~(ADC1_CFG_TS_BUF_EN);								//•	сбросить биты TS_BUF_EN    ((uint32_t)0x00040000)
	MDR_ADC->ADC1_CFG &= ~(ADC1_CFG_SEL_TS);									//•	сбросить биты SEL_TS  		 ((uint32_t)0x00040000)
	MDR_ADC->ADC1_CHSEL	=0x0;																	//•	в регистре ADC1_CHSEL выбрать только 31 канал
	iadc=0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Пуск процесса чтения напряжений АЭ
void TReadAE_Step_GO (void)
{	Var_init();
	add_chanl = 4;		iadc=1;																									// Добавка для вычисления канала измерения АЦП

	if (nTwoAEs)	{
		Set_Adr_AE_Step(KodDeshifr[nTwoAEs-1]);
	}
	else	{	
		Set_Adr_AE_Step(0);																											// Отключить каналы измерения напряжений
	}	
	ADC_GO();																																	// Запуск преобразования c 4-го канала
	MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;																			// Запуск преобразования
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// ************ Основная процедура программы. Производит чтение коммутируемых каналов МИБ и ПСД *************
void PutParamADC (void)																											// С 27.11.2019
{	float TempTerm;
	
	switch (StatRdADC)		{																										// Переключатель режимов чтения АЦП
			
		//.............................................................................................................................
		case ReadAE:						// Чтение напряжений 1-2, 5-6, ..., 60-70АЭ, UИЗМ12, UИЗМ22, 4-3, 8-7, ..., 72-71АЭ, UИЗМ11, UИЗМ21
																																						// Измерение АЭ, iadc=4,5	
			switch (nTwoAEs)	{	
			case 37:	//Uop[iadc] = kUop[iadc]*Uadc;															//
								TUop[iadc] = kUop[iadc]*Uadc;																//
								break;																											// Uop
			case 38:	//cUsm[nMUK_BE-1][iadc] = Uadc;																					//cUsm[nMUK_BE-1][ind]
								TUsm[iadc] = Uadc;																					//
								break;																											// Usm;		
			default:	if (nTwoAEs%2)	{
									akkum[((nTwoAEs-1)*2)+(!iadc)].Fdata  = Uadc;							//
									Takkum[((nTwoAEs-1)*2)+(!iadc)] = Uadc;										//
									}
								else	{
									akkum[((nTwoAEs-1)*2)+(iadc)].Fdata  = Uadc;							//
									Takkum[((nTwoAEs-1)*2)+(iadc)] = Uadc;										//
								}
								if (nTwoAEs==36)	{
									nTwoAEs=nTwoAEs;	
								}															// Счётчик значений U аккумуляторов равных НУЛЮ
			}	

			if (!iadc)		{	
				if (nTwoAEs < MaxTwoAEs-1)		{	iadc=1;	
					nTwoAEs++;
					Set_Adr_AE_Step(KodDeshifr[nTwoAEs-1]);														// Установить след адрес дешифратора для 4-го канала АЦП
				}
				else	{
					new_mode = AK_AR_READY;		}																				// Нормализация результатов измерения (на обработку массива напряжений АЭ)
			}
			else				{	iadc=0;	}	
		
		break;
			
	//.............................................................................................................................
	case ReadDatch:						// Чтение  АЦП напряжений датчиков и преобразование по формуле U = Uизм * Iтар/Iизм* Когр/Коу

	 if (bReadDatch)	{

			if ((Uadc>0)&&(Uadc<3.2))	{		OldUadc[a] = Uadc; cBadUadc[a] = 0;		}
			else				{
				if (cBadUadc[a]<1)	{ Uadc = OldUadc[a];	cBadUadc[a]++; }													//else	Uadc = 0;
			}
			//........................................................................................................... 
			switch (a)
			{					// datch[6] Значения Общ_ДД, Общ_ДТ, U_опДД, U_опДТ, U_АБ1, U_АБ2 заполняемые в процессе опроса АЦП
			case 10:	datch[a-10].Fdata = Uadc*KuGndP[nMUK_BE-1];																	// Общий ДД
								//fdatch[a-10] 			= Uadc*KuGndP[nMUK_BE-1];
								break;		
			case 11:	datch[a-10].Fdata = Uadc*KuGndT[nMUK_BE-1];																	// Общий ДТ
								//fdatch[a-10] 			= Uadc*KuGndT[nMUK_BE-1];
								break;
			case 12:	datch[a-10].Fdata = Uadc*KuOpornP[nMUK_BE-1];																// Опорное напржение ДД
								//fdatch[a-10] 			= Uadc*KuOpornP[nMUK_BE-1];
								break;																											
			case 13:	datch[a-10].Fdata = Uadc*KuOpornT[nMUK_BE-1];																// Опорное напржение ДТ
								//fdatch[a-10] 			= Uadc*KuOpornT[nMUK_BE-1];
								break;
			case 14:	TempTerm = kTempAB[nMUK_BE-1]*(TempCod-kodADC_t25[nMUK_BE-1]);
								AB1 = (KuAB1[nMUK_BE-1]*Uadc) + DuAB1[nMUK_BE-1] - TempTerm; 								// Вычисление реального напряжения АБ1
								datch[a-10].Fdata = AB1;
								//fdatch[a-10] 			= Uadc;
								iadc = 1;
								break;																											
			case 15:	TempTerm = kTempAB[nMUK_BE-1]*(TempCod-kodADC_t25[nMUK_BE-1]);
								AB2 = (KuAB2[nMUK_BE-1]*Uadc) + DuAB2[nMUK_BE-1] - TempTerm; 								// Вычисление реального напряжения АБ2
								datch[a-10].Fdata = AB2;
								//fdatch[a-10] 			= Uadc;
//								iUnalad = 0;																																// Выбор опорного напряжения для вычисления ПСД
								TempTerm = datch[0].Fdata;																									// Значения Общ_ДД, Общ_ДТ, U_опДД, U_опДТ, U_АБ1,  U_АБ2		// Значения U_АБ1,  U_АБ2,  U_опДД, U_опДТ, Общ_ДД,	Общ_ДТ									надо									получили
								datch[0].Fdata = datch[4].Fdata;	datch[4].Fdata = TempTerm;								// перестановка для CAN протокола 
								TempTerm = datch[1].Fdata;
								datch[1].Fdata = datch[5].Fdata;	datch[5].Fdata = TempTerm;

								break;																											
			case 16:	TempCod = Result;																														// Код текущего значения температуры

//								TtMUK 		 = FACTORY_TEMP25 - (FACTORY_ADC_TEMP25-TempCod)/FACTORY_ADC_AVG_SLOPE + dTempC[nMUK_BE-1];
//								tMUK.Fdata = FACTORY_TEMP25 - (FACTORY_ADC_TEMP25-TempCod)/FACTORY_ADC_AVG_SLOPE + dTempC[nMUK_BE-1];

								TtMUK 		 = FACTORY_TEMP25 - (kodADC_t25[nMUK_BE-1]-TempCod)/FACTORY_ADC_AVG_SLOPE + dTempC[nMUK_BE-1];
								tMUK.Fdata = FACTORY_TEMP25 - (kodADC_t25[nMUK_BE-1]-TempCod)/FACTORY_ADC_AVG_SLOPE + dTempC[nMUK_BE-1];

								DeInitReadADC_DT();
								break;																																			// Нормализация результатов измерения (обработка массива ДД, ДТ)
			case 17:	iadc=0;	add_chanl = 2;																											// Чтение канала PD0 (Uref+) и коррекция AUcc в обработчике прер
								break;
								
			default:	// Пересчёт напряжения с ДД и ДТ в массив реальных значение

				if (a%2)	{																																					// DT - нечётные.  
					tUadcDT[iDT] = Uadc;		iDT++;																										// Значения, прочитанные АЦП в вольтах
				}						
				else			{																																					// DD - чётные. 
					tUadcDD[iDD] = Uadc;		iDD++;
				}	
			}	
			
			//.......................................................................................................................
			a++;
			if ((B3001001)&&(a==10))	{	a=18;	}
			//.......................................................................................................................
			switch (a)
			{	//			case 10:	break;		case 11:	break;	case 12:	break;	case 13:	break;
			case 14:	iadc=0;	add_chanl = 6;																			// Чтение канала PD6 (AB1)
								break;																											
			case 15:	iadc=1;	add_chanl = 6;																			// Чтение канала PD7 (AB2)
								break;																											
			case 16:	iadc=0;	add_chanl = 31;	InitReadADC_DT();										// Чтение значение датчика температуры
								break;																											// Нормализация результатов измерения (обработка массива ДД, ДТ)
			case 17:	iadc=0;	add_chanl = 0;																			// Чтение канала PD0 (Uref+)
								break;																											
			case 18:	new_mode=AK_AR_READY;																				// Закончить цикл чтения напряжений на датчиках
								str=7;	a=0;																								// Начальные значения для формирования адреса коммутатации выводов ДД и ДТ 
								break;																											
			default:																															// Пересчёт напряжения с ДД и ДТ в массив реальных значение
					if (iadc)		{	iadc=0;		Set_Adr_Datch();		}											// Иниц для след цикла измерения АЦП. Установить след адрес дешифратора
					else				{	iadc++;	}																						// 
			}

			bReadDatch=0;
			break;
		}	
		//.............................................................................................................................
		default: StatRdADC=ReadDatch;		iDT=0;	iDD=0;													// Переход в режим чтения датчиков
	 }
 
		bReadDatch=1;		OkDataADC=0;		summa=0;	NumReadAdc=0;		iReadAdc=0;
	 
		if (new_mode!=AK_AR_READY)	{
			ADC_Start(iadc);																											// Выбор канала для чтения
			MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO & ADC1_REG_GO;										// Запуск преобразования
		}
		else {
			a=a;	
		}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void ControlP (void)									// Контроль температуры, давления, напряжения АБ от 14.02.20
{//	float fp, ft;
	// З А П Р Е Т   З А Р Я Д А .............................................................
//	if ((pkiDT[0].Fdata > 35)&&					// Среднее значение температуры на ДТ > 35град С
//			(pkiDD[0].Fdata > 45))		{			// Среднее значение давления 		на ДД > 45 кг/см2
//			MDR_PORTE->RXTX |=  0x4;	}			// PE2=1
//	else
//			MDR_PORTE->RXTX &= ~0x4;				// PE2=0
//pkiDD[0].Fdata = 50;
//	AB1 = 80;
//fp = pkiDD[0].Fdata;
ft = pkiDT[0].Fdata;
	
	if (fp < 42)		{											// Среднее значение давления 		на ДД < 42 кг/см2
		if (ft > 32)		{										// Среднее значение температуры на ДТ > 32 град С
			if (fp > 34)		{									// Среднее значение давления 		на ДД > 34 кг/см2
						MDR_PORTE->RXTX |=  0x4;	}	// PE2=1
			else	MDR_PORTE->RXTX &=  0xfffb;	// PE2=0	~0x4
		}		
		else	MDR_PORTE->RXTX &= 0xfffb;		// PE2=0
	}		
	else	MDR_PORTE->RXTX |= 0x4;					// PE2=1

//	if (pkiDD[0].Fdata < 42)		{					// Среднее значение давления 		на ДД < 42 кг/см2
//		if (pkiDT[0].Fdata > 32)		{				// Среднее значение температуры на ДТ > 32 град С
//			if (pkiDD[0].Fdata > 34)		{			// Среднее значение давления 		на ДД > 34 кг/см2
//						MDR_PORTE->RXTX |=  0x4;	}	// PE2=1
//			else	MDR_PORTE->RXTX &=  0xfffb;	// PE2=0	~0x4
//		}		
//		else	MDR_PORTE->RXTX &= 0xfffb;		// PE2=0
//	}		
//	else	MDR_PORTE->RXTX |= 0x4;					// PE2=1

	// З А П Р Е Т   Р А З Р Я Д А ...........................................................
	if ((AB1 < 72)||((minU.Fdata < 0.2)&&(nMUK_BE != nMUK3_BE)))	{													// Напряжение АБ < 72в AB1
	//if (datch[0].Fdata < 72)	{							// Напряжение АБ < 72в AB1
			MDR_PORTE->RXTX |= 0x8;	}						// PE3=1
	else
		if (AB1 > 88)													// 
			MDR_PORTE->RXTX &= 0xfff7;					// PE3=0
}	

//-------------------------------------------------------------------------------------------------------------------------------------------------
void ControlP_imp (void)								// Контроль температуры, давления, напряжения АБ от 14.02.20
{
	unsigned char  bZaprZar, b1ZaprZar=1, bZaprRazr, b1ZaprRazr=1;
	
	// З А П Р Е Т   З А Р Я Д А .............................................................
	if (pkiDD[0].Fdata < 42)		{					// Среднее значение давления 		на ДД < 42 кг/см2
		if (pkiDT[0].Fdata > 32)		{				// Среднее значение температуры на ДТ > 32 град С
			if (pkiDD[0].Fdata > 34)		{			// Среднее значение давления 		на ДД > 34 кг/см2
						bZaprZar =  1;	}						// PE2=1
			else	bZaprZar =  0;							// PE2=0
		}		
		else	bZaprZar =  0;								// PE2=0
	}		
	else	bZaprZar =  1;									// PE2=1

	// З А П Р Е Т   Р А З Р Я Д А ...........................................................
	if (AB1 < 72)	bZaprRazr = 1;					// PE3=1		Напряжение АБ < 72в AB1
	else					bZaprRazr = 0;					// PE3=0


	// В обработчике системного таймера ========================================================================================
	// З А П Р Е Т   З А Р Я Д А .............................................................
	if (bZaprZar)	{																									// PE2=1
		if (b1ZaprZar)			MDR_PORTE->RXTX |=  0x4;
		else								MDR_PORTE->RXTX &= ~0x4;
		b1ZaprZar = !b1ZaprZar;
	}
	else	{																													// PE2=0
		if (b1ZaprZar>4)	{	MDR_PORTE->RXTX |=  0x4;	b1ZaprZar=0;}
		else							{	MDR_PORTE->RXTX &= ~0x4;	b1ZaprZar++;}
	}	

	// З А П Р Е Т   Р А З Р Я Д А ...........................................................
	if (bZaprRazr)	{																								// PE3=1										// Напряжение АБ < 72в AB1
		if (b1ZaprRazr)			MDR_PORTE->RXTX |=  0x8;
		else								MDR_PORTE->RXTX &= ~0x8;
		b1ZaprRazr = !b1ZaprRazr;
	}
	else	{																													// PE3=0
		if (b1ZaprRazr>4)	{	MDR_PORTE->RXTX |=  0x8;	b1ZaprRazr=0;}
		else							{	MDR_PORTE->RXTX &= ~0x8;	b1ZaprRazr++;}
	}	
}	

//-------------------------------------------------------------------------------------------------------------------------------------------------
//void ControlP (void)												До 14.02.20																// Контроль давления, minDD	= 10;	maxDD	= 50;	
//{
//	float lockTMaxDatchT;
////#define lockPMaxDatchD	50				// Максимальное давление для ДД (для автономного управление)
////#define lockPMinDatchD	10				// Минимальное давление для ДД (для автономного управление)
////#define lockMaxTzaryd		30				// Максимальная темпаратура при ЗАРЯДЕ (для автономного управления)
////#define lockMaxTrazryd	45				// Максимальная темпаратура при РАЗРЯДЕ (для автономного управления)

//	if (status_AB)	lockTMaxDatchT = lockMaxTzaryd;					// Статус АБ: 0 - разряд, 1 - заряд
//	else						lockTMaxDatchT = lockMaxTrazryd;
//	
//	if (pkiDT[0].Fdata < lockTMaxDatchT)	{	// Среднее значение темпаратуры < 30
//		
//		if (pkiDD[0].Fdata > lockPMaxDatchD)	{
//				MDR_PORTE->RXTX &= ~0x8;
//				MDR_PORTE->RXTX |=  0x4;	}			// Среднее значение давления на ДД > Pв,  PE2=1
//		else	MDR_PORTE->RXTX &= ~0x4;			// Среднее значение давления на ДД <= Pв, PE2=0

//		if (pkiDD[0].Fdata < lockPMinDatchD)	{	
//				MDR_PORTE->RXTX &= ~0x4;
//				MDR_PORTE->RXTX |=  0x8;	}			// Среднее значение давления на ДД < Pн,  PE3=1
//		else	MDR_PORTE->RXTX &= ~0x8;			// Среднее значение давления на ДД >= Pв, PE3=0
//	}	
//	
//	else	{																// Среднее значение темпаратуры >= 30
//		MDR_PORTE->RXTX |= 0xC;							// PE2=1	Запрет заряда
//		//MDR_PORTE->RXTX |= 0x8;							// PE3=1	Запрет разряда
//	}
//}	


//-------------------------------------------------------------------------------------------------------------------------------------------------
// Разбор принятого пакета по CAN,	uint32_t	RecievCanID[32];	RecievCanDLC[32];		RecievCanDATAH_RecievCanDATAL[32];
//void DecoderPackCAN (void)
//{	unsigned char  cod, adrTx;	//, adrRx, msg;
//	
////	MDR_ADC->ADC1_STATUS &= (!ADC_STATUS_ECOIF_IE);													// Запрет прерываний от АЦП
//	NVIC_DisableIRQ(CAN1_IRQn);	NVIC_DisableIRQ(CAN2_IRQn);
//	
//		//msg		= 0x0000001f & RecievCanID;																			// номер сообщения, если данных в пакете больше 8 байт
//		cod		= 0x0000000f & RecievCanID >> 12;																	// Код пакета
//		//adrRx = 0x0000000f & RecievCanID >> 16;																// Адрес узла-приёмника пакета
//		adrTx = 0x0000000f & RecievCanID >> 20;																	// Адрес узла-передатчика пакета
//	
//	if ((adrTx>3)&&(cod>0x04)&&(cod<0x0C)&&(!bMagorOk))	{											//  обслужить запросы только 4, 5, 6 адрес МУК1..3 ЗРУ, КПА
//		
//		if (bWeitTime)	{																												// Флаг времени ожидания прихода всех команд от МУК1..3 ЗРУ (1 секунда)
//			bStrtGetCmd = 1;																											// Стартовал процесс прихода команд от МУК1..3 ЗРУ
//			bWeitTime = 0;																												// Сброс при получении первой команды
//		}	

//		nMUK_ZRU[adrTx-4] = 1;																									// Флаг активности узла-передатчика пакета
//		codMUK_ZRU[adrTx-4] = cod;																							// Код (команда) пакета
//		lastCMD = cod;
//		if (MagorOk())	bMagorOk=1;

//	}/*	if (adrTx>3)																													//  4, 5, 6, 7	адрес МУК1..3 ЗРУ, КПА */

//	new_mode = ADC_ERR;
//	NVIC_EnableIRQ(CAN1_IRQn);	NVIC_EnableIRQ(CAN2_IRQn);
//}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Разбор принятого пакета по CAN,	uint32_t	RecievCanID[32];	RecievCanDLC[32];		RecievCanDATAH_RecievCanDATAL[32];
//union uBytesFloat16 aIrazr, aIzar;					// Получены по CAN из ЗРУ для коррекции U AKi
void RunCmdCAN (void)
{	unsigned char  cod;
	
		i=0;
		while ((!(codMUK_ZRU[i]))&&(i<3))	i++;
		cod = codMUK_ZRU[i];
		new_mode = ADC_ERR;
	
		switch (cod)		{																												// Выбор команды:
 			case CAN_MSG_Ok:																											// Подтверждение о получении пакета
				// Фрагмент кода подготовки данных для передачи в программе ЗРУ	
				// MDR_CAN1->CAN_BUF[lbuf_TX].DATAL = (aIrazr.b[0] <<24)|(aIzar.b[1]<<16)|(aIzar.b[0] <<8)|confcmd;	// Четвёртый..первый байт в пакете
				// MDR_CAN1->CAN_BUF[lbuf_TX].DATAH =  aIrazr.b[1];																									// Восьмой..пятый байт в пакете
				// Получение токов заряда и разряда, и состояния ЗРУ
				aIzar.b[0] = 0xff & (RecievCanDATAL>>8);			
				aIzar.b[1] = 0xff & (RecievCanDATAL>>16);			
				aIrazr.b[0] = 0xff & (RecievCanDATAL>>24);			
				aIrazr.b[1] = 0xff & (RecievCanDATAH);			
				bVklZRU = 0x80 & RecievCanDATAL;			
				//				bOkCAN = 1;		Это не актуально!!!
 				break;

			case CAN_Vkl_RS:																											// Подключить разрядный резистор
				new_mode = RRazr_On;
				break;

			case CAN_Otkl_RS:																											// Отключить разрядный резистор
				new_mode = RRazr_Off;
				break;

 			case CAN_Step_On:																											// Включить шаговый режим с АКПА
				bStep = 1;
				TransmCAN_Msg(CAN_Step_On,1,0);																			// Ответ о результате включения	
 				break;

 			case CAN_Step_Off:																										// Отключить шаговый режим с АКПА
				bStep = 0;
				TransmCAN_Msg(CAN_Step_Off,1,0);																		// Ответ о результате отключения	
				new_mode = Init_Run;
 				break;

			case CAN_U_AK:																												// Выдать напряжение пары АК
				if (bStep)	{
					nTwoAEs  = 0x0000003f & RecievCanDATAL;														// Номер пары АЭ, полуенный от КПА (для измерения напряжения)
					new_mode = Read_U_AK;
				}
				break;

			case CAN_SetPeriod:																										// Установить период выдачи телеметрии датчиков
				PeriodSndTelem  = 0x000000ff & RecievCanDATAL;			
				if (PeriodSndTelem) deleyST = PeriodSndTelem;
				TransmCAN_Msg(CAN_SetPeriod,1,0);																		// Ответ о результате отключения	
				break;

			case CAN_Pasport_Get:																									// Паспортные данные ДД, ДТ													*** МУК БЭ 
				TransmCAN_PasportDach();
				break;
			
			case CAN_NumBadAk:																										// Номера отказавших АЭ АБ													***	АРК, КПА 
				for (i=0; i<4; i++)
				{nBadAE_BCU[i] = 0x000000ff & RecievCanDATAL>>i*8;	}								// Номера отказавших АЭ, полученные по CAN от БЦУ
				nBadAE_BCU[4]  = 0x000000ff & RecievCanDATAH;			
				TransmCAN_Msg(CAN_NumBadAk,1,0);																		// Ответ для ЗРУ	
				break;
		}
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
// Получение среднего значения параметра измерения в вольтах по nReadADC точкам
void bOkDataADC (void)
{
	OkResult = 0;
	iReadAdc++;																																		// 

	if (iReadAdc < nReadADC)	{																										// Промежуточный массив значений текущего канала заполнен
		if (Result>0)	{	
			summa+=Result;	NumReadAdc++;		}																					// Допустимый результат
		MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO & ADC1_REG_GO;													// Повторить измерение
	}
	
	else	{
		if ((NumReadAdc)&&(summa)) {
			Result = summa/NumReadAdc;
			if (add_chanl)	{	Uadc = ((Result)*(AUcc[nMUK_BE-1]/0xfff));					}		// Пересчёт значения АЦП в вольты
			else						{	
												AUcc[nMUK_BE-1] = (Uref[nMUK_BE-1]*0xfff)/(Result);			// Коррекция AUcc
			}
		}	
		else	{	Uadc=0;	}																														// АЭ вышел из строя
		OkDataADC = 1;																															//summa=0;	NumReadAdc=0;
	}	
}

//void loop (void)	{
//	while (1)			{	}
//}	

// *********************************************** MAIN ****************************************************/
int main (void)
{		
	Clock_Init();													// Инициализация тактового генератора
	Ports_Init(); 
	//Ports_Init_Tst(); 										// Инициализация портов для отладочной платы
 	ADC_Init();
 	CAN1_Init(); 
 	CAN2_Init();
	__enable_irq ();											// Глобальное разрешение прерываний
	SysTickInit(100000);

	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//initInternalWatchdog();								// Запуск сторожевого таймера
	
	mode = Init_Run;											// Начальрый режим
//	bPauza5 = 1;	sCount5 =0;							// Флаг отсчёта паузы 5 сек
//	bOneSec = 0;		cnt_tst30=0;

	while (1)												
	{
		//..............................................................................................................................
		if (bPeriod)	{	bPeriod=0;					// Отправка телеметрии через Period сек.
			
			if	(bDataDatchOk)				 	TransmCAN_Dt(); 							// ДД, ДТ
			//TransmCAN_Err();																					// Статус ошибок
			if	((bDataABOk)&&(!bStep))	TransmCAN_AB();								// АБ
			vtstTime++;	
		}

		//..............................................................................................................................
//		if (!bPauza5)	{				bPauza5 = 1;	sCount5 =0;							// Контроль работоспособности CAN через 5 сек.
//					
//			if (bOkCAN)	{				bControlP = 0;	bOkCAN = 0;	MDR_PORTE->RXTX &= ~0xC;	}					// Проверка и Сброс флага "CAN работает"
//			else	{
//				if (!bStep)	{	bControlP = 1;	}		}											// Установка флага "Контроль давления" без связи по CAN с ЗРУ (если не шаговый режим)
//			
//		}
			
		//..............................................................................................................................
		switch (mode)												// Обработчик состояний
		{
			case Init_Run:										// Инициализация всех процессов при старте или сбоях
				Var_init();																							// Инициализация переменных
				bPauza5 = 1;	sCount5 =0;																// Флаг отсчёта паузы 5 сек
				bOneSec = 0;		cnt_tst30=0;
			
			case AK_START:										// Начальный запуск или переход в нормальный режим
				Start();																								// Пуск процесса чтения напряжений датчиков
				ReadDatch_GO();																					// Пуск процесса чтения напряжений датчиков
				new_mode=NORMAL;																				// Переход в штатный рабочий режим
				break;
			
			case NORMAL:											// Штатная работа. Режим измерения напряжений АБ и датчиков АЦП
				if (OkResult)		{	bOkDataADC();	}												// Значение одной точки параметра получено
				if (OkDataADC)	{																				// Среднее значение параметра получено
					PutParamADC();	}																			// Запись напряжения соответствующего параметра ДД, ДТ, АБ, АЭ
				break;
			
			case AK_AR_READY:									// Обработка массива напряжений аккумуляторов
				
				if (StatRdADC==ReadDatch) {		
					RaschotArrayDatch();																	// Обработка массива значений напряжений датчиков
					if (bStep)		{	ReadDatch_GO();		}										// Шаговый режим с КПА
					else					{	ReadAE_GO();			}									
				}
				else	{													
					RaschotArrayAE();																			// Обработка массива значений напряжений АЭ
					//_ReadAE_GO();
					ReadDatch_GO();																				//
				}
				new_mode=NORMAL;																				// Возврат в нормальный рабочий режим
				break;
				
			case ADC_ERR:											// Повтор чтения текущих каналов АЦП
				ADC_Start(iadc);																				// Перезапуск опроса каналов АЦП
				MDR_ADC->ADC1_CFG |= ADC1_CFG_REG_GO;										// Запуск преобразования
				new_mode=NORMAL;																				// Переход в нормальный рабочий режим
				break;
			
			case Read_U_AK:										// Чтение пары напряжений AK(i) и AK(i+1)
				if (nTwoAEs<=MaxTwoAEs)	{
					MDR_ADC->ADC1_CFG &= (~ADC1_CFG_REG_ADON);						// Остановка АЦП
					TReadAE_Step_GO();																		// Запустить чтение пары напряжений AЭ(i) и AЭ(i+1)
					new_mode=SEND_U_AK;																		// Возврат в нормальный рабочий режим
				}	
				else	{
					TransmCAN_Msg(CAN_U_AK,2,1);													// Ответ "НЕдоустимое значение"
					new_mode=NORMAL;																			// Возврат в нормальный рабочий режим
				}	
				break;
			
			case SEND_U_AK:										// Отправка пары напряжений AK(i) и AK(i+1)
				if (OkResult)		{	bOkDataADC();	}												// Значение одной точки параметра получено
				if (OkDataADC)	{																				// Среднее значение параметра получено
					PutParamADC_AE();																			// Запись напряжения соответствующего AK(i)
				}
				break;
			
			case RRazr_On:										// Включить разрядные резисторы
				RRazrOn();																						
				TransmCAN_Msg(CAN_Vkl_RS,1,0);													// Ответ о результате включения
				new_mode=NORMAL;																				// Возврат в нормальный рабочий режим
				break;
			
			case RRazr_Off:										// Отключить разрядные резисторы
				RRazrOff();
				TransmCAN_Msg(CAN_Otkl_RS,1,0);													// Ответ о результате отключения	
				new_mode=NORMAL;																				// Возврат в нормальный рабочий режим
				break;
			
			default:	{	mode = Init_Run;		}													// Начальрый режим
			
			
		} //end of switch (mode)
		
		//..............................................................................................................................
		if (MagorOk())	bMagorOk=1;					// Мажоритар собрался

		//..............................................................................................................................
		if (((bWeitTime)&&(bStrtGetCmd))||(bMagorOk))	{							// Время ожидания прихода всех команд от МУК1..3 ЗРУ ИСЧЕРПАНО!!!(1 секунда)
			if (bMagorOk)		{	RunCmdCAN ();	}													// Мажоритар собрался. Команды от МУК1..3 ЗРУ собраны
			else							TransmCAN_Msg(lastCMD,0,0);							// Не собрался мажоритар
			bStrtGetCmd = 0;																					// Сброс Старта процесса прихода команд от МУК1..3 ЗРУ - команда не быдет отрабатываться
			for (i=0; i<3; i++)	{																			// 
				nMUK_ZRU[i] = 0;																				// Сброс Флага активности узла-передатчика пакета
				codMUK_ZRU[i] = 0;																			// Код (команда) пакета
			}		
			coun=0;	bMagorOk=0;
		}	
		//..............................................................................................................................
//		if (bControlP) {	ControlP ();	}														// Контроль давления АБ
		ControlP ();																								// Контроль давления АБ
		
		mode = new_mode;
		
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//resetInternalWatchdog();						// Перезапуск (сброс) внутреннего сторожевого таймера.
	}
}
// end of file main.c
