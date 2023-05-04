/**********************************************************************
*																																			*
*		Файл параметров для АБ 72НВ-40 ЖЦПИ.563532.011 зав. № ________		*
*																																			*
***********************************************************************/

#ifndef PARAMETERS_A3001006_H
#define PARAMETERS_A3001006_H

//#define nMaxReadADC			3000			// Максимальное количество чтений АЦП для одного канала
//#define nWrkReadADC			1000			// Количество чтений АЦП для одного канала в штатом режиме
//#define nChanlADCd			2					// Число рабочих каналов АЦП. 2,3 - для датчиков; 4,5 - АЭ
//#define nDatch					10				// Число датчиков (5+5)

#define nDatchD					5					// Число датчиков давления
#define nDatchT					5					// Число датчиков температуры

//#define nMaxBadAE				5					// Число вышедших из строя АЭ, при котором АБ ещё "живёт"
//#define nMaxBadDatchD		2					// Число вышедших из строя ДД, при котором АБ ещё "живёт"
//#define nMaxBadDatchT		2					// Число вышедших из строя ДТ, при котором АБ ещё "живёт"

//#define lockPMaxDatchD	50				// Максимальное давление для ДД (для автономного управление)
//#define lockPMinDatchD	10				// Минимальное давление для ДД (для автономного управление)

//#define PMaxDatchD			65				// Максимальное давление для ДД (для автономного управление)
//#define PMinDatchD			0					// Минимальное давление для ДД (для автономного управление)

//#define TMaxDatchT			67				// Максимальная температура для ДТ
//#define TMinDatchT		 -18				// Минимальная температура для ДТ

//#define addd_dTm1				0.0002
//#define addd_dTm2				0.0004
//#define addd_dTm3				0.0004

//**************************************************************************************************
const int	kodADC_t25[3]	= {1688, 1717, 1837};

const float	dTempC[3]	= {0.00, 1.5, 1.5};	// (Грд Цельс) дельта датчика температуры МК

const float kdelta	= 0.05;											// Допустимый диапазон разброса ошибки 5% от диапазона измерений

			float	AUcc[3]	= {3.271, 3.254, 3.266};		// (В) Напряжение АЦП МУК1, МУК2, МУК3 (С19)
const float	Uref[3]	= {3.0194, 3.0108, 3.0199};		// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3 на PD0 (С13)


// .............................................................................................................................
// *** М И Б *******************************************************************************************************************

const int nCorrectAE 	 = 2;											// Числ пропусков ложного значения напряжений АЭ
const int koefZadergAE = 4;											// Коэфф задержки после коммутации дешифр АЭ (koefZadergAE*40000)

const float kTempAE[3]	= {0.00010, 0.000100001, 0.00000001};	// Температурная коррекция АЭ

const float kTempAB[3]	= {0.00000001, 0.00000001, 0.00000001};		// Температурная коррекция АБ

//float minUAE = 0.6;															// (В) Нижняя граница значения рабочего напряжения АЭ
//float maxUAE = 2.0;															// (В) Верхняя граница значения рабочего напряжения АЭ

//													 1					 2					3				 4					5					6					7				 8				9					10		
const float	kAE_[2][72]	={{ 
					/* МУК1 */				2.467344,2.244374,2.430109,2.269658,2.408321,2.282537,2.386898,2.295538,2.386907,2.315349,
														2.372832,2.308696,2.352052,2.315350,2.358934,2.315334,2.358943,2.335492,2.358934,2.322007,
														2.358943,2.349113,2.345198,2.342260,2.345204,2.342286,2.352043,2.322007,2.352052,2.342283,
														2.352043,2.328723,2.345202,2.342283,2.345204,2.328723,2.338395,2.349116,2.352037,2.342268,
														2.331621,2.355989,2.345191,2.342271,2.331623,2.349119,2.338382,2.342271,2.338395,2.355989,
														2.345194,2.349103,2.324892,2.355990,2.338385,2.349097,2.324894,2.362903,2.345189,2.342271,
														2.324889,2.349119,2.338380,2.355977,2.331621,2.355992,2.338380,2.349103,2.324891,2.355995,
														2.345192,2.244331},

	// ...................................................................................................
					
					/* МУК2 */			 {2.498493,2.250870,2.468047,2.250864,2.438342,2.282574,2.402193,2.302025,2.409340,2.308593,
2.388030,2.321816,2.388037,2.321819,2.381015,2.335199,2.395094,2.335203,2.367105,2.335199,
2.374046,2.335206,2.374038,2.335195,2.367105,2.328497,2.346535,2.348734,2.374040,2.328497,
2.367103,2.348737,2.360210,2.335206,2.360208,2.348737,2.360211,2.335207,2.339761,2.341948,
2.339761,2.335210,2.346534,2.335206,2.353354,2.341958,2.346538,2.341950,2.346539,2.335209,
2.339758,2.348745,2.333026,2.348746,2.339761,2.348737,2.353356,2.341958,2.339758,2.341951,
2.339764,2.341958,2.339759,2.348741,2.339761,2.341962,2.339759,2.355565,2.339761,2.348749,
2.353351,2.244598}};
//													 1		 2			3			 4			5			 6			7			 8			9			10
/* 													{ 
														1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
														1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
														1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0
														}};
*/					
//														 1					 2					3					 4					5					6							7					 8					9						10
const float	dAE_[2][72]	={
													{0.007003,-0.018542,0.014909,-0.039744,0.030278,-0.035521,0.032070,-0.047707,0.036818,-0.046423,
													0.035762,-0.055441,0.048713,-0.044576,0.043176,-0.056570,0.047868,-0.055353,0.043176,-0.063264,
													0.047868,-0.063267,0.050502,-0.077972,0.051425,-0.056495,0.044035,-0.063264,0.048713,-0.058364,
													0.044035,-0.062564,0.049554,-0.058364,0.051425,-0.064423,0.054121,-0.059518,0.038405,-0.070498,
													0.053081,-0.062559,0.043018,-0.066760,0.054942,-0.055769,0.045738,-0.066760,0.054121,-0.062559,
													0.046760,-0.069813,0.057612,-0.060679,0.049470,-0.075435,0.059466,-0.063733,0.041147,-0.066760,
													0.053902,-0.055769,0.043873,-0.071005,0.053081,-0.058800,0.043873,-0.069813,0.055757,-0.056921,
													0.044889,-0.058817},

					/* МУК2 */			{-0.026113,-0.004246,-0.020237,-0.013637,-0.001891,-0.018109,0.006452,-0.032569,0.009788,-0.023992,
0.008403,-0.039432,0.018366,-0.035270,0.013146,-0.047161,0.013624,-0.039270,0.020673,-0.047161,
0.025884,-0.037418,0.015979,-0.050864,0.023039,-0.030801,0.025286,-0.054978,0.020236,-0.032648,
0.018795,-0.053116,0.025827,-0.037418,0.021595,-0.053116,0.027699,-0.035566,0.029898,-0.050129,
0.032237,-0.031862,0.023424,-0.039753,0.028598,-0.038502,0.027146,-0.048272,0.031354,-0.033714,
0.026187,-0.045665,0.036816,-0.039590,0.029898,-0.053116,0.030465,-0.036644,0.026187,-0.046415,
0.034093,-0.038502,0.028042,-0.049390,0.032237,-0.032930,0.028042,-0.056119,0.032237,-0.037728,
0.022513,-0.042925}};
					// .................................................................................................
/*		  									{ 
													0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
													0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
													0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
													0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
												  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
													0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0
													}};
*/				

// .............................................................................................................................

const float kUop[2]	=	{4.325, 4.325};						// Коэфф масштаб для измеренного опорного (аппаратного) напряжения

float cUsm[3][2] =  {{1.363, 1.361},						// (В) Измеренное напряжение средней точкой (смещение) для МУКов
									 	 {1.368, 1.369},
										 {0, 0}};
float Uop[2] = {1.981, 1.996};									// (В) Измеренное опорное напряжение

float t_Usm[2]	=	{1.364, 1.36};								// (В) Измеренное напряжение средней точкой (смещение) для отладки
float t_Uop[2]	=	{2.006, 1.992};

float TUsm[2];																	// (В) Измеренное напряжение средней точкой (смещение) для отладки
float TUop[2];																	// (В) Измеренное опорное напряжение для отладки
//
volatile float NumIzmAE[nDatchD]	= {6, 7, 16, 59, 66};	// 7, 8, 17, 60, 67 - Номера измерительных АЭ (ДД, ДТ) 


// .............................................................................................................................
// *** П С Д *******************************************************************************************************************

const int nCorrectDD = 1;												// Числ пропусков ложного значения напряжений датчиков ДАВЛЕНИЯ
const int nCorrectDT = 1;												// Числ пропусков ложного значения напряжений датчиков ТЕЧПЕРАТУРЫ
const int nCorrectDAB = 1;											// Числ пропусков ложного значения напряжений датчиков ТЕЧПЕРАТУРЫ

const int koefZadergDat = 2;										// Коэфф задержки после коммутации дешифр датчиков (koefZadergDat*40000)

const float	KuGndP[3]	= {1.0, 1.0, 1.0};
const float	KuGndT[3]	= {1.0, 1.0, 1.0};
const float	KuOpornP[3]	= {1.0, 1.0, 1.0};
const float	KuOpornT[3]	= {1.0, 1.0, 1.0};

// ******************************** Напряжение АБ: ********************************
const float	KuAB1[3]	= {64.489952, 64.489952, 64.489952};
const float	DuAB1[3]	= {-0.340721, -0.440721,  -0.440721};

const float	KuAB2[3]	= {64.489952, 64.489952, 64.489952};
const float	DuAB2[3]	= {-0.340721, -0.440721, -0.440721};

// ******************************** Датчики давления: ********************************
const float	MinUdatchD = 0.04;															 
const float	MaxUdatchD = 3.18;
	
// ------------------------------- Паспортные данные
float a0[3][nDatchD]				  =	{{1.1784,	1.1784, 1.1784, 1.1784, 1.1784},
														  	 {1.1784,	1.1784, 1.1784, 1.1784, 1.1784},
	 													  	 {1.1784,	1.1784, 1.1784, 1.1784, 1.1784}};

float a1[3][nDatchD]				  = {{0.4732, 0.4732, 0.4732, 0.4732, 0.4732},
	 													  	 {0.4732, 0.4732, 0.4732, 0.4732, 0.4732},
	 													  	 {0.4732, 0.4732, 0.4732, 0.4732, 0.4732}};

float a2[3][nDatchD]				  =	{{0.0002, 0.0002, 0.0002, 0.0002, 0.0002},
	 													  	 {0.0002, 0.0002, 0.0002, 0.0002, 0.0002},
	 													  	 {0.0002, 0.0002, 0.0002, 0.0002, 0.0002}};
																 
// ------------------------------- Тарировочные данные
float ItarD[nDatchD]	= {0.0020052,  0.0019966,  0.0020038, 0.0020018, 0.0019985};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDD[3][nDatchD]		= {{0.055955,0.055869,0.055621,0.055362,0.055436},
																 {0.055819,0.055847,0.055593,0.055384,0.055417},
																 {0.055921,0.055831,0.055663,0.055450,0.055411}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDD[3][nDatchD]		= {{-0.009120,-0.008713,-0.008908,-0.008865,-0.008957},
																 {-0.009001,-0.008674,-0.008808,-0.008839,-0.008836},
																 {-0.009089,-0.008764,-0.008885,-0.008907,-0.008772}};								 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},


// ******************************** Датчики температуры: ********************************
const float	MinUdatchT = 0.004;															 
const float	MaxUdatchT = 3.18;
	
// ------------------------------- Паспортные данные
float R0[nDatchT]			= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 0°C (по паспорту);
//float R100[nDatchT]		= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 100°C (по паспорту);
												 
float alpha[nDatchT]	= {0.004149, 0.004149, 0.004149, 0.004149, 0.004149};			// Температурный коэффициент по паспорту.

// ------------------------------- Тарировочные данные																																	
float ItarT[nDatchT]	= {0.0040024, 0.0040311, 0.0040198, 0.0040263, 0.0040304};		// (A) Ток опроса датчиков ДТ в БЭ
																
const float	kDT[3][nDatchD]		= {{0.048752,0.048650,0.048725,0.048683,0.048656},	
																 {0.048576,0.048494,0.048676,0.048476,0.048683},
																 {0.048478,0.048548,0.048503,0.048696,0.048624}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDT[3][nDatchD]		= {{0.369407,0.369583,0.369826,0.369508,0.370097},	// 0.371696 dДт1 2-4 точка, 0.370504 2-5 точка
																 {0.368690,0.368752,0.368595,0.368666,0.368852},
																 {0.369744,0.369625,0.369853,0.369268,0.369901}};								 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},

const float	kTemp[3][5]	= {0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001,
													 0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001,
													 0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001};

//const float	dKT[3][nDatchD]	= {{ 0.01,   0.01,  0.010,  0.01,  0.01},
//															 {-0.018, -0.02, -0.018, -0.019, -0.02},
//															 { 0.0245, 0.024, 0.0245, 0.025,  0.024}};						// Коэфф динамической дельты датч темпертуры

//========== коррекция первой уставки ====================================================================
const float	LimUadc[3][nDatchD]	= {{ 0.9, 0.9, 0.9, 0.9, 0.9},
																		{0.9, 0.9, 0.9, 0.9, 0.9},
																		{0.9, 0.9, 0.9, 0.9, 0.9}};											// Коэфф динамической дельты датч темпертуры

const float	kLimUadc[3][nDatchD]	= {{0.0015, 0.0009, 0.0008, 0.0006, 0.0012},
																	 	 {0.0010, 0.0005, 0.0003, 0.0005, 0.0005},
																		 {0.0014, 0.0005, 0.0009, 0.0005, 0.0005}};						// Коэфф динамической дельты датч хвостика темпертуры

const float	kLimUadcT[3][nDatchD]	= {{0.000005, 0.000005, 0.000005, 0.000005, 0.000005},
	                                   {0.000000, 0.000000, 0.000000, 0.000000, 0.000000},                      // Коэфф динамической дельты датч темпертуры для 1 уставки 
																		 {0.000000, 0.000000, 0.000000, 0.000000, 0.000000}};																			 

//========== коррекция пятой уставки ====================================================================
const float	LimUadc_5[3][nDatchD]	= {{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{0.0, 0.0, 0.0, 0.0, 0.0},
																			{0.0, 0.0, 0.0, 0.0, 0.0}};

const float	kLimUadc_5[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэфф динамической дельты датч темпертуры

const float	kLimUadc_5_T[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэф динамической дельты датч темпертуры для 5 уставки


#endif
