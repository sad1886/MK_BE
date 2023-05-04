/**********************************************************************
*																																			*
*		Файл параметров для АБ 72НВ-40 ЖЦПИ.563532.011 зав. № ________		*
*																																			*
***********************************************************************/

#ifndef PARAMETERS_E3001002_H
#define PARAMETERS_E3001002_H

#define nDatchD					5																				// Число датчиков давления
#define nDatchT					5																				// Число датчиков температуры

//**************************************************************************************************
const int	kodADC_t25[3]	= {1648, 1700, 1737};

const float	dTempC[3]	= {14.7, 3.69, 12.71};											// (Грд Цельс) дельта датчика температуры МК

const float kdelta	= 0.05;																			// Допустимый диапазон разброса ошибки 5% от диапазона измерений

			float	AUcc[3]	= {3.216, 3.291, 3.287};										// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
const float	Uref[3]	= {3.0169, 3.0197, 3.0214};										// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0

						 
// .............................................................................................................................
// *** М И Б *******************************************************************************************************************

const int nCorrectAE 	 = 2;																			// Числ пропусков ложного значения напряжений АЭ
const int koefZadergAE = 2;//4;																	// Коэфф задержки после коммутации дешифр АЭ (koefZadergAE*10000)

const float kTempAE[3]	= {0.00020001, 0.00015001, 0.00000001};	// Температурная коррекция АЭ

const float kTempAB[3]	= {0.00000001, 0.00000001, 0.00000001};	// Температурная коррекция АБ

//float minUAE = 0.6;																							// (В) Нижняя граница значения рабочего напряжения АЭ
//float maxUAE = 2.0;																							// (В) Верхняя граница значения рабочего напряжения АЭ

//														 1				 2				3				 4				5				 6				7				 8				9				10
const float	kAE_[2][72]	={{	/* МУК1 */
														2.530452,2.220004,2.465911,2.270316,2.421212,2.251183,2.399465,2.296335,2.371069,2.302934,
														2.364075,2.316247,2.364075,2.309571,2.357122,2.322960,2.350210,2.322959,2.350209,2.322959,
														2.350210,2.336504,2.336505,2.322959,2.329713,2.336504,2.406671,2.329712,2.329713,2.350208,
														2.343337,2.357121,2.336505,2.329712,2.350209,2.283251,2.343338,2.371069,2.329713,2.385182,
														2.329713,2.343336,2.336505,2.343336,2.329713,2.350208,2.329713,2.343336,2.329713,2.336504,
														2.336505,2.343336,2.329713,2.357120,2.336505,2.302934,2.350209,2.343336,2.336505,2.357120,
														2.406671,2.336504,2.329713,2.350208,2.322960,2.343336,2.329713,2.350208,2.316246,2.357120,2.336505,2.251182},

//													{
//														1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
//													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
//													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
//													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
//													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0,
//													  1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		1.0, 1.0}};
															
														// /* МУК2 */......................................................................................
														{ 
														2.493089,2.220370,2.435029,2.251381,2.398794,2.283272,2.370573,2.316077,2.391676,2.309441,
														2.343008,2.329464,2.356710,2.316078,2.349839,2.336216,2.363621,2.329465,2.336216,2.336216,
														2.349839,2.329465,2.336217,2.336216,2.343008,2.329465,2.343008,2.336216,2.349839,2.329465,
														2.343008,2.343009,2.349839,2.329465,2.343008,2.343009,2.356710,2.336217,2.336216,2.343009,
														2.343008,2.343008,2.329465,2.349839,2.349839,2.343008,2.309440,2.336218,2.322752,2.435030,
														2.336216,2.363622,2.336217,2.336217,2.329465,2.349840,2.343008,2.343008,2.336216,2.356711,
														2.329465,2.377566,2.316077,2.349839,2.343008,2.343008,2.329465,2.349840,2.356710,2.343008,2.336216,2.251382}};
					
// .............................................................................................................................
const float	dAE_[2][72]	={{	// /* МУК1 */
														-0.051147,0.029872,-0.025858,-0.012468,0.004928,0.016528,0.001246,-0.009218,0.027009,-0.003787,
														0.015682,-0.021483,0.024174,-0.006623,0.022271,-0.024386,0.033522,-0.012342,0.021338,-0.020687,
														0.033522,-0.018128,0.030648,-0.022536,0.039922,-0.016268,-0.001693,-0.025450,0.043631,-0.029595,
														0.026006,-0.041033,0.035321,-0.015227,0.019467,-0.001763,0.034424,-0.042332,0.025989,-0.058884,
														0.030649,-0.015450,0.025068,-0.025734,0.038067,-0.022111,0.029698,-0.029465,0.036213,-0.012548,
														0.025068,-0.023868,0.032503,-0.023182,0.023208,-0.008394,0.022297,-0.015450,0.023208,-0.037279,
														-0.000712,-0.016268,0.025989,-0.026812,0.033400,-0.013585,0.025989,-0.028683,0.037980,-0.021306,0.025068,-0.023817},

//													  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
//														0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
//													  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
//													  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
//													  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,
//													  0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0,		0.0, 0.0}};

														// /* МУК2 */......................................................................................
													{
													  -0.056513,0.016058,-0.006009,0.030361,-0.013325,-0.013057,0.020091,0.004905,-0.010377,-0.018954,
														0.031253,0.001113,0.002237,-0.023658,0.028487,-0.001724,-0.000631,-0.031303,0.034003,-0.001724,
														0.005087,-0.027616,0.032153,-0.001724,0.004212,-0.025772,0.027544,-0.003574,0.006947,-0.031303,
														0.031253,-0.002724,0.003227,-0.031303,0.029398,-0.004579,-0.003360,-0.028687,0.024757,-0.000870,
														-0.001352,-0.027910,0.033049,-0.001869,-0.000493,-0.031619,0.046673,0.001973,0.018164,-0.096408,
														0.030304,-0.009499,0.005191,-0.028687,0.033049,-0.007449,0.004212,-0.029765,0.028455,-0.010337,
														0.008009,-0.059714,0.038491,-0.001869,0.000503,-0.029765,0.029361,-0.003729,-0.003360,-0.031619,0.030304,0.000064}};

// .............................................................................................................................

const float kUop[2]	=	{4.325, 4.325};						// Коэфф масштаб для измеренного опорного (аппаратного) напряжения

float cUsm[3][2] =  {{1.361, 1.363},						// (В) Измеренное напряжение средней точкой (смещение) для МУКов
									 	 {1.365, 1.359},
										 {0, 0}};
float Uop[2] = {1.981, 1.996};									// (В) Измеренное опорное напряжение

float t_Usm[2]	=	{1.364, 1.36};								// (В) Измеренное напряжение средней точкой (смещение) для отладки
float t_Uop[2]	=	{2.006, 1.992};

float TUsm[2];																	// (В) Измеренное напряжение средней точкой (смещение) для отладки
float TUop[2];																	// (В) Измеренное опорное напряжение для отладки
//
volatile float NumIzmAE[nDatchD]	= {54, 25, 17, 60, 50};	// Номера измерительных АЭ (ДД, ДТ) от 23.01.20
//volatile float NumIzmAE[nDatchD]	= {6, 7, 16, 59, 66};	// 7, 8, 17, 60, 67 - Номера измерительных АЭ (ДД, ДТ) 


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
const float	KuAB1[3]	= {64.203035, 64.426245, 62.138699};
const float	DuAB1[3]	= {0.537773,   0.081076, 0.000000};

const float	KuAB2[3]	= {64.203035, 64.569411, 62.138699};
const float	DuAB2[3]	= {0.537773,  -0.024430, -0.000000};

// ******************************** Датчики давления: ********************************
const float	MinUdatchD = 0.1;															 
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
float ItarD[nDatchD]	= {0.002003,  0.0019898,  0.002002, 0.001993, 0.002002};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDD[3][nDatchD]		= {{0.050717,0.050724,0.050735,0.050722,0.050796},
																 {0.050769,0.050685,0.050677,0.050639,0.050731},
																 {0.048798,0.048761,0.048708,0.048716,0.048829}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDD[3][nDatchD]		= {{-0.007266,-0.007290,-0.007023,-0.007184,-0.007190},
																 {-0.007303,-0.007133,-0.006855,-0.006987,-0.007045},
																 {-0.007291,-0.007238,-0.006879,-0.007053,-0.007175}};							 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},
																 

// ******************************** Датчики температуры: ********************************
const float	MinUdatchT = 0.001;															 
const float	MaxUdatchT = 3.18;
															 
// ------------------------------- Паспортные данные
float R0[nDatchT]			= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 0°C (по паспорту);
//float R100[nDatchT]		= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 100°C (по паспорту);
												 
float alpha[nDatchT]	= {0.004149, 0.004149, 0.004149, 0.004149, 0.004149};			// Температурный коэффициент по паспорту.

// ------------------------------- Тарировочные данные																																	
float ItarT[nDatchT]	= {0.0040124, 0.0040274, 0.0040072, 0.00402, 0.0040166};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDT[3][nDatchD]		= {{0.044094,0.044123,0.044143,0.044317,0.044398},
																 {0.044168,0.044138,0.044160,0.044125,0.044184},
																 {0.042501,0.042455,0.042421,0.042423,0.042498}
//																 {1.0, 1.0, 1.0, 1.0, 1.0},
																 };

const float	dDT[3][nDatchD]		= {{0.371285,0.370949,0.371017,0.370491,0.370311},
																 {0.372777,0.372699,0.372575,0.372698,0.372337},
																 {0.371999,0.371962,0.372067,0.372073,0.371758}
//																 {0.0, 0.0, 0.0, 0.0, 0.0},
																};								 

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

const float	kLimUadc[3][nDatchD]	= {{0.0035, 0.0070, 0.004, 0.0027, 0.0027},
																	 	 {0.0035, 0.0070, 0.004, 0.0037, 0.0037},
																		 {0.0035, 0.0073, 0.0045, 0.004, 0.004}};						// Коэфф динамической дельты датч темпертуры

const float	kLimUadcT[3][nDatchD]	= {{0.000005, 0.000005, 0.000005, 0.000005, 0.000005},
	                                   {0.000005, 0.000005, 0.000005, 0.000005, 0.000005},                      // Коэфф динамической дельты датч темпертуры для 1 уставки 
																		 {0.0, 0.000005, 0.000005, 0.000005, 0.000005}};																			 

//========== коррекция пятой уставки ====================================================================
const float	LimUadc_5[3][nDatchD]	= {{ 3.3, 3.3, 3.3, 3.3, 3.3},
																			{3.3, 3.3, 3.3, 3.3, 3.3},
																			{3.3, 3.3, 3.3, 3.3, 3.3}};

const float	kLimUadc_5[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэфф динамической дельты датч темпертуры

const float	kLimUadc_5_T[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэф динамической дельты датч темпертуры для 5 уставки


#endif
