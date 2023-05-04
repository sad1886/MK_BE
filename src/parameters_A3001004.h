/**********************************************************************
*																																			*
*		Файл параметров для АБ 72НВ-40 ЖЦПИ.563532.011 зав. № ________		*
*																																			*
***********************************************************************/

#ifndef PARAMETERS_A3001004_H
#define PARAMETERS_A3001004_H

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
const int	kodADC_t25[3]	= {1642, 1762, 1687};										// Значения в битах при 25 град Цельсия

const float	dTempC[3]	= {0.00, 0.2, 0.5};												// (Грд Цельс) дельта датчика температуры МК

const float kdelta	= 0.05;																			// Допустимый диапазон разброса ошибки 5% от диапазона измерений

			float	AUcc[3]	= {3.275, 3.268, 3.282};										// (В) напряжение АЦП МУК1, МУК2, МУК3
const float	Uref[3]	= {3.0164, 3.0242, 3.013};									// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3 на PD0


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
					/* МУК1 */				2.429205,2.241997,2.429199,2.248305,2.414501,2.248314,2.392770,2.261053,2.399973,2.254673,
														2.371433,2.267479,2.378508,2.261060,2.364405,2.280443,2.364410,2.293560,2.352331,2.307124,
														2.350479,2.306829,2.350473,2.300163,2.336707,2.306831,2.343565,2.300165,2.350479,2.306824,
														2.329881,2.300169,2.336708,2.300176,2.336703,2.306816,2.336708,2.300179,2.316346,2.306819,
														2.316352,2.327016,2.336699,2.313513,2.329881,2.306832,2.329877,2.320234,2.329882,2.320247,
														2.329875,2.333815,2.309648,2.327014,2.316348,2.320236,2.329882,2.333821,2.323093,2.313508,
														2.309648,2.320247,2.316344,2.333815,2.316353,2.327016,2.316344,2.352331,2.309648,2.333823,
														2.323091,2.223224},

	// ...................................................................................................
					
					/* МУК2 */			 {	2.405674,2.290445,2.391317,2.279356,2.393616,2.272877,2.384199,2.297024,2.391306,2.297008,
															2.363076,2.310294,2.363071,2.303618,2.349211,2.316972,2.363060,2.310273,2.349205,2.323707,
															2.363065,2.323676,2.335501,2.323703,2.349205,2.344092,2.328716,2.330463,2.335485,2.337251,
															2.335506,2.337268,2.321935,2.337251,2.328711,2.337264,2.335490,2.337247,2.315239,2.337273,
															2.315234,2.350970,2.315244,2.350983,2.321939,2.344086,2.321961,2.357914,2.335496,2.350976,
															2.321950,2.350993,2.315239,2.357902,2.308556,2.344117,2.315234,2.350966,2.321961,2.337264,
															2.308551,2.350989,2.308567,2.350983,2.308556,2.357891,2.315250,2.357914,2.315234,2.357891,
															2.321961,2.251887}};
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
													{0.026185,-0.026205,0.025219,-0.037121,0.031958,-0.030837,0.037625,-0.037465,0.035743,-0.028289,
													0.046028,-0.040357,0.044189,-0.034760,0.048796,-0.046190,0.049736,-0.049346, 0.049555,-0.059308,
													0.055217,-0.049769,0.054283,-0.058742,0.056899,-0.047925,0.053256,-0.056905,0.055217,-0.055299,
													0.062393,-0.051389,0.058767,-0.050476,0.057838,-0.058059,0.058767,-0.046800,0.062168,-0.054371,
													0.063088,-0.056920,0.054103,-0.055523,0.057733,-0.046081,0.058669,-0.064096,0.059595,-0.055758,
													0.056807,-0.064610,0.067589,-0.058779,0.064019,-0.062242,0.059595,-0.061818,0.061352,-0.059220,
													0.067589,-0.055758,0.060317,-0.064610,0.064940,-0.056920,0.060317, -0.077834,0.067589,-0.059953,
													0.059495,-0.047103},

					/* МУК2 */			{	0.021903,-0.031331,0.019507,-0.032801,0.023360,-0.020397,0.020464,-0.038365,0.025718,-0.030564,
														0.030850,-0.047934,0.035101,-0.033473,0.034545,-0.047209,0.038875,-0.038243,0.036422,-0.053903,
														0.036988,-0.040443,0.040063,-0.052047,0.038771,-0.049409,0.039078,-0.053191,0.047994,-0.048269,
														0.038198,-0.056207,0.053401,-0.048269,0.040938,-0.054341,0.046129,-0.046403,0.046370,-0.058074,
														0.050535,-0.050554,0.044521,-0.056661,0.051547,-0.047536,0.041808,-0.061599,0.044264,-0.052432,
														0.045516,-0.060417,0.048685,-0.055473,0.050907,-0.061114,0.050535,-0.048677,0.041808,-0.054341,
														0.055059,-0.056189,0.047220,-0.056661,0.053215,-0.051707,0.042672,-0.061599,0.050535,-0.051707,
														0.041808,-0.054283}};
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
const float	KuAB1[3]	= {64.538886, 64.25182, 64.25182};
const float	DuAB1[3]	= {-0.344533, -0.085494, 0.009307};

const float	KuAB2[3]	= {64.467300, 64.25182, 64.25182};
const float	DuAB2[3]	= {-0.265820, 0.176091, -0.019952};

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
float ItarD[nDatchD]	= {0.001997,  0.001999,  0.001988, 0.001989, 0.001997};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDD[3][nDatchD]		= {{0.055874,0.055947,0.055618,0.055437,0.055456},
																{0.055874,0.055958,0.055549,0.055438,0.055439},
																{0.055880,0.055977,0.055608,0.055459,0.055461}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDD[3][nDatchD]		= {{-0.008801,-0.008952,-0.009003,-0.008970,-0.009236},
																{ -0.008759,-0.008936,-0.008931,-0.009063,-0.009197},
																 {-0.008769,-0.008978,-0.009019,-0.009019,-0.009246}};								 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},


// ******************************** Датчики температуры: ********************************
const float	MinUdatchT = 0.005;															 
const float	MaxUdatchT = 3.18;
	
// ------------------------------- Паспортные данные
float R0[nDatchT]			= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 0°C (по паспорту);
//float R100[nDatchT]		= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 100°C (по паспорту);
												 
float alpha[nDatchT]	= {0.004149, 0.004149, 0.004149, 0.004149, 0.004149};			// Температурный коэффициент по паспорту.

// ------------------------------- Тарировочные данные																																	
float ItarT[nDatchT]	= {0.004017, 0.003998, 0.004000, 0.0040126, 0.004014};		// (A) Ток опроса датчиков ДТ в БЭ
																
const float	kDT[3][nDatchD]		= {{0.048737,0.048659,0.048698,0.048738,0.048733},	
																 {0.048739,0.048662,0.048653,0.048613,0.048608},
																 {0.048601,0.048662,0.048584,0.048589,0.048515}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDT[3][nDatchD]		= {{0.369218,0.369415,0.369017,0.368885,0.368686},	// 0.371696 dДт1 2-4 точка, 0.370504 2-5 точка
																 { 0.370195,0.370523,0.370198,0.370125,0.369912},
																 { 0.368872,0.368826,0.368725,0.368478,0.368464}};								 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},

const float	kTemp[3][5]	= {0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001,
													 0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001,
													 0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001};

//const float	dKT[3][nDatchD]	= {{ 0.01,   0.01,  0.010,  0.00,  0.01},
//															 {-0.018, -0.02, -0.018, -0.019, -0.02},
//															 { 0.0245, 0.024, 0.0245, 0.025,  0.024}};						// Коэфф динамической дельты датч темпертуры

//========== коррекция первой уставки ====================================================================
const float	LimUadc[3][nDatchD]	= {{ 0.9, 0.9, 0.9, 0.9, 0.9},
																		{0.9, 0.9, 0.9, 0.9, 0.9},
																		{0.9, 0.9, 0.9, 0.9, 0.9}};											// Порог напряжения корректировки темпертуры

const float	kLimUadc[3][nDatchD]	= {{0.0013, 0.0012, 0.0009, 0.0006, 0.0008},
																	 	 {0.0010, 0.0020, 0.0020, 0.0012, 0.0013},
																		 {0.0011, 0.0020, 0.0020, 0.0010, 0.0010}};						// Коэфф динамической дельты датч темпертуры

const float	kLimUadcT[3][nDatchD]	= {{0.000000, 0.000000, 0.000000, 0.000000, 0.000000},
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