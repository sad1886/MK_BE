/**********************************************************************
*																																			*
*		Файл параметров для АБ 72НВ-40 ЖЦПИ.563532.011 зав. № ________		*
*																																			*
***********************************************************************/

#ifndef PARAMETERS_A3001003_H
#define PARAMETERS_A3001003_H

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
const int	kodADC_t25[3]	= {1711, 1811, 1831};

const float	dTempC[3]	= {-0.00, 0.0, 0.0};	// (Грд Цельс) дельта датчика температуры МК

const float kdelta	= 0.05;											// Допустимый диапазон разброса ошибки 5% от диапазона измерений

			float	AUcc[3]	= {3.266, 3.265, 3.272};		// (В)  напряжение АЦП МУК1, МУК2, МУК3
const float	Uref[3]	= {3.014, 3.023, 3.019};		// (В) опрное напряжение АЦП МУК1, МУК2, МУК3 на PD0


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
					/* МУК1 */				2.457748,2.256695,2.435497,2.263025,2.442870,2.269388,2.413646,2.269388,2.413646,2.269388,
														2.392184,2.282223,2.366567,2.264476,2.352661,2.270897,2.359594,2.277356,2.345769,2.277356,
														2.352661,2.303560,2.352661,2.290383,2.345769,2.290382,2.332106,2.296953,2.338918,2.283851,
														2.345770,2.303560,2.356315,2.296952,2.338918,2.296952,2.338918,2.296951,2.325334,2.283851,
														2.311907,2.310205,2.318601,2.303561,2.325334,2.316890,2.318601,2.303561,2.325334,2.316889,
														2.325334,2.316889,2.311907,2.330375,2.311907,2.296952,2.325334,2.323613,2.318601,2.303561,
														2.311907,2.316888,2.318601,2.310206,2.305252,2.323612,2.325334,2.316889,2.318601,2.316889,
														2.318601,2.220521},

	// ...................................................................................................
					
					/* МУК2 */			 {2.466100,2.245989,2.443701,2.264890,2.436327,2.258558,2.414461,2.290598,2.414463,2.284121,
2.378887,2.310263,2.378887,2.316903,2.378884,2.316898,2.378889,2.323569,2.364946,2.323565,
2.371899,2.330276,2.364948,2.330274,2.351173,2.343810,2.358041,2.330272,2.364952,2.330276,
2.351171,2.330268,2.344346,2.323567,2.337558,2.330264,2.358041,2.323569,2.337558,2.337019,
2.330808,2.330274,2.344343,2.337015,2.337556,2.343806,2.344345,2.337017,2.344346,2.343808,
2.337558,2.350621,2.324099,2.343804,2.330804,2.337017,2.330806,2.343806,2.337556,2.350630,
2.324097,2.343808,2.330808,2.343796,2.330808,2.343806,2.330806,2.337017,2.324101,2.343800,
2.337558,2.246013}};
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
													{ 0.020260,-0.034737,0.026058,-0.049284,0.024124,-0.040444,0.030771,-0.041329,0.031713,-0.029596,
														0.039212,-0.045288,0.048324,-0.029215,0.054755,-0.038390,0.052949,-0.034955,0.055597,-0.041286,
														0.057547,-0.050304,0.052881,-0.052603,0.056512,-0.040760,0.059123,-0.053735,0.062936,-0.037850,
														0.057465,-0.056708,0.045184,-0.043689,0.058297,-0.051905,0.057345,-0.040028,0.058091,-0.044199,
														0.062475,-0.042232,0.066303,-0.053038,0.060851,-0.048883,0.068150,-0.054874,0.058999,-0.047036,
														0.063650,-0.059015,0.068000,-0.060441,0.065257,-0.046415,0.062704,-0.055571,0.066303,-0.054874,
														0.068000,-0.054419,0.062608,-0.054178,0.070626,-0.051867,0.059944,-0.060862,0.063512,-0.047036,
														0.064455,-0.052951},

					/* МУК2 */			{0.001904,-0.025690,0.010291,-0.032809,0.013712,-0.033118,0.023818,-0.044305,0.024308,-0.044584,
0.034115,-0.049417,0.036494,-0.061135,0.037907,-0.056066,0.034597,-0.060428,0.041543,-0.057196,
0.037375,-0.061575,0.039657,-0.062047,0.045612,-0.069492,0.040525,-0.060190,0.038252,-0.061575,
0.045135,-0.056474,0.046457,-0.056724,0.046823,-0.052759,0.042883,-0.060428,0.046823,-0.063201,
0.051848,-0.059717,0.047850,-0.059475,0.051024,-0.065755,0.045982,-0.061338,0.046457,-0.067623,
0.046823,-0.061781,0.052668,-0.063887,0.053234,-0.061338,0.053706,-0.065755,0.048687,-0.069277,
0.054521,-0.067623,0.049518,-0.058756,0.051848,-0.065755,0.051376,-0.061338,0.050816,-0.060150,
0.046823,-0.056580}};
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
const float	KuAB1[3]	= {64.435583, 64.400736, 64.39395};
const float	DuAB1[3]	= { -0.258021, -0.243950, -0.180370};

const float	KuAB2[3]	= {64.409430, 64.429353, 64.344427};
const float	DuAB2[3]	= {-0.210527, -0.224102, -0.193235};

// ******************************** Датчики давления: ********************************
const float	MinUdatchD = 0.05;															 
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
float ItarD[nDatchD]	= {0.001980,  0.0019827,  0.001975, 0.001989, 0.0019746};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDD[3][nDatchD]		= {{0.055810,0.055939,0.055681,0.055490,0.055519},
																 {0.055805,0.055913,0.055650,0.055482,0.055430},
																 {0.055742,0.055867,0.055584,0.055415,0.055438}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDD[3][nDatchD]		= {{-0.008647,-0.008800,-0.009043,-0.008836,-0.009127},
																 {-0.008651,-0.008768,-0.009039,-0.008828,-0.009102},
																 {-0.008683,-0.008797,-0.009061,-0.008851,-0.009125}};								 
//																 {0.0, 0.0, 0.0, 0.0, 0.0},


// ******************************** Датчики температуры: ********************************
const float	MinUdatchT = 0.005;															 
const float	MaxUdatchT = 3.18;
	
// ------------------------------- Паспортные данные
float R0[nDatchT]			= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 0°C (по паспорту);
//float R100[nDatchT]		= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 100°C (по паспорту);
												 
float alpha[nDatchT]	= {0.004149, 0.004149, 0.004149, 0.004149, 0.004149};			// Температурный коэффициент по паспорту.

// ------------------------------- Тарировочные данные																																	
float ItarT[nDatchT]	= {0.003996, 0.003993, 0.004006, 0.004000, 0.003990};		// (A) Ток опроса датчиков ДТ в БЭ
																
const float	kDT[3][nDatchD]		= {{0.048609,0.048485,0.048692,0.048569,0.048774},	
																 {0.048603,0.048560,0.048668,0.048626,0.048837},
																 {0.048632,0.048543,0.048629,0.048586,0.048681}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDT[3][nDatchD]		= {{0.369050,0.369236,0.368480,0.368731,0.368619},	// 0.371696 dДт1 2-4 точка, 0.370504 2-5 точка
																 {0.370370,0.370416,0.369815,0.369914,0.369588},
																 {0.369421,0.369597,0.369092,0.369194,0.369268}};								 
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

const float	kLimUadc[3][nDatchD]	= {{0.0015, 0.0009, 0.0009, 0.0006, 0.0008},
																	 	 {0.0017, 0.0017, 0.0012, 0.0015, 0.0015},
																		 {0.0017, 0.0015, 0.0011, 0.0015, 0.0016}};						// Коэфф динамической дельты датч темпертуры

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
