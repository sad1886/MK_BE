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
const int	kodADC_t25[3]	= {1600, 1670, 1655};

const float	dTempC[3]	= {0.0, 0.0, 0.0};											// (Грд Цельс) дельта датчика температуры МК

const float kdelta	= 0.05;																			// Допустимый диапазон разброса ошибки 5% от диапазона измерений

			float	AUcc[3]	= {3.295, 3.277, 3.287};										// (В) Опорное напряжение АЦП МУК1, МУК2, МУК3
const float	Uref[3]	= {3.0201, 3.0238, 3.0296};										// (В) Прецизионное напряжение АЦП МУК1, МУК2, МУК3 на PD0

						 
// .............................................................................................................................
// *** М И Б *******************************************************************************************************************

const int nCorrectAE 	 = 2;																			// Числ пропусков ложного значения напряжений АЭ
const int koefZadergAE = 2;//4;																	// Коэфф задержки после коммутации дешифр АЭ (koefZadergAE*10000)

const float kTempAE[3]	= {0.00005001, 0.00010001, 0.00000001};                                                          // {0.00020001, 0.00015001, 0.00000001};	// Температурная коррекция АЭ

const float kTempAB[3]	= {0.00000001, 0.00000001, 0.00000001};	// Температурная коррекция АБ

//float minUAE = 0.6;																							// (В) Нижняя граница значения рабочего напряжения АЭ
//float maxUAE = 2.0;																							// (В) Верхняя граница значения рабочего напряжения АЭ

//														 1				 2				3				 4				5				 6				7				 8				9				10
const float	kAE_[2][72]	={{	/* МУК1 */
														  2.510748,2.214428 ,2.494244,2.194893,2.446008,2.227101,2.407214,2.267019,2.362254,2.260268,
	                            2.3549242,2.273811,2.347639,2.308396,2.347639,2.287520,2.347639,2.308396,2.347638,2.315437,
	                            2.333202,2.308396,2.333203,2.315438,2.340398,2.308396,2.326051,2.315437,2.318943,2.315438,
	                            2.336802,2.318970,2.329628,2.326077,2.336802,2.311906,2.344019,2.326077,2.329628,2.347665,
	                            2.322499,2.340424,2.322498,2.340424,2.322498,2.340425,2.315412,2.347665,2.308369,2.333230,
	                            2.308369,2.340425,2.301369,2.340424,2.308369,2.318970,2.322498,2.327801,2.316433,2.320652,                                             //2.530452,2.220004,2.465911,2.270316,2.421212,2.251183,2.399465,2.296335,2.371069,2.302934,
												      2.323554,2.327801,2.323553,2.327795,2.330714,2.327801,2.316435,2.320651,2.330712,2.334988,2.316431,2.225022},                           //2.350210,2.336504,2.336505,2.322959,2.329713,2.336504,2.406671,2.329712,2.329713,2.350208,
                                                                                                                                                                     //2.343337,2.357121,2.336505,2.329712,2.350209,2.283251,2.343338,2.371069,2.329713,2.385182,
													                                                                                                                                           //2.329713,2.343336,2.336505,2.343336,2.329713,2.350208,2.329713,2.343336,2.329713,2.336504,
													                                                                                                                                           //2.336505,2.343336,2.329713,2.357120,2.336505,2.302934,2.350209,2.343336,2.336505,2.357120,
													                                                                                                                                           //2.406671,2.336504,2.329713,2.350208,2.322960,2.343336,2.329713,2.350208,2.316246,2.357120,2.336505,2.251182},

//												     1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,
//										      	 1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		
//											    	 1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		
//												     1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		
//												     1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		
//												     1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,		
//												     1.0, 1.0,		1.0, 1.0,   1.0, 1.0,   1.0, 1.0,		1.0, 1.0,	1.0, 1.0},
															
														   //* МУК2 */......................................................................................*/
														{ 
					                     2.511413,2.168788,2.470895,2.199941,2.439410,2.245090,2.378787,2.285290,2.419058,2.245090,									// 2.493089,2.220370,2.435029,2.251381,2.398794,2.283272,2.370573,2.316077,2.391676,2.309441,
							                 2.349592,2.292131,2.356823,2.278491,2.364100,2.305936,2.371421,2.292131,2.356824,2.305935,									// 2.343008,2.329464,2.356710,2.316078,2.349839,2.336216,2.363621,2.329465,2.336216,2.336216,
							                 2.349592,2.299012,2.349592,2.305935,2.364099,2.299011,2.349592,2.305935,2.349592,2.312901,									// 2.349839,2.329465,2.336217,2.336216,2.343008,2.329465,2.343008,2.336216,2.349839,2.329465,
							                 2.349592,2.312900,2.364100,2.312900,2.335262,2.305935,2.349592,2.305935,2.335261,2.312900,									// 2.343008,2.343009,2.349839,2.329465,2.343008,2.343009,2.356710,2.336217,2.336216,2.343009,
						                   2.342405,2.305936,2.342405,2.326956,2.342405,2.312900,2.335261,2.334049,2.349592,2.312900,										// 2.343008,2.343008,2.329465,2.349839,2.349839,2.343008,2.309440,2.336218,2.322752,2.435030,
						                   2.328162,2.326956,2.342405,2.305936,2.335262,2.326956,2.342405,2.312900,2.335262,2.319908,									// 2.336216,2.363622,2.336217,2.336217,2.329465,2.349840,2.343008,2.343008,2.336216,2.356711,
			                         2.328162,2.319907,2.335262,2.326956,2.342405,2.319907,2.321105,2.319907,2.342405,2.319907,2.342405,2.232003}};	// 2.329465,2.377566,2.316077,2.349839,2.343008,2.343008,2.329465,2.349840,2.356710,2.343008,2.336216,2.251382}};
					
// .............................................................................................................................
//														 1				 2				3					 4				5					6			  7					 8				9				 10
const float	dAE_0[2][72]	={	/* МУК1 ------------------------------------------------------------------------------------*/
													{	
														
						                 -0.056837, 0.029820,-0.056871, 0.036203,-0.025680, 0.031850,-0.019994, 0.008153,0.009441, 0.017432,								                             //0.079651, 0.004928,0.068895, 0.002266,0.081395,-0.047611,0.055590,-0.055168,0.075772,-0.043681,		//0.059651
		                          0.002163, 0.005181, 0.013695,-0.001646, 0.005250,-0.004470, 0.013695,-0.001646,0.003375,-0.013030, 	                                           //0.043396,-0.070275,0.100043,-0.050005,0.058197,-0.100094,0.085275,-0.056332,0.072140,-0.082635,
													    0.017897,-0.001646, 0.011367,-0.014880, 0.012999,-0.001646, 0.012540,-0.013030,0.022047, 0.000845, 	                                           //0.075275,-0.061798,0.054459,-0.088254,0.098077,-0.050308,0.070283,-0.038827,0.074031,-0.052659,
													    0.005175,-0.013648, 0.016609,-0.006530, 0.003309,-0.008715, 0.012406,-0.004673,0.008229,-0.026191,                  	                         //0.051106,-0.080091,0.090862,-0.050308,0.051576,-0.095831,0.063030,-0.043652,0.071316,-0.102777,
													    0.015909,-0.009001, 0.013119,-0.019289, 0.019619,-0.012739, 0.016130,-0.024317,0.027426,-0.007763,                  	                         //0.068845,-0.058092,0.083557,-0.101425,0.087180,-0.052002,0.058425,-0.085772,0.092538,-0.048986,
													    0.017279,-0.021158, 0.028538,-0.009001, 0.015436,-0.006241, 0.014055,-0.014271,0.018957,-0.021356,	                                           //0.053168,-0.092065,0.100914,-0.084929,0.079164,-0.083919,0.096383,-0.049848,0.048887,-0.087625,
										          0.024299,-0.014271, 0.017797,-0.024500, 0.019431,-0.014271, 0.020806,-0.023209,0.017571,-0.019267,0.017108,-0.011250},				                 //0.076289,-0.056886,0.062343,-0.092102,0.087867,-0.043005,0.078050,-0.068928,0.100893,-0.056886,0.063557,-0.101841
														
														
														
														
//													  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,		
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,
//														0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.0},
													
														
														   
														
													{		/* МУК2 ------------------------------------------------------------------------------------*/
		                         -0.081315, 0.050015,-0.041223, 0.065238,-0.052244, 0.018370,-0.000527, 0.027164,-0.045380, 0.018370,											          // 0.013846,02.511413 .055656,0.003684,0.030703,0.010939,-0.005552,0.009718,0.026868,0.015053,0.019870,
		                          0.009893, 0.027880,-0.014755, 0.007372, 0.000038, 0.020200,-0.021040,-0.000385, 0.004981, 0.023850,											          //	0.012708,0.014734,0.002300,0.011224,0.008991,-0.004689,0.032314,-0.001230,0.021570,-0.009525,
		                         -0.015361,-0.003388, 0.006174, 0.022025,-0.023501, 0.001568, 0.008034, 0.022025,-0.015361,-0.009449,											          //	0.029419,0.007195,0.022652,0.008939,0.018368,0.008307,0.011244,-0.001709,0.019870,-0.001230,
	                            0.006174, 0.017241,-0.025372,-0.007619, 0.012236, 0.022025,-0.015361, 0.000890, 0.010388, 0.026393,											          //	0.021570,0.001945,0.017270,0.015205,0.013092,0.001945,0.028642,0.023459,0.024798,0.001945,
		                         -0.017816,-0.000935, 0.005507, 0.018635,-0.014108,-0.005788, 0.010388, 0.011949,-0.019079,-0.005788,											          //	0.014685,0.004249,0.011244,0.006718,0.020210,-0.002378,0.005274,0.012559,0.011336,0.006075,
		                          0.015239, 0.016793,-0.014108,-0.000935, 0.008540, 0.016793,-0.015962,-0.003958, 0.006692, 0.017935,											          //	0.034961,-0.014938,0.000886,0.019537,0.020816,0.022990,0.032577,-0.008379,0.011244,0.013279,
		                         -0.006100,-0.008835, 0.008540, 0.016793,-0.015962,-0.008835, 0.016387, 0.019772,-0.014108,-0.010671,.001800,0.023353 											//	0.011728,0.007195,0.031321,-0.000339,0.016527,0.004949,0.009397,0.000807,0.022377,0.004949,0.016044,-0.016569
													}};
const float	dAE_w[2][72]	={	/* МУК1 ------------------------------------------------------------------------------------*/
													{
														0.089651,-0.004928,0.083895,0.007266,0.061395,-0.017611,0.025590,-0.035168,0.055772,-0.003681,
														0.033396,-0.030275,0.080043,-0.020005,0.038197,-0.080094,0.065275,-0.016332,0.042140,-0.042635,
														0.065275,-0.050798,0.034459,-0.038254,0.068077,-0.010308,0.040283,-0.018827,0.054031,-0.012659,
														0.041106,-0.040091,0.070862,-0.010308,0.051576,-0.055831,0.053030,-0.023652,0.041316,-0.062777,
														0.058845,-0.018092,0.053557,-0.061425,0.067180,-0.012002,0.038425,-0.045772,0.062538,-0.008986,
														0.043168,-0.042065,0.080914,-0.044929,0.059164,-0.043919,0.076383,-0.029848,0.048887,-0.047625,
														0.066289,-0.016886,0.042343,-0.070102,0.057867,-0.003005,0.048050,-0.048928,0.070893,-0.016886,0.053557,-0.081841
													},
													{		/* МУК2 ------------------------------------------------------------------------------------*/
														0.013846,0.055656,0.003684,0.030703,0.010939,-0.005552,0.009718,0.026868,0.015053,0.019870,
														0.012708,0.014734,0.002300,0.011224,0.008991,-0.004689,0.032314,-0.001230,0.021570,-0.009525,
														0.029419,0.007195,0.022652,0.008939,0.018368,0.008307,0.011244,-0.001709,0.019870,-0.001230,
														0.021570,0.001945,0.017270,0.015205,0.013092,0.001945,0.028642,0.023459,0.024798,0.001945,
														0.014685,0.004249,0.011244,0.006718,0.020210,-0.002378,0.005274,0.012559,0.011336,0.006075,
														0.034961,-0.014938,0.000886,0.019537,0.020816,0.022990,0.032577,-0.008379,0.011244,0.013279,
														0.011728,0.007195,0.031321,-0.000339,0.016527,0.004949,0.009397,0.000807,0.022377,0.004949,0.016044,-0.016569
													}};

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
const float	KuAB1[3]	= {64.211986 , 64.426245, 63.766868 };
const float	DuAB1[3]	= { 0.284171,   0.081076,   0.853970};

const float	KuAB2[3]	= {64.211986 , 64.569411, 63.840293};
const float	DuAB2[3]	= { 0.284171,  -0.024430, 0.876000};

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
float ItarD[nDatchD]	= {0.001971,  0.001977,  0.001975, 0.001975, 0.001968};		// (A) Ток опроса датчиков ДД в БЭ

const float	kDD[3][nDatchD]		= {{0.050717,0.050724,0.050735,0.050722,0.050796},
																 {0.050785, 0.050790, 0.050780, 0.050781, 0.050768},       
																 {0.050648, 0.050689, 0.050661, 0.050663 , 0.050702 }};                                              
//																 {1.0, 1.0, 1.0, 1.0, 1.0},

const float	dDD[3][nDatchD]		= {{-0.007266,-0.007290,-0.007023,-0.007184,-0.007000},
															{-0.006876, -0.006931, -0.006732,  -0.006625,  -0.006518},                              
														 { -0.006803, -0.006952, -0.006703, -0.006703, -0.006666}};		                                                         
//																 {0.0, 0.0, 0.0, 0.0, 0.0},
																 

// ******************************** Датчики температуры: ********************************
const float	MinUdatchT = 0.001;															 
const float	MaxUdatchT = 3.18; 
															 
// ------------------------------- Паспортные данные
float R0[nDatchT]			= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 0°C (по паспорту);
//float R100[nDatchT]		= {99.94, 99.94, 99.94, 99.94, 99.94};										// (ом) Сопротивление датчика при t = 100°C (по паспорту);
												 
float alpha[nDatchT]	= {0.004149, 0.004149, 0.004149, 0.004149, 0.004149};			// Температурный коэффициент по паспорту.

// ------------------------------- Тарировочные данные																																	
float ItarT[nDatchT]	= {0.004019, 0.003977, 0.004003, 0.004005, 0.003997};		// (A) Ток опроса датчиков ДТ в БЭ

const float	kDT[3][nDatchD]		= {{0.043893 , 0.043918 , 0.043892 , 0.043848, 0.043961},                                       //{{0.044094,0.044123,0.044143,0.044317,0.044398},
																  {0.043982, 0.044077, 0.044064, 0.043900, 0.044000},                                         //{0.044168,0.044138,0.044160,0.044125,0.044184},
																    {0.043928, 0.044028, 0.043973, 0.043944, 0.043976,}};                                     //{0.042501,0.042455,0.042421,0.042423,0.042498}};
//																 {1.0, 1.0, 1.0, 1.0, 1.0},};
														
const float	dDT[3][nDatchD]		={{ 0.370889,  0.370907,  0.3706680, 0.370572, 0.370811},                                     //{{0.371285,0.370949,0.371017,0.370491,0.370311},
																   { 0.372539, 0.372485,  0.372107, 0.372212, 0.372519},                                    //{0.372777,0.372699,0.372575,0.372698,0.372337},
																     {0.371609,  0.371497, 0.371432, 0.371338, 0.371729}};                                  //{0.371999,0.371962,0.372067,0.372073,0.371758}};
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
																		{0.9, 0.9, 0.9, 0.9, 0.9}};											// пороговое напряжение АЦП для коррекции 1 уставки датч темпертуры

const float	kLimUadc[3][nDatchD]	= {{0.0035, 0.0070, 0.004, 0.0027, 0.004},
																	 	 {0.0035, 0.0070, 0.004, 0.0037, 0.0037},
																		 {0.0035, 0.0073, 0.0045, 0.003, 0.004}};						// Коэфф динамической дельты датч темпертуры

const float	kLimUadcT[3][nDatchD]	= {{0.000005, 0.000005, 0.000005, 0.000005, 0.000009},
	                                   {0.000005, 0.000005, 0.000005, 0.000005, 0.000005},                      // Коэфф динамической дельты датч темпертуры для 1 уставки 
																		 {0.0, 0.000005, 0.000005, 0.000005, 0.000005}};																			 

//========== коррекция пятой уставки ====================================================================
const float	LimUadc_5[3][nDatchD]	= {{ 3.3, 3.3, 3.3, 3.3, 3.3},
																			{3.3, 3.3, 3.3, 3.3, 3.3},                // ограничение напряжения для АЦП температуры
																			{3.3, 3.3, 3.3, 3.3, 3.3}};

const float	kLimUadc_5[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэфф динамической дельты датч темпертуры для 2-4 уставки

const float	kLimUadc_5_T[3][nDatchD]	= {{0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0},
																			{ 0.0, 0.0, 0.0, 0.0, 0.0}};						// Коэф динамической дельты датч темпертуры для 5 уставки


#endif
