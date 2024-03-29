/**
  ******************************************************************************
  * @file    system_MDR32F9x.h
  * @author  Phyton Application Team
  * @version V1.0.1
  * @date    10/28/2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Header File.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
  ******************************************************************************
  * FILE system_MDR32F9x.h
  */

/** @addtogroup __CMSIS CMSIS
  * @{
  */

/** @defgroup __MDR32F9x MDR32F9x
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_MDR32F9x_H
#define __SYSTEM_MDR32F9x_H

/** @addtogroup __MDR32F9x_System_Exported_Variables MDR32F9x System Exported Variables
  * @{
  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock)
                                           *   default value */

/** @} */ /* End of group __MDR32F9x_System_Exported_Variables */

/** @addtogroup __MDR32F9x_System_Exported_Functions MDR32F9x System Exported Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);

/** @} */ /* End of group __MDR32F9x_System_Exported_Functions */

#endif /*__SYSTEM_MDR32F9x_H */

/** @} */ /* End of group __MDR32F9x */

/** @} */ /* End of group __CMSIS */

/******************* (C) COPYRIGHT 2011 Phyton *********************************
*
* END OF FILE system_MDR32F9x.h */
