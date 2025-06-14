/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H17R1_spi.h
 Description: Header for SPI configuration functions.
 SPI: Declares SPI1 handle and initialization function prototypes.
*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Variables ******************************************************/
extern SPI_HandleTypeDef  hspi1;

/* Exported Functions ******************************************************/
void MX_SPI_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
