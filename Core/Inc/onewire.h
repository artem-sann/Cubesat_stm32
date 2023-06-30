#ifndef __ONEWIRE_H__
#define __ONEWIRE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);
uint16_t DS18B20_Getemp(void);

#ifdef __cplusplus
}
#endif
#endif
