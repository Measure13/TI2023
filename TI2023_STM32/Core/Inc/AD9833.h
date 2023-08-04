//#ifndef __AD9833_H
//#define __AD9833_H

//#include "gpio.h"
//#include "stdbool.h"

//#define CS_0 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
//#define CS_1 HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

//#define FSYNC_0 HAL_GPIO_WritePin(GPIOA, SPI1_FSYNC_Pin, GPIO_PIN_RESET)
//#define FSYNC_1 HAL_GPIO_WritePin(GPIOA, SPI1_FSYNC_Pin, GPIO_PIN_SET)
//#define SCK_0 HAL_GPIO_WritePin(GPIOA, SPI1_SCK_Pin, GPIO_PIN_RESET)
//#define SCK_1 HAL_GPIO_WritePin(GPIOA, SPI1_SCK_Pin, GPIO_PIN_SET)
//#define DAT_0 HAL_GPIO_WritePin(GPIOA, SPI1_DAT_Pin, GPIO_PIN_RESET)
//#define DAT_1 HAL_GPIO_WritePin(GPIOA, SPI1_DAT_Pin, GPIO_PIN_SET)

//void AD9833_Init(void);
//void AD9833_Set_Amplitude(uint8_t Amp);
//void AD9833_Transmit(uint16_t content);
//void AD9833_Reset(void);
//void AD9833_Set_Freq(uint32_t output_freq);
//void AD9833_Set_Phase(uint16_t init_phase);
//void AD9833_Set(void);
//void AD9833_Default_Set(uint32_t output_freq);

//#endif

#ifndef _AD9833_H
#define _AD9833_H

#include "gpio.h"
#define TRI_WAVE 	0  		//输出三角波
#define SIN_WAVE 	1		//输出正弦波
#define SQU_WAVE 	2		//输出方波


void AD9833_WaveSeting(double frequence,unsigned int frequence_SFR,unsigned int WaveMode,unsigned int Phase, unsigned char channel );
void AD9833_WaveSeting_Double(double Freq,unsigned int Freq_SFR,unsigned int WaveMode,unsigned int Phase);

#endif

