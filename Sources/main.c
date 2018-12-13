/*
` * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "MKL25Z4.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define CNT 100000

int16_t test[64] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,\
					33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64};
int16_t data[64];

int16_t ready;

float get_decayValue(float curr_value, float prev_value)
{
    float d_coff = 0.90, result = 0;

    if (curr_value < prev_value) {
        result = prev_value * d_coff;

        if (result < curr_value) {
            result = curr_value;
        }
    }
    else {
        result = curr_value;
    }

    return result;
}

void uart_config()
{
	uint16_t Baud;
	SIM_SCGC5 |= SIM_SCGC5_PORTA(1); // Enable port A as UART0 is part of port A
	SIM_SCGC4 |= SIM_SCGC4_UART0(1); // Enable UART0 through system clock gating register

	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); // Activates the UART0 clock
	PORTA_PCR1 |= PORT_PCR_MUX(2); // Enable alternative functionality 2 for allowing UART0 on PTA1
	PORTA_PCR2 |= PORT_PCR_MUX(2); // Enable alternative functionality 2 for allowing UART0 on PTA2
	UART0_C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK); // Disable Transmitter and Receiver

	UART0_S1 = 0;
	UART0_C1 = 0;
	UART0_C4 |= (0x0F & UART0_C4_OSR_MASK);

	//Baud = (uint16_t)(SystemCoreClock/(BAUD_RATE * 16)); // Setting Baud to 57600
	Baud = 12;

	UART0_BDH &= ~(UART0_BDH_SBR_MASK);
	UART0_BDH |= UART_BDH_SBR((Baud>>8));
	UART0_BDL &= ~(UART0_BDL_SBR_MASK);
	UART0_BDL |= UART_BDH_SBR(Baud);

	UART0_C2 |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK); // Enable Transmitter and Receiver

}

void uart_interrupt_config()
{
	NVIC_EnableIRQ(UART0_IRQn);
	UART0_C2 |= UART0_C2_RIE_MASK;
}

char uart_transmit()
{
	while(!(UART0->S1 & UART_S1_RDRF_MASK));

	return UART0_D;
}

void uart_receive(char tx_data)
{
	while(!(UART0_S1 & UART_S1_TDRE_MASK) && !(UART0_S1 & UART_S1_TC_MASK));

	UART0_D = (tx_data);
}

/* Code to print string */
void dispStr(char *data)
{
	while(*data)
	{
		uart_receive(*data);
		data++;
	}
}

void reverseItoa(char *revStr, int len)
{
	int i = 0; // Start index
	int j = len - 1; // Last index
	char temp;
	while(i<j)
	{
		temp = revStr[j];
		revStr[j] = revStr[i];
		revStr[i] = temp;
		i++;
		j--;
	}
}

/* Code to convert integers to ascii values */
char *my_itoa(int number, char *str, int base)
{
	int i =0,temp;
	bool isNegative = false;
	if(number == 0)
	{
		str[i++] ='0';
		str[i] = '\0';
	}

	if (number < 0 && base == 10)
	    {
	        isNegative = true;
	        number = -number;
	    }
	while(number!=0)
	{
		temp = number%base;
		str[i++] = temp + '0';  //(rem > 9)? (rem-10) + 'a' : rem + '0'
		number = number/base;
	}

	// If number is negative, append '-'
	    if (isNegative)
	        str[i++] = '-';

	str[i] = '\0';
	reverseItoa(str,i);
	return str;
}

void adc_config()
{
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK; // Enable clock gating for ADC0

	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; // Enable clock for portE

	PORTE_PCR20 = PORT_PCR_MUX(0);
	PORTE_PCR21 = PORT_PCR_MUX(0);

	ADC0_SC3 |= ADC_SC3_CAL_MASK;

	ADC0_CFG1 |= ADC_CFG1_ADICLK(0); // Input clock select (Bus clock)
	ADC0_CFG1 |= ADC_CFG1_MODE(3); // Conversion mode select 16 bit
	ADC0_CFG1 |= ADC_CFG1_ADIV(3); // Clock divide select (input clock / 8)

	ADC0_SC1A = 0;
	ADC0_SC1A |= ADC_SC1_DIFF(1);

	ADC0_SC2 = 0;
	ADC0_SC2 |= ADC_SC2_DMAEN_MASK; // DMA enable mask
	ADC0_SC3 |= ADC_SC3_AVGE_MASK;
	ADC0_SC3 |= ADC_SC3_ADCO_MASK; // Enable Continuous conversion

	ADC0_SC1A = (ADC_SC1_ADCH(0) | (ADC0_SC1A & ADC_SC1_DIFF_MASK)); // Enable Differential mode


}
void dma_config()
{
	ready = 0;
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	DMAMUX0_CHCFG0 = 0x00; // Disabling the DMA mmux

	DMA_SAR0 = (uint32_t)&ADC0_RA; // Assing DMA source address
	DMA_DAR0 = (uint32_t)&data; // Assign DMA destination address

	DMA_DSR_BCR0 |= DMA_DSR_BCR_BCR(128);
	DMA_DCR0 |= DMA_DCR_EINT_MASK | DMA_DCR_SSIZE(2) | DMA_DCR_DSIZE(2) | DMA_DCR_DINC_MASK;// | DMA_DCR_SINC_MASK | DMA_DCR_DMOD(3);

	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable the DMA channel 0  and set ADC0 as source

	NVIC_EnableIRQ(DMA0_IRQn); // Enable interrupt
	DMA_DCR0 |= DMA_DCR_START_MASK; // Start DMA
}

void DMA0_IRQHandler(void)
{
	NVIC_DisableIRQ(DMA0_IRQn);

	if(DMA_DSR_BCR_DONE_MASK)
	{
		GPIOB->PTOR |= (1<<3); // Toggle GPIO pin

		DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK; // Clear Done Flag
		DMA_DSR_BCR0 |= DMA_DSR_BCR_BCR(128);
		DMA_SAR0 = (uint32_t)&ADC0_RA;
		DMA_DAR0 = (uint32_t)&data;
		DMA_DCR0 |= DMA_DCR_START_MASK;
		dispStr("Fetching \r\n");
		ready = 1;
	}
}

void compute_logValue(int16_t log_table[])
{
    int i = 1;

    for (i = 1; i <= 100; i++) {
        log_table[i] = (20 * (log10(i) - log10(100)));
    }
}

int main(void)
{
	uint32_t counter = 0;
	int16_t log_table[100];
	int i;
	float result = 0;
	float c_decay = 0;
	float p_decay = 0;
	char val;
	char s[100];
	char t[10];
	float logCal;
	adc_config();
	dma_config();
	uart_config();

	compute_logValue(log_table);
	dispStr("Welcome\r\n");

	while(1)
	{
		if (ready) {
			for(i=0;i<64;i++)
			{
				dispStr(my_itoa(data[i],s,10));
								dispStr("\r\n");
				if (result < abs(data[i])) {
					result = abs(data[i]);
				}

			}
			sprintf(t, "%0.2f", result);
			dispStr(t);
			dispStr("\t");
			c_decay = get_decayValue(result, p_decay);
			sprintf(t, "%0.2f", c_decay);
			dispStr("Decay Value = ");
			dispStr(t);
			dispStr("\t");
			dispStr("Log Value = ");
			logCal = (result/32767) * 100;
			logCal = log_table[abs((int)logCal)];
			sprintf(t, "%0.2f",logCal);
			dispStr(t);
			dispStr("\r\n");
			dispStr("\r\nDone \r\n");
			p_decay = c_decay;
			ready = 0;
			result = 0;
			NVIC_EnableIRQ(DMA0_IRQn);
		}
	}
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
