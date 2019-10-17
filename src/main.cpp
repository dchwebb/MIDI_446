#include "initialisation.h"
#include "USB.h"

USB usb;
uint32_t usbEvents[200];
uint8_t usbEventNo = 0;
uint8_t eventOcc = 0;
uint8_t mouseBuffer[4];
bool usbReady = false;

extern "C" {
#include "interrupts.h"
}


extern uint32_t SystemCoreClock;
int main(void)
{
	SystemInit();							// Activates floating point coprocessor and resets clock
	SystemClock_Config();					// Configure the clock and PLL - NB Currently done in SystemInit but will need updating for production board
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of oscillators, prescalers and PLL
	InitBtnLED();							// PC13 blue button; PB7 is LD2 Blue; PB14 is LD3 Red
	usb.InitUSB();

	int i = 0;

	while (1)
	{
		if (GPIOC->IDR & GPIO_IDR_IDR_13) {
			GPIOB->BSRR |= GPIO_BSRR_BS_7;
			usbReady = true;
//			GPIOB->BSRR |= GPIO_BSRR_BS_14;
		}
		else
			GPIOB->BSRR |= GPIO_BSRR_BR_7;

		mouseBuffer[0] = (GPIOC->IDR & GPIO_IDR_IDR_13) ? 2 : 0;

		if (usbReady) 		usb.SendReport (mouseBuffer, 4);
		i++;
	}
}

