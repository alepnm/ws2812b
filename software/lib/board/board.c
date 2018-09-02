
#include "stm32f0xx_hal.h"
#include "board.h"


#define LEDn    2

GPIO_TypeDef* LED_PORT[] = {LD3_GPIO_Port, LD4_GPIO_Port};
const uint16_t LED_PIN[] = {LD3_Pin, LD4_Pin};


/*  */
void BRD_LED_On(uint8_t led) {
    if(led >= LEDn) return;
    HAL_GPIO_WritePin(LED_PORT[led], LED_PIN[led], GPIO_PIN_SET);
}

/*  */
void BRD_LED_Off(uint8_t led) {
    if(led >= LEDn) return;
    HAL_GPIO_WritePin(LED_PORT[led], LED_PIN[led], GPIO_PIN_RESET);
}

/*  */
void BRD_LED_Toggle(uint8_t led) {
    if(led >= LEDn) return;
    HAL_GPIO_TogglePin(LED_PORT[led], LED_PIN[led]);
}
