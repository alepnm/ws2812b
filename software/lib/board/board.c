
#include "stm32f0xx_hal.h"
#include "board.h"


#define LEDn    4

GPIO_TypeDef* LED_PORT[] = {LD2_GPIO_Port, LD5_GPIO_Port, LD6_GPIO_Port, LD7_GPIO_Port,};
const uint16_t LED_PIN[] = {LD2_Pin, LD5_Pin, LD6_Pin, LD7_Pin};


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
