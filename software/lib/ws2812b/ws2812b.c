
#include "stm32f0xx_hal.h"
#include "ws2812b.h"
#include "presets.h"



extern TIM_HandleTypeDef htim16;

uint8_t ErrCode = 0;
FlagStatus SendToLedsRequired = RESET;
FlagStatus BufferIsFilled = RESET;


Meteo_TypeDef MeteoBlue = {
    .Body = meteo_blue,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 0
};

Meteo_TypeDef MeteoRed = {
    .Body = meteo_red,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 0
};

Meteo_TypeDef MeteoGreen = {
    .Body = meteo_green,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 0
};


/* Statics */
static uint8_t dma_buffer[DMA_BUFFER_SIZE];   // taimerio PULSE reiksme vienam sviesdiodziui


/* Inicializavimas */
void WS2812B_Init(void){

    uint16_t i = 0;
    uint8_t* pdata = dma_buffer + RST_BYTES;

    /* isvalom buferi */
    while(i < DMA_BUFFER_SIZE){

       if(i < RST_BYTES) *(pdata+i) = 0;
       else *(pdata+i) = LOW_LVL;

       i++;
    }
}


/*  */
void WS2812B_SendOverDMA(void){

    if(SendToLedsRequired == RESET) return;

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
    SendToLedsRequired = RESET;
}


/* buferio uzpildymas. priima pointeri i preseto buferi */
void WS2812B_FillDMABuffer(const uint32_t* pdata){

    uint16_t i = 0, j = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer+RST_BYTES;

    BufferIsFilled = RESET;

    /* ruosiam buferi */
    while(i < nLEDS){

        color = *(pdata+i);

        while(j < 24){
            *(pdmabuf+j+24*i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
            color = color<<1;
            j++;
        }

        j = 0;
        i++;
    }

    BufferIsFilled = SET;
}


/* visu triju pikselio ledu nustatymas */
void WS2812B_SetPixel_RGB(uint16_t led, uint8_t red, uint8_t green, uint8_t blue){

    uint8_t i = 0;

    if(red > MAX_LIGHT) red = MAX_LIGHT;
    if(green > MAX_LIGHT) green = MAX_LIGHT;
    if(blue > MAX_LIGHT) blue = MAX_LIGHT;

    uint32_t color = (green<<16) | (red<<8) | blue;

    uint8_t* pdata = WS2812B_GetLedBuferPointer(led);    // pointeris i reikiama ledo buferi

    while(i < 24){
        *(pdata+i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
        color = color<<1;
        i++;
    }
}


/* vieno pikselio ledo busenos keitimas nekeiciant kitu ledu busenos */
void WS2812B_SetPixelLed(uint16_t led, uint8_t color, uint8_t bright){

    uint8_t i = 0;

    if(bright > MAX_LIGHT) bright = MAX_LIGHT;

    uint8_t* pdata = WS2812B_GetLedBuferPointer(led) + color*8;

    while(i < 8){
        *(pdata+i) = ( (bright & 0x80) == 0 ) ? LOW_LVL : HIGH_LVL;
        bright = bright<<1;
        i++;
    }
}




/*  */
void WS2812B_ShowMeteo(Meteo_TypeDef* meteo) {

    uint8_t i = 0, j = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer + RST_BYTES;

    do {

        color = *( meteo->Body + i);

        if(meteo->CurrentDirection == FORWARD) {

            j = 0;

            while(j < 24){
                *(pdmabuf+j+24*i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
                color = color<<1;
                j++;
            }

        } else {


        }

    } while(++i < meteo->Lenght);
}


/* grazina pasirinkto ledo 3 baitu pozicija ledu buferyje */
uint8_t* WS2812B_GetLedBuferPointer( uint16_t led){
    return dma_buffer + (RST_BYTES + 24*(led-1) );
}


/*  */
void WS2812B_LedsOff(void){

    uint16_t i = 0;
    uint8_t* pdata = dma_buffer + RST_BYTES;

    while(i < 24*nLEDS){
        *(pdata+i) = LOW_LVL;
        i++;
    }

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
    HAL_Delay(10);
}
