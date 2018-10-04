
#include "globals.h"
#include "ws2812b.h"
#include "presets.h"


FlagStatus SendToLedsRequired = RESET;
FlagStatus BufferIsFilled = RESET;


Meteo_TypeDef MeteoBlue = {
    .Body = meteo_blue,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 10,
    .lPosition = 10
};

Meteo_TypeDef MeteoRed = {
    .Body = meteo_red,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 0,
    .lPosition = 0
};

Meteo_TypeDef MeteoGreen = {
    .Body = meteo_green,
    .Lenght = 10,
    .CurrentSpeed = 0,
    .CurrentDirection = FORWARD,
    .CurrentPosition = 0,
    .lPosition = 0
};


/* Statics */
static uint8_t dma_buffer[DMA_BUFFER_SIZE];   // taimerio PULSE reiksme vienam sviesdiodziui


/* Inicializavimas */
void WS2812B_Init(void) {

    uint16_t i = 0;

    /* isvalom buferi */
    while(i < DMA_BUFFER_SIZE) {

        if(i < RST_BYTES) {
            dma_buffer[i] = 0;
        } else {
            dma_buffer[i] = LOW_LVL;
        }

        i++;
    }
}


/*  */
void WS2812B_SendOverDMA(void) {

    if(SendToLedsRequired == RESET) return;

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
    SendToLedsRequired = RESET;
}


/* buferio uzpildymas. priima pointeri i preseto buferi */
void WS2812B_FillDMABuffer(const uint32_t* pdata) {

    uint16_t i = 0, j = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer+RST_BYTES;

    BufferIsFilled = RESET;

    /* ruosiam buferi */
    while(i < nLEDS) {

        color = *(pdata+i);

        while(j < 24) {
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
void WS2812B_SetPixel_RGB(uint16_t led, uint8_t red, uint8_t green, uint8_t blue) {

    uint8_t i = 0;

    if(red > MAX_LIGHT) red = MAX_LIGHT;
    if(green > MAX_LIGHT) green = MAX_LIGHT;
    if(blue > MAX_LIGHT) blue = MAX_LIGHT;

    uint32_t color = (green<<16) | (red<<8) | blue;

    uint8_t* pdata = WS2812B_GetLedBuferPointer(led);    // pointeris i reikiama ledo buferi

    while(i < 24) {
        *(pdata+i) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;
        color = color<<1;
        i++;
    }
}


/* vieno pikselio ledo busenos keitimas nekeiciant kitu ledu busenos */
void WS2812B_SetPixelLed(uint16_t led, uint8_t color, uint8_t bright) {

    uint8_t i = 0;

    if(bright > MAX_LIGHT) bright = MAX_LIGHT;

    uint8_t* pdata = WS2812B_GetLedBuferPointer(led) + color*8;

    while(i < 8) {
        *(pdata+i) = ( (bright & 0x80) == 0 ) ? LOW_LVL : HIGH_LVL;
        bright = bright<<1;
        i++;
    }
}




/*  */
void WS2812B_ShowMeteo(Meteo_TypeDef* meteo) {

    uint8_t j = 0;
    uint16_t i = 0;
    uint32_t color = 0;
    uint8_t* pdmabuf = dma_buffer + RST_BYTES;

    do {
        j = 0;


        if(meteo->CurrentDirection == FORWARD) color = meteo->Body[(meteo->Lenght-1) - i];
        else color = meteo->Body[i];

        while(j < 24) {

            if(meteo->CurrentDirection == FORWARD){
                //*( pdmabuf + (meteo->lPosition+i)*24 + j ) = LOW_LVL;     // gesinam sena pozicija
                *( pdmabuf + (meteo->CurrentPosition+i)*24 + j ) = ( (color & 0x800000) == 0 ) ? LOW_LVL : HIGH_LVL;   // rodom esama pozicija

            }else{



            }

            color = color<<1;
            j++;
        }

    } while(++i < meteo->Lenght);

    meteo->lPosition = meteo->CurrentPosition;

    if(meteo->CurrentDirection == FORWARD){
        meteo->CurrentPosition++;
        if(++meteo->CurrentPosition > nLEDS) meteo->CurrentPosition = 0;
    }else{
        meteo->CurrentPosition--;
        if(++meteo->CurrentPosition < 0) meteo->CurrentPosition = 0;
    }
}


/* grazina pasirinkto ledo 3 baitu pozicija ledu buferyje */
uint8_t* WS2812B_GetLedBuferPointer( uint16_t led) {
    return dma_buffer + (RST_BYTES + 24*(led-1) );
}


/*  */
void WS2812B_LedsOff(void) {

    uint16_t i = 0;
    uint8_t* pdata = dma_buffer + RST_BYTES;

    while(i < 24*nLEDS) {
        *(pdata+i) = LOW_LVL;
        i++;
    }

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*)dma_buffer, DMA_BUFFER_SIZE );
    HAL_Delay(10);
}
