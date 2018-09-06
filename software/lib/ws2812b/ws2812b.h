#ifndef WS2812B_H_INCLUDED
#define WS2812B_H_INCLUDED


#define nLEDS       20

#define MAX_LIGHT   0xA0    // sviesos diodo MAX ryskumas

#define TMR_COUNTER 60  //60 -> 1.25us
#define LOW_LVL     18  // loginis 0
#define HIGH_LVL    44  // loginis 1
#define RST_BYTES   46

#define DMA_BUFFER_SIZE   (RST_BYTES + nLEDS*24 + 1)

enum { FORWARD = 0, BACKWARD };
enum { GREEN = 0, RED, BLUE };


typedef struct{
    const uint32_t*     Body;       // pointeris i objekto forma/ilgi ir spalva
    uint8_t             Lenght;
    uint8_t             CurrentSpeed;      // speed    (0-7)
    uint8_t             CurrentDirection;  // kriptis  (FORWARD/BACK)
    int16_t             CurrentPosition;   // nuo pirmo sviesos diodo iki nLEDS. nurodo objekto pradzia. int todel, kad objekto pozicija gali buti uz buferio ribu, bet dar reikia rodyti objekto "uodega"
}Meteo_TypeDef;




extern FlagStatus SendToLedsRequired;
extern FlagStatus BufferIsFilled;

extern Meteo_TypeDef MeteoRed;
extern Meteo_TypeDef MeteoGreen;
extern Meteo_TypeDef MeteoBlue;



void WS2812B_Init(void);
void WS2812B_SendOverDMA(void);

void WS2812B_LedsOff(void);
void WS2812B_FillDMABuffer(const uint32_t* pdata);
void WS2812B_SetPixel_RGB(uint16_t led, uint8_t red, uint8_t green, uint8_t blue);
void WS2812B_SetPixelLed(uint16_t led, uint8_t color, uint8_t bright);
uint8_t* WS2812B_GetLedBuferPointer( uint16_t led);

void WS2812B_ShowMeteo(Meteo_TypeDef* meteo);

#endif /* WS2812B_H_INCLUDED */
