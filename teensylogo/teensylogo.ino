extern "C" {
  #include "iopins.h"  
  #include "emuapi.h"  
}
#include "ili9341_t3dma.h"
#include "keyboard_osd.h"
#include <math.h>
#include "mcume.h"

ILI9341_t3DMA tft = ILI9341_t3DMA(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO, TFT_TOUCH_CS, TFT_TOUCH_INT);
static int xOffLogo=0;
static int swipeAngle=0;

void setup() {
  tft.begin();
  tft.flipscreen(true);  
  tft.start();
  
  emu_init(); 
}

static uint8_t col=0x00;

void loop(void) 
{
  uint16_t bClick = emu_DebounceLocalKeys();

  if (menuActive()) {
    int action = handleMenu(bClick);
    char * filename = menuSelection();    
    if (action == ACTION_RUNTFT) {
      //tft.setFrameBuffer((uint16_t *)malloc((ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH)*sizeof(uint16_t)));     
      Serial.print("TFT init: ");  
      //Serial.println(tft.getFrameBuffer()?"ok":"ko");       
      toggleMenu(false);
      tft.fillScreenNoDma(RGBVAL16(0x00,0x00,0x00));
      tft.refresh();
      tft.drawRect(0,ILI9341_TFTHEIGHT/2, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT/2, RGBVAL16(0x00,0x00,0x80));
      tft.drawText(0,ILI9341_TFTHEIGHT/2, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), true); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+16, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+24, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+32, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+40, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+48, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+56, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
      tft.drawText(0,ILI9341_TFTHEIGHT/2+64, "hello hello hello hello hello hello hell", RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), false); 
    }    
    else if (action == ACTION_RUNVGA)  {
      toggleMenu(false);
    }         
   
    delay(20);
  }
  else {
    //handleVirtualkeyboard();
    //if ( (!virtualkeyboardIsActive()) ) {     
      delay(20);
      //emu_printf("step");
      xOffLogo = 16*sin((2*3.14*swipeAngle)/256)+30;
      swipeAngle = (swipeAngle + 4)&0xff;
      tft.drawSprite(xOffLogo,10,(uint16_t*)logo);
    //}
  }  
}


