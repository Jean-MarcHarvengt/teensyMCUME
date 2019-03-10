extern "C" {
  #include "iopins.h"  
  #include "emuapi.h"  
}
#include "keyboard_osd.h"
#include "ili9341_t3dma.h"
//extern "C" {
//#include "Colem.h"
//}

ILI9341_t3DMA tft = ILI9341_t3DMA(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO, TFT_TOUCH_CS, TFT_TOUCH_INT);

bool vgaMode = false;

static unsigned char  palette8[PALETTE_SIZE];
static unsigned short palette16[PALETTE_SIZE];
static IntervalTimer myTimer;
volatile boolean vbl=true;
static int skip=0;
static elapsedMicros tius;

static void vblCount() { 
  if (vbl) {
    vbl = false;
  } else {
    vbl = true;
  }
}

void emu_SetPaletteEntry(unsigned char r, unsigned char g, unsigned char b, int index)
{
  if (index<PALETTE_SIZE) {
    //Serial.println("%d: %d %d %d\n", index, r,g,b);
    palette8[index]  = RGBVAL8(r,g,b);
    palette16[index] = RGBVAL16(r,g,b);    
  }
}

void emu_DrawVsync(void)
{
  volatile boolean vb=vbl;
  skip += 1;
  skip &= VID_FRAME_SKIP;
  if (!vgaMode) {
    while (vbl==vb) {};
  }
  else {
  }
}

void emu_DrawLine(unsigned char * VBuf, int width, int height, int line) 
{
  if (!vgaMode) {
    tft.writeLine(width,1,line, VBuf, palette16);
  }
  else {
  }
}  

void emu_DrawScreen(unsigned char * VBuf, int width, int height, int stride) 
{
  if (!vgaMode) {
    if (skip==0) {
      tft.writeScreen(width,height-TFT_VBUFFER_YCROP,stride, VBuf+(TFT_VBUFFER_YCROP/2)*stride, palette16);
    }
  }
  else {
  }
}


// ****************************************************
// the setup() method runs once, when the sketch starts
// ****************************************************
void setup() {
  tft.begin();
  tft.flipscreen(true);  
  tft.start();
  
  emu_init(); 

  myTimer.begin(vblCount, 20000);  //to run every 20ms  
}

// ****************************************************
// the loop() method runs continuously
// ****************************************************
void loop(void) 
{
  uint16_t bClick = emu_DebounceLocalKeys();

  if (menuActive()) {
    int action = handleMenu(bClick);
    char * filename = menuSelection();    
    if (action == ACTION_RUNTFT) {
      //tft.setFrameBuffer((uint16_t *)malloc((ILI9341_TFTHEIGHT*ILI9341_TFTWIDTH)*sizeof(uint16_t)));     
      Serial.print("TFT init: ");  
      Serial.println(tft.getFrameBuffer()?"ok":"ko");       
      toggleMenu(false);
      vgaMode = false;   
      tft.fillScreenNoDma( RGBVAL16(0x00,0x00,0x00) );
      emu_Init(filename);      
      tft.refresh();
    }    
    else if (action == ACTION_RUNVGA)  {
      toggleMenu(false);
    }         
   
    delay(20);
  }
  else {
    //handleVirtualkeyboard();
    //if ( (!virtualkeyboardIsActive()) ) {     
      //emu_printf("step");
      //delay(20);
      
      //uint16_t * fb = tft.getLineBuffer(0);
      //for (int i=0;i<ILI9341_BLOCK;i++) fb[i]=RGBVAL16(col,0x00,0x00);
      //tft.fillScreen(RGBVAL16(col,0x00,0x00));
      //col++;
      
      emu_Step();
    //}
  }  
}

#ifdef HAS_SND

#include <Audio.h>
#include "AudioPlaySystem.h"

AudioPlaySystem mymixer;
AudioOutputAnalog dac1;
AudioConnection   patchCord1(mymixer, dac1);

void emu_sndInit() {
  Serial.println("sound init");  

  AudioMemory(16);
  mymixer.start();
}

void emu_sndPlaySound(int chan, int volume, int freq)
{
  if (chan < 6) {
    mymixer.sound(chan, freq, volume); 
  }
  
  /*
  Serial.print(chan);
  Serial.print(":" );  
  Serial.print(volume);  
  Serial.print(":" );  
  Serial.println(freq); 
  */ 
}

void emu_sndPlayBuzz(int size, int val) {
  mymixer.buzz(size,val); 
  //Serial.print((val==1)?1:0); 
  //Serial.print(":"); 
  //Serial.println(size); 
}
#endif




