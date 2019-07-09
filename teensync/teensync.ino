#include "iopins.h"  
#include "emuapi.h"  

#include "ili9341_t_dma.h"
#include "bmpvbar.h"

#include <SD.h>
#include "uSDFS.h"


// Title:     <                                        >
#define TITLE "       Teensy Norton Commander          "

#define MAX_PATHNAME_SIZE   128
#define MAX_FILES           32
#define MIN_FILENAME_SIZE   16
#define MAX_FILENAME_SIZE   32
#define MAX_MENULINES       ((TEXT_HEIGHT == 16)?10:20)
#define TEXT_HEIGHT         8
#define TEXT_WIDTH          8
#define MENU_FILE_LXOFFSET  0
#define MENU_FILE_RXOFFSET  (ILI9341_TFTWIDTH/2+2*8)
#define MENU_FILE_YOFFSET   (3*TEXT_HEIGHT)
#define MENU_FILE_W         (MIN_FILENAME_SIZE*TEXT_WIDTH)
#define MENU_FILE_H         (MAX_MENULINES*TEXT_HEIGHT)
#define MENU_FILE_SBGCOLOR  RGBVAL16(0x00,0x00,0xA0)
#define MENU_FILE_UBGCOLOR  RGBVAL16(0x00,0x00,0x50)
#define MENU_VBAR_XOFFSET   (ILI9341_TFTWIDTH/2-3*8)
#define MENU_VBAR_YOFFSET   (MENU_FILE_YOFFSET)

#define LEFT_PANEL          0
#define RIGHT_PANEL         1

#define CHUNK_SIZE          1024


ILI9341_t_DMA tft = ILI9341_t_DMA(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO, TFT_TOUCH_CS, TFT_TOUCH_INT);

static int SelectedPanel = LEFT_PANEL; 

static char SDpath[MAX_PATHNAME_SIZE]="";
static File file;
static int  SDnbFiles=0;
static int  SDcurFile=0;
static int  SDtopFile=0;
static char SDselection[MAX_FILENAME_SIZE+1]="";
static char SDfiles[MAX_FILES][MAX_FILENAME_SIZE];

const char USBpath[MAX_PATHNAME_SIZE]={'2',':',0};
static FATFS fatfs;      /* File system object */
static FIL fil;          /* File object */
static FRESULT rc;       /* Result code */
static int  USBnbFiles=0;
static int  USBcurFile=0;
static int  USBtopFile=0;
static char USBselection[MAX_FILENAME_SIZE+1]={0};
static char USBfiles[MAX_FILES][MAX_FILENAME_SIZE];

static int SDreadNbFiles(void) {
  int totalFiles = 0;
  File entry;    
  file = SD.open(SDpath);
  while ( (true) && (totalFiles<MAX_FILES) ) {
    entry = file.openNextFile();
    if (!entry) {
      // no more files
      break;
    }

    char * filename = entry.name();
    //Serial.println(filename); 
    if (!entry.isDirectory())  {
      strncpy(&SDfiles[totalFiles][0], filename, MAX_FILENAME_SIZE-1);
      totalFiles++;
    }
    else {
      if ( (strcmp(filename,".")) && (strcmp(filename,"..")) ) {
        strncpy(&SDfiles[totalFiles][0], filename, MAX_FILENAME_SIZE-1);
        totalFiles++;
      }
    }  
    entry.close();
  }
  file.close();
  return totalFiles;  
}  

static int USBreadNbFiles(void) {
  DIR dir;
  FILINFO entry; 
  int totalFiles = 0;
  f_opendir(&dir, USBpath);  
  while ( (true) && (totalFiles<MAX_FILES) ) {
    f_readdir(&dir, &entry);
    if (!entry.fname[0]) {
      // no more files
      break;
    }
    char * filename = entry.fname;
    //Serial.print(filename);
    if ( !(entry.fattrib & AM_DIR) ) {
      strncpy(&USBfiles[totalFiles][0], filename, MAX_FILENAME_SIZE-1);
      totalFiles++;
    }
    else {
      if ( (strcmp(filename,".")) && (strcmp(filename,"..")) ) {
        strncpy(&USBfiles[totalFiles][0], filename, MAX_FILENAME_SIZE-1);
        totalFiles++;
      }
    }  
  }
  return totalFiles;  
}  

static int SDhandleMenu(uint16_t bClick, uint16_t bgColor, bool menuRedraw)
{
  char newpath[MAX_PATHNAME_SIZE];
  strcpy(newpath, SDpath);
  strcat(newpath, "/");
  strcat(newpath, SDselection);
  int retval = 0;
  if (bClick & MASK_JOY2_BTN)  {
      File file = SD.open(newpath);
      if (file.isDirectory())  {
        strcpy(SDpath,newpath);
        SDcurFile = 0;
        SDnbFiles = SDreadNbFiles();             
      }
      else {
        retval = 1;               
      }
      menuRedraw=true;
  }
  else if (bClick & MASK_JOY2_UP) {
    if (SDcurFile!=0) {
      menuRedraw=true;
      SDcurFile--;
    }
  }
  else if (bClick & MASK_JOY2_DOWN)  {
    if ((SDcurFile<(SDnbFiles-1)) && (SDnbFiles)) {
      SDcurFile++;
      menuRedraw=true;
    }
  }

  if (menuRedraw && SDnbFiles) {
    int fileIndex = 0;
    tft.drawRectNoDma(MENU_FILE_RXOFFSET,MENU_FILE_YOFFSET, MENU_FILE_W, MENU_FILE_H, bgColor);
    if (SDcurFile <= (MAX_MENULINES/2-1)) SDtopFile=0;
    else SDtopFile=SDcurFile-(MAX_MENULINES/2);
//    if (SDcurFile <= (MAX_MENULINES-1)) SDtopFile=0;
//    else SDtopFile=SDcurFile-(MAX_MENULINES/2);

    int i=0;
    while (i<MAX_MENULINES) {
      if (fileIndex>=SDnbFiles) {
          // no more files
          break;
      }
      char filename[MAX_FILENAME_SIZE];
      strncpy(filename,&SDfiles[fileIndex][0],MAX_FILENAME_SIZE);    
      if (fileIndex >= SDtopFile) {              
        if ((i+SDtopFile) < SDnbFiles ) {
          if ((i+SDtopFile)==SDcurFile) {
            strcpy(SDselection,filename);
            if (strlen(filename) > MIN_FILENAME_SIZE) {filename[MIN_FILENAME_SIZE] = 0;}
            tft.drawTextNoDma(MENU_FILE_RXOFFSET,i*TEXT_HEIGHT+MENU_FILE_YOFFSET, filename, RGBVAL16(0xff,0xff,0x00), RGBVAL16(0xff,0x00,0x00), (TEXT_HEIGHT == 16));           
            strcpy(newpath, SDpath);
            strcat(newpath, "/");
            strcat(newpath, SDselection);
            File file = SD.open(newpath);        
            tft.drawRectNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, ILI9341_TFTWIDTH, 16, RGBVAL16(0x00,0x80,0x00));
            if ( file.isDirectory() ) {
              tft.drawTextNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, "Dir", RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
            }
            else {
              tft.drawTextNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, "File", RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
              tft.drawTextNoDma(5*8,MENU_FILE_YOFFSET+MENU_FILE_H, String((int)file.size()).c_str(), RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
            }
 
          }
          else {
            if (strlen(filename) > MIN_FILENAME_SIZE) {filename[MIN_FILENAME_SIZE] = 0;}
            tft.drawTextNoDma(MENU_FILE_RXOFFSET,i*TEXT_HEIGHT+MENU_FILE_YOFFSET, filename, RGBVAL16(0xff,0xff,0xff), bgColor, (TEXT_HEIGHT == 16));      
          }
        }
        i++; 
      }
      fileIndex++;    
    }  
  }

  return(retval);  
}



static int USBhandleMenu(uint16_t bClick, uint16_t bgColor, bool menuRedraw)
{
  FILINFO entry;  
  char newpath[MAX_PATHNAME_SIZE];
  strcpy(newpath, USBpath);
  strcat(newpath, "/");
  strcat(newpath, USBselection);
  int retval = 0;
  if (bClick & MASK_JOY2_BTN) {
      f_stat(newpath, &entry);
      if ( (entry.fattrib & AM_DIR) ) {
        strcpy(USBpath,newpath);        
        USBcurFile = 0;
        USBnbFiles = USBreadNbFiles();             
      }
      else {
        retval = 1;               
      }
      menuRedraw=true;
  }
  else if (bClick & MASK_JOY2_UP) {
    if (USBcurFile!=0) {
      menuRedraw=true;
      USBcurFile--;
    }
  }
  else if (bClick & MASK_JOY2_DOWN)  {
    if ((USBcurFile<(USBnbFiles-1)) && (USBnbFiles)) {
      USBcurFile++;
      menuRedraw=true;
    }
  }

  if (menuRedraw && USBnbFiles) {
    int fileIndex = 0;
    tft.drawRectNoDma(MENU_FILE_LXOFFSET,MENU_FILE_YOFFSET, MENU_FILE_W, MENU_FILE_H, bgColor);
    if (USBcurFile <= (MAX_MENULINES/2-1)) USBtopFile=0;
    else USBtopFile=USBcurFile-(MAX_MENULINES/2);
//    if (USBcurFile <= (MAX_MENULINES-1)) USBtopFile=0;
//    else USBtopFile=USBcurFile-(MAX_MENULINES/2);

    int i=0;
    while (i<MAX_MENULINES) {
      if (fileIndex>=USBnbFiles) {
          // no more files
          break;
      }   
      char filename[MAX_FILENAME_SIZE];
      strncpy(filename,&USBfiles[fileIndex][0],MAX_FILENAME_SIZE);      
      if (fileIndex >= USBtopFile) {              
        if ((i+USBtopFile) < USBnbFiles ) {
          if ((i+USBtopFile)==USBcurFile) {
            strcpy(USBselection,filename);
            if (strlen(filename) > MIN_FILENAME_SIZE) {filename[MIN_FILENAME_SIZE] = 0;}
            tft.drawTextNoDma(MENU_FILE_LXOFFSET,i*TEXT_HEIGHT+MENU_FILE_YOFFSET, filename, RGBVAL16(0xff,0xff,0x00), RGBVAL16(0xff,0x00,0x00), (TEXT_HEIGHT == 16));
            strcpy(newpath, USBpath);
            strcat(newpath, "/");
            strcat(newpath, USBselection);           
            f_stat(newpath, &entry);
            tft.drawRectNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, ILI9341_TFTWIDTH, 16, RGBVAL16(0x00,0x80,0x00));
            if ( (entry.fattrib & AM_DIR) ) {
              tft.drawTextNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, "Dir", RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
            }
            else {
              tft.drawTextNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H, "File", RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
              tft.drawTextNoDma(5*8,MENU_FILE_YOFFSET+MENU_FILE_H, String((int)entry.fsize).c_str(), RGBVAL16(0xff,0xff,0xff), RGBVAL16(0x00,0x80,0x00), true);
            }
          }
          else {
            if (strlen(filename) > MIN_FILENAME_SIZE) {filename[MIN_FILENAME_SIZE] = 0;}
            tft.drawTextNoDma(MENU_FILE_LXOFFSET,i*TEXT_HEIGHT+MENU_FILE_YOFFSET, filename, RGBVAL16(0xff,0xff,0xff), bgColor, (TEXT_HEIGHT == 16));      
          }
        }
        i++; 
      }
      fileIndex++;    
    }   
  }

  return(retval);  
}

// ****************************************************
// the setup() method runs once, when the sketch starts
// ****************************************************
void setup() {
  tft.begin();
  tft.flipscreen(true);  

  emu_init();
  Serial.begin(115200);
  //while (!Serial) {}

  // mount FS
  if (!SD.begin(SD_CS)) {  
    emu_printf("SdFat.begin() for SD failed");
  }
  if((rc = f_mount (&fatfs, USBpath, 1))) { 
    emu_printf("f_mount() for USB MSC  failed");
  }
  
  // Read root dir
  SDnbFiles  = SDreadNbFiles(); 
  USBnbFiles = USBreadNbFiles();

  // Init display
  tft.fillScreenNoDma(RGBVAL16(0x00,0x00,0x00));
  tft.drawTextNoDma(0,0, TITLE, RGBVAL16(0x00,0xff,0xff), RGBVAL16(0x00,0x00,0xff), true);  
  tft.drawSpriteNoDma(MENU_VBAR_XOFFSET,MENU_VBAR_YOFFSET,(uint16_t*)bmpvbar);
  USBhandleMenu(0,MENU_FILE_SBGCOLOR,true);
  SDhandleMenu(0,MENU_FILE_UBGCOLOR,true);
}



// ****************************************************
// the loop() method runs continuously
// ****************************************************
void loop(void) 
{
  uint16_t bClick = emu_DebounceLocalKeys();
  if (bClick & MASK_JOY2_LEFT) {
    SelectedPanel = LEFT_PANEL;
    USBhandleMenu(0,MENU_FILE_SBGCOLOR,true); 
    SDhandleMenu(0,MENU_FILE_UBGCOLOR,true);    
  }
  else if (bClick & MASK_JOY2_RIGHT) {
      SelectedPanel = RIGHT_PANEL; 
      USBhandleMenu(0,MENU_FILE_UBGCOLOR,true);
      SDhandleMenu(0,MENU_FILE_SBGCOLOR,true);    
  }
    
  if (SelectedPanel == LEFT_PANEL) {
    int action = USBhandleMenu(bClick,MENU_FILE_SBGCOLOR,false);
    if (action) 
    {
      char srcpath[MAX_PATHNAME_SIZE];
      strcpy(srcpath, USBpath);
      strcat(srcpath, "/");
      strcat(srcpath, USBselection);
      FILINFO entry;
      f_stat(srcpath, &entry);
      int filesize = (int)entry.fsize;
      
      char dstpath[MAX_PATHNAME_SIZE];
      strcpy(dstpath, SDpath);
      strcat(dstpath, "/");
      strcat(dstpath, USBselection);        
      emu_printf(srcpath);
      emu_printf(dstpath);
      if (filesize) {
        file = SD.open(&dstpath[1], O_WRITE);
        if (file) {
          if(!(rc = f_open(&fil, srcpath, FA_READ))) {
            int max=ILI9341_TFTWIDTH;
            tft.drawRectNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H+16, ILI9341_TFTWIDTH, 16, RGBVAL16(0x00,0x00,0x00));
            unsigned char buffer[CHUNK_SIZE];
            int remaining = filesize;
            int byteread = 0; 
            while (remaining >= CHUNK_SIZE) {
              //int retval = file.read(buffer, CHUNK_SIZE);
              unsigned int retval;
              f_read (&fil, buffer, CHUNK_SIZE, &retval);
              if (retval>0) {
                //memcpy(buf,buffer,retval);
                //buf += retval;
                byteread += retval;     
                remaining -= retval;
                tft.drawRectNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H+16, (ILI9341_TFTWIDTH*byteread)/filesize, 16, RGBVAL16(0xff,0x00,0x00));          
              }
            }
            if (remaining) {
              //int retval = file.read(buffer, remaining);
              unsigned int retval;
              f_read (&fil, buffer, remaining, &retval);
              if (retval>0) {
                //memcpy(buf,buffer,retval);
                byteread += retval;
                tft.drawRectNoDma(0,MENU_FILE_YOFFSET+MENU_FILE_H+16, (ILI9341_TFTWIDTH*byteread)/filesize, 16, RGBVAL16(0xff,0x00,0x00));
              }
            }           
           
            f_close(&fil);
            SDcurFile = 0;
            SDnbFiles = SDreadNbFiles();
            SDhandleMenu(0,MENU_FILE_UBGCOLOR,true); 
            USBhandleMenu(0,MENU_FILE_SBGCOLOR,true); 
          }
          file.close(); 
        }
        else {
          emu_printf("failed opening SD");
        }
      }  
    }
  }
  else {
   SDhandleMenu(bClick,MENU_FILE_SBGCOLOR,false);
  }
       
  delay(20);  
}








