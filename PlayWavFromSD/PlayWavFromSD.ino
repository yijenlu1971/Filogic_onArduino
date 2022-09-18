/* 
 * Audio Playback from SD.
 */
#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "filogic_audio.h"
#include "asound.h"

extern "C" {
#include "audio_test_utils.h"

int ListWavFiles(FileName *list);
void PlayWavFile(char *fname);
}

int sd_cd = 12; // SD card
//int sd_cd = 35; // microSD
bool bMount = false;
int     fNum = 0;
FileName  flist[16] = {0};

void TaskSDDetect(void *pvParameters) {
  while(1) {
    if (digitalRead(sd_cd) == 0) {
      if( !bMount ) {
        bMount = true;
        vTaskDelay(500);
        if( ff_SD_mount(1) == 1 ) fNum = ListWavFiles(flist);
      }
    }
    else {
      if( bMount ) {
        bMount = false;
        ff_SD_mount(0);
      }
    }

    vTaskDelay(100);
  }
}

void TaskPlayMusic(void *param)
{
  int idx = 0;

  for(;;) {
    while( fNum == 0 ) vTaskDelay( 1000 );
    printf("# of wav_file: %d\n", fNum);
  
    connect_route("track0", "INTDAC out", 1, CONNECT_FE_BE);
    connect_route("I_22", "O_20", 1, CONNECT_IO_PORT);
    connect_route("I_23", "O_21", 1, CONNECT_IO_PORT);
  
    while(bMount) {
      PlayWavFile( flist[idx].name );
  
      if( ++idx >= fNum ) idx = 0;
      vTaskDelay( 1500 );
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(sd_cd, INPUT_PULLUP);

  // DSP & voice assistant
//  enable_adsp();
//  va_enable();
  
  // Priority, (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  xTaskCreate( TaskPlayMusic, "PLAY", 8192/sizeof(portSTACK_TYPE), NULL, 1, NULL);
  xTaskCreate( TaskSDDetect, "SDCD", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

void loop() {
}
