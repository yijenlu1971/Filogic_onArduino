/* 
 * Audio Playback from internal RAM.
 */
#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"

extern "C" {
#include "tinypcm.h"
#include "audio_test_utils.h"

// defined in va_golden_tone.c (Linux SDK)
extern short tone1[]; // (len = 12800)
extern short tone2[]; // (len = 8394)
}

#define SW_KEY_2     47
#define SW_KEY_3     49

int count = 0;
void TaskPlayAlarm(void *arg);

void setup() {
  Serial.begin(115200);

  pinMode(SW_KEY_2, INPUT_PULLUP);  // play "I'm here"
  pinMode(SW_KEY_3, INPUT_PULLUP);  // play "got it"

  // Priority, (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  xTaskCreate( TaskPlayAlarm, "PLAY", 8192/sizeof(portSTACK_TYPE), NULL, 1, NULL);
}

void loop() {
}

void TaskPlayAlarm(void *arg)
{
  sound_t *w_snd;
  int ret = 0;
  int remain_frame = 0;
  int written_frame = 0;
  unsigned int once_write = 0;

  int frames;
  void *data;
  struct msd_hw_params params;

  params.format = MSD_PCM_FMT_S16_LE; // PCM 16-bit little-endian
  params.channels = 2;
  params.period_count = 4;
  params.rate = 16000; // sampling rate 16K
  params.period_size = 1024;

  while (1) {
    vTaskDelay(200);

    if (digitalRead(SW_KEY_2) == 0) {
      data = tone1;
      frames = 12800;
    } else if (digitalRead(SW_KEY_3) == 0) {
      data = tone2;
      frames = 8394;
    }
    else {
      continue;
    }

    connect_route("track0", "INTDAC out", 1, CONNECT_FE_BE);
    connect_route("I_22", "O_20", 1, CONNECT_IO_PORT);
    connect_route("I_23", "O_21", 1, CONNECT_IO_PORT);

    ret = snd_pcm_open(&w_snd, "track0", MSD_STREAM_PLAYBACK, 0);
    if (ret != 0) {
      printf("track0 open fail %d.\n", ret);
      continue;
    }

    snd_pcm_hw_params(w_snd, &params);

    ret = snd_pcm_prepare(w_snd);
    if (ret != 0) goto exit;

    remain_frame = frames;
    while (remain_frame) {
      if (remain_frame >= params.period_size * 4) {
        once_write = params.period_size * 4;
      } else {
        once_write = remain_frame;
      }

      ret = snd_pcm_write(w_snd, (void*)((unsigned int *)data + written_frame), once_write);
      if (ret != once_write) printf("[%s]Warning: write frames: %d", __func__, ret);

      written_frame += once_write;
      remain_frame -= once_write;
      //printf("[%s]written_frame: %d,  remain_frame: %d\n", __func__, written_frame, remain_frame);
    }

    //printf("[%s]Total write frames: %d\n", __func__, written_frame);
    written_frame = 0;
    ret = snd_pcm_drain(w_snd);
    if (ret) goto exit;

    ret = snd_pcm_hw_free(w_snd);
    if (ret) goto exit;

    ret = snd_pcm_close(w_snd);
    if (ret) goto exit;

    connect_route("track0", "INTDAC out", 0, CONNECT_FE_BE);
    connect_route("I_22", "O_20", 0, CONNECT_IO_PORT);
    connect_route("I_23", "O_21", 0, CONNECT_IO_PORT);

    printf("[%s], end\n", __func__);
    printf("#### Alert Count %d ####\n", ++count);
  }

exit:
  vTaskDelete(NULL);
}

