/* Copyright Statement:
 *
 * (C) 2022-2022  MediaTek Inc. All rights reserved.
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. ("MediaTek") and/or its licensors.
 * Without the prior written permission of MediaTek and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 * You may only use, reproduce, modify, or distribute (as applicable) MediaTek Software
 * if you have agreed to and been bound by the applicable license agreement with
 * MediaTek ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User").  If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES
 * ARE PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 */


#include <hal.h>


#include "filogic_audio.h"
#include "log_dump.h"
#include "ff.h"
#include "aud_log.h"
#include "asound.h"
#include "tinypcm.h"
#include "tinycompress.h"
#include "compress_params.h"
#include "afe_reg_rw.h"
#include "aud_memory.h"
#include "va.h"
#include "errno.h"
#include "audio_test_utils.h"
#include "mt7933-adsp-pcm.h"
#include "audio_messenger_ipi.h"
#include "audio_shared_info.h"

extern FATFS    FatFS[FF_VOLUMES];               /* Fatfs target */

typedef struct
{
  char    tag[4];
  uint32_t  length;
}BlockHdr;

typedef struct
{
  BlockHdr  blk;
  char    wav[4];
}RiffHdr;

typedef struct
{
  BlockHdr  blk;
  uint16_t  format;
  uint16_t  channel;
  uint32_t  sampleRate;
  uint32_t  byteRate;
  uint16_t  align;
  uint16_t  depth;  
}FmtHdr;


int ff_SD_mount(int opt)
{
    FRESULT  res;                 /* fs status infor*/
    char     mountStr[64] = {0};
    int      vol = -1;
    int      res_sptf;
    
    res_sptf = sprintf(mountStr, "SD:/");
    if( res_sptf < 0 )
    {
        return -1;
    }

    vol = get_pdstr(mountStr, mountStr, sizeof(mountStr) );
    if (vol<0)
    {
        printf("Get PD Str Fail\r\n");
        return 0;
    }
    printf("\r\nVol(%u)\r\n", vol);

    res = f_mount(&FatFS[vol], "", opt);
    if(res)
    {
        printf("[FS]: Mount Fail - res(%u)\r\n", res);
        return 0;
    }

    printf(opt ? "[FS]: Mount OK!\r\n" : "[FS]: UnMount\r\n");
    return 1;
}

int ff_SD_write(char *filename, char *wr_buf)
{
    FRESULT  res;                 /* fs status infor*/
    FIL      fdst;                /* file target */
    int      ret = -1;
    char    *pbuff     = NULL;
    char     path[128] = {0};
    UINT     length_written;

    ret = get_pdstr(filename, path, sizeof(path) );
    if (ret<0)
    {
        printf("Get PD Str Fail\r\n");
        return 0;
    }

    pbuff = wr_buf;

    res = f_open(&fdst, path, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
    if (res)
    {
        printf("[FS]: File Open Fail - res(%u)\r\n", res);
        return 0;\
    }

    res = f_write(&fdst, pbuff, strlen(pbuff), &length_written);
    if (res)
    {
        res = f_close(&fdst);
        printf("[FS_Write]: Write File Fail - res(%u)\r\n", res);
        return 0;
    }
    res = f_close(&fdst);

    printf("[FS]: Write - pfile(%s), pbuff(%s), len(%u)\r\n", path, pbuff, strlen(pbuff));

    return 0;
}

int ff_SD_read(char *filename)
{
    FRESULT  res;                 /* fs status infor*/
    FIL      fdst;                /* file target */
    int      ret = -1;
    char     buff[256] = {0};
    char     path[128] = {0};
    UINT     length_read;

    ret = get_pdstr(filename, path, sizeof(path) );
    if (ret<0)
    {
        printf("Get PD Str Fail\r\n");
        return 0;
    }

    res = f_open(&fdst, path, FA_OPEN_EXISTING | FA_READ);
    if (res)
    {
        printf("[FS]: File Open Fail - res(%u)\r\n", res);
        return 0;
    }

    res = f_read(&fdst, buff, sizeof(buff), &length_read);
    if (res)
    {
        res = f_close(&fdst);
        printf("[FS_Read]: Read File Fail - res(%u)\r\n", res);
        return 0;
    }

    printf("[FS]: Read - Path(%s), pbuff(%s), len(%u)\r\n", path, buff, sizeof(buff));

    return 0;
}

int ff_SD_ls(void)
{
    FRESULT  res;                 /* fs status infor*/
    DIR      dir;               /* Directory object */
    FILINFO  Finfo;
    int      ret = -1;

    res = f_opendir(&dir, "/");
    if (res)
    {
        printf("[FS]: Folder Open Fail - res(%u)\r\n", res);
        return 0;
    }

    for(;;) {
        res = f_readdir(&dir, &Finfo);
        if (res || !Finfo.fname[0])
        {
            break;
        }
        printf("%s%s%s%s%s %u/%02u/%02u %02u:%02u %lu ",
            (Finfo.fattrib & AM_DIR) ? "DIR" : "-",
            (Finfo.fattrib & AM_RDO) ? "RDO" : "-",
            (Finfo.fattrib & AM_HID) ? "HID" : "-",
            (Finfo.fattrib & AM_SYS) ? "SYS" : "-",
            (Finfo.fattrib & AM_ARC) ? "ARC" : "-",
            (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
            (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63, (unsigned long)Finfo.fsize);
        printf("%s\n", Finfo.fname);
    }
    f_closedir(&dir);

    return 0;
}

void dlm_intdac_file_SD(int channel_num, int bitdepth, int sample_rate,
                     int period_size, int period_count, int time_len,
                     char *file_name)
{
    sound_t *w_snd;
    int ret;
    struct msd_hw_params params;

    FIL fid;
    FRESULT f_ret;
    UINT f_br;

    if (bitdepth != 32 && bitdepth != 16) {
        aud_error("bitdepth error: %d", bitdepth);
        return;
    }

    params.format = bitdepth == 16 ? MSD_PCM_FMT_S16_LE : MSD_PCM_FMT_S32_LE;
    params.channels = channel_num;
    params.period_count = period_count;
    params.period_size = period_size;
    params.rate = sample_rate;

    int bytes_per_frame = bitdepth * params.channels / 8;
    int data_size = bytes_per_frame * params.period_size * params.period_count;

    void *data_src = malloc(data_size);
    if (!data_src) {
        aud_error("%s: memory error\n", __FUNCTION__);
        return;
    }

    memset(data_src, 0, data_size);

    f_ret = f_open(&fid, file_name, FA_READ);
    if (f_ret) {
        printf("f_open error: %d", f_ret);
        goto exit0;
    }
    aud_msg("data_src = %p, data_size = 0x%x", data_src, data_size);

    connect_route("track0", "INTDAC out", 1, CONNECT_FE_BE);
    connect_route("I_22", "O_20", 1, CONNECT_IO_PORT);
    connect_route("I_23", "O_21", 1, CONNECT_IO_PORT);

    ret = snd_pcm_open(&w_snd, "track0", 0, 0);
    if (ret)
        goto exit1;
    ret = snd_pcm_hw_params(w_snd, &params);
    if (ret)
        goto exit2;
    ret = snd_pcm_prepare(w_snd);
    if (ret)
        goto exit2;

    while (!f_eof(&fid)) {
        f_read(&fid, data_src, data_size, &f_br);

        ret = snd_pcm_write(w_snd, data_src, f_br / bytes_per_frame);
        if (ret != (int)f_br / bytes_per_frame)
            aud_msg("ret: %d", ret);
    }

    ret = snd_pcm_drain(w_snd);
    if (ret)
        goto exit2;
    ret = snd_pcm_hw_free(w_snd);
    if (ret)
        goto exit2;

exit2:
    snd_pcm_close(w_snd);

exit1:
    f_close(&fid);

exit0:
    free(data_src);

    connect_route("track0", "INTDAC out", 0, CONNECT_FE_BE);
    connect_route("I_22", "O_20", 0, CONNECT_IO_PORT);
    connect_route("I_23", "O_21", 0, CONNECT_IO_PORT);
}

int audio_play_file_cmd(char audio_file[])
{
    int play_type;
    int channel_num;
    int bitdepth;
    int sample_rate;
    int period_size;
    int period_count;
    int time_len;
    char file_name[256] = {0};

    play_type = 1;
    channel_num = 4;
    bitdepth = 32;
    sample_rate = 16000;
    time_len = 10;
    period_size = 960;
    period_count = 4;

    snprintf(file_name, sizeof(file_name), "SD:/%s", audio_file);

    aud_msg("play_type = %d\n", play_type);
    aud_msg("channel_num = %d\n", channel_num);
    aud_msg("bitdepth = %d\n", bitdepth);
    aud_msg("sample_rate = %d\n", sample_rate);
    aud_msg("time_len = %d\n", time_len);
    aud_msg("period_size = %d\n", period_size);
    aud_msg("period_count = %d\n", period_count);
    aud_msg("file_name = %s\n", file_name);
    switch (play_type) {
        case 0:
#ifdef AUD_CODEC_SUPPORT
            //      dlm_gsrc_intdac(channel_num, bitdepth, sample_rate, period_size, period_count, time_len);
#else /* #ifdef AUD_CODEC_SUPPORT */
            aud_msg("Codec is not supported\n");
#endif /* #ifdef AUD_CODEC_SUPPORT */
            break;
        case 1:
//#ifdef AUD_CODEC_SUPPORT
            dlm_intdac_file_SD(channel_num, bitdepth, sample_rate, period_size,
                            period_count, time_len, file_name);
//#else /* #ifdef AUD_CODEC_SUPPORT */
//            printf("Codec is not supported\n");
//#endif /* #ifdef AUD_CODEC_SUPPORT */
            break;
        case 2:
            //      dlm_etdmout2(channel_num, bitdepth, sample_rate, period_size, period_count, time_len);
            break;
        default:
            aud_err("play_type error\n");
    }
    return 0;
}

int pdct_f1(uint32_t check_mask)
{
    return 0;
}

int enable_adsp(void)
{
    int *value = (int *)malloc(2 * sizeof(int));
        if (!value) {
            aud_error("malloc error");
            return 1;
        }
    value[0] = 1;
    value[1] = 1;
    control_cset("ADSP_Enable", 1, value);
    control_cget("ADSP_Enable", 1);
    free(value);

    aud_msg("enable_adsp OK!\n");

    return 0;
}

TaskHandle_t g_handler_1 = NULL;


static TaskHandle_t get_handle(void)
{
    return g_handler_1;
}

static void set_handle(TaskHandle_t thread_handler)
{
    g_handler_1 = thread_handler;
}

struct va_task *task_constructor_va(void)
{
    struct va_task *p_va_task;
    struct msd_hw_params *params;

    p_va_task = (struct va_task *)pvPortMalloc(sizeof(struct va_task));
    memset(p_va_task, 0, sizeof(struct va_task));

    p_va_task->thread_stack_dep = configMINIMAL_STACK_SIZE * 4;
    p_va_task->thread_priority = configMAX_PRIORITIES - 2;

    p_va_task->priv = pvPortMalloc(sizeof(struct msd_hw_params));
    memset(p_va_task->priv, 0, sizeof(struct msd_hw_params));

    params = p_va_task->priv;

    params->format = MSD_PCM_FMT_S16_LE;
    params->channels = 4;
    params->period_count = 12;
    params->rate = 16000;
    params->period_size = params->rate / 100; //10ms;

    return p_va_task;
}

void va_enable(void)
{
    int ret = 0;
    struct va_task *p_va_task = task_constructor();
    struct msd_hw_params *params = (struct msd_hw_params *)p_va_task->priv;

    connect_route("ADSP_HOSTLESS_VA", "ADSP_UL9_IN BE", 1, CONNECT_FE_BE);
    connect_route("ADSP_VA_FE", "ADSP_UL9_IN BE", 1, CONNECT_FE_BE);
    connect_route("ADSP_UL9_IN BE", "INTADC in", 1, CONNECT_FE_BE);
    connect_route("ADSP_UL9_IN BE", "GASRC0_C", 1, CONNECT_FE_BE);
    connect_route("GASRC0_C", "dummy_end_c", 1, CONNECT_FE_BE);
    connect_route("I_60", "O_26", 1, CONNECT_IO_PORT);
    if (hal_boot_get_hw_ver() != 0x8A00) {
        connect_route("I_61", "O_27", 1, CONNECT_IO_PORT);
    } else {
        connect_route("I_08", "O_27", 1, CONNECT_IO_PORT);
    }
    connect_route("I_22", "O_28", 1, CONNECT_IO_PORT);
    connect_route("I_23", "O_29", 1, CONNECT_IO_PORT);

    xTaskCreate(va_capture_loop,
                "ap_aud_t_va",
                p_va_task->thread_stack_dep,
                (void *)p_va_task,
                p_va_task->thread_priority,
                &p_va_task->thread_handler);

    va_alarm_create();

    set_handle(p_va_task->thread_handler);
}

int enhance_mic_gain(void)
{
    int value[2] = {1,15};

    control_cset("Audio_Uplink_Vol", 2, value);
    control_cget("Audio_Uplink_Vol", 2);

    aud_msg("enhance_mic_gain OK!\n");

    return 0;
}

int ListWavFiles(FileName *list)
{
  FRESULT  res;                 /* fs status infor*/
  DIR      dir;               /* Directory object */
  FILINFO fileInfo;
  int   rc = 0;

  res = f_opendir(&dir, "/");
  if (res)
  {
      printf("[FS]: OpenDir Fail - res(%u)\r\n", res);
      return 0;
  }

  for(;;)
  {
    memset( &fileInfo, 0, sizeof(FILINFO) );
    f_readdir( &dir, &fileInfo );
    if( fileInfo.fname[0] == 0 ) break;
    
    if( strstr(fileInfo.fname, ".wav") != NULL )
    {
      sprintf(list[rc].name, "%s", fileInfo.fname);
      printf("%s\n", fileInfo.fname );
      rc++;
    }
  }
  
  f_readdir( &dir, NULL ); // Rewind the directory object
  return rc;
}

void PlayWavFile(char *fname)
{
extern bool bMount;

  sound_t *w_snd;
  struct msd_hw_params params;

  FIL *Fil = (FIL *) pvPortMalloc( sizeof(FIL) );
  unsigned int dwRead;
  
  RiffHdr riff;
  FmtHdr  fmt;
  BlockHdr dataTag;

  if( f_open(Fil, fname, FA_READ) == FR_OK )
  {
    if( f_read(Fil, (uint8_t*)&riff, sizeof(riff), &dwRead) != FR_OK ) goto CLOSE;
    if( !memcmp(riff.blk.tag, "RIFF", 4) && !memcmp(riff.wav, "WAVE", 4))
    {
      if( f_read(Fil, (uint8_t*)&fmt, sizeof(fmt), &dwRead) != FR_OK ) goto CLOSE;
      if( f_read(Fil, (uint8_t*)&dataTag, sizeof(dataTag), &dwRead) != FR_OK ) goto CLOSE;
      
      printf("%s  rate:%ld,%ld, len=%ld\n", fname, fmt.sampleRate, fmt.byteRate, dataTag.length);
      
      params.format = (fmt.depth == 16) ? MSD_PCM_FMT_S16_LE : MSD_PCM_FMT_S8;
      params.channels = fmt.channel;
      params.rate = fmt.sampleRate;
      params.period_size = 960;
      params.period_count = 4;

      int bytes_per_frame = fmt.depth * params.channels / 8;
      int data_size = bytes_per_frame * params.period_size * params.period_count;
  
      if( snd_pcm_open(&w_snd, "track0", MSD_STREAM_PLAYBACK, 0) != 0 ) goto CLOSE;
      if( snd_pcm_hw_params(w_snd, &params) != 0 ) goto EXIT_PCM;
      if( snd_pcm_prepare(w_snd) != 0 ) goto EXIT_PCM;

      void *data_src = pvPortMalloc(data_size);
      printf("playing.....\n");
      
      while( dataTag.length > 0 )
      {
        data_size = snd_pcm_avail(w_snd);
        if( data_size <= 0 )
        {
          vTaskDelay(4);
          continue;
        }

        if( !bMount ) break;
        if( f_read(Fil, (uint8_t*)data_src, data_size*bytes_per_frame, &dwRead) != FR_OK ) break;
        
        dataTag.length = (dataTag.length >= dwRead) ? (dataTag.length - dwRead) : 0;
        snd_pcm_write(w_snd, data_src, dwRead / bytes_per_frame);
        vTaskDelay(1);
      }
      
      printf("play complete\n");
      snd_pcm_drop(w_snd);
      vPortFree(data_src);

EXIT_PCM:
      snd_pcm_hw_free(w_snd);
      snd_pcm_close(w_snd);
    }

CLOSE:
    f_close ( Fil );
  }

  vPortFree( Fil );
}

