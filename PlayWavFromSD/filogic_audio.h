

#ifndef __FILOGIC_AUDIO_H__
#define __FILOGIC_AUDIO_H__


#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  char    name[12];
}FileName;

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 * Prerequisites
 ******************************************************************************/


/*****************************************************************************
 * Structures
 *****************************************************************************/


/*****************************************************************************
 * Functions
 *****************************************************************************/

int ff_SD_mount(int opt);

int ff_SD_ls(void);

int ff_SD_write(char *filename, char *wr_buf);

int ff_SD_read(char *filename);

int audio_play_file_cmd(char audio_file[]);

int enable_adsp(void);

extern void va_capture_loop(void *void_this);

void va_enable(void);

int enhance_mic_gain(void);

#ifdef __cplusplus
}
#endif


#endif /* __FILOGIC_AUDIO_H__ */
