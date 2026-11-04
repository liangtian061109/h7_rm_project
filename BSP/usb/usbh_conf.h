#ifndef USBH_CONF_H
#define USBH_CONF_H

#include <stdlib.h>
#include <string.h>

#include "main.h"

/* ST USB Host Library 配置。单设备、单 class，够接一只手柄。 */

#define USBH_MAX_NUM_ENDPOINTS 8
#define USBH_MAX_NUM_INTERFACES 8 /* 接收器是复合设备，接口不止一个 */
#define USBH_MAX_NUM_CONFIGURATION 1
#define USBH_KEEP_CFG_DESCRIPTOR 1
#define USBH_MAX_NUM_SUPPORTED_CLASS 1
#define USBH_MAX_SIZE_CONFIGURATION 512
#define USBH_MAX_DATA_BUFFER 512
#define USBH_MAX_PIPES_NBR 8

#define USBH_DEBUG_LEVEL 0

/* 0 = 裸机轮询。Host 状态机由我们的任务调 USBH_Process 推进，
 * 不用 ST 的 OS 抽象层（那套是给 RTOS 版本准备的，这里没必要）。 */
#define USBH_USE_OS 0

/* 0 = NAK 不在 Core 里重试，由 class 自己按 interval 重新提交。
 * 无线接收器空闲时大量 NAK，交给 Core 会把状态机拖死。 */
#define USBH_IN_NAK_PROCESS 0

#define USBH_malloc malloc
#define USBH_free free
#define USBH_memset memset
#define USBH_memcpy memcpy

#define USBH_UsrLog(...) \
  do {                   \
  } while (0)
#define USBH_ErrLog(...) \
  do {                   \
  } while (0)
#define USBH_DbgLog(...) \
  do {                   \
  } while (0)

/* Core 枚举字符串时请求 255 字节，缓冲区不能按 64 字节端点缩小 */
_Static_assert(USBH_MAX_DATA_BUFFER >= 255, "descriptor buffer too small");

#endif
