/*
 * drivers/video/sun3i/disp/drv_disp_i.h
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Danling <danliang@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __DRV_DISP_I_H__
#define __DRV_DISP_I_H__

#include "../drv_display.h"
#include "bsp_display.h"

#define __wrn printk

#if 1
#define __msg(msg...) do{}while(0)
#define __inf(msg...) do{}while(0)
#else
#define __HERE__ {printk("File:%s,Line:%d;\t", __FILE__, __LINE__);}
#define __msg printk
#define __inf printk
#endif


typedef enum
{
   DIS_SUCCESS=0,
   DIS_FAIL=-1,
   DIS_PARA_FAILED=-2,
   DIS_PRIO_ERROR=-3,
   DIS_OBJ_NOT_INITED=-4,
   DIS_NOT_SUPPORT=-5,
   DIS_NO_RES=-6,
   DIS_OBJ_COLLISION=-7,
   DIS_DEV_NOT_INITED=-8,
   DIS_DEV_SRAM_COLLISION=-9,
   DIS_TASK_ERROR = -10,
   DIS_PRIO_COLLSION = -11
}__disp_return_value;

#define HANDTOID(handle)  ((handle) - 100)
#define IDTOHAND(ID)  ((ID) + 100)


#define DISP_IO_NUM     7
#define DISP_IO_SCALER0 0
#define DISP_IO_SCALER1 1
#define DISP_IO_IMAGE0  2
#define DISP_IO_IMAGE1  3
#define DISP_IO_LCDC0   4
#define DISP_IO_LCDC1   5
#define DISP_IO_TVEC    6

#define DISP_IO_CCMU    7
#define DISP_IO_SDRAM   8
#define DISP_IO_PIO     9



#endif
