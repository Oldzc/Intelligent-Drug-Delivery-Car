#ifndef __APP_TASK_H
#define __APP_TASK_H

#include "bsp_sys.h"

/*==========================================================================
 * 智能送药小车 —— 主任务状态机（对外接口）
 *
 * 原先这套流程用 Do_count / Do2_count + 大 switch 写在 main.c 的主循环里，
 * 现在重构成了「路线表 + 通用执行器」，具体路线放在 app_task.c。
 *
 * 用法（main.c 的主循环）：
 *      App_Task_Init();
 *      while (1) { App_Task_Loop(); }
 *========================================================================*/

/* 主任务阶段，对应原来的 TASK / Load_flag 组合 */
typedef enum
{
  APP_PHASE_WAIT_ROOM = 0,   /* 原 TASK==1              : 等 OpenMV 识别目标房号 */
  APP_PHASE_DELIVER,         /* 原 TASK==2 Load_flag==1 : 送药去病房 */
  APP_PHASE_WAIT_UNLOAD,     /* 已到病房，等药品被取走（Load_flag 变成 2） */
  APP_PHASE_RETURN,          /* 原 TASK==2 Load_flag==2 : 空车返回药房 */
  APP_PHASE_FINISH           /* 全部动作完成 */
} app_phase_t;

void        App_Task_Init(void);          /* 上电初始化，进主循环前调用一次 */
void        App_Task_Loop(void);          /* 主循环里反复调用，不阻塞 */
app_phase_t App_Task_Get_Phase(void);     /* 当前阶段，可用于 OLED 调试 */
char       *App_Task_Get_Phase_Name(void);/* 当前阶段的名字，直接给 OLED_ShowString 用 */

#endif
