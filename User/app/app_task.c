#include "app_task.h"

#include "control.h"
#include "bsp_led.h"
#include "menu.h"
#include "openmv.h"
#include "bsp_graySensor.h"

/*==========================================================================
 * 智能送药小车 —— 主任务状态机
 *
 * 本文件是原来写在 main.c 主循环里、用 Do_count / Do2_count + 大 switch
 * 实现的那套「送药 / 返回」逻辑的重构版本，结构改成：
 *
 *   1) 路线表：每个房间一条步骤序列（app_step_t 数组），下表就是全部路线；
 *   2) 执行器：App_Engine_Run() 每次进主循环推进一步，靠控制层给出的完成
 *      标志位（Stop_Flag / Spin_succeed_flag）判断这一步做完没有；全程不
 *      阻塞，所以跑车过程中 OLED、按键、OpenMV 收数都照常工作；
 *   3) 分叉：凡是要"看识别结果决定往哪走"的地方，步骤里直接写三条分支
 *      路线，不再用 Do_count 的魔法数字跳转，也不必再靠 TargetRoom 反推
 *      转向方向。
 *
 * 调车时只改下面的路线表（距离 / 转向 / 亮灯），执行器不用动。
 * 每条路线上面都写了它对应原来 main.c 里的哪几个 case。
 *========================================================================*/

/*------------ 原 main.c 主循环里三件事的时间间隔（单位 ms，SysTick 累加） ------------*/
#define APP_OPENMV_SEND_PERIOD    21    /* 原 SendTime   >= 21  : 给 OpenMV 发数据 */
#define APP_OLED_REFRESH_PERIOD  200    /* 原 showOLEDTime >= 200: 刷 OLED + 查药品 */

/* 一次进主循环最多连推多少个"瞬时步骤"，防止路线表写错（比如绕成环）时死循环 */
#define APP_ENGINE_GUARD          32

/* main.c 里定义、在 SysTick 中断里累加的 1ms 节拍 */
extern u16 showOLEDTime;
extern u16 SendTime;


/*==========================================================================
 * 一、步骤与路线表的类型定义
 *========================================================================*/

/* 步骤类型 */
typedef enum
{
  ST_END = 0,      /* 路线结束 */
  ST_GO,           /* 直行 arg 厘米 */
  ST_TURN,         /* 原地转向，arg 取 spin_dir_t（left_90 / right_90 / back_180） */
  ST_FIND,         /* 等 OpenMV 识别目标数字，命中或超时后跳到不同分支 */
  ST_SET_ROOM,     /* 把 arg 写进 TargetRoom（房间字母） */
  ST_LIGHT,        /* 点灯，arg 取 app_light_t */
  ST_DELAY         /* 延时 arg 毫秒 */
} app_step_kind_t;

/* 灯。注意 bsp_led.h 里 GreenSignal_* 实际接的是 blue_sig 那个引脚，沿用原写法 */
typedef enum
{
  LIGHT_RED_ON = 0,
  LIGHT_RED_OFF,
  LIGHT_GREEN_ON,
  LIGHT_GREEN_OFF,
  LIGHT_YELLOW_ON,
  LIGHT_YELLOW_OFF
} app_light_t;

typedef struct app_step  app_step_t;
typedef struct app_route app_route_t;

struct app_step
{
  uint8_t            kind;        /* app_step_kind_t */
  int16_t            arg;         /* 距离(cm) / 转向 / 房间字母 / 灯 / 毫秒 */
  const app_route_t *on_left;     /* ST_FIND: 识别到目标数字且偏左 -> 走这里 */
  const app_route_t *on_right;    /* ST_FIND: 识别到目标数字且偏右 -> 走这里 */
  const app_route_t *on_timeout;  /* ST_FIND: 超时没识别到          -> 走这里 */
};

struct app_route
{
  const app_step_t *steps;        /* 这条路线的第一步 */
  char              room;         /* 进入这条路线时写进 TargetRoom；0 表示不改 */
};

/* 路线表书写宏，纯粹为了表格看起来像"流水账" */
#define STEP_END()          { ST_END,      0,             0, 0, 0 }
#define STEP_GO(cm)         { ST_GO,       (cm),          0, 0, 0 }
#define STEP_TURN(dir)      { ST_TURN,     (int16_t)(dir),0, 0, 0 }
#define STEP_ROOM(ch)       { ST_SET_ROOM, (ch),          0, 0, 0 }
#define STEP_DELAY(ms)      { ST_DELAY,    (ms),          0, 0, 0 }
#define STEP_RED_ON()       { ST_LIGHT,    LIGHT_RED_ON,  0, 0, 0 }
#define STEP_RED_OFF()      { ST_LIGHT,    LIGHT_RED_OFF, 0, 0, 0 }
#define STEP_GREEN_ON()     { ST_LIGHT,    LIGHT_GREEN_ON,0, 0, 0 }
#define STEP_GREEN_OFF()    { ST_LIGHT,    LIGHT_GREEN_OFF,0,0, 0 }
#define STEP_FIND(l, r, t)  { ST_FIND,     0, (l), (r), (t) }


/*==========================================================================
 * 二、路线表
 *
 * 房间与房号的对应（同原代码）：
 *   1 -> A(近端左)   2 -> B(近端右)
 *   3 -> C(中端左)   4 -> D(中端右)
 *   5 -> E   6 -> F   （远端左支线）
 *   7 -> G   8 -> H   （远端右支线）
 *========================================================================*/

/*---------------- 远端病房的收尾段（原 case 5 命中后的 case 6/7/8） ----------------
 * 原 case 5 无论命中左还是右、还是超时，都是 Do_count++ 然后 Car_go(52)，
 * 区别只体现在 case 6 往哪边转（C/E/G 左转，D/F/H 右转）。
 * 所以这里动作序列只准备"左转收尾"和"右转收尾"两条，房间字母靠 route 上的
 * room 字段区分，E 和 G 共用左转序列，F 和 H 共用右转序列。
 *----------------------------------------------------------------------------*/
static const app_step_t steps_tail_left[] =
{
  STEP_GO(52),
  STEP_TURN(left_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_step_t steps_tail_right[] =
{
  STEP_GO(52),
  STEP_TURN(right_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_route_t route_tail_E = { steps_tail_left,  'E' };
static const app_route_t route_tail_F = { steps_tail_right, 'F' };
static const app_route_t route_tail_G = { steps_tail_left,  'G' };
static const app_route_t route_tail_H = { steps_tail_right, 'H' };


/*---------------- 远端支线（原 case 2 命中 5/7 之后的 case 3/4/5/6/7/8） ----------------
 * 原 case 3: 到路口停车后 TargetRoom=='E' 左转，=='G' 右转
 * 原 case 4: 转完 Car_go(36)
 * 原 case 5: 再识别一次，左 -> E/G，右 -> F/H，超时保持原房间
 *----------------------------------------------------------------------------*/
static const app_step_t steps_far_E[] =      /* 远端左支线：GO(48) -> 左转 -> GO(36) -> 识别 5/6 */
{
  STEP_GO(48),
  STEP_TURN(left_90),
  STEP_GO(36),
  STEP_FIND(&route_tail_E, &route_tail_F, &route_tail_E),
  STEP_END()
};

static const app_step_t steps_far_G[] =      /* 远端右支线：GO(48) -> 右转 -> GO(36) -> 识别 7/8 */
{
  STEP_GO(48),
  STEP_TURN(right_90),
  STEP_GO(36),
  STEP_FIND(&route_tail_G, &route_tail_H, &route_tail_G),
  STEP_END()
};

static const app_route_t route_far_E = { steps_far_E, 'E' };
static const app_route_t route_far_G = { steps_far_G, 'G' };


/*---------------- 远端入口（原 case 2：第一次识别没命中，直行到远端路口再识别） ----------------
 * 原 case 2: GO(88) 后识别，左 -> TargetRoom='E'，右 -> TargetRoom='G'，
 *            超时则保留 SetTargetRoom 给的 'G'（所以超时也走右支线）。
 *----------------------------------------------------------------------------*/
static const app_step_t steps_far_entry[] =
{
  STEP_GO(88),
  STEP_FIND(&route_far_E, &route_far_G, &route_far_G),
  STEP_END()
};

static const app_route_t route_far_entry = { steps_far_entry, 0 };


/*---------------- 中端病房 C/D（原 case 1 命中后 Do_count=6 那段） ----------------
 * 原 case 1: 命中偏左 -> TargetRoom='C'，偏右 -> 'D'，并 Car_go(50)
 * 原 case 6/7/8: 转向 -> GO(36) -> 亮红灯
 *----------------------------------------------------------------------------*/
static const app_step_t steps_deliver_C[] =
{
  STEP_GO(50),
  STEP_TURN(left_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_step_t steps_deliver_D[] =
{
  STEP_GO(50),
  STEP_TURN(right_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_route_t route_deliver_C = { steps_deliver_C, 'C' };
static const app_route_t route_deliver_D = { steps_deliver_D, 'D' };


/*---------------- 房号 3~8 的总入口（原 case 0/1） ----------------
 * 原 case 0: Car_go(115)
 * 原 case 1: 在第一个中端路口识别目标数字
 *             命中 -> 中端病房 C/D（上面那两条）
 *             超时 -> 说明不在这里，继续去远端（route_far_entry）
 *----------------------------------------------------------------------------*/
static const app_step_t steps_room_search[] =
{
  STEP_GO(115),
  STEP_FIND(&route_deliver_C, &route_deliver_D, &route_far_entry),
  STEP_END()
};

static const app_route_t route_room_search = { steps_room_search, 0 };


/*---------------- 近端病房 A/B（原 case 0/1/2/3） ----------------
 * Car_go(75) -> 左/右转 -> Car_go(36) -> 亮红灯
 * （另一版测试代码里这里用的是 Car_go(72)，要换回来改这个数即可）
 *----------------------------------------------------------------------------*/
static const app_step_t steps_deliver_A[] =
{
  STEP_GO(75),
  STEP_TURN(left_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_step_t steps_deliver_B[] =
{
  STEP_GO(75),
  STEP_TURN(right_90),
  STEP_GO(36),
  STEP_RED_ON(),
  STEP_END()
};

static const app_route_t route_deliver_A = { steps_deliver_A, 'A' };
static const app_route_t route_deliver_B = { steps_deliver_B, 'B' };


/*---------------- 返回药房（原 Load_flag==2 那一大段） ----------------
 * 统一都是：关红灯 -> 掉头 -> GO(36) -> 按进入病房时的反方向转回主通道 ->
 *           直行回药房 -> 亮绿灯。
 * 近端回 70cm，中端回 160cm，远端要过两个路口所以是 88 + 转 + 238。
 * 注意：原代码是先 spin_Turn(back_180) 再 RedSignal_off，这里把关灯放在掉头
 * 前面，灯灭的时机与原代码一致（执行器马上就会走到下一步，差别只在几毫秒）。
 *----------------------------------------------------------------------------*/
static const app_step_t steps_back_A[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(right_90),
  STEP_GO(70),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_B[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(left_90),
  STEP_GO(70),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_C[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(right_90),
  STEP_GO(160),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_D[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(left_90),
  STEP_GO(160),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_E[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(right_90),
  STEP_GO(88),
  STEP_TURN(right_90),
  STEP_GO(240 - 5 + 3),      /* 保持原写法，方便和原代码对照：= 238 */
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_F[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(left_90),
  STEP_GO(88),
  STEP_TURN(right_90),
  STEP_GO(240 - 5 + 3),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_G[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(right_90),
  STEP_GO(88),
  STEP_TURN(left_90),
  STEP_GO(240 - 5 + 3),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_step_t steps_back_H[] =
{
  STEP_RED_OFF(),
  STEP_TURN(back_180),
  STEP_GO(36),
  STEP_TURN(left_90),
  STEP_GO(88),
  STEP_TURN(left_90),
  STEP_GO(240 - 5 + 3),
  STEP_GREEN_ON(),
  STEP_END()
};

static const app_route_t route_back_A = { steps_back_A, 'A' };
static const app_route_t route_back_B = { steps_back_B, 'B' };
static const app_route_t route_back_C = { steps_back_C, 'C' };
static const app_route_t route_back_D = { steps_back_D, 'D' };
static const app_route_t route_back_E = { steps_back_E, 'E' };
static const app_route_t route_back_F = { steps_back_F, 'F' };
static const app_route_t route_back_G = { steps_back_G, 'G' };
static const app_route_t route_back_H = { steps_back_H, 'H' };


/*==========================================================================
 * 三、执行器
 *========================================================================*/

typedef struct
{
  const app_step_t *step;      /* 当前步骤，NULL 表示路线已结束 */
  uint8_t           started;   /* 当前步骤的"启动动作"是否已经执行过 */
} app_engine_t;

static app_engine_t g_engine;
static app_phase_t  g_phase = APP_PHASE_WAIT_ROOM;


/**
  * @brief  执行一条灯的开关动作
  */
static void App_Light_Apply(int16_t light)
{
  switch (light)
  {
    case LIGHT_RED_ON:      RedSignal_on;      break;
    case LIGHT_RED_OFF:     RedSignal_off;     break;
    case LIGHT_GREEN_ON:    GreenSignal_on;    break;
    case LIGHT_GREEN_OFF:   GreenSignal_off;   break;
    case LIGHT_YELLOW_ON:   YellowSignal_on;   break;
    case LIGHT_YELLOW_OFF:  YellowSignal_off;  break;
    default:                                   break;
  }
}


/**
  * @brief  执行当前步骤的"启动动作"（只做一次）
  * @note   直行/转向类步骤在这里把动作发出去，之后由 App_Engine_Run 轮询完成标志
  */
static void App_Step_Start(const app_step_t *s)
{
  switch (s->kind)
  {
    case ST_GO:
      Car_go((int32_t)s->arg);          /* 内部会把 Stop_Flag 清零 */
      break;

    case ST_TURN:
      spin_Turn((spin_dir_t)s->arg);    /* 内部会把 Spin_succeed_flag 清零 */
      break;

    case ST_FIND:
      FindTimeCount = 0;
      FindStartFlag = 1;                /* 交给 SysTick 计时，超时判据同原代码 */
      break;

    case ST_DELAY:
      WaitTimeCount = 0;
      WaitFlag      = 1;
      break;

    case ST_SET_ROOM:
      TargetRoom = (char)s->arg;
      break;

    case ST_LIGHT:
      App_Light_Apply(s->arg);
      break;

    default:
      break;
  }
}


/**
  * @brief  跳到某条路线（NULL 表示不再有动作）
  * @note   顺带把路线代表的房间写进 TargetRoom，返回时用来选回程路线
  */
static void App_Route_Jump(const app_route_t *r)
{
  if (r == 0)
  {
    g_engine.step    = 0;
    g_engine.started = 0;
    return;
  }

  if (r->room != 0)
  {
    TargetRoom = r->room;
  }

  g_engine.step    = r->steps;
  g_engine.started = 0;
}


/**
  * @brief  推进状态机（不阻塞，没做完就直接返回，下次再查）
  */
static void App_Engine_Run(void)
{
  uint8_t guard = 0;

  while ((g_engine.step != 0) && (g_engine.step->kind != ST_END))
  {
    const app_step_t *cur  = g_engine.step;
    uint8_t           done = 0;

    if (++guard >= APP_ENGINE_GUARD)
    {
      break;                            /* 路线表写成环了，保护一下 */
    }

    if (g_engine.started == 0)          /* 当前步骤第一次进来，先发动作 */
    {
      App_Step_Start(cur);
      g_engine.started = 1;
    }

    switch (cur->kind)
    {
      case ST_GO:
        done = (Stop_Flag != 0) ? 1 : 0;
        break;

      case ST_TURN:
        done = (Spin_succeed_flag != 0) ? 1 : 0;
        break;

      case ST_DELAY:
        done = (WaitTimeCount >= (u16)cur->arg) ? 1 : 0;
        break;

      case ST_FIND:
        if ((RoomNum == TargetNum) && (LoR == 1))          /* 识别到目标数字且偏左 */
        {
          FindStartFlag = 0;
          FindTimeCount = 0;
          App_Route_Jump(cur->on_left);
          continue;
        }
        else if ((RoomNum == TargetNum) && (LoR == 2))     /* 识别到目标数字且偏右 */
        {
          FindStartFlag = 0;
          FindTimeCount = 0;
          App_Route_Jump(cur->on_right);
          continue;
        }
        else if (FindTimeCount >= WaitTime_ms)             /* 超时，说明不在这里 */
        {
          FindStartFlag = 0;
          FindTimeCount = 0;
          App_Route_Jump(cur->on_timeout);
          continue;
        }
        break;                                             /* 还没结果，这一步算没完成 */

      case ST_SET_ROOM:
      case ST_LIGHT:
      default:
        done = 1;                                          /* 瞬时步骤，马上过 */
        break;
    }

    if (done == 0)
    {
      break;                                               /* 等下一次主循环再查 */
    }

    if (cur->kind == ST_DELAY)                             /* 延时结束，停掉软件计时 */
    {
      WaitFlag      = 0;
      WaitTimeCount = 0;
    }

    g_engine.step    = cur + 1;                            /* 下一步 */
    g_engine.started = 0;
  }
}


/**
  * @brief  当前路线是否已经走完
  */
static uint8_t App_Engine_Finished(void)
{
  return ((g_engine.step == 0) || (g_engine.step->kind == ST_END)) ? 1 : 0;
}


/**
  * @brief  按房号选送药路线（原 if(TargetRoom=='A') ... else if('B') ... else ...）
  */
static const app_route_t *App_Select_Deliver_Route(u8 room_num)
{
  switch (room_num)
  {
    case 1:  return &route_deliver_A;      /* 房号 1 -> 近端左 A */
    case 2:  return &route_deliver_B;      /* 房号 2 -> 近端右 B */
    default: return &route_room_search;    /* 房号 3~8 -> 中端/远端，沿途识别 */
  }
}


/**
  * @brief  按已经确定的房间字母选回程路线（原 if(Load_flag==2) 里那一大段）
  */
static const app_route_t *App_Select_Return_Route(char room)
{
  switch (room)
  {
    case 'A': return &route_back_A;
    case 'B': return &route_back_B;
    case 'C': return &route_back_C;
    case 'D': return &route_back_D;
    case 'E': return &route_back_E;
    case 'F': return &route_back_F;
    case 'G': return &route_back_G;
    case 'H': return &route_back_H;
    default:  return 0;
  }
}


/**
  * @brief  周期性杂事：给 OpenMV 发数据、刷 OLED、查药品有没有放上/取走
  */
static void App_Housekeeping(void)
{
  if (SendTime >= APP_OPENMV_SEND_PERIOD)
  {
    SendTime = 0;
    SendDataToOpenmv();          /* 发太快会撑爆 OpenMV 的接收缓冲 */
  }

  if (showOLEDTime >= APP_OLED_REFRESH_PERIOD)
  {
    showOLEDTime = 0;
    OLED_Display(Menu_Item);
    LoadOrNot();                 /* 顺便更新 Load_flag */
  }
}


/*==========================================================================
 * 四、对外接口
 *========================================================================*/

/**
  * @brief  初始化主任务状态机，进主循环前调用一次
  */
void App_Task_Init(void)
{
  g_engine.step    = 0;
  g_engine.started = 0;
  g_phase          = APP_PHASE_WAIT_ROOM;

  RedSignal_off;
  GreenSignal_off;
  YellowSignal_off;

  FindStartFlag = 0;
  FindTimeCount = 0;
  WaitFlag      = 0;
  WaitTimeCount = 0;
}


/**
  * @brief  主循环任务：杂事 + 阶段调度
  * @note   不阻塞，多久调用一次都行（越频繁越跟手）
  */
void App_Task_Loop(void)
{
  App_Housekeeping();

  switch (g_phase)
  {
    /* 原 TASK==1：一直查 OpenMV 传来的房号，识别到了就定目标房间 */
    case APP_PHASE_WAIT_ROOM:
      SetTargetRoom();
      if ((TASK == 2) && (RoomNum >= 1) && (RoomNum <= 8))
      {
        App_Route_Jump(App_Select_Deliver_Route(RoomNum));
        g_phase = APP_PHASE_DELIVER;
      }
      break;

    /* 原 TASK==2 && Load_flag==1：送药 */
    case APP_PHASE_DELIVER:
      App_Engine_Run();
      if (App_Engine_Finished() != 0)      /* 已到病房并点亮红灯 */
      {
        g_phase = APP_PHASE_WAIT_UNLOAD;
      }
      break;

    /* 到病房了，等药品被取走（原来没有这个阶段，是直接看 Load_flag 就切返回，
       这里改成必须等送药路线走完才允许切，避免药还没送到就掉头） */
    case APP_PHASE_WAIT_UNLOAD:
      if (Load_flag == 2)
      {
        App_Route_Jump(App_Select_Return_Route(TargetRoom));
        g_phase = APP_PHASE_RETURN;
      }
      break;

    /* 原 TASK==2 && Load_flag==2：空车返回药房 */
    case APP_PHASE_RETURN:
      App_Engine_Run();
      if (App_Engine_Finished() != 0)      /* 已回到药房并点亮绿灯 */
      {
        g_phase = APP_PHASE_FINISH;
      }
      break;

    case APP_PHASE_FINISH:
    default:
      break;
  }
}


/**
  * @brief  当前阶段
  */
app_phase_t App_Task_Get_Phase(void)
{
  return g_phase;
}


/**
  * @brief  当前阶段的名字（OLED 调试用，可直接传给 OLED_ShowString）
  */
char *App_Task_Get_Phase_Name(void)
{
  switch (g_phase)
  {
    case APP_PHASE_WAIT_ROOM:    return "WAIT ROOM";
    case APP_PHASE_DELIVER:      return "DELIVER";
    case APP_PHASE_WAIT_UNLOAD:  return "WAIT UNLOAD";
    case APP_PHASE_RETURN:       return "RETURN";
    case APP_PHASE_FINISH:       return "FINISH";
    default:                     return "?";
  }
}
