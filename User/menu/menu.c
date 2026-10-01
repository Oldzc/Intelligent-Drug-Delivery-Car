#include "menu.h"
#include <stdio.h>
#include "bsp_key.h"
#include "control.h"
#include "oled.h"
#include "encoder.h"
#include "tim.h"
#include "openmv.h"
#include "bsp_GraySensor.h"
#include "app_task.h"

//extern unsigned char BMP1[];   //无用的图片首页
////extern unsigned char BMP4[];  //HCF  //不要这个
//extern unsigned char BMP3[];  //START

//OLED菜单有四个数据缓冲区就够了吧
u8 str_buff1[64];
u8 str_buff2[64];
u8 str_buff3[64];
u8 str_buff4[64];   
u8 str_buff5[64];
u8 str_buff6[64];

u8 Menu_Item = 122;  //这个表示一开始就跳转哪个界面


void OLED_Display(u8 num)
{
	switch(num)      //这里光放要显示的内容
	{
		case 0: 
	    OLED_ShowString(40, 3, "WELCOME", 16);
		break;
		
		//一级界面，选中第 1 项 TASK
		case 1:
			OLED_ShowString(40, 0, "MENU", 16);
		  OLED_ShowString(20, 3, "TASK", 8);
		  OLED_ShowString(20, 4, "TEST", 8);
		  OLED_ShowString(0, 3, ">>", 8);
		break;
		
		//TASK 二级界面：任务运行状态（原来这 4 项标签是空字符串，页面也是空的）
		case 10:
		 {
			  OLED_ShowString(0, 0, "TASK", 16);
		    OLED_ShowString(20, 3, "phase", 8);
		    OLED_ShowString(20, 4, "motion", 8);
		    OLED_ShowString(20, 5, "route", 8);
			  OLED_ShowString(20, 6, "all", 8);
		    OLED_ShowString(0, 3, ">>", 8);
		 }
		 break;
		 
		 case 11:
		 {
			  OLED_ShowString(0, 0, "TASK", 16);
		    OLED_ShowString(20, 3, "phase", 8);
		    OLED_ShowString(20, 4, "motion", 8);
		    OLED_ShowString(20, 5, "route", 8);
			  OLED_ShowString(20, 6, "all", 8);
		    OLED_ShowString(0, 4, ">>", 8);
		 }
		 break;
		 
		 case 12:
		 {
			  OLED_ShowString(0, 0, "TASK", 16);
		    OLED_ShowString(20, 3, "phase", 8);
		    OLED_ShowString(20, 4, "motion", 8);
		    OLED_ShowString(20, 5, "route", 8);
			  OLED_ShowString(20, 6, "all", 8);
		    OLED_ShowString(0, 5, ">>", 8);
		 }
		 break;
		 
		  case 13:
		 {
			  OLED_ShowString(0, 0, "TASK", 16);
		    OLED_ShowString(20, 3, "phase", 8);
		    OLED_ShowString(20, 4, "motion", 8);
		    OLED_ShowString(20, 5, "route", 8);
			  OLED_ShowString(20, 6, "all", 8);
		    OLED_ShowString(0, 6, ">>", 8);
		 }
		 break;
		 
		 //TASK 三级界面：真正显示任务状态机当前的情况
		 case 110:                                    //当前阶段
				OLED_ShowString(0, 0, "TASK PHASE", 8);
				OLED_ShowString(0, 2, "phase:", 8);
				OLED_ShowString(0, 3, (u8 *)App_Task_Get_Phase_Name(), 8);
				sprintf((char *)str_buff1, "Room: %c", TargetRoom);
				OLED_ShowString(0, 5, str_buff1, 8);
				sprintf((char *)str_buff2, "Load: %d", Load_flag);
				OLED_ShowString(0, 6, str_buff2, 8);
			break;
				
			case 111 :                                   //运动/巡线标志位
				OLED_ShowString(0, 0, "TASK MOTION", 8);
				sprintf((char *)str_buff1, "Stop_Flag: %d", Stop_Flag);
				OLED_ShowString(0, 2, str_buff1, 8);
				sprintf((char *)str_buff2, "Spin_flag: %d", Spin_succeed_flag);
				OLED_ShowString(0, 3, str_buff2, 8);
				sprintf((char *)str_buff3, "Line_Num: %d", Line_Num);
				OLED_ShowString(0, 4, str_buff3, 8);
			break;
			
			case 112:                                    //识别与目标房间
				OLED_ShowString(0, 0, "TASK ROUTE", 8);
				sprintf((char *)str_buff1, "Room: %c", TargetRoom);
				OLED_ShowString(0, 2, str_buff1, 8);
				sprintf((char *)str_buff2, "TargetNum: %d", TargetNum);
				OLED_ShowString(0, 3, str_buff2, 8);
				sprintf((char *)str_buff3, "Num: %d", Num);
				OLED_ShowString(0, 4, str_buff3, 8);
				sprintf((char *)str_buff4, "LoR: %d", LoR);
				OLED_ShowString(0, 5, str_buff4, 8);
				sprintf((char *)str_buff5, "Finded: %d", Finded_flag);
				OLED_ShowString(0, 6, str_buff5, 8);
			break;
			
			case 113:                                    //一屏看全
				OLED_ShowString(0, 0, "TASK ALL", 8);
				OLED_ShowString(0, 2, (u8 *)App_Task_Get_Phase_Name(), 8);
				sprintf((char *)str_buff1, "Room:%c Num:%d", TargetRoom, Num);
				OLED_ShowString(0, 3, str_buff1, 8);
				sprintf((char *)str_buff2, "Load:%d Find:%d", Load_flag, Finded_flag);
				OLED_ShowString(0, 4, str_buff2, 8);
		  break;
		
		//一级界面，选中第 2 项 TEST
		case 2:
			OLED_ShowString(40, 0, "MENU", 16);   //记得与前面主菜单界面保持一直
		  OLED_ShowString(20, 3, "TASK", 8);
		  OLED_ShowString(20, 4, "TEST", 8);
		  OLED_ShowString(0, 4, ">>", 8);
		  break;
		
		//TEST 二级界面
		case 20:
		 {
			  OLED_ShowString(0, 0, "TEST", 16);
		    OLED_ShowString(20, 3, "motor", 8);
		    OLED_ShowString(20, 4, "ADC", 8);
		    OLED_ShowString(20, 5, "openmv", 8);
			  OLED_ShowString(20, 6, "Load", 8);
		    OLED_ShowString(0, 3, ">>", 8);
		 }
		 break;
		 
		 case 21:
		 {
			  OLED_ShowString(0, 0, "TEST", 16);
		    OLED_ShowString(20, 3, "motor", 8);
		    OLED_ShowString(20, 4, "ADC", 8);
		    OLED_ShowString(20, 5, "openmv", 8);
			  OLED_ShowString(20, 6, "Load", 8);
		    OLED_ShowString(0, 4, ">>", 8);
		 }
		 break;
		 
		 case 22:
		 {
			  OLED_ShowString(0, 0, "TEST", 16);
		    OLED_ShowString(20, 3, "motor", 8);
		    OLED_ShowString(20, 4, "ADC", 8);
		    OLED_ShowString(20, 5, "openmv", 8);
			  OLED_ShowString(20, 6, "Load", 8);
		    OLED_ShowString(0, 5, ">>", 8);
		 }
		 break;
		 
		 case 23:
		 {
			  OLED_ShowString(0, 0, "TEST", 16);
		    OLED_ShowString(20, 3, "motor", 8);
		    OLED_ShowString(20, 4, "ADC", 8);
		    OLED_ShowString(20, 5, "openmv", 8);
			  OLED_ShowString(20, 6, "Load", 8);
		    OLED_ShowString(0, 6, ">>", 8);
		 }
		 break;
		 
		 //TEST 三级界面
		 case 120:
			 
		 /*********测试编码器是否正常**************/

		 
   		  OLED_ShowString(0, 0, "test 1:", 16);   //在test 1 里面调试获取编码器的数值   //下面第二个表示每秒转动的圈数
		    sprintf((char *)str_buff1, "M_EncNum: %5d", g_nMotorPulse);            //这个数值取决与调用GetMotorPulse的周期  （50ms）， 下面的也要对应1000/25=40,下面要乘上40
				sprintf((char *)str_buff2, "rps:      %2.2f", g_nMotorPulse*(1000/SPEED_PID_PERIOD)*0.25/56/11);    //两相  上升沿下降沿都捕获 减速比1：56   单项编码器脉冲数11  
				sprintf((char *)str_buff3, "M2_EncNum:%5d", g_nMotor2Pulse);  
				sprintf((char *)str_buff4, "rps:      %2.2f", g_nMotor2Pulse*(1000/SPEED_PID_PERIOD)*0.25/56/11);
		 
				OLED_ShowString(0, 3,str_buff1, 8);
				OLED_ShowString(0, 4,str_buff2, 8);
				OLED_ShowString(0, 5,str_buff3, 8);
		    OLED_ShowString(0, 6,str_buff4, 8);		

			break;
		 
		 case 121:
				
		   
       sprintf((char *)str_buff1, "L2_Val: %d", L2_Val);   //为什么第一位总是0？
		   sprintf((char *)str_buff2, "L1_Val: %d", L1_Val);    //看成33是因为我将它用sprintf转换为了十进制的数
		   sprintf((char *)str_buff3, "M_Val: %d",  M_Val); 
		   sprintf((char *)str_buff4, "R1_Val: %d", R1_Val);
		   sprintf((char *)str_buff5, "R2_Val: %d", R2_Val);
			 sprintf((char *)str_buff6, "Load_flag:%d", Load_flag);
		 
			  OLED_ShowString(0, 0, "GraySensor:", 16);
		    OLED_ShowString(0, 2,str_buff1, 8);
				OLED_ShowString(0, 3,str_buff2, 8);
				OLED_ShowString(0, 4,str_buff3, 8);
		    OLED_ShowString(0, 5,str_buff4, 8);
        OLED_ShowString(0, 6,str_buff5, 8);		 
				OLED_ShowString(0, 7,str_buff6, 8);	
		 break;
		 
		 case 122:
		 			sprintf((char *)str_buff1, "TargetRoom: %c", TargetRoom);
		 			sprintf((char *)str_buff2, "LoR: %d", LoR);
		 			sprintf((char *)str_buff3, "Num: %d", Num);
		 			sprintf((char *)str_buff4, "FindTask: %d", FindTask);
		 			sprintf((char *)str_buff5, "Load_flag:%d", Load_flag);
		 
		 	    OLED_ShowString(0, 0, "openmv:", 16);
		 			OLED_ShowString(0, 2,str_buff1, 8);
		 			OLED_ShowString(0, 3,str_buff2, 8);
		 			OLED_ShowString(0, 4,str_buff3, 8);
		 	    OLED_ShowString(0, 5,str_buff4, 8);
		         OLED_ShowString(0, 6,str_buff5, 8);
		 			break;
		 
		 case 123:
				OLED_ShowString(0, 0, "test 2:", 16);   //在test 1 里面调试获取编码器的数值   //下面第二个表示每秒转动的圈数
				sprintf((char *)str_buff1, "ADC_Volt: %2.2f", ADC_Volt );         
    		sprintf((char *)str_buff3, "Load_flag:%d", Load_flag);  

				OLED_ShowString(0, 3,str_buff1, 8);
        OLED_ShowString(0, 6,str_buff3, 8);
		   
			break;
			
		}
}


void MENU_Item_KEY(void)              //按键按下才给调参数，所以代码逻辑要放这里面
{
			switch(g_nButton)
			{
				
				//选择键，调参键 。 加数，当到了最后一个选项后返回第一个选项，或者在最后一级界面去调参
				case KEY1_PRES:     
			   	OLED_Clear();
				  Menu_Item++;
				
				     //下面是控制Menu_Item的switch
			    	switch(Menu_Item) 
					{
						//一级菜单只有 2 项
						case 3: 
							Menu_Item  = 1; 
						break;
						
						//TASK 二级界面 10~13，到底回到 10
						case 14:
							Menu_Item = 10;
						break;
						
						//TASK 三级界面 110~113，不可再往后翻
						case 111:
						case 112:
						case 113:
						case 114:
							Menu_Item--;
						break;	
						
						//TEST 二级界面 20~23，到底回到 20
						case 24:
							Menu_Item = 20;
						break;
						
						//TEST 三级界面 120~123，不可再往后翻
						case 121:
						case 122:
						case 123:
						case 124:
							Menu_Item--;
						break;
					}	
 					
				  break;
				
				case KEY2_PRES:     //确定键
						OLED_Clear();
					switch(Menu_Item)
					{
						//一级 -> 二级
						case 1: 
						case 2:
							Menu_Item *= 10;   //这里以乘10操作来实现确认的效果
						break;
						
						//TASK 二级 -> 三级
						case 10:      
						case 11:	
						case 12:
						case 13:
							Menu_Item  += 100;   //这里以进百位操作来实现确认的效果
						break;
						
						//TEST 二级 -> 三级
						case 20:      
						case 21:	
						case 22:
						case 23:
							Menu_Item  += 100;   //这里以进百位操作来实现确认的效果
						break;
					}					
				break;
				
					
					//返回主菜单或者状态清零
				case KEY3_PRES :              //home键 
					 Menu_Item = 0;    
//				  PWM_update(0,0,0,0);      //所有状态恢复默认
				
				 break;
			}
}
