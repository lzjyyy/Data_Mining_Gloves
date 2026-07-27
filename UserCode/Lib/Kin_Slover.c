#include "Kin_Slover.h"
#include "math.h"
#include "arm_math.h"
#include "Kmodel.h"

extern ExtY rtY;
extern ExtU rtU;


/************************************/
OUTPUT modelCalculate(INPUT in)
{
	OUTPUT out;
	rtU.Tar_muzhi_wanqu_jiaodu_2  	 	=  in.Tar_muzhi_wanqu_jiaodu_2 ;
	rtU.Tar_muzhi_baidong_jiaodu_2 		=  in.Tar_muzhi_baidong_jiaodu_2; 
	rtU.Tar_shizhi_wanqu_jiaodu_2  		=	in.Tar_shizhi_wanqu_jiaodu_2; 
	rtU.Tar_zhongzhi_wanqu_jiaodu_2 		=	in.Tar_zhongzhi_wanqu_jiaodu_2; 
	rtU.Tar_wumingzhi_wanqu_jiaodu_2 	=  in.Tar_wumingzhi_wanqu_jiaodu_2;             
	rtU.Tar_xiaomuzhi_wanqu_jiaodu_2		=	in.Tar_xiaomuzhi_wanqu_jiaodu_2;	
													
	rtU.Tar_muzhi_wanqu_jiaosudu_2  		= 	in.Tar_muzhi_wanqu_jiaosudu_2; 
	rtU.now_M1_pos                 		=	in.now_M1_pos;             
	rtU.Tar_muzhi_baidong_jiaosudu  		=	in.Tar_muzhi_baidong_jiaosudu;  
	rtU.now_M2_pos                  		=	in.now_M2_pos;               
	rtU.Tar_shizhi_wanqu_jiaosudu_2		=	in.Tar_shizhi_wanqu_jiaosudu_2;	
	rtU.now_M3_pos                 		=	in.now_M3_pos;       
	rtU.Tar_zhongzhi_wanqu_jiaosudu_2 	=  in.Tar_zhongzhi_wanqu_jiaosudu_2;                 
	rtU.now_M4_pos            				=	in.now_M4_pos;         
	rtU.Tar_wumingzhi_wanqu_jiaosudu_2  =  in.Tar_wumingzhi_wanqu_jiaosudu_2;               
	rtU.now_M5_pos                   	=	in.now_M5_pos;   
	rtU.Tar_xiaomuzhi_wanqu_jiaosudu_2  =  in.Tar_xiaomuzhi_wanqu_jiaosudu_2;                   
	rtU.now_M6_pos  							=	in.now_M6_pos;
		
	rtU.now_M1_pos2  =  in.now_M1_pos2;      
	rtU.now_M2_pos2  =  in.now_M2_pos2;            
	rtU.now_M3_pos2  =  in.now_M3_pos2;             
	rtU.now_M4_pos2  =  in.now_M4_pos2;               
	rtU.now_M5_pos2  =  in.now_M5_pos2;             
	rtU.now_M6_pos2  =  in.now_M6_pos2;
	  
	rtU.now_M1_pos1  =  in.now_M1_pos1;            
	rtU.now_M1_spd   =  in.now_M1_spd ;          
	rtU.now_M2_pos1  =  in.now_M2_pos1;              
	rtU.now_M2_spd   =  in.now_M2_spd ;            
	rtU.now_M3_pos1  =  in.now_M3_pos1;           
	rtU.now_M3_spd   =  in.now_M3_spd ;           
	rtU.now_M4_pos1  =  in.now_M4_pos1;            
	rtU.now_M4_spd   =  in.now_M4_spd ;             
	rtU.now_M5_pos1  =  in.now_M5_pos1;               
	rtU.now_M5_spd   =  in.now_M5_spd ;             
	rtU.now_M6_pos1  =  in.now_M6_pos1;              
	rtU.now_M6_spd   =  in.now_M6_spd ;
	
	Kmodel_step();
	
/* 用户期望的关节角度&角速度--->映射到对应电机的位置&速度 */
	out.Tar_M1_pos  =   rtY.Tar_M1_pos;  /*1号电机位置Tar*/           
	out.Tar_M2_pos  =   rtY.Tar_M2_pos;  /*2号电机位置Tar*/
	out.Tar_M3_pos  =   rtY.Tar_M3_pos;  /*3号电机位置Tar*/      
	out.Tar_M4_pos  =   rtY.Tar_M4_pos;  /*4号电机位置Tar*/      
	out.Tar_M5_pos  =   rtY.Tar_M5_pos;  /*5号电机位置Tar*/
	out.Tar_M6_pos	 =	  rtY.Tar_M6_pos;  /*6号电机位置Tar*/

	out.Tar_M1_spd  =   rtY.Tar_M1_spd;  /*1号电机速度Tar*/          
	out.Tar_M2_spd  =   rtY.Tar_M2_spd;  /*2号电机速度Tar*/      
	out.Tar_M3_spd  =   rtY.Tar_M3_spd;  /*3号电机速度Tar*/   
	out.Tar_M4_spd  =   rtY.Tar_M4_spd;  /*4号电机速度Tar*/    
	out.Tar_M5_spd  =   rtY.Tar_M5_spd;  /*5号电机速度Tar*/   
	out.Tar_M6_spd  =	  rtY.Tar_M6_spd;  /*6号电机速度Tar*/ 
	
/* 回传当前电机的位置&速度-->映射到当前关节角度&角速度---> */
	out.now_muzhi_wanqu_jiaodu_2			=	 rtY.now_muzhi_wanqu_jiaodu_2;         /*当前拇指弯曲关节角度*/
	out.now_muzhi_baidong_jiaodu_2		=	 rtY.now_muzhi_baidong_jiaodu_2;		   /*当前拇指摆动关节角度*/
	out.now_shizhi_wanqu_jiaodu_2			=	 rtY.now_shizhi_wanqu_jiaodu_2;		   /*当前食指弯曲关节角度*/
	out.now_zhongzhi_wanqu_jiaodu_2		=	 rtY.now_zhongzhi_wanqu_jiaodu_2;		/*当前中指弯曲关节角度*/
	out.now_wumingzhi_wanqu_jiaodu_2 	=   rtY.now_wumingzhi_wanqu_jiaodu_2;     /*当前无名指弯曲关节角度*/
	out.now_xiaomuzhi_wanqu_jiaodu_2		=	 rtY.now_xiaomuzhi_wanqu_jiaodu_2;		/*当前小拇指弯曲关节角度*/
												 
	out.now_muzhi_wanqu_jiaosudu_2  		=	 rtY.now_muzhi_wanqu_jiaosudu_2;			/*当前拇指弯曲关节角速度*/
	out.now_muzhi_baidong_jiaosudu  		=	 rtY.now_muzhi_baidong_jiaosudu;			/*当前拇指摆动关节角速度*/
	out.now_shizhi_wanqu_jiaosudu_2		=	 rtY.now_shizhi_wanqu_jiaosudu_2;		/*当前食指弯曲关节角速度*/
	out.now_zhongzhi_wanqu_jiaosudu_2	=   rtY.now_zhongzhi_wanqu_jiaosudu_2;    /*当前中指弯曲关节角速度*/           
	out.now_wumingzhi_wanqu_jiaosudu_2	=   rtY.now_wumingzhi_wanqu_jiaosudu_2;   /*当前无名指弯曲关节角速度*/                     
	out.now_xiaomuzhi_wanqu_jiaosudu_2	=	 rtY.now_xiaomuzhi_wanqu_jiaosudu_2;	/*当前小拇指弯曲关节角速度*/
  
  return out;
}


