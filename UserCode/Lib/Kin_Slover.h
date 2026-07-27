#ifndef KIN_SLOVER_H
#define KIN_SLOVER_H

#ifdef __cplusplus
extern "C" {
#endif


typedef struct{
  double Tar_muzhi_wanqu_jiaodu_2;    
  double Tar_muzhi_baidong_jiaodu_2;   
  double Tar_shizhi_wanqu_jiaodu_2;    
  double Tar_zhongzhi_wanqu_jiaodu_2; 
  double Tar_wumingzhi_wanqu_jiaodu_2;                
  double Tar_xiaomuzhi_wanqu_jiaodu_2;
                                    
  double Tar_muzhi_wanqu_jiaosudu_2;   
  double now_M1_pos;                   
  double Tar_muzhi_baidong_jiaosudu;   
  double now_M2_pos;                   
  double Tar_shizhi_wanqu_jiaosudu_2; 
  double now_M3_pos;                   
  double Tar_zhongzhi_wanqu_jiaosudu_2;                     
  double now_M4_pos;                   
  double Tar_wumingzhi_wanqu_jiaosudu_2;                    
  double now_M5_pos;                   
  double Tar_xiaomuzhi_wanqu_jiaosudu_2;                       
  double now_M6_pos;  
	
  double now_M1_pos2;                 
  double now_M2_pos2;                  
  double now_M3_pos2;                  
  double now_M4_pos2;                  
  double now_M5_pos2;                  
  double now_M6_pos2; 
  
  double now_M1_pos1;                  
  double now_M1_spd;                   
  double now_M2_pos1;                  
  double now_M2_spd;                   
  double now_M3_pos1;                 
  double now_M3_spd;                  
  double now_M4_pos1;                  
  double now_M4_spd;                   
  double now_M5_pos1;                  
  double now_M5_spd;                   
  double now_M6_pos1;                  
  double now_M6_spd;                   
}INPUT;


typedef struct{
  double Tar_M1_pos;                   
  double Tar_M2_pos;                   
  double Tar_M3_pos;                   
  double Tar_M4_pos;                   
  double Tar_M5_pos;                   
  double Tar_M6_pos;
	
  double Tar_M1_spd;                   
  double Tar_M2_spd;                   
  double Tar_M3_spd;                  
  double Tar_M4_spd;                   
  double Tar_M5_spd;                  
  double Tar_M6_spd;
	
  double now_muzhi_wanqu_jiaodu_2;    
  double now_muzhi_baidong_jiaodu_2;   
  double now_shizhi_wanqu_jiaodu_2;   
  double now_zhongzhi_wanqu_jiaodu_2; 
  double now_wumingzhi_wanqu_jiaodu_2;                     
  double now_xiaomuzhi_wanqu_jiaodu_2;
                                     
  double now_muzhi_wanqu_jiaosudu_2;  
  double now_muzhi_baidong_jiaosudu;   
  double now_shizhi_wanqu_jiaosudu_2; 
  double now_zhongzhi_wanqu_jiaosudu_2;                       
  double now_wumingzhi_wanqu_jiaosudu_2;                                  
  double now_xiaomuzhi_wanqu_jiaosudu_2;
}OUTPUT;


OUTPUT modelCalculate(INPUT in);



#ifdef __cplusplus
}
#endif

#endif 


