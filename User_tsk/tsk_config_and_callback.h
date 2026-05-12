#ifndef TSK_CONFIG_AND_CALLBACK_H
#define TSK_CONFIG_AND_CALLBACK_H

#ifdef __cplusplus
extern "C" {
#endif

void Task_Init();
void Task_Loop();

void Task1ms_TIM2_Callback();
#ifdef __cplusplus
};
#endif

#endif