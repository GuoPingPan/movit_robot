#ifndef __ETHERCAT_MOTOR_H__
#define __ETHERCAT_MOTOR_H__

#include "stdint.h"
#include "ethercat.h"


#ifdef __cplusplus
extern "C" {
#endif

/******* 是否打印电机状态信息 *******/
#define PRINT_MOTOR     0

/******* EtherCAT同步模式 *******/
typedef enum{
    Free_Run = 0,
    DC_Sync,
}EtherCAT_SyncMode;

/******* DC同步参数定义 *******/
#define DC_ShiftTime        250000      // DC偏移时间， 以ns为单位

typedef enum{
    YKS = 0,
    GQ
}Motor_Manufacturer;

typedef enum{
    YKS_36 = 0,
    YKS_94,
    YKS_150
}YKS_Motor_Type;

typedef enum{
    GQ_4538 = 0,
    GQ_5046,
    GQ_5047_Single,
    GQ_5047_Double
}GQ_Motor_Type;

typedef enum{
    MIX_CONTROL = 0,        // PD控制
    POS_CONTROL,            // 伺服位置
    SPD_CONTROL,            // 伺服速度
    TOR_CONTROL             // 伺服力矩
}Control_Mode;

typedef enum{
    Un_Initialized = 0,
    Request_Initialize,
    Complete_Initialize
}Init_Mode;

/******* PDO定义 *******/
typedef struct{
    float Position_Actual;
    float Speed_Actual;
    float Torque_Actual;
    uint16_t Error_flag;
} __attribute__((packed)) Motor_Information;

typedef struct{
    uint16_t Control_Mode;              // 控制模式，详见Control_Mode枚举类型
    float KP;
    float KD;
    float Position;
    float Speed;
    float Torque;
} __attribute__((packed)) Motor_Control;

typedef struct{
    Motor_Control Motor1, Motor2;
} __attribute__((packed)) PDO_Output;

typedef struct{
    Motor_Information Motor1, Motor2;
} __attribute__((packed)) PDO_Input;

typedef struct{
    uint8_t manufacturer :4;
    uint8_t model :4;
} __attribute__((packed)) Motor_TypeDef;

typedef struct{
    Motor_TypeDef Motor_Type;
    uint16_t Motor_ID;
    uint8_t Init_flag;
} __attribute__((packed)) Motor_Config;


/******* EtherCAT电机控制结构体 *******/
typedef struct{
    uint8_t slave;
    uint8_t passage;
    Motor_Config config;
    Motor_Control control;              // 控制指令
    Motor_Information information;      // 反馈信息
} __attribute__((packed)) EtherCAT_Motor_TypeDef;

extern EtherCAT_Motor_TypeDef   EtherCAT_Motor[];

uint8_t EtherCAT_Init(EtherCAT_SyncMode syncMode, uint32_t syncTime_us);
void EtherCAT_DeInit(void);
void EtherCAT_Loop(void);
uint8_t Get_MotorCount(void);

#ifdef __cplusplus
}
#endif

#endif