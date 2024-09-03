#include "EtherCAT_Motor.h"
#include "EtherCAT_Timer.h"
#include "stdio.h"

/******* EtherCAT电机定义 *******/
EtherCAT_Motor_TypeDef EtherCAT_Motor[] = {
// 从站号， 从站通信， {{电机厂商， 电机型号}， 电机ID， 电机初始化状态}， {控制模式， KP， KD， POS， SPD， TOR}， {R_POS, R_SPD, R_TOR, R_Error}
    // 左臂为 0
	{5, 2, {{GQ, GQ_4538}, 1, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},				// 负
	{5, 1, {{GQ, GQ_4538}, 2, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},				// 正
	{4, 2, {{GQ, GQ_4538}, 3, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},				// 负	
	{4, 1, {{GQ, GQ_5047_Double}, 4, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},		// 正
	{3, 2, {{YKS, YKS_36}, 5, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 负
	{3, 1, {{YKS, YKS_36}, 6, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 正
	{2, 2, {{YKS, YKS_36}, 7, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 负
	{2, 1, {{YKS, YKS_36}, 8, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 负	
	{6, 1, {{YKS, YKS_36}, 9, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 正
	{6, 2, {{YKS, YKS_36}, 10, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 正
	{7, 2, {{YKS, YKS_36}, 11, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 负
	{7, 1, {{YKS, YKS_36}, 12, Un_Initialized},{MIX_CONTROL, 10, 0.5, 0, 0, 0},{0, 0, 0, 0}},			// 正
	{8, 1, {{GQ, GQ_5047_Double}, 13, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},		// 负
	{8, 2, {{GQ, GQ_4538}, 14, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},			// 正
	{9, 2, {{GQ, GQ_4538}, 15, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},				// 负
	{9, 1, {{GQ, GQ_4538}, 16, Un_Initialized},{MIX_CONTROL, 1, 1, 0, 3, 3},{0, 0, 0, 0}},
};

/******* IOmap定义 *******/
char IOmap[72 * EC_MAXSLAVE];

/******* 状态切换重试次数 *******/
#define Retry_Count		3

/**
 * @brief 获取电机数量
 * @return 电机数量
 */

uint8_t Get_MotorCount(void){
	uint8_t count = sizeof(EtherCAT_Motor) / sizeof(EtherCAT_Motor_TypeDef);
	return count;
}

/**
 * @brief Motor初始化
 * @return Motor初始化状态
 */
uint8_t Motor_Init(void){
	while(1){
		// 发送从站初始化命令
		for(uint8_t i=0;i<Get_MotorCount();i++){
			if(EtherCAT_Motor[i].slave <= ec_slavecount){
				// 从站数量符合
				uint16_t slave = EtherCAT_Motor[i].slave;
				uint16_t index;
				if(EtherCAT_Motor[i].passage == 1){
					index = 0x8000;
				}
				else if(EtherCAT_Motor[i].passage == 2){
					index = 0x8001;
				}
				EtherCAT_Motor[i].config.Init_flag = Request_Initialize;

				ec_SDOwrite(slave, index, 1, FALSE, sizeof(EtherCAT_Motor[i].config.Motor_Type), &(EtherCAT_Motor[i].config.Motor_Type), EC_TIMEOUTRXM);
				ec_SDOwrite(slave, index, 2, FALSE, sizeof(EtherCAT_Motor[i].config.Motor_ID), &(EtherCAT_Motor[i].config.Motor_ID), EC_TIMEOUTRXM);
				ec_SDOwrite(slave, index, 3, FALSE, sizeof(EtherCAT_Motor[i].config.Init_flag), &(EtherCAT_Motor[i].config.Init_flag), EC_TIMEOUTRXM);
			}	
			else{
				EC_PRINT("Error: 超出最大从站数量！\n");
				EC_PRINT("Expected %d Slave, actually detected %d slaves!\n", EtherCAT_Motor[i].slave , ec_slavecount);
			}
		}

		// 等待从站初始化完成
		osal_usleep(1000 * 500);

		static uint8_t count = 0;
		uint16_t init_finished = 0;
		for(uint8_t i=0;i<Get_MotorCount();i++){
			if(EtherCAT_Motor[i].slave <= ec_slavecount){
				// 从站数量符合
				uint16_t slave = EtherCAT_Motor[i].slave;
				uint16_t index;
				if(EtherCAT_Motor[i].passage == 1){
					index = 0x8000;
				}
				else if(EtherCAT_Motor[i].passage == 2){
					index = 0x8001;
				}
				int size = sizeof(EtherCAT_Motor[i].config.Init_flag);
				ec_SDOread(slave, index, 3, FALSE, &size, &(EtherCAT_Motor[i].config.Init_flag), EC_TIMEOUTRXM);

				if(EtherCAT_Motor[i].config.Init_flag != Complete_Initialize){
					init_finished ++;
				}
				EC_PRINT("_%d_", EtherCAT_Motor[i].config.Init_flag);
			}
		}
		EC_PRINT("\n");

		if(init_finished == 0){
			EC_PRINT("电机初始化完成！\n");
			return TRUE;
		}

		count ++;
		if(count >= 10){
			printf("Error: 退出电机初始化！\n");
			return FALSE;
		}
		else{
			printf("电机初始化失败！尝试次数%d\n", count);
		}
	}
}

/**
 * @brief EtherCAT主站初始化
 * @return EtherCAT主站初始化状态
 */
int expected_wkc = 0;

uint8_t EtherCAT_Init(EtherCAT_SyncMode syncMode, uint32_t syncTime_us){
	EC_PRINT("/****** EtherCAT主站初始化 ******/\n");

	struct ec_adapter *adapter = ec_find_adapters();

	while(TRUE){
		/****** 开始EtherCAT主站初始化 ******/
		EC_PRINT("网口%s初始化！\n", adapter->name);
		if(ec_init(adapter->name)){
			EC_PRINT("/***** 网卡初始化成功 *****/\n");
			if(ec_config_init(FALSE) > 0){
				EC_PRINT("SOEM主站识别到 %d 个从站！\n", ec_slavecount);
				ec_config_map(&IOmap);


				// 获得电机所需最大从站数量
				uint8_t max_slave = 0;
				for(uint8_t i=0;i<Get_MotorCount();i++){
					if(EtherCAT_Motor[i].slave > max_slave){
						max_slave = EtherCAT_Motor[i].slave;
					}
				}
				if(max_slave != ec_slavecount){
					EC_PRINT("Error: 从站数量不符！\n");
					EC_PRINT("Expected %d Slaves, actually detected %d slaves!\n", max_slave, ec_slavecount);
				}
				
				// 电机初始化
				// SDO通信
				if(Motor_Init() == FALSE){
					EC_PRINT("/***** 电机初始化失败 *****/\n");
					return FALSE;
				}

				EC_PRINT("从机映射，转换状态至SAFE_OP!\n");
				/***** 等待所有从站到达SAFE_OP状态 *****/
				uint8_t cnt = 0;
				do{
					ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
					ec_readstate();
					for(uint16_t idx=0; idx<=ec_slavecount; idx++){
						EC_PRINT("_%x_", ec_slave[idx].state);
					}
					EC_PRINT(".");
					cnt ++;
					if(cnt == Retry_Count){			// 超出重复次数
						EC_PRINT("\n从站状态转换失败！\n");
						EC_PRINT("/****** EtherCAT主站初始化失败 ******/\n");
						return FALSE;
					}
				}while(ec_slave[0].state != EC_STATE_SAFE_OP);
				EC_PRINT("\n从站状态转换成功！\n");

				/***** 配置分布式时钟 *****/
				ec_configdc();

				/***** 打印从站信息 *****/
				for(uint16_t cnt = 1 ; cnt <= ec_slavecount ; cnt++)
				{
					EC_PRINT("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
							cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
							ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
					EC_PRINT(" Configured address: %x\n", ec_slave[cnt].configadr);
					EC_PRINT(" Outputs address: %p\n", ec_slave[cnt].outputs);
					EC_PRINT(" Inputs address: %p\n", ec_slave[cnt].inputs);

					for(uint16_t j = 0 ; j < ec_slave[cnt].FMMUunused ; j++)
					{
						EC_PRINT(" FMMU%1d Ls:%x Ll:%4d Lsb:%d Leb:%d Ps:%x Psb:%d Ty:%x Act:%x\n", j,
								(int)ec_slave[cnt].FMMU[j].LogStart, ec_slave[cnt].FMMU[j].LogLength, ec_slave[cnt].FMMU[j].LogStartbit,
								ec_slave[cnt].FMMU[j].LogEndbit, ec_slave[cnt].FMMU[j].PhysStart, ec_slave[cnt].FMMU[j].PhysStartBit,
								ec_slave[cnt].FMMU[j].FMMUtype, ec_slave[cnt].FMMU[j].FMMUactive);
					}
					EC_PRINT(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
								ec_slave[cnt].FMMU0func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU2func, ec_slave[cnt].FMMU3func);
				}

				expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

				EC_PRINT("请求所有从站到OP状态！\n");

				/***** 发送一帧数据，准备进入OP状态 *****/
				ec_send_processdata();
				ec_receive_processdata(EC_TIMEOUTRET);

				/***** 请求所有从站到OP状态 *****/
				ec_slave[0].state = EC_STATE_OPERATIONAL;
				ec_writestate(0);

				cnt = 0;
				do{
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
					ec_readstate();
					for(uint16_t idx=0; idx<=ec_slavecount; idx++){
						EC_PRINT("_%x_", ec_slave[idx].state);
					}
					EC_PRINT(".");

					cnt ++;
					EC_PRINT("try %d", cnt);
					if(cnt == Retry_Count){			// 超出重复次数
						EC_PRINT("\n从站状态转换失败！\n");
						ec_readstate();
						for(uint16_t idx=0; idx<ec_slavecount; idx++){
							EC_PRINT("Slave %d State=0x%04x StatusCode=0x%04x\n",
															idx, ec_slave[idx].state, ec_slave[idx].ALstatuscode);
						}
						EC_PRINT("/****** EtherCAT主站初始化失败 ******/\n");
						return FALSE;
					}
				}while(ec_slave[0].state != EC_STATE_OPERATIONAL);

				EC_PRINT("\n从站状态转换成功！\n");

				// 开启定时线程
				if(syncMode == DC_Sync){
					for(uint16_t idx=1;idx<=ec_slavecount;idx++){
						ec_dcsync0(idx, TRUE, syncTime_us * 1000, DC_ShiftTime);
						EC_PRINT("从站 %d 启用DC同步模式！\n", idx);
					}
				}

				
				Timer_Init(&EtherCAT_Loop);
				Start_Timer(syncTime_us * 1000);

				// /****** 将PDO数据映射至本地 ******/
				// PDO_Input_1 = (PDO_Input *)ec_slave[1].inputs;
				// PDO_Output_1 = (PDO_Output *)ec_slave[1].outputs;

				EC_PRINT("/****** EtherCAT主站初始化成功 ******/\n");
				return TRUE;
			}else{
				EC_PRINT("SOEM主站未识别到从站！\n");
				if(adapter->next != NULL){
					adapter = adapter->next;
					continue;
				}
			}
		}else{
			EC_PRINT("/***** 网卡初始化失败 *****/\n");
		}
		return FALSE;
	}
	
}

void EtherCAT_Loop(void){

	static uint8_t wkc_err_count = 0;

#if PRINT_MOTOR
	ec_timet current_time = osal_current_time();
	static double last_second = 0;
	double current_second = (double)current_time.sec + (double)current_time.usec / 1000000.0f;
	double Freq = 1.0f / (current_second - last_second);
	last_second = current_second;
	printf("Freq: %f, Sec: %d, uSec: %d!\n", Freq, current_time.sec, current_time.usec);
#endif

	// 映射TXPDO数据
	for(uint8_t i=0;i<Get_MotorCount();i++){
		if(EtherCAT_Motor[i].slave <= ec_slavecount){
			// 从站数量符合
			PDO_Output *output = (PDO_Output *)ec_slave[EtherCAT_Motor[i].slave].outputs;

			if(EtherCAT_Motor[i].passage == 1){
				output->Motor1 = EtherCAT_Motor[i].control;
			}
			else if(EtherCAT_Motor[i].passage == 2){
				output->Motor2 = EtherCAT_Motor[i].control;
			}
			
		}	
		else{
			EC_PRINT("\x1b[31m[EtherCAT Error]  超出最大从站数量！\x1b[0m\n");
			EC_PRINT("Expected %d Slave, actually detected %d slaves!\n", EtherCAT_Motor[i].slave , ec_slavecount);
		}
	}

	ec_send_processdata();
	int wkc = ec_receive_processdata(EC_TIMEOUTRET);

	// 映射RXPDO数据
	for(uint8_t i=0;i<Get_MotorCount();i++){
		if(EtherCAT_Motor[i].slave <= ec_slavecount){
			// 从站数量符合
			PDO_Input *input = (PDO_Input *)ec_slave[EtherCAT_Motor[i].slave].inputs;

			if(EtherCAT_Motor[i].passage == 1){
				EtherCAT_Motor[i].information = input->Motor1;
			}
			else if(EtherCAT_Motor[i].passage == 2){
				EtherCAT_Motor[i].information = input->Motor2;
			}
			if(EtherCAT_Motor[i].information.Error_flag != 0){
				EC_PRINT("\x1b[31m[EtherCAT Error]  Motor %d disconnect, ErrorCode: %d!\x1b[0m\n", i, EtherCAT_Motor[i].information.Error_flag);
				EtherCAT_DeInit();
			}
		}	
		else{
			EC_PRINT("\x1b[31m[EtherCAT Error]  超出最大从站数量！\x1b[0m\n");
			EC_PRINT("Expected %d Slave, actually detected %d slaves!\n", EtherCAT_Motor[i].slave , ec_slavecount);
		}
	}
	
#if PRINT_MOTOR
	for(uint8_t i=0;i<Get_MotorCount();i++){
		EC_PRINT("Motor %d: Pos %f, Spd %f, Tor %f, Err %d\n", i, EtherCAT_Motor[i].information.Position_Actual, EtherCAT_Motor[i].information.Speed_Actual, \
		EtherCAT_Motor[i].information.Torque_Actual, EtherCAT_Motor[i].information.Error_flag);
	}
#endif
	
	if(wkc < expected_wkc){
		EC_PRINT("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
        wkc_err_count++;
		if(wkc_err_count >= 10){
			EC_PRINT("Error: 退出运行模式！\n");
			EtherCAT_DeInit();
		}
	}else{
		wkc_err_count = 0;
	}
}

void EtherCAT_DeInit(void){
	Stop_Timer();
	ec_slave[0].state = EC_STATE_INIT;
	ec_writestate(0);

	EC_PRINT("从站状态切换至INIT状态！\n");
	EC_PRINT("/****** EtherCAT主站停止运行 ******/\n");
}