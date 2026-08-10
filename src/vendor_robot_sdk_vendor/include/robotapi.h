#ifndef ROBOTAPI_H
#define ROBOTAPI_H

#include"basestruct.h"
#ifdef WIN32
    #ifdef  CRSDK_EXPORT
    #define CRSDKAPI __declspec(dllexport)
    #else
    #define CRSDKAPI __declspec(dllimport)
    #endif
#else
    #ifdef  CRSDK_EXPORT
    #define CRSDKAPI
    #else
    #define CRSDKAPI
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CR_ROBOT_SDK_VERSION  "v2.50c"

/**
 * @brief   获取版本号
 * @return  返回版本信息
 */
CRSDKAPI CRresult cr_get_sdk_version(char *version);

/**
 * @brief   机器人构造函数
 * @param[out]   robotHandle   指向创建的机械臂实例句柄
 * @param[in]    ipAddr        机械臂IP地址
 * @param[in]    port          机械臂TCP端口号，例如连接真实机械臂为：2323，虚拟控制柜：2325
 * @param[in]    passwd        登录密码
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_create_robot(RobotHandle* robotHandle, const char * ipAddr, int port,const char * passwd);

/**
 * @brief   机器人释放函数
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_destroy_robot(RobotHandle robotHandle);

/*
*************机器人控制*************
*/

/**
 * @brief   读控制柜硬件序列号
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   productSn     读取的控制柜硬件序列号
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_productInfo(RobotHandle robotHandle, char* productSn);

/**
 * @brief   读取整臂序列号
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   Sn            读取整臂序列号
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_productSn(RobotHandle robotHandle, char *Sn);

/**
 * @brief   读控制柜系统日期时间
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[out]   systemDateTime 读取的控制柜系统日期时间
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_SystemDateTime(RobotHandle robotHandle, char* systemDateTime);

/**
 * @brief   设置控制柜系统日期时间
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[out]   systemDateTime 读取的控制柜系统日期时间
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_system_date_time(RobotHandle robotHandle, WorldTime systemDateTime);

/**
 * @brief   关闭机器人
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_shutdown(RobotHandle robotHandle);

/**
 * @brief   机器人上电
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_poweron(RobotHandle robotHandle);

/**
 * @brief   机器人断电
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_poweroff(RobotHandle robotHandle);

/**
 * @brief   机器人使能
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_enable(RobotHandle robotHandle);

/**
 * @brief   机器人关使能
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_disable(RobotHandle robotHandle);

/**
 * @brief   机器人故障复位
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_FaultReset(RobotHandle robotHandle);

/**
 * @brief   获取机器人电源按钮是否被按下的状态
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   status        机器人电源按钮状态
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_power_button_pressed(RobotHandle robotHandle, BOOL *status);

/**
 * @brief   加载程序
 * @param[in]   robotHandle   机械臂实例句柄
 * @param[in]   programfile   需加载的程序
 * @return      CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_downloadProgram(RobotHandle robotHandle, const char* programfile);

/**
 * @brief   加载程序（加密）
 * @param[in]   robotHandle   机械臂实例句柄
 * @param[in]   programfile   需加载的程序
 * @return      CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_downloadProgram_encryption(RobotHandle robotHandle, const char* programfile);

/**
 * @brief   上传程序
 * @param[in]   robotHandle   机械臂实例句柄
 * @param[out]   programfile   上传的程序
 * @return      CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_uploadProgram(RobotHandle robotHandle,const char* programfile);


/**
 * @brief   运行当前程序
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_play(RobotHandle robotHandle);

/**
 * @brief   停止运行
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_stop(RobotHandle robotHandle);

/**
 * @brief   暂停运行
 * @param[in]    robotHandle   机械臂实例句柄
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_pause(RobotHandle robotHandle);

/**
 * @brief   获取机械臂日志
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   logMsgs       机械臂日志信息数组
 * @param[out]   logMsgNumber  日志个数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_logMsgs(RobotHandle robotHandle, RobotLogMsg *logMsgs, int* logMsgNumber);

/**
 * @brief   获取机械臂历史所有日志
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   logMsgs       机械臂日志信息数组
 * @param[out]   logMsgNumber  日志个数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_sys_history_log_get(RobotHandle robotHandle,RobotLogMsg *logMsgs, int* logMsgNumber);

/**
 * @brief   获取控制柜的自动或手动状态
 * @param[in]    robotHandle        机械臂实例句柄
 * @param[out]   controlmode        0-自动 1-手动
 * @return       CRresult           返回处理结果
 */
CRSDKAPI CRresult cr_get_controlMode(RobotHandle robotHandle, int *controlmode);

/**
 * @brief   获取版本号信息
 * @param[in]    robotHandle        机械臂实例句柄
 * @param[out]   version            返回版本号信息
 * @return       CRresult           返回处理结果
 */
CRSDKAPI CRresult cr_get_sysVersion(RobotHandle robotHandle, SysVersion *version);

/**
 * @brief 读取机型型号
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      robotModel       机械臂型号
 * @param[in]       len              机械臂型号数组长度
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_sys_robotModel_get(RobotHandle robotHandle,char *robotModel,int len);

/**
 * @brief  获取控制柜型号
 * @param[in]   robotHandle         机械臂实例句柄
 * @param[out]  controllerType      控制柜型号
 * @return      CRresult            返回处理结果
 */
CRSDKAPI CRresult cr_get_controllerType(RobotHandle robotHandle,char *controllerType);

/**
 * @brief 触发黑匣子数据
 * @param[in]       robotHandle      机械臂实例句柄
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_sys_blackBoxData_trigger(RobotHandle robotHandle);

/**
 * @brief 触发系统更新
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      errCode          控制柜返回错误码
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_sys_update_trigger(RobotHandle robotHandle, int* errCode);

/**
 * @brief 关闭手持示教器电源（未保存的工程数据将丢失），控制器继续正常运行
 * @param[in]       robotHandle      机械臂实例句柄
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_sys_tp_poweroff(RobotHandle robotHandle);
/*
*************IO控制*************
*/

/**
 * @brief   写控制柜标准数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[in]    val           待设定的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_stdDigitalOut(RobotHandle robotHandle, int index, BOOL val);

/**
 * @brief   写控制柜可配置数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[in]    val           待设定的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_configDigitalOut(RobotHandle robotHandle, int index, BOOL val);

/**
 * @brief   写控制柜模拟量输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[in]    val           待设定的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_stdAnalogOut(RobotHandle robotHandle, int index, double val);

/**
 * @brief   写工具端数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[in]    val           待设定的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_toolDigitalOut(RobotHandle robotHandle, int index, BOOL val);

/**
 * @brief   读控制柜标准数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_stdDigitalOut(RobotHandle robotHandle,int index, BOOL* val);

/**
 * @brief   读控制柜可配置数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_configDigitalOut(RobotHandle robotHandle, int index, BOOL* val);

/**
 * @brief   读控制柜所有可配置数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   arrayVal      待读取的值所存放的位置
 * @param[in]    arraySize     读取的数量
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_all_configDigitalOut(RobotHandle robotHandle, BOOL* arrayVal,int arraySize);

/**
 * @brief   读控制柜模拟量输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_stdAnalogOut(RobotHandle robotHandle, int index, double* val);

/**
 * @brief   读工具端数字输出
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_toolDigitalOut(RobotHandle robotHandle, int index, BOOL* val);

/**
 * @brief   读控制柜标准数字输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_stdDigitalIn(RobotHandle robotHandle, int index, BOOL* val);

/**
 * @brief   读控制柜可配置数字输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_configDigitalIn(RobotHandle robotHandle, int index, BOOL* val);

/**
 * @brief   读控制柜所有可配置数字输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   arrayVal      待读取的值所存放的位置
 * @param[in]    arraySize     读取的数量
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_all_configDigitalIn(RobotHandle robotHandle, BOOL* arrayVal,int arraySize);

/**
 * @brief   读控制柜模拟量输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_stdAnalogIn(RobotHandle robotHandle, int index, double* val);

/**
 * @brief   读工具端数字输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_toolDigitalIn(RobotHandle robotHandle, int index, BOOL* val);

/**
 * @brief   读工具端模拟量输入
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    index         索引号
 * @param[out]   val           待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_toolAnalogIn(RobotHandle robotHandle, int index, double* val);

/**
 * @brief   读机械臂控制柜和工具端上所有输入数字量和模拟量
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   controllerDI  控制柜标准数字输入量
 * @param[out]   controllerCI  控制柜可配置数字输入量
 * @param[out]   controllerAI  控制柜模拟输入量
 * @param[out]   toolDI        工具端标准数字输入量
 * @param[out]   toolAI        工具端模拟输入量
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_allDAInput(RobotHandle robotHandle, BOOL controllerDI[CR6_DI_NUM], BOOL controllerCI[CR6_CI_NUM], double controllerAI[CR6_AI_NUM], BOOL toolDI[CR6_TOOL_DI_NUM], double toolAI[CR6_TOOL_AI_NUM]);

/**
 * @brief   读机械臂控制柜和工具端上所有输出数字量和模拟量
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   controllerDO  控制柜标准数字输出量
 * @param[out]   controllerCO  控制柜可配置数字输出量
 * @param[out]   controllerAO  控制柜模拟输出量
 * @param[out]   toolDO        工具端标准数字输出量
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_allDAOutput(RobotHandle robotHandle, BOOL controllerDO[CR6_DO_NUM], BOOL controllerCO[CR6_CO_NUM], double controllerAO[CR6_AO_NUM], BOOL toolDO[CR6_TOOL_DO_NUM]);

/*
*************关节数据*************
*/

/**
 * @brief   读实际关节位置
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   pos           待读取的值，关节1~6（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualPos(RobotHandle robotHandle, double pos[ROB_AXIS_NUM]);

/**
 * @brief   读目标关节位置
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   pos           待读取的值，关节1~6（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointTargetPos(RobotHandle robotHandle, double pos[ROB_AXIS_NUM]);

/**
 * @brief   读实际关节速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   velocity      待读取的值，关节1~6（单位：°/s）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualVelocity(RobotHandle robotHandle, double velocity[ROB_AXIS_NUM]);

/**
 * @brief   读目标关节速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   velocity      待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointTargetVelocity(RobotHandle robotHandle, double velocity[ROB_AXIS_NUM]);

/**
 * @brief   读实际关节加速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   acceleration  待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualAcceleration(RobotHandle robotHandle, double acceleration[ROB_AXIS_NUM]);

/**
 * @brief   读目标关节加速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   acceleration  待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointTargetAcceleration(RobotHandle robotHandle, double acceleration[ROB_AXIS_NUM]);

/**
 * @brief   读实际关节电机电流
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   current       待读取的值，关节1~6（单位：mA）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualCurrent(RobotHandle robotHandle, double current[ROB_AXIS_NUM]);

/**
 * @brief   读目标关节电机电流
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   current       待读取的值，关节1~6（单位：mA）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointTargetCurrent(RobotHandle robotHandle, double current[ROB_AXIS_NUM]);

/**
 * @brief   读实际关节转矩
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   torque        待读取的值，关节1~6（单位：额定转矩/1000）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualTorque(RobotHandle robotHandle, double torque[ROB_AXIS_NUM]);

/**
 * @brief   读实际关节采集的母线电压
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   voltage       待读取的值，关节1~6（单位：V)
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointActualVoltage(RobotHandle robotHandle, double voltage[ROB_AXIS_NUM]);

/**
 * @brief   读关节温度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   temperature   待读取的值，关节1~6（单位：℃)
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointTemperature(RobotHandle robotHandle, double temperature[ROB_AXIS_NUM]);

/**
 * @brief   读关节模式
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   jointMode     待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_jointMode(RobotHandle robotHandle,enum JointModes jointMode[ROB_AXIS_NUM]);

/*
*************TCP数据*************
*/
/**
 * @brief   读实际tcp位置
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   pose          待读取的值：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_tcpActualPose(RobotHandle robotHandle, double pose[ROB_AXIS_NUM]);

/**
 * @brief   读目标tcp位置
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   pose          待读取的值：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_tcpTargetPose(RobotHandle robotHandle, double pose[ROB_AXIS_NUM]);

/**
 * @brief   读实际tcp速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   speed         待读取的值，关节1~6（单位：mm/s)
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_tcpActualSpeed(RobotHandle robotHandle, double speed[ROB_AXIS_NUM]);

/**
 * @brief   读目标tcp速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   speed         待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_tcpTargetSpeed(RobotHandle robotHandle, double speed[ROB_AXIS_NUM]);

/**
 * @brief   读实际tcp加速度
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   acceleration  待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_tcpActualAcceleration(RobotHandle robotHandle, double acceleration[ROB_AXIS_NUM]);

/**
 * @brief   读所有TCP偏移信息列表
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   tcpMsgList    读取所有TCP偏移信息所在的数组
 * @param[out]   validNumber   读取TCP偏移信息数组中有效数据的长度
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_allTCPmsg(RobotHandle robotHandle, TCPMsg *tcpMsgList, int *validNumber);

/**
 * @brief   读当前使用的TCP偏移信息
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   tcpMsg        当前使用的TCP偏移信息
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_currentTCPmsg(RobotHandle robotHandle, TCPMsg *tcpMsg);

/**
 * @brief   读所有负载列表
 * @param[in]    robotHandle       机械臂实例句柄
 * @param[out]   payloadMsgList    读取所有负载所在的数组
 * @param[out]   validNumber       读取负载数组中有效数据的长度
 * @return       CRresult          返回处理结果
 */
CRSDKAPI CRresult cr_get_allPayloadmsg(RobotHandle robotHandle, PayLoad *payloadMsgList, int *validNumber);

/**
 * @brief   读取当前负载信息
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   payloadMsg    读取当前负载信息
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_currentPayloadmsg(RobotHandle robotHandle, PayLoad *payloadMsg);

/**
 * @brief   获取指定TCPoffset的位姿
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    dstTCPoffset   需变换后TCP偏移量：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[out]   dstPose        变化为dstTCPoffset后的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_AssignTCP_Pose(RobotHandle robotHandle, double dstTCPoffset[ROB_AXIS_NUM], double dstPose[ROB_AXIS_NUM]);

/*
*************通讯变量读写操作*************
*/
/**
 * @brief   写bool寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[in]    regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_boolRegValue(RobotHandle robotHandle, unsigned int startAddress,unsigned int offsetLength, BOOL* regData);
/**
 * @brief   读bool寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[out]   regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_boolRegValue(RobotHandle robotHandle,  unsigned int startAddress,unsigned int offsetLength, BOOL* regData);
/**
 * @brief   写int16寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[in]    regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_int16RegValue(RobotHandle robotHandle, unsigned int startAddress,unsigned int offsetLength, int* regData);
/**
 * @brief   读int16寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[out]   regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_int16RegValue(RobotHandle robotHandle,  unsigned int startAddress,unsigned int offsetLength, int* regData);
/**
 * @brief   写int32寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[in]    regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_int32RegValue(RobotHandle robotHandle, unsigned int startAddress,unsigned int offsetLength, int* regData);
/**
 * @brief   读int32寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[out]   regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_int32RegValue(RobotHandle robotHandle,  unsigned int startAddress,unsigned int offsetLength, int* regData);
/**
 * @brief   写float寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[in]    regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_floatRegValue(RobotHandle robotHandle, unsigned int startAddress,unsigned int offsetLength, float* regData);
/**
 * @brief   读float寄存器
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    startAddress   寄存器起始地址
 * @param[in]    offsetLength   寄存器偏移地址
 * @param[out]   regData        寄存器数据
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_floatRegValue(RobotHandle robotHandle,  unsigned int startAddress,unsigned int offsetLength, float* regData);
/*
*************机械臂状态*************
*/

/*
*************安装变量读写操作*************
*/
/**
 * @brief   读安装变量
 * @param[in]    robotHandle     机械臂实例句柄
 * @param[in]    installVarName  需读取安装变量名称
 * @param[out]   installVar      读取的安装变量值
 * @return       CRresult        返回处理结果
 */
CRSDKAPI CRresult cr_get_intallVarValue(RobotHandle robotHandle, const char* installVarName, VariableMsg *installVar);

/**
 * @brief   写安装变量
 * @param[in]    robotHandle     机械臂实例句柄
 * @param[in]    installVarName  需写入的安装变量名称
 * @param[in]    installVar      需赋值的安装变量值
 * @return       CRresult        返回处理结果
 */
CRSDKAPI CRresult cr_set_installVarValue(RobotHandle robotHandle, const char* installVarName, VariableMsg* installVar);

/**
 * @brief   读机械臂当前状态
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   robotMode     机械臂当前状态
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_robotMode(RobotHandle robotHandle, enum RobotModes* robotMode);
/**
 * @brief   读机械臂运动状态
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   isRobotMoving 机械臂当前是否运动
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_robotMoveStatus(RobotHandle robotHandle, BOOL* isRobotMoving);

/**
 * @brief   读机械臂DH参数
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   dhPara         机械臂DH参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_DhParmeter(RobotHandle robotHandle, double dhPara[DH_PARA_MAX_NUM]);

/**
 * @brief   读机械臂标准DH参数
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   dhPara         机械臂DH参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_stdDhParmeter(RobotHandle robotHandle, double dhPara[DH_PARA_MAX_NUM]);

/**
 * @brief   读机械臂动力学参数
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   dynamicPara   机械臂动力学参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_DynamicParmeter(RobotHandle robotHandle, double dynamicPara[DY_PARA_MAX_NUM]);

/**
 * @brief   读机械臂零位补偿值
 * @param[in]    robotHandle            机械臂实例句柄
 * @param[out]   zeroCompensationOffset  机械臂零位补偿值
 * @return       CRresult               返回处理结果
 */
CRSDKAPI CRresult cr_get_zeroCompensationOffset(RobotHandle robotHandle, double zeroCompensationOffset[ROB_AXIS_NUM]);

/**
 * @brief   读机械臂减速比
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   reducerRatio  机械臂减速比
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_reducerRatio(RobotHandle robotHandle, double reducerRatio[ROB_AXIS_NUM]);

/*
*************运动控制*************
*/
/**
 * @brief   写机械臂速度百分比
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    speedPercent  机械臂速度百分比（1~100）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_robotSpeedPercent(RobotHandle robotHandle, unsigned int speedPercent);
/**
 * @brief   读机械臂速度百分比
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   speedPercent  机械臂速度百分比
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_robotSpeedPercent(RobotHandle robotHandle, unsigned int *speedPercent);

/**
 * @brief   PointControlParaSimple转换为PointControlPara
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    pointControlParaSimple  简化运动参数
 * @param[out]   pointControlPara 运动参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_move_pointControlPara_transfer(RobotHandle robotHandle,PointControlParaSimple  pointControlParaSimple,PointControlPara  *pointControlPara);

/**
 * @brief   轴空间运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]   pointControlPara  运动相关参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_moveJ(RobotHandle robotHandle,PointControlPara  pointControlPara);

/**
 * @brief   轴空间运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    pointControlPara  运动相关参数
 * @param[in]    isBlock  TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult  返回处理结果
 */
CRSDKAPI CRresult cr_move_joint(RobotHandle robotHandle,PointControlPara  pointControlPara,BOOL isBlock);

/**
 * @brief   直线运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]   pointControlPara  运动相关参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_moveL(RobotHandle robotHandle,PointControlPara  pointControlPara);

/**
 * @brief   直线运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    pointControlPara  运动相关参数
 * @param[in]    isBlock  TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_move_line(RobotHandle robotHandle,PointControlPara  pointControlPara,BOOL isBlock);

/**
 * @brief   点动运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    pointControlPara  运动相关参数
 * @return       CRresult      返回处理结果
 * @attention   该指令需调用者实现心跳机制，每150ms调用该指令一次，以实现连续点动运动
 */
CRSDKAPI CRresult cr_moveJog(RobotHandle robotHandle,PointControlPara  pointControlPara);

/**
 * @brief   圆弧运动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    pointControlParaList  圆弧运动相关参数
 * @param[in]    isBlock  TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_move_circle(RobotHandle robotHandle,PointControlParaList  pointControlParaList,BOOL isBlock);

/**
    * @brief       读取移动缓存区中移动指令的数量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  cacheNum   缓存区中移动指令数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_move_cache_num_get(RobotHandle robotHandle,int *cacheNum);

/**
    * @brief       设置阻塞移动中参数阈值
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   poseDelt   位姿阈值  默认0.02
    * @param[in]   jointDelt  关节阈值  默认0.1
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_move_block_threshold_set(RobotHandle robotHandle,double poseDelt,double jointDelt);

/**
 * @brief   运动控制
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    moveType       运动控制参数
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_moveControl(RobotHandle robotHandle, enum MoveType moveType);

/**
 * @brief   设置软件自由驱动启动
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[in]    softFreeDriveEnabled  待设定值，TRUE-使能
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_set_softFreeDriveEnabled(RobotHandle robotHandle, BOOL softFreeDriveEnabled);


///********获取串口数据********/
///**
// * @brief   获取串口数据
// * @param[in]    robotHandle         机械臂实例句柄
// * @param[in]    rsnum               0表示RS485工具端，1表示RS485-1,2表示RS485-2,3表示RS232
// * @param[out]   getSerialPortData   获取到的串口数据
// * @return       CRresult            返回处理结果
// */
//CRSDKAPI CRresult cr_get_serialPortData(RobotHandle robotHandle, SerialCommSettings *getSerialPortData,int rsnum);
///**
// * @brief   写串口数据
// * @param[in]    robotHandle        机械臂实例句柄
// * @param[in]    setSerialPortData  需写入的串口数据
// * @param[in]    rsnum              0表示RS485工具端，1表示RS485-1,2表示RS485-2,3表示RS232
// * @return       CRresult           返回处理结果
// */
//CRSDKAPI CRresult cr_set_serialPortData(RobotHandle robotHandle, SerialCommSettings setSerialPortData,int rsnum);

/*
*************数学库*************
*/
/**
 * @brief   位姿转换,位姿整体右乘叠加计算
 * @param[in]    poseFrom      原位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[in]    poseTrans     变换位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[out]   poseTo        结果位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_poseTrans(double poseFrom[ROB_AXIS_NUM], double poseTrans[ROB_AXIS_NUM], double poseTo[ROB_AXIS_NUM]);

/**
 * @brief   位姿变换，轴角转欧拉角
 * @param[in]    axisAnglePose   轴角位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[out]   eulePose        欧拉角位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult        返回处理结果
 */
CRSDKAPI CRresult cr_AxisAngle2Eule(double axisAnglePose[ROB_AXIS_NUM], double eulePose[ROB_AXIS_NUM]);

/**
 * @brief   位姿变换，欧拉角转轴角
 * @param[in]    eulePose      欧拉角位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[out]   axisAnglePose 轴角位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_Eule2AxisAngle(double eulePose[ROB_AXIS_NUM], double axisAnglePose[ROB_AXIS_NUM]);

/**
 * @brief 正解
 * @param[in]    robotHandle  机器人控制句柄
 * @param[in]    srcJointPose 关节空间位置:关节1~6（单位：°）
 * @param[out]   tarPose      笛卡尔空间位姿计算结果：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return       CRresult     返回处理结果
 */

CRSDKAPI CRresult cr_kineForward(RobotHandle robotHandle, double srcJointPose[ROB_AXIS_NUM], double tarPose[ROB_AXIS_NUM]);

/**
 * @brief 逆解
 * @param[in]      robotHandle  机器人控制句柄
 * @param[in]      srcPose      笛卡尔空间位姿值：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[in]      refJointPose 逆解计算用的参考关节空间位置:关节1~6（单位：°）
 * @param[out]     tarJointPose 计算成功时关节空间位置计算结果:关节1~6（单位：°）
 * @return         CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_kineInverse(RobotHandle robotHandle, double srcPose[ROB_AXIS_NUM], double refJointPose[ROB_AXIS_NUM],double tarJointPose[ROB_AXIS_NUM]);

/**
 * @brief TCP位姿转法兰位姿
 * @param[in]      tcpPose      tcp位姿在基座标系下的描述：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[in]      toolPose     当前使用的tcp：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @param[out]     flangePose   法兰在基座标系下的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
 * @return         CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_TcpToFlangePose( double tcpPose[ROB_AXIS_NUM], double toolPose[ROB_AXIS_NUM],double flangePose[ROB_AXIS_NUM]);

/**
 * @brief  目标点旋转变换
 * @param[in]   axisPosition       旋转轴位置(x,y,z（单位：mm）)
 * @param[in]   axisDirection      旋转轴方向(x,y,z（单位：mm）)
 * @param[in]   angle              旋转角度（°）
 * @param[in]   srcPose            原始目标位姿(x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）)
 * @param[out]  dstPose            旋转后目标位姿(x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）)
 * @return      CRresult           返回处理结果
 */
CRSDKAPI CRresult cr_compute_rotate_around_axis(double axisPosition[3], double axisDirection[3], double angle, double srcPose[ROB_AXIS_NUM], double dstPose[ROB_AXIS_NUM]);

/**
 * @brief  工程文件上传 上传crp文件和脚本程序
 * @param[in]   robotHandle         机械臂实例句柄
 * @param[in]   FilePath            文件保存路径
 * @param[in]   filename            .crp和脚本程序的文件名
 * @return      CRresult            返回处理结果
 */
CRSDKAPI CRresult cr_uploadProject(RobotHandle robotHandle, char *FilePath, char*filename);

/**
 * @brief 工程文件下载 下载crp文件和脚本程序
 * @param[in]   robotHandle              机械臂实例句柄
 * @param[in]   crpFilepathname          .crp文件路径名
 * @param[in]   crscriptFilepathname      脚本程序路径名
 * @return      CRresult                 返回处理结果
 */
CRSDKAPI CRresult cr_downloadProject(RobotHandle robotHandle, char* crpFilepathname, char *crscriptFilepathname);

/**
 * @brief 读取所有多程序的索引及其名字
 * @param[in]   robotHandle             机械臂实例句柄
 * @param[out]  progfileCount           程序数量
 * @param[out]  progfileInfo            程序信息
 * @return      CRresult                返回处理结果
 */
CRSDKAPI CRresult cr_get_all_multiple_program(RobotHandle robotHandle, int* progfileCount, ProgFileInfo* progfileInfo);

/**
 * @brief 运行多程序
 * @param[in]   robotHandle             机械臂实例句柄
 * @param[in]   progfileIndex           程序索引
 * @return      CRresult                返回处理结果
 */
CRSDKAPI CRresult cr_play_multiple_program(RobotHandle robotHandle, int progfileIndex);

/**
    * @brief       后台程序下载
    * @param[in]   robotHandle      机械臂实例句柄
    * @param[in]   progfileIndex    程序索引
    * @param[in]   progfileName     程序名字
    * @param[in]   programfile      需加载的程序
    * @return      CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_download_background_program(RobotHandle robotHandle, int progfileIndex, const char *progfileName, const char* programfile);

/**
    * @brief       获取后台程序状态
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[out]   progfileIndex           运行中的程序索引（程序停止时该值为-1）
    * @param[out]   backgroundProgramState  后台程序状态 0-运行 1-停止
    * @return      CRresult                 返回处理结果
 */
CRSDKAPI CRresult cr_get_background_program_state(RobotHandle robotHandle, int *progfileIndex, int *backgroundProgramState);
/**
    * @brief       设置后台程序运行和停止
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[in]   progfileIndex            程序索引（设置停止时，该值可以给任意值）
    * @param[in]   backgroundProgramState   后台程序状态 0-运行 1-停止
    * @return      CRresult                 返回处理结果
 */
CRSDKAPI CRresult cr_set_background_program_state(RobotHandle robotHandle, int progfileIndex, int backgroundProgramState);

/**
 * @brief   读脚本运行状态
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   scriptstatus  当前脚本运行状态
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_lua_scriptstatus(RobotHandle robotHandle, Lua_ScriptStatus *scriptstatus);

/**
    * @brief       获取配置改变标志
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  changeCode   配置改变标志
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_change_code_get(RobotHandle robotHandle,int* changeCode);

/**
    * @brief       获取日志数量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  logNumber    日志数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_log_number_get(RobotHandle robotHandle,int* logNumber);

/**
    * @brief       设置安装角度
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   installAngle  安装角度设置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_install_angle_set(RobotHandle robotHandle,InstallAngle installAngle);
/**
    * @brief       读取安装角度
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  installAngle  安装角度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_install_angle_get(RobotHandle robotHandle,InstallAngle *installAngle);
/**
    * @brief       自计算安装角度
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  installAngle  安装角度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_install_angle_calculate(RobotHandle robotHandle,InstallAngle *installAngle);
/**
    * @brief       读取TCP个数
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  count  tcp个数
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取TCP数据
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index   tcp索引
    * @param[out]  tcpMsg  tcp数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_get(RobotHandle robotHandle,int index,TCPMsg *tcpMsg);
/**
    * @brief       读取TCP数据(通过名称)
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   tcpName   tcp名称
    * @param[out]  tcpMsg  tcp数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_accord_name_get(RobotHandle robotHandle, char* tcpName, TCPMsg *tcpMsg);
/**
    * @brief       增加TCP
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]  tcpMsg  tcp数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_add(RobotHandle robotHandle,TCPMsg tcpMsg);
/**
    * @brief       删除TCP
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index   tcp索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_delete(RobotHandle robotHandle,int index);
/**
    * @brief       编辑TCP数据
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index   tcp索引
    * @param[in]   tcpMsg  tcp数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_set(RobotHandle robotHandle,int index,TCPMsg tcpMsg);
/**
    * @brief       读取当前激活的TCP索引
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  index   tcp索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_active_get(RobotHandle robotHandle,int *index);
/**
    * @brief       激活TCP
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index   tcp索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_tcp_active_set(RobotHandle robotHandle,int index);
/**
    * @brief       读取负载个数
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  count  负载个数
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取负载数据
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index    负载索引
    * @param[out]  payLoad  负载信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_get(RobotHandle robotHandle,int index,PayLoad *payLoad);
/**
    * @brief       读取负载数据(通过名称)
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   payloadName    负载名称
    * @param[out]  payLoad  负载信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_accord_name_get(RobotHandle robotHandle, char* payloadName, PayLoad *payLoad);
/**
    * @brief       增加负载
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   payLoad  负载信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_add(RobotHandle robotHandle,PayLoad payLoad);
/**
    * @brief      删除负载
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index    负载索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_delete(RobotHandle robotHandle,int index);
/**
    * @brief      编辑负载
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[in]   index    负载索引
    * @param[in]  payLoad  负载信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_set(RobotHandle robotHandle,int index,PayLoad payLoad);
/**
    * @brief      读取当前激活的负载索引
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  index    负载索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_active_get(RobotHandle robotHandle,int *index);
/**
    * @brief      激活负载
    * @param[in]  robotHandle   机械臂实例句柄
    * @param[in]  index    负载索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_payload_active_set(RobotHandle robotHandle,int index);

/**
    * @brief      设置初始位
    * @param[in]  robotHandle   机械臂实例句柄
    * @param[in]  homeJointPose    初始位关节角度
    * @param[in]  len    数组长度
    * @return     success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_home_pose_set(RobotHandle robotHandle,double *homeJointPose,int len);
/**
    * @brief      读取初始位
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  homeJointPose    初始位关节角度
    * @param[in]   len    数组长度
    * @return     success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_home_pose_get(RobotHandle robotHandle,double *homeJointPose,int len);
/**
    * @brief      读取包装位
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  packJointPose    包装位关节角度
    * @param[in]   len    数组长度
    * @return     success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_pack_pose_get(RobotHandle robotHandle,double *packJointPose,int len);
/**
    * @brief     删除初始位
    * @param[in]  robotHandle  机械臂实例句柄
    * @return     success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_home_pose_delete(RobotHandle robotHandle);
/**
    * @brief       设置机器人是否自动上电使能
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   isAuto  是否自动上电
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_poweron_auto_set(RobotHandle robotHandle,BOOL isAuto);
/**
    * @brief       读取机器人自动上电状态
    * @param[in]    robotHandle  机械臂实例句柄
    * @param[out]   isAuto  是否自动上电的状态
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_poweron_auto_get(RobotHandle robotHandle,BOOL *isAuto);

/**
    * @brief       设置机器人拖动阻尼
    * @param[in]   robotHandle          机械臂实例句柄
    * @param[in]   jointDargDamping     机器人拖动阻尼数组(关节1-6)
    * @param[in]   arrSize              机器人拖动阻尼数组大小(一般为6)
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_joint_drag_damping_set(RobotHandle robotHandle, int* jointDargDamping, const int arrSize);
/**
    * @brief       读取机器人拖动阻尼
    * @param[in]   robotHandle          机械臂实例句柄
    * @param[out]  jointDargDamping     机器人拖动阻尼数组(关节1-6)
    * @param[in]   arrSize              机器人拖动阻尼数组大小(一般为6)
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_joint_drag_damping_get(RobotHandle robotHandle, int* jointDargDamping, const int arrSize);
/**
    * @brief       设置转矩异常门限倍率
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[in]   torqueThresholdRatio     机器人转矩异常门限倍率数组(关节1-6)
    * @param[in]   arrSize                  机器人转矩异常门限倍率数组大小(一般为6)
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_torque_threshold_ratio_set(RobotHandle robotHandle, double* torqueThresholdRatio, const int arrSize);
/**
    * @brief       读转矩异常门限倍率
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[out]  torqueThresholdRatio     机器人转矩异常门限倍率数组(关节1-6)
    * @param[in]   arrSize                  机器人转矩异常门限倍率数组大小(一般为6)
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_torque_threshold_ratio_get(RobotHandle robotHandle, double* torqueThresholdRatio, const int arrSize);
/**
    * @brief       设置扩展轴步长及速度倍率
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[in]   exjMoveSpeedRatio        扩展轴步长及速度倍率
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_exj_move_speed_ratio_set(RobotHandle robotHandle, double exjMoveSpeedRatio);
/**
    * @brief       读扩展轴步长及速度倍率
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[out]  exjMoveSpeedRatio        扩展轴步长及速度倍率
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_exj_move_speed_ratio_get(RobotHandle robotHandle, double* exjMoveSpeedRatio);
/**
    * @brief       设置扩展轴直线方向
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[in]   exjLineDirection         扩展轴直线运动方向
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_exj_line_direction_set(RobotHandle robotHandle, ExjLineDirection exjLineDirection);
/**
    * @brief       读取扩展轴直线方向
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[out]  exjLineDirection         扩展轴直线运动方向
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_exj_line_direction_get(RobotHandle robotHandle, ExjLineDirection *exjLineDirection);
/**
    * @brief       设置掉电抬升状态
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[in]   poweroffLift             是否开启掉电抬升  0-关闭 1-开启
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_poweroff_lift_set(RobotHandle robotHandle, BOOL poweroffLift);
/**
    * @brief       读取掉电抬升状态
    * @param[in]   robotHandle              机械臂实例句柄
    * @param[out]  poweroffLift             是否开启掉电抬升  0-关闭 1-开启
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_poweroff_lift_get(RobotHandle robotHandle, BOOL *poweroffLift);
/**
    * @brief       设置静止时碰撞保护
    * @param[in]   robotHandle                 机械臂实例句柄
    * @param[in]   staticCollisionProtection   是否开启静止时碰撞保护
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_static_collision_protection_set(RobotHandle robotHandle, StaticCollisionProtectionMode staticCollisionProtection);
/**
    * @brief       读取静止时碰撞保护
    * @param[in]   robotHandle                 机械臂实例句柄
    * @param[out]  staticCollisionProtection   是否开启静止时碰撞保护
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_static_collision_protection_get(RobotHandle robotHandle, StaticCollisionProtectionMode *staticCollisionProtection);
/**
    * @brief       设置串口超时重连
    * @param[in]   robotHandle                 机械臂实例句柄
    * @param[in]   status                      是否开启串口超时重连
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_serialport_timeout_reconnect_set(RobotHandle robotHandle, BOOL status);
/**
    * @brief       读取串口超时重连状态
    * @param[in]   robotHandle                 机械臂实例句柄
    * @param[out]  status                      是否开启串口超时重连
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_serialport_timeout_reconnect_get(RobotHandle robotHandle, BOOL *status);

/**
    * @brief      设置数字输入信号配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  IO类型
    * @param[in]   index  IO索引
    * @param[in]   inputConfiguration  数字量输入配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_di_set(RobotHandle robotHandle,VarDigitialIOType type,int index,IOInputConfiguration inputConfiguration);
/**
    * @brief      读取数字输入信号配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  IO类型
    * @param[in]   index  IO索引
    * @param[out]  inputConfiguration  数字量输入O配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_di_get(RobotHandle robotHandle,VarDigitialIOType type,int index,IOInputConfiguration *inputConfiguration);
/**
    * @brief      设置数字输出信号配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  IO类型
    * @param[in]   index  IO索引
    * @param[in]   outputConfiguration  数字量输出配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_do_set(RobotHandle robotHandle,VarDigitialIOType type,int index,IOOutputConfiguration outputConfiguration);
/**
    * @brief      读取数字输出信号配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  IO类型
    * @param[in]   index  IO索引
    * @param[out]  outputConfiguration  数字量输出配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_do_get(RobotHandle robotHandle,VarDigitialIOType type,int index,IOOutputConfiguration *outputConfiguration);

/**
    * @brief      设置工具IO类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  IO索引
    * @param[in]   toolIoType  工具IO类型
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_tool_io_type_set(RobotHandle robotHandle,int index,ToolIoType toolIoType);

/**
    * @brief       读取工具IO类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  IO索引
    * @param[out]   toolIoType  工具IO类型
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_tool_io_type_get(RobotHandle robotHandle,int index,ToolIoType *toolIoType);

/**
    * @brief      设置模拟量输入信号类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  模拟量类型
    * @param[in]   index  模拟量索引
    * @param[in]   analogInputConfiguration  模拟量输入配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_ai_set(RobotHandle robotHandle,VarDigitialIOType type,int index,AnalogInputConfiguration analogInputConfiguration);
/**
    * @brief      读取模拟量输入信号类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  模拟量类型
    * @param[in]   index  模拟量索引
    * @param[out]  analogInputConfiguration  模拟量输入配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_ai_get(RobotHandle robotHandle,VarDigitialIOType type,int index,AnalogInputConfiguration *analogInputConfiguration);
/**
    * @brief      设置模拟量输出信号类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  模拟量索引
    * @param[in]   analogOutputConfiguration  模拟量输出配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_ao_set(RobotHandle robotHandle,int index,AnalogOutputConfiguration analogOutputConfiguration);
/**
    * @brief      读取模拟量输出信号类型
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  模拟量索引
    * @param[out]  analogOutputConfiguration  模拟量输出配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_ao_get(RobotHandle robotHandle,int index,AnalogOutputConfiguration *analogOutputConfiguration);
/**
    * @brief      设置工具电压输出值
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   toolPower  电压值
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_to_set(RobotHandle robotHandle,ToolPower toolPower);
/**
    * @brief      读取工具电压输出值
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  toolPower  电压值
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_to_get(RobotHandle robotHandle,ToolPower *toolPower);

/**
    * @brief      读取编码器配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index        编码器索引
    * @param[out]  encoderConfig  编码器状态
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_encoder_config_get(RobotHandle robotHandle, int index, EncoderConfiguration *encoderConfig);

/**
    * @brief      设置编码器配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index        编码器索引
    * @param[in]   encoderConfig  编码器状态
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_encoder_config_set(RobotHandle robotHandle, int index, EncoderConfiguration encoderConfig);

/**
    * @brief      设置位寄存器配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[in]   bitRegisterConfiguration  寄存器配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_bit_reg_set(RobotHandle robotHandle,int index,BitRegisterConfiguration bitRegisterConfiguration);
/**
    * @brief       读取位寄存器配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[out]  bitRegisterConfiguration  寄存器配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_bit_reg_get(RobotHandle robotHandle,int index,BitRegisterConfiguration *bitRegisterConfiguration);
/**
    * @brief       读取所有位寄存器配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  bitRegisterConfigurationArray  存放寄存器配置的地址
    * @param[in]   arraySize  读取的数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_all_bit_reg_get(RobotHandle robotHandle,BitRegisterConfiguration *bitRegisterConfigurationArray, int arraySize);
/**
    * @brief       设置16位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[in]   name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_int16_reg_name_set(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取16位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[out]  name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_int16_reg_name_get(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取所有16位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  registerConfigurationArray  存放寄存器名字的地址
    * @param[in]   arrSize  读取的名字数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_all_int16_reg_name_get(RobotHandle robotHandle, RegisterConfiguration *registerConfigurationArray,int arrSize);
/**
    * @brief       设置32位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[in]   name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_int32_reg_name_set(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取32位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[out]  name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_int32_reg_name_get(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取所有32位整数寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  registerConfigurationArray  存放寄存器名字的地址
    * @param[in]   arrSize  读取的名字数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_all_int32_reg_name_get(RobotHandle robotHandle, RegisterConfiguration *registerConfigurationArray,int arrSize);
/**
    * @brief       设置浮点寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[in]   name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_float_reg_name_set(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取浮点寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  寄存器索引值
    * @param[out]  name  寄存器名字
    * @param[in]   len  名字长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_float_reg_name_get(RobotHandle robotHandle,int index, char *name,int len);
/**
    * @brief       读取所有浮点寄存器变量名
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  registerConfigurationArray  存放寄存器名字的地址
    * @param[in]   arrSize  读取的名字数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_all_float_reg_name_get(RobotHandle robotHandle,RegisterConfiguration *registerConfigurationArray,int arrSize);
/**
    * @brief       读取安装变量个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count  安装变量数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_install_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取安装变量数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安装变量索引
    * @param[out]  variableMsg  安装变量数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_install_get(RobotHandle robotHandle,int index,VariableMsg *variableMsg);
/**
    * @brief       增加安装变量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   variableMsg  安装变量数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_install_add(RobotHandle robotHandle,VariableMsg *variableMsg);
/**
    * @brief       删除安装变量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安装变量索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_install_delete(RobotHandle robotHandle,int index);
/**
    * @brief       修改安装变量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安装变量索引
    * @param[in]   variableMsg  安装变量数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_var_install_set(RobotHandle robotHandle,int index,VariableMsg *variableMsg);


/**
    * @brief       读取点坐标系个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count  点坐标系数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取点坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  点坐标系索引
    * @param[out]  pointCSNode  点坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_get(RobotHandle robotHandle,int index, PointCSNode *pointCSNode);
/**
    * @brief       读取点坐标系数据(通过名称)
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   csName  点坐标系名称
    * @param[out]  pointCSNode  点坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_accord_name_get(RobotHandle robotHandle, char* csName, PointCSNode *pointCSNode);
/**
    * @brief       增加点坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   pointCSNode  点坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_add(RobotHandle robotHandle,PointCSNode pointCSNode);
/**
    * @brief       删除点坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  点坐标系索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_delete(RobotHandle robotHandle,int index);
/**
    * @brief       修改点坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  点坐标系索引
    * @param[in]   pointCSNode  点坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_point_set(RobotHandle robotHandle,int index, PointCSNode pointCSNode);
/**
    * @brief       读取线坐标系个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count  线坐标系个数
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取线坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  线坐标系线索引
    * @param[out]  lineCSNode  线坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_get(RobotHandle robotHandle,int index, LineCSNode *lineCSNode);
/**
    * @brief       读取线坐标系数据(通过名称)
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   csName  线坐标系名称
    * @param[out]  lineCSNode  线坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_accord_name_get(RobotHandle robotHandle, char* csName, LineCSNode *lineCSNode);
/**
    * @brief       增加线坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   lineCSNode  线坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_add(RobotHandle robotHandle,LineCSNode lineCSNode);
/**
    * @brief       删除线坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  线坐标系线索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_delete(RobotHandle robotHandle,int index);
/**
    * @brief       修改线坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  线坐标系线索引
    * @param[in]   lineCSNode  线坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_line_set(RobotHandle robotHandle,int index, LineCSNode lineCSNode);

/**
    * @brief       读取面坐标系个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count  面坐标系数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取面坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  面坐标系索引
    * @param[out]  planeCSNode  面坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_get(RobotHandle robotHandle,int index, PlaneCSNode *planeCSNode);
/**
    * @brief       读取面坐标系数据(通过名称)
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   csName  面坐标系名称
    * @param[out]  planeCSNode  面坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_accord_name_get(RobotHandle robotHandle, char* csName, PlaneCSNode *planeCSNode);
/**
    * @brief       增加面坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   planeCSNode  面坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_add(RobotHandle robotHandle,PlaneCSNode planeCSNode);
/**
    * @brief      删除面坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  面坐标系索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_delete(RobotHandle robotHandle,int index);
/**
    * @brief       修改面坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  面坐标系索引
    * @param[in]   planeCSNode  面坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_plane_set(RobotHandle robotHandle,int index, PlaneCSNode planeCSNode);
/**
    * @brief       读取基坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  csPose  基坐标系数据
    * @param[in]   len  数据长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_base_get(RobotHandle robotHandle,double *csPose,int len);
/**
    * @brief       读取工具坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  csPose  工具坐标系数据
    * @param[in]   len  数据长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_tool_get(RobotHandle robotHandle,double *csPose,int len);

/**
    * @brief       读取工件坐标系个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count        工件坐标系数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_count(RobotHandle robotHandle,int *count);
/**
    * @brief       读取工件坐标系数据(通过索引)
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index        工件坐标系索引
    * @param[out]  wcsConfig    工件坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_get(RobotHandle robotHandle,int index, WCSConfig *wcsConfig);
/**
    * @brief       读取工件坐标系数据(通过名称)
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   csName       工件坐标系名称
    * @param[out]  wcsConfig    工件坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_accord_name_get(RobotHandle robotHandle, char* csName, WCSConfig *wcsConfig);
/**
    * @brief       读取所有工件坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  wcsConfig    工件坐标系数据
    * @param[out]  validNumber  工件坐标系的数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_get_all(RobotHandle robotHandle, WCSConfig *wcsConfig, int *validNumber);

/**
    * @brief       增加工件坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   wcsConfig    工件坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_add(RobotHandle robotHandle,WCSConfig wcsConfig);
/**
    * @brief       删除工件坐标系
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index        工件坐标系索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_delete(RobotHandle robotHandle,int index);
/**
    * @brief       修改工件坐标系数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index        工件坐标系索引
    * @param[in]   wcsConfig    工件坐标系数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_cs_workpiece_set(RobotHandle robotHandle,int index, WCSConfig wcsConfig);

/**
    * @brief       设置串口配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   serialType  串口类型
    * @param[in]   serialCommSettings  串口配置数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_serial_setting_set(RobotHandle robotHandle,SerialType serialType, SerialCommSettings serialCommSettings);

/**
    * @brief       读取串口配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   serialType  串口类型
    * @param[out]  serialCommSettings  串口配置数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_serial_setting_get(RobotHandle robotHandle,SerialType serialType, SerialCommSettings *serialCommSettings);

/**
    * @brief       设置限制模式
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   safetyLimitsValuesType  限制模式
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_limit_type_set(RobotHandle robotHandle,SafetyLimitsValuesType safetyLimitsValuesType);
/**
    * @brief       读取限制模式
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]   safetyLimitsValuesType  限制模式
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_limit_type_get(RobotHandle robotHandle,SafetyLimitsValuesType *safetyLimitsValuesType);

/**
    * @brief       设置自定义限制模式及对应的碰撞检测比例
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   collisionPercent  碰撞检测比例,默认值为75
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_custom_limit_type_set(RobotHandle robotHandle,int collisionPercent);
/**
    * @brief       读取自定义限制模式下的碰撞检测比例
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  collisionPercent  碰撞检测比例
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_custom_limit_type_get(RobotHandle robotHandle,int *collisionPercent);

/**
    * @brief       设置碰撞后处理方式
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   safetyCollisionHandleMode  碰撞后处理方式
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_collihandle_type_set(RobotHandle robotHandle,SafetyCollisionHandleMode safetyCollisionHandleMode);

/**
    * @brief       读取碰撞后处理方式
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  safetyCollisionHandleMode  碰撞后处理方式
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_collihandle_type_get(RobotHandle robotHandle,SafetyCollisionHandleMode *safetyCollisionHandleMode);

/**
    * @brief       设置自定义模式下参数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   mode  模式选择
    * @param[in]   safetyLimitsValues  模式参数
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_limit_para_set(RobotHandle robotHandle,SafetyLimitsMode mode,SafetyLimitsValues safetyLimitsValues);
/**
    * @brief       读取自定义模式下参数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   mode  模式选择
    * @param[out]  safetyLimitsValues  模式参数
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_limit_para_get(RobotHandle robotHandle,SafetyLimitsMode mode,SafetyLimitsValues *safetyLimitsValues);

/**
    * @brief       设置关节限值
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   mode  模式选择
    * @param[in]   jointIndex  关节序号
    * @param[in]   safetyLimitJoint  关节限值
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_joint_limit_set(RobotHandle robotHandle,SafetyLimitsMode mode,int jointIndex,SafetyLimitsJointAngle safetyLimitJoint);
/**
    * @brief       读取关节限值
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   mode  模式选择
    * @param[in]   jointIndex  关节序号
    * @param[out]   safetyLimitJoint  关节限值
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_joint_limit_get(RobotHandle robotHandle,SafetyLimitsMode mode,int jointIndex,SafetyLimitsJointAngle *safetyLimitJoint);

/**
    * @brief       读取安全平面个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  count  安全平面数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_plane_count(RobotHandle robotHandle,int *count);

/**
    * @brief      读取安全平面信息
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安全平面序号
    * @param[out]  safetyLimitsBoundaryPlane  安全平面信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_plane_get(RobotHandle robotHandle,int index,SafetyLimitsBoundaryPlane *safetyLimitsBoundaryPlane);

/**
    * @brief     增加安全平面
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]  safetyLimitsBoundaryPlane  安全平面信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_plane_add(RobotHandle robotHandle,SafetyLimitsBoundaryPlane safetyLimitsBoundaryPlane);

/**
    * @brief      删除安全平面
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安全平面序号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_plane_delete(RobotHandle robotHandle,int index);

/**
    * @brief      修改安全平面数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  安全平面序号
    * @param[in]   safetyLimitsBoundaryPlane  安全平面信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_plane_set(RobotHandle robotHandle,int index, SafetyLimitsBoundaryPlane safetyLimitsBoundaryPlane);

/**
    * @brief      设置可配置输入信号
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  索引，输入范围:0-3
    * @param[in]   safetyinput  可配置输入信号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_io_input_set(RobotHandle robotHandle,int index,SafetyIOInput safetyinput);

/**
    * @brief      读取可配置输入信号
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  索引，输入范围:0-3
    * @param[out]  safetyinput  可配置输入信号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_io_input_get(RobotHandle robotHandle,int index,SafetyIOInput *safetyinput);

/**
    * @brief      设置可配置输出信号
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  索引，输入范围:0-3
    * @param[in]   safetyoutput  可配置输出信号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_io_output_set(RobotHandle robotHandle,int index,SafetyIOOutput safetyoutput);
/**
    * @brief      读取可配置输出信号
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   index  索引，输入范围:0-3
    * @param[out]  safetyoutput  可配置输出信号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_io_output_get(RobotHandle robotHandle,int index,SafetyIOOutput *safetyoutput);
/**
    * @brief      设置是否启用示教器
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   isUse  是否启用示教器
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_tp_use_set(RobotHandle robotHandle,BOOL isUse);
/**
    * @brief      读取当前启用示教器
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  isUse  启用示教器状态
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_safety_tp_use_get(RobotHandle robotHandle,BOOL *isUse);
/**
    * @brief       设置控制柜网络配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   ipconfig  网络地址配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_ethernet_ip_set(RobotHandle robotHandle,IPConfig ipconfig);
/**
    * @brief       读取控制柜网络配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  ipconfig  网络地址配置
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_ethernet_ip_get(RobotHandle robotHandle,IPConfig *ipconfig);
/**
    * @brief       设置控制柜从站配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   modbusSlaveNo  从站号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_ethernet_modbus_slave_num_set(RobotHandle robotHandle,int modbusSlaveNo);
/**
    * @brief       读取控制柜从站配置
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  modbusSlaveNo  从站号
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_ethernet_modbus_slave_num_get(RobotHandle robotHandle,int *modbusSlaveNo);

/**
    * @brief       读取从站个数
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[out]  count  从站数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_count(RobotHandle robotHandle,SerialType type,int *count);
/**
    * @brief       读取从站数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   index  从站索引
    * @param[out]  modbusConfig  从站配置信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_get(RobotHandle robotHandle,SerialType type,int index, ModbusMasterConfig *modbusConfig);
/**
    * @brief       添加从站
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]  modbusConfig  从站配置信息
    * @return     success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_add(RobotHandle robotHandle,SerialType type,ModbusMasterConfig *modbusConfig);

/**
    * @brief       删除从站
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   index  从站索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_delete(RobotHandle robotHandle,SerialType type,int index);
/**
    * @brief      修改从站数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   index  从站索引
    * @param[in]   modbusConfig  从站配置信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_set(RobotHandle robotHandle,SerialType type,int index, ModbusMasterConfig modbusConfig);

/**
    * @brief       从站连接/断开
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   index  从站索引
    * @param[in]   modbusOperate  从站操作
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_operate(RobotHandle robotHandle,SerialType type,int index, ModbusMasterOperate modbusOperate);

/**
    * @brief       获取从站连接状态
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   index  从站索引
    * @param[out]  status  从站连接状态
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_slave_connect_get(RobotHandle robotHandle,SerialType type,int index, int* status);

/**
    * @brief       读取地址映射数量
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   serialIndex  从站索引
    * @param[out]  count  地址映射数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_addr_map_count(RobotHandle robotHandle,SerialType type,int serialIndex,int *count);
/**
    * @brief       读取地址映射数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   serialIndex  从站索引
    * @param[in]   index  地址映射索引
    * @param[out]  addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_addr_map_get(RobotHandle robotHandle,SerialType type,int serialIndex,int index,ModbusAddrMap *addrMap);
/**
    * @brief       添加地址映射
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   serialIndex  从站索引
    * @param[in]   addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_addr_map_add(RobotHandle robotHandle,SerialType type,int serialIndex,ModbusAddrMap addrMap);

/**
    * @brief       删除地址映射
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   serialIndex  从站索引
    * @param[in]   index  地址映射索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_addr_map_delete(RobotHandle robotHandle,SerialType type,int serialIndex,int index);

/**
    * @brief       修改地址映射数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  从站类型
    * @param[in]   serialIndex  从站索引
    * @param[in]   index  地址映射索引
    * @param[in]   addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_modbus_addr_map_set(RobotHandle robotHandle,SerialType type,int serialIndex,int index,ModbusAddrMap addrMap);

/**
    * @brief       读取主站通讯所有数据（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[out]  masterConfig  主站配置信息
    * @param[out]  count  地址映射数量
    * @param[out]  addrMapArray  存放地址映射数据的数组地址（地址映射最大数量为 100 个）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_master_config_get_all(RobotHandle robotHandle, SerialType type, CustomMasterConfig *masterConfig, int *count, CustomAddrMap *addrMapArray);

/**
    * @brief       修改主站通讯配置（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   masterConfig  主站配置信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_master_config_set(RobotHandle robotHandle, SerialType type, CustomMasterConfig masterConfig);

/**
    * @brief       主站通讯连接/断开（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   masterOperate  主站操作
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_master_operate(RobotHandle robotHandle, SerialType type, ModbusMasterOperate masterOperate);

/**
    * @brief       读取主站通讯连接状态（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[out]  status  主站连接状态  0-已断开 1-已连接
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_master_connect_get(RobotHandle robotHandle, SerialType type, int* status);

/**
    * @brief       读取主站通讯地址映射（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   index  地址映射索引
    * @param[out]  addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_get(RobotHandle robotHandle, SerialType type, int index, CustomAddrMap *addrMap);

/**
    * @brief       添加主站通讯地址映射（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_add(RobotHandle robotHandle, SerialType type, CustomAddrMap addrMap);

/**
    * @brief       删除主站通讯地址映射（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   index  地址映射索引
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_delete(RobotHandle robotHandle, SerialType type, int index);
/**
    * @brief       修改主站通讯地址映射数据（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   index  地址映射索引
    * @param[in]   addrMap  地址映射数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_set(RobotHandle robotHandle, SerialType type, int index, CustomAddrMap addrMap);
/**
    * @brief       删除所有主站通讯地址映射数据（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_delete_all(RobotHandle robotHandle, SerialType type);
/**
    * @brief       添加所有主站通讯地址映射数据（直接覆盖非追加）（自定义协议）
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   type  主站类型
    * @param[in]   addrMapArray  存放地址映射数据的数组
    * @param[in]   arraySize     数组长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_cfg_comm_custom_addr_map_add_all(RobotHandle robotHandle, SerialType type, CustomAddrMap *addrMapArray, int arraySize);


/**
    * @brief       计算TCP位置
    * @param[in]   poses  多组位姿数据，每组位姿数据都是轴角：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   posesLen 位姿数量，至少为4
    * @param[in]   positionLen   xyz的长度，至少为3
    * @param[out]  position    tcp的xyz值：x,y,z（单位：mm）
    * @param[out]  err    位置误差，单位：mm
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_tcp_position(double poses[][6], int posesLen, double *position, int positionLen, double *err);
/**
    * @brief       计算TCP角度
    * @param[in]   csPose   参考坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   csLen    参考坐标系位姿的长度，至少为6
    * @param[in]   pointPose 任意一点的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   pointLen 点位姿的长度，至少为6
    * @param[in]   orientationLen  rxyz的长度，至少为3
    * @param[out]  orientation   tcp的rxyz值：Rx,Ry,Rz（单位：°）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_tcp_orientation(double *csPose,int csLen,double *pointPose,int pointLen, double *orientation,int orientationLen);



/**
    * @brief       计算点坐标系
    * @param[in]   pointPose   计算点坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   pLen        位姿的长度，至少为6
    * @param[out]  pointCS     点坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   csLen       坐标系的长度，至少为6
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_cs_point(double *pointPose,int pLen,double *pointCS,int csLen);
/**
    * @brief       计算线坐标系
    * @param[in]   point1Pose  计算点线标系的位姿1：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   p1Len       位姿的长度，至少为6
    * @param[in]   point2Pose  计算点坐标系的位姿2：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   p2Len       位姿的长度，至少为6
    * @param[out]  lineCS      线坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   csLen       线坐标系的长度，至少为6
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_cs_line(double *point1Pose,int p1Len,double *point2Pose,int p2Len,double *lineCS,int csLen);
/**
    * @brief       计算面坐标系
    * @param[in]   point1Pose  计算面线标系的位姿1：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   p1Len       位姿的长度，至少为6
    * @param[in]   point2Pose  计算面坐标系的位姿2：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   p2Len       位姿的长度，至少为6
    * @param[in]   point3Pose  计算面坐标系的位姿3：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   p3Len       位姿的长度，至少为6
    * @param[out]  planeCS     面坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   csLen       面坐标系的长度，至少为6
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_cs_plane(double *point1Pose,int p1Len,double *point2Pose,int p2Len,double *point3Pose,int p3Len,double *planeCS,int csLen);

/**
    * @brief       基坐标系下的tcp转到指定工件坐标系下
    * @param[in]   robotHandle      机械臂实例句柄
    * @param[in]   basePose         基座标系下的轴角位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   index            指定的工件坐标系索引
    * @param[in]   exjPos           坐标转换时对应的外部轴位置：exj1,exj2,exj3,exj4,exj5,exj6,exj7,exj8,exj9,exj10，输入NULL时使用当前外部轴位置（单位：mm / °）
    * @param[out]  poseInWorkpiece  工件坐标系下的位姿数据（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_base_to_workpiece(RobotHandle robotHandle, double *basePose, int index, double *exjPos, double *poseInWorkpiece);

/**
    * @brief       指定工件坐标系下的tcp转到基坐标系下
    * @param[in]   robotHandle      机械臂实例句柄
    * @param[in]   poseInWorkpiece  工件坐标系下的位姿数据（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   index            工件坐标系索引
    * @param[in]   exjPos           坐标转换时对应的外部轴位置：exj1,exj2,exj3,exj4,exj5,exj6,exj7,exj8,exj9,exj10，输入NULL时使用当前外部轴位置（单位：mm / °）
    * @param[out]  basePose         基坐标系下的位姿数据（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_workpiece_to_base(RobotHandle robotHandle, double *poseInWorkpiece, int index, double *exjPos, double *basePose);

/**
    * @brief       基坐标系下的tcp位姿转到世界坐标系下
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   basePose     基座标系下的轴角位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   exjPos       坐标转换时对应的外部轴位置：exj1,exj2,exj3,exj4,exj5,exj6,exj7,exj8,exj9,exj10，输入NULL时使用当前外部轴位置（单位：mm / °）
    * @param[out]  poseInWorld  世界坐标系下的位姿数据（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_base_to_world(RobotHandle robotHandle, double *basePose, double *exjPos, double *poseInWorld);

/**
    * @brief       世界坐标系下的tcp位姿转到基坐标系下
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   poseInWorld  世界坐标系下的轴角位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   exjPos       坐标转换时对应的外部轴位置：exj1,exj2,exj3,exj4,exj5,exj6,exj7,exj8,exj9,exj10，输入NULL时使用当前外部轴位置（单位：mm / °）
    * @param[out]  poseInBase   基坐标系下的位姿数据（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_world_to_base(RobotHandle robotHandle, double *poseInWorld, double *exjPos, double *poseInBase);

/**
    * @brief       读取脚本当前运行行号
    * @param[in]   robotHandle   机械臂实例句柄
    * @param[out]  currentLine   脚本当前行号
    * @param[out]  lineLen       脚本行号数组长度
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_current_line_get(RobotHandle robotHandle,int *currentLine,int *lineLen);

/**
    * @brief       读取程序变量数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   varName  程序变量名
    * @param[out]  variableMsg  程序变量数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_var_get(RobotHandle robotHandle, char *varName, VariableMsg *variableMsg);

/**
    * @brief       修改程序变量数据
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   variableMsg  程序变量数据
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_var_set(RobotHandle robotHandle, VariableMsg *variableMsg);

/**
    * @brief       位姿求逆
    * @param[in]   pose   轴角位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   poseLen 位姿长度，至少为6
    * @param[out]  poseInv  位姿的逆：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   poseInvLen  位姿长度，至少为6
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_inv(double *pose,int poseLen,double *poseInv,int poseInvLen);
/**
    * @brief       基坐标系下的位姿转到用户坐标系下
    * @param[in]   basePose   轴角位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   basePoseLen 位姿长度，至少为6
    * @param[in]   userPose    用户坐标系位姿（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   userPoseLen  位姿长度，至少为6
    * @param[out]  poseInUser   用户坐标系下的位姿（轴角）：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[in]   poseInUserLen  位姿长度，至少为6
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_compute_pose_base_to_user(double *basePose,int basePoseLen,double *userPose,int userPoseLen, double *poseInUser,int poseInUserLen);


/**
    * @brief       日志功能是否开启
    * @param[in]   enable  是否开启 TRUE-开启  FALSE-不开启
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_log_enable(BOOL enable);

/**
    * @brief       日志大小设置
    * @param[in]   size  单个日志文件大小 字节
    * @param[in]   num  日志文件数量
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_log_set_size(long size,int num);

/**
    * @brief       是否存在弹窗
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  hasPopup  弹窗结果
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_popup_exist(RobotHandle robotHandle,BOOL *hasPopup);

/**
    * @brief       读取弹窗信息
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[out]  popupMsg  弹窗信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_popup_msg_get(RobotHandle robotHandle,PopUpMsg *popupMsg);

/**
    * @brief       设置输入弹窗信息
    * @param[in]   robotHandle  机械臂实例句柄
    * @param[in]   invarMsg    弹窗信息
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_popup_msg_set(RobotHandle robotHandle,VariableMsg *invarMsg);

/**
    * @brief       关闭弹窗
    * @param[in]   robotHandle  机械臂实例句柄
    * @return      success/0 成功，其它失败
*/
CRSDKAPI CRresult cr_script_popup_close(RobotHandle robotHandle);

///新增接口

/**
 * @brief   读取控制柜当前存在的轨迹索引
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   allPathIndex  所有轨迹索引,最大为51
 * @param[out]   pathIndexLen  轨迹索引数组有效长度
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_path_all_index_get(RobotHandle robotHandle,int *allPathIndex,int *pathIndexLen);
/**
 * @brief   上传轨迹
 * @param[in]    robotHandle  机械臂实例句柄
 * @param[in]    index        轨迹索引：-1~49
 * @param[out]   pathData     上传的轨迹数据
 * @return       CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_path_upload(RobotHandle robotHandle,int index,PathData *pathData);

/**
 * @brief   下载轨迹
 * @param[in]    robotHandle       机械臂实例句柄
 * @param[in]    pathDownloadData  下载轨迹数据
 * @return       CRresult          返回处理结果
 */
CRSDKAPI CRresult cr_path_download(RobotHandle robotHandle,PathDownloadData pathDownloadData);

/**
 * @brief   轨迹控制
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    pathIndex      轨迹索引：0~49
 * @param[in]    runControl     0-停止，1-启动，2-开启实时透传，3-开启离线透传
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_path_action(RobotHandle robotHandle,int pathIndex,int runControl);

/**
 * @brief   轨迹数据转换为轨迹文件
 * @param[in]    pathData      轨迹数据
 * @param[out]   filePath      文件保存路径
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_path_pathData2file(PathData pathData,char *filePath);

/**
 * @brief   轨迹文件转换为轨迹数据
 * @param[in]    filePath      文件路径
 * @param[out]   pathData      轨迹数据
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_path_file2pathData(char *filePath,PathData *pathData);

/**
 * @brief   设置轨迹记录参数
 * @param[in]    robotHandle     机械臂实例句柄
 * @param[in]    recordPathPara  轨迹记录参数
 * @return       CRresult        返回处理结果
 */
CRSDKAPI CRresult cr_path_recordPara_set(RobotHandle robotHandle,RecordPathPara recordPathPara);

/**
 * @brief   获取当前轨迹记录状态
 * @param[in]    robotHandle      机械臂实例句柄
 * @param[out]   pathRecordStatus 当前轨迹记录状态
 * @return       CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_path_recordStatus_get(RobotHandle robotHandle,PathRecordStatus *pathRecordStatus);

/**
 * @brief   获取当前轨迹运行状态
 * @param[in]    robotHandle  机械臂实例句柄
 * @param[out]   pathRunMsg   当前轨迹运行状态
 * @return       CRresult     返回处理结果
 */
CRSDKAPI CRresult cr_path_currentRunStatus_get(RobotHandle robotHandle,PathRunMsg *pathRunMsg);
///力控
/**
* @brief 设置力控配置
* @param[in]       robotHandle      机械臂实例句柄
* @param[in]       forceConfig      力控配置参数
* @return          CRresult         返回处理结果
*/
CRSDKAPI CRresult cr_force_cfg_set(RobotHandle robotHandle,ForceConfig forceConfig);
/**
 * @brief 读力控配置
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      forceConfig      力控配置参数
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_cfg_get(RobotHandle robotHandle,ForceConfig *forceConfig);
/**
 * @brief 开启力控并下发相关设置
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[in]       forceSetting     力控设置参数
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_open(RobotHandle robotHandle,ForceSetting forceSetting);
/**
 * @brief 关闭力控
 * @param[in]       robotHandle      机械臂实例句柄
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_close(RobotHandle robotHandle);
/**
 * @brief 设置力控控制参数
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[in]       forceCtlPara     力控控制参数
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_para_set(RobotHandle robotHandle,ForceCtlPara forceCtlPara);
/**
 * @brief 读力控控制参数
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      forceCtlPara     力控控制参数
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_para_get(RobotHandle robotHandle,ForceCtlPara *forceCtlPara);
/**
 * @brief 读力控数据
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]       forceData       力控数据
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_force_data_get(RobotHandle robotHandle,ForceData *forceData);

/**
 * @brief 获取机械臂状态数据
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      robotStateData   机械臂状态数据
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_get_robotStateData(RobotHandle robotHandle,RobotStateData* robotStateData);

/**
 * @brief 获取编码器计数数量
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[in]       encoderChannel   编码器通道
 * @param[out]      encoderTickCount 编码器计数数量
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_get_encoderTickCnt(RobotHandle robotHandle,int encoderChannel,long long* encoderTickCount);

/**
 * @brief 设置工具输出电压
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[in]       toolPower        工具输出电压
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_set_ToolOutputVoltage(RobotHandle robotHandle, enum ToolPower toolPower);

/**
 * @brief 读取工具输出电压
 * @param[in]       robotHandle      机械臂实例句柄
 * @param[out]      toolPower        工具输出电压
 * @return          CRresult         返回处理结果
 */
CRSDKAPI CRresult cr_get_ToolOutputVoltage(RobotHandle robotHandle, enum ToolPower* toolPower);

/*
*************扩展轴*************
*/

/**
 * @brief   读取机械臂和扩展轴数据
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   robotExjData  待读取的值
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_robot_exj_data(RobotHandle robotHandle, RobotExjData* robotExjData);

/**
 * @brief   读取扩展轴使能状态
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    index          扩展轴索引
 * @param[out]   status         扩展轴状态
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_get_exj_enableStatus(RobotHandle robotHandle, int index, BOOL* status);

/**
 * @brief   设置扩展轴使能状态
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    index          扩展轴索引
 * @param[in]    status         扩展轴状态
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_set_exj_enableStatus(RobotHandle robotHandle, int index, BOOL status);

/**
 * @brief   扩展轴关节移动
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    moveExjPara    扩展轴关节移动相关参数
 * @param[in]    isBlock        TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_move_joint_exj(RobotHandle robotHandle, MoveExjPara moveExjPara, BOOL isBlock);

/**
 * @brief   扩展轴直线移动
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    moveExjPara    扩展轴直线移动相关参数
 * @param[in]    isBlock        TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_move_line_exj(RobotHandle robotHandle, MoveExjPara moveExjPara, BOOL isBlock);

/**
 * @brief   扩展轴点动运动
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    moveJogExjPara 扩展轴点动运动相关参数
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_moveJog_exj(RobotHandle robotHandle, MoveJogExjPara moveJogExjPara);

/**
 * @brief   扩展轴运动
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    exjMovePara    扩展轴运动相关参数
 * @param[in]    isBlock        TRUE-阻塞  FALSE-非阻塞
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_exj_move(RobotHandle robotHandle, ExjMovePara exjMovePara, BOOL isBlock);

/**
 * @brief   获取扩展轴配置
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    exjName        扩展轴名称
 * @param[out]   exjConfig      扩展轴配置
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_get(RobotHandle robotHandle, const char* exjName, ExjConfig* exjConfig);

/**
 * @brief   设置扩展轴配置
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    exjName        扩展轴名称
 * @param[in]    exjConfig      扩展轴配置
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_set(RobotHandle robotHandle, const char* exjName, ExjConfig exjConfig);

/**
 * @brief   增加扩展轴
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    exjConfig      扩展轴配置
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_add(RobotHandle robotHandle, ExjConfig exjConfig);

/**
 * @brief   删除扩展轴（只能删除最后一个）
 * @param[in]    robotHandle    机械臂实例句柄
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_delete(RobotHandle robotHandle);

/**
 * @brief   获取所有扩展轴配置
 * @param[in]    robotHandle        机械臂实例句柄
 * @param[out]   exjConfigArray     读取扩展轴配置所在的数组
 * @param[out]   validNumber        读取扩展轴配置中有效数据的长度
 * @return       CRresult           返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_get_all(RobotHandle robotHandle, ExjConfig* exjConfigArray, int* validNumber);

/**
 * @brief   根据索引获取扩展轴配置
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    index          扩展轴索引
 * @param[out]   exjConfig      扩展轴配置
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_get_by_index(RobotHandle robotHandle, int index, ExjConfig* exjConfig);

/**
 * @brief   根据索引设置扩展轴配置
 * @param[in]    robotHandle    机械臂实例句柄
 * @param[in]    index          扩展轴索引
 * @param[in]    exjConfig      扩展轴配置
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_exj_set_by_index(RobotHandle robotHandle, int index, ExjConfig exjConfig);

/*
*************机械单元*************
*/
/**
 * @brief   获取所有机械单元配置
 * @param[in]    robotHandle                    机械臂实例句柄
 * @param[out]   mechanicalUnitConfigArray      读取机械单元配置所在的数组
 * @param[out]   validNumber                    读取机械单元配置中有效数据的长度
 * @return       CRresult                       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_mechanical_unit_get_all(RobotHandle robotHandle, MechanicalUnitConfig* mechanicalUnitConfigArray, int* validNumber);
/**
 * @brief   获取机械单元配置
 * @param[in]    robotHandle                机械臂实例句柄
 * @param[in]    index                      机械单元索引
 * @param[out]   mechanicalUnitConfig       机械单元配置
 * @return       CRresult                   返回处理结果
 */
CRSDKAPI CRresult cr_cfg_mechanical_unit_get(RobotHandle robotHandle, int index, MechanicalUnitConfig* mechanicalUnitConfig);

/**
 * @brief   设置机械单元配置
 * @param[in]    robotHandle                机械臂实例句柄
 * @param[in]    index                      机械单元索引
 * @param[in]    mechanicalUnitConfig       机械单元配置
 * @return       CRresult                   返回处理结果
 */
CRSDKAPI CRresult cr_cfg_mechanical_unit_set(RobotHandle robotHandle, int index, MechanicalUnitConfig mechanicalUnitConfig);

/**
 * @brief   添加机械单元
 * @param[in]    robotHandle                机械臂实例句柄
 * @param[in]    mechanicalUnitConfig       机械单元配置
 * @return       CRresult                   返回处理结果
 */
CRSDKAPI CRresult cr_cfg_mechanical_unit_add(RobotHandle robotHandle, MechanicalUnitConfig mechanicalUnitConfig);

/**
 * @brief   删除机械单元（只能删除最后一个）
 * @param[in]    robotHandle    机械臂实例句柄
 * @return       CRresult       返回处理结果
 */
CRSDKAPI CRresult cr_cfg_mechanical_unit_delete(RobotHandle robotHandle);

/**
    * @brief       计算扩展轴坐标系
    * @param[in]   mecUnitIndex  机械单元索引
    * @param[in]   exjIndex      外部轴在机械单元中的索引
    * @param[in]   point1Pose    计算扩展轴坐标系的机械臂和外部轴位姿1：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）;e0,e1...e9（单位：mm或°）
    * @param[in]   point2Pose    计算扩展轴坐标系的机械臂和外部轴位姿2：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）;e0,e1...e9（单位：mm或°）
    * @param[in]   point3Pose    计算扩展轴坐标系的机械臂和外部轴位姿3：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）;e0,e1...e9（单位：mm或°）
    * @param[in]   point4Pose    计算扩展轴坐标系的机械臂和外部轴位姿4：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）;e0,e1...e9（单位：mm或°）
    * @param[out]  exjCS         扩展轴坐标系的位姿：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    * @param[out]  errorValue    计算误差
    * @return      success/0     成功，其它失败
*/
CRSDKAPI CRresult cr_compute_cs_exj(RobotHandle robotHandle, int mecUnitIndex, int exjIndex, double *point1Pose, double *point2Pose, double *point3Pose, double *point4Pose, double *exjCS, double *errorValue);

/**
 * @brief   读取机械臂报警时锁存的数据
 * @param[in]    robotHandle   机械臂实例句柄
 * @param[out]   alarmData     报警数据
 * @return       CRresult      返回处理结果
 */
CRSDKAPI CRresult cr_get_alarm_data(RobotHandle robotHandle, AlarmData *alarmData);



#ifdef __cplusplus
}
#endif

#endif // ROBOTAPI_H

