
#ifndef CDRTDEAPI_H
#define CDRTDEAPI_H

#include <stdlib.h>
#include <stdint.h>
#include "cdrtdebasedefine.h"


#ifdef _WIN32
    #ifdef RTDE_DLL_EXPORTS
        #define RTDEAPI_API __declspec(dllexport)
    #else
        #define RTDEAPI_API
    #endif
#else
    #define RTDEAPI_API
#endif

#ifdef __cplusplus
extern "C" {
#endif


/*
用户函数，创建rtde连接
输入参数：
rtdeHandle：返回rtde句柄
ip：ip地址
if_name：网卡名称
srcmac：源mac地址（可选）uint8_t srcmac[6]={0xxx,0xxx,0xxx,0xxx,0xxx,0xxx},如果设置（NULL），则使用系统默认值
version：协议版本
nack：是否启用nack机制：0-不启用；1-启用（仅在协议版本3.0及以上有效）
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_create(RtdeHandle *rtdeHandle,const char *ip,const char* if_name,uint8_t* srcmac,int64_t version,int nack);


/*
用户函数，关闭rtde连接
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_destroy(RtdeHandle rtdeHandle);


/*
用户函数，配置输入配方，控制器只记住最新一条输入配方
输入参数：
rtdeHandle：rtde句柄
varNames：输入变量名
recipeId：返回输入配方ID
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_input_setup(RtdeHandle rtdeHandle,char *varNames, int *recipeId);

/*
用户函数，配置输出配方，控制器只记住最新一条输出配方
输入参数：
rtdeHandle：rtde句柄
varNames：输出变量名
frequency：输出频率
recipeId：返回输出配方ID
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_output_setup(RtdeHandle rtdeHandle, char *varNames, int frequency,int *recipeId);

/*
用户函数，获取SDK版本号
输入参数：
version：返回版本号
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_get_sdk_version(uint8_t* version);


/*
用户函数，输出数据控制
输入参数：
rtdeHandle：rtde句柄
recipeId：配方ID
control： 1开始  2-停止
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_output_control(RtdeHandle rtdeHandle,uint32_t recipeId,int control);

/*
用户函数，接收输出数据
输入参数：
rtdeHandle：rtde句柄
recipeId：配方ID
data：输出数据指针
data_size：输出数据大小
返回值：
0-正常；其他-错误

注意：由于此接口对应唯一物理接口，因此在多线程环境下，需要保证同一时刻只有一个线程调用此接口
*/
RTDEAPI_API RtdeResult cd_rtde_output_data_receive(RtdeHandle rtdeHandle,uint32_t recipeId,void *data,int data_size);


/*
用户函数，发送输入数据
输入参数：
rtdeHandle：rtde句柄
recipeId：配方ID
data：输入数据指针
返回值：
0-正常；其他-错误

注意：由于此接口对应唯一物理接口，因此在多线程环境下，需要保证同一时刻只有一个线程调用此接口
*/
RTDEAPI_API RtdeResult cd_rtde_input_data_send(RtdeHandle rtdeHandle,uint32_t recipeId,void *data);

/*
用户函数，设置速度百分比
输入参数：
rtdeHandle：rtde句柄
percentVelocity：速度百分比
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_percentvelocity_set(RtdeHandle rtdeHandle,double percentVelocity);

//方法
/*
用户函数，关节透传运动
输入参数：
has_robot：是否控制机械臂
jointPose：关节给定位置（°）
exjNum: 外部轴数量
exjPose：外部轴给定位置（°/mm）
cutoffFreq：滤波截止频率（rad/s）
返回值：
0-正常；其他：-错误

注意：由于此接口对应唯一物理接口，因此在多线程环境下，需要保证同一时刻只有一个线程调用此接口
*/
RTDEAPI_API RtdeResult cd_rtde_servoj(RtdeHandle rtdeHandle,uint8_t has_robot,double jointPose[6],uint8_t exjNum,double *exjPose,double cutoffFreq);

/*
用户函数，停止关节透传运动
输入参数：
rtdeHandle：rtde句柄
deceleration：关节降速的加速度值（°/s²）
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_servo_stop(RtdeHandle rtdeHandle,double deceleration);


/*
用户函数，重置错误
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_reseterror(RtdeHandle rtdeHandle);



/*
用户函数，获取控制器版本信息
输入参数：
rtdeHandle：rtde句柄
controller_info： 版本信息结构体指针
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_get_control_version(RtdeHandle rtdeHandle,controller_info_t *controller_info);



/*
用户函数，设置标准输出
输入参数：
rtdeHandle：rtde句柄
channel：通道号
level：输出电平
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_standarddigitalout_set(RtdeHandle rtdeHandl,uint8_t channel,uint8_t level);



/*
用户函数，设置可配置输出
输入参数：
rtdeHandle：rtde句柄
channel：通道号
level：输出电平
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_configurabledigitalout_set(RtdeHandle rtdeHandl,uint8_t channel,uint8_t level);


/*
用户函数，设置工具输出
输入参数：
rtdeHandle：rtde句柄
channel：通道号
level：输出电平
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_tooldigitalout_set(RtdeHandle rtdeHandle,uint8_t channel,uint8_t level);


/*
用户函数，获取所有输入位状态
输入参数：
rtdeHandle：rtde句柄
digitalInputBits：输入位状态指针,0-7: Standard   8-15: Configurable  16-31: Tool(依据型号与配置确定)
返回值：
0-正常；其他：-错误     
*/
RTDEAPI_API RtdeResult cd_rtde_alldigitalinputbits_get(RtdeHandle rtdeHandle,uint64_t *digitalInputBits);


/*
用户函数，获取所有输出位状态
输入参数：
rtdeHandle：rtde句柄
digitalOutputBits：输出位状态指针,0-7: Standard   8-15: Configurable  16-31: Tool(依据型号与配置确定)
返回值：
0-正常；其他：-错误     
*/
RTDEAPI_API RtdeResult cd_rtde_alldigitaloutputbits_get(RtdeHandle rtdeHandle,uint64_t *digitalOutputBits);



/*
用户函数，启动惯性补偿
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他：-错误
*/
RTDEAPI_API RtdeResult cd_rtde_startinertiacomp(RtdeHandle rtdeHandle);


/*
用户函数，关闭惯性补偿
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_stopinertiacomp(RtdeHandle rtdeHandle);

/*
用户函数，开启力矩补偿
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_starttorquecomp(RtdeHandle rtdeHandle);


/*
用户函数，关闭力矩补偿
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_stoptorquecomp(RtdeHandle rtdeHandle);


/*
用户函数，开启日志输出
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_startlogoutput(RtdeHandle rtdeHandle);


/*
用户函数，开启传感器力输入
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_startsensorforceinput(RtdeHandle rtdeHandle);



/*
用户函数，停止传感器力输入
输入参数：
rtdeHandle：rtde句柄
返回值：
0-正常；其他-错误
*/
RTDEAPI_API RtdeResult cd_rtde_stopsensorforceinput(RtdeHandle rtdeHandle);




#ifdef __cplusplus
}
#endif

#endif /* CDRTDEAPI_H */
