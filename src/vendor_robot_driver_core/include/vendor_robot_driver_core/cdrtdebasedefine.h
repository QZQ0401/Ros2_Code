
#ifndef CDRTDEBASEDEFINE_H
#define CDRTDEBASEDEFINE_H

typedef int RtdeHandle;

typedef struct
{
    unsigned short model;
    unsigned char firmware_version[20];
    unsigned char hardware_version[20]; 
    unsigned char bootloader_version[20];
}controller_info_t;

typedef enum RtdeResult
{
    RTDE_ERR_UNKNOW                         =-1,
    RTDE_ERR_NONE                           = 0,//	success
    RTDE_ERR_PACKAGE_TYPE                   = 1,//	package type is unidentify
    RTDE_ERR_PORT_UNMATCH                   = 2,//	interface unmatch with method
    RTDE_ERR_PAYLOAD_LEN                    = 3,//	package len is not correct
    RTDE_ERR_PROTOCOL_VER_NSUPPORT          = 4,//	the request proto version is not support
    RTDE_ERR_DATA_TYPE                      = 5,//	data type is unidentify
    RTDE_ERR_PARAM                          = 6,//	package params is not correct
    RTDE_ERR_EXECUTE_NALLOW                 = 7,//	package type not allow exec
    RTDE_ERR_IO_NSUPPORT                    = 8,//	the IO-dev is not support
    RTDE_ERR_METHOD_INDEX                   = 100,//	the method index is not support
    RTDE_ERR_VARIABLE_CODE                  = 101,//	the variable code is not identify
    RTDE_ERR_VARIABLE_NSUPPORT              = 102,//	the variable is not support
    RTDE_ERR_RECIPE_LEN                     = 103,//	the recipe length is override
    RTDE_ERR_RECIPE_INPUT_VAR_CODE_REUSED   = 104,//	variable reused in input recipe
    RTDE_ERR_RECIPE_INPUT_LIMIT             = 105,//	the input recipe reach the upper limit
    
    RTDE_ERR_ULTRALIMIT                     = 200,//	the input value is out of ultraralimit
    RTDE_ERR_METHOD_OVER                    = 201,//	the method is over the limit

    //控制器返回错误码
    
    RTDE_ERR_DATALEN_ULTRALIMIT             = 300,//	数据长度超出限制
    RTDE_ERR_DATA_NOT_EXIST                 = 301,//	数据不存在
    RTDE_ERR_SOCKET_ERROR                   = 302,//	socket错误
    RTDE_ERR_REQUEST_ERROR                  = 303,//    请求错误	
    RTDE_ERR_RAW_SOCK_ERROR                 = 304,//    原始套接字错误
    //

} RtdeResult;

#endif // CDRTDEBASEDEFINE_H

