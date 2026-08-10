#ifndef BASESTRUCT_H
#define BASESTRUCT_H


#define ROB_AXIS_NUM 6
#define CR6_DI_NUM 8
#define CR6_DO_NUM 8
#define CR6_CI_NUM 8
#define CR6_CO_NUM 8
#define CR6_AI_NUM 3
#define CR6_AO_NUM 1
#define CR6_TOOL_DI_NUM 10
#define CR6_TOOL_DO_NUM 10
#define CR6_TOOL_AI_NUM 2

#define ModbusMasterConfigNumber 20
#define ModbusAddrMapNumber 100
#define CustomAddrMapNumber 100
#define CR6_SAFETY_LIMITS_BOUNDARY_PLANE_NUM 8

#define CR6_BOOL_REG_NUM 1000
#define CR6_INT16_REG_NUM 1000
#define CR6_INT32_REG_NUM 1000
#define CR6_FLOAT_REG_NUM 1000
#define CR6_REG_MAX_NUM 100

#define ENCODER_TICK_NUM 2

#define DH_PARA_MAX_NUM 100
#define DY_PARA_MAX_NUM 100

#define CR6_ISTALL_VAR_NUM 100
#define CR6_ISTALL_VAR_NAME_LENGTH 100
#define CR6_INSTALL_VAR_MAX_LENGTH  100
#define CR6_INSTALL_VAE_TABLE_MAX_LENGTH 10000
#define Script_MAX_TableElement 1000
#define Script_MAX_DataLen 100
#define Global_Var_Max_Num 100  ///读全局变量最大个数
#define Set_Global_Var_Max_Num 1 ///写读全局变量最大个数
#define COR_MAX_NUM 50 ///协程最大个数
#define CR6_EQEP_NUM    2
#define TCP_MAX_NUM 20
#define NAMELENTH  50

#define CR6_DIGITIAL_INPUT_NUM 8
#define CR6_DIGITIAL_OUTPUT_NUM 8
#define CR6_CONFIGURABLED_INPUT_NUM 8
#define CR6_CONFIGURABLED_OUTPUT_NUM 8
#define CR6_ANALOG_INPUT_NUM 3
#define CR6_ANALOG_OUTPUT_NUM 1
#define CR6_TOOL_ANALOG_INPUT_NUM 2

#define CR6_POSE_NODE_NUM 10
#define CR6_LINE_NODE_NUM 10
#define CR6_PLANE_NODE_NUM 10
#define CR6_WCS_CONF_NUM 5
#define CR6_COM_NUM 4
#define LOG_MAX_NUM 3000
#define LOG_RPC_MAX_NUM 10
#define MOVECPOINT_NUM 2
#define PATHMAXPOINTS 10000   ///<轨迹最大数量为10000个点

#define VERSION_MAX_LENGTH 50

#define EXTERNAL_MAX_NUM 10 //扩展轴最大个数
#define ROB_EXJ_AXIS_NUM 12
#define ROB_EXJOG_AXIS_NUM 16

#define MECHANICAL_MAX_NUM 10 //机械单元最大个数
#define REF_EXJ_CONFIG_MAX_NUM 6 //每个机械单元最大扩展轴数量

#define SENSORPOSE_MAX_NUM 5 //力传感器安装位置最大个数
#define SENSOR_RANGE_DIM 6//力传感器量程最大维度

typedef int RobotHandle;

typedef int BOOL;
#define TRUE 1
#define FALSE 0

#define sucess success

typedef enum CRresult
{
    success = 0,             ///<成功；分别有以下3种：连接、获取参数、写入参数
    error = 1,              ///<一般错误
    thread_running = 2,     ///<通讯连接重复建立
    operate_timeout = 3,    ///<操作超时，无返回值；注：处理超时导致无结果输出
    result_invalid = 4,     ///<返回结果无效；原因：rpc获取结果为空
    out_of_range = 5,       ///<输入数值越界；需输入在范围内的值
    mutex_invalid = 6,      ///<内部错误
    para_error = 7,         ///<参数无效；需检查输入参数是否为空
    no_result = 8,          ///<无对应数值
    no_assignTCPindex = 9,   ///<未找到指定索引号TCP偏移量
    no_handle = 10,          ///<未创建相关句柄
    handle_repeat = 11,      ///<重复建立相同的句柄
    repeat_name = 12,        ///<重命名
    delete_invalid = 13,     ///<删除唯一的TCP或者负载
    set_bit_reg_invalid = 14,   ///<同时设置位寄存器输入与输出
    repeat_id = 15,           ///<id重复
    file_encryption = 16,      ///文件加密
    robotmode_error = 17,    ///<当前状态不能执行该操作
    move_error = 18,   /// 移动阻塞运动时,返回的移动状态错误
    forcectrl_wrongpara = 19,    ///力控SDK参数设置错误：包括参数长度错误、payload为负数
    forcectrl_invaildcmd = 20,   ///力控SDK无效的指令：包括forceEna等于1,0以外的值forcebasetype除0以外的值 forcectrltype除1,0以外  auto/mannual 1,0以外的值
    forcectrl_parablocked = 21,   ///力控SDK freedrive模式下部分参数失效被屏蔽
    forcectrl_cmdblocked = 22,   ///力控SDK 已在力控模式下时禁止再打开力控
    forcectrl_configblocked = 23,   ///力控SDK 机器人运动时或已在力控模式下时禁止下发配置
    forcectrl_notenable = 24,   ///力控SDK 机器人使能之前不能打开传感器
    forcectrl_wrongsensor = 25,  ///力控SDK 传感器型号选择错误
    outputrule_error = 26,  ///<未设置系统输出，禁止启用自复位
    analoginputtype_error = 27   ///<模拟量输入类型错误：注：工具端模拟量输入类型只有电压
}CRresult;

typedef enum CommuVarType
{
    CommuVarType_Bool = 0,      ///<bool类型
    CommuVarType_Int16 = 1,     ///<16bit整数类型
    CommuVarType_Int32 = 2,     ///<32bit整数类型
    CommuVarType_Float = 3      ///<32bit浮点类型
}CommuVarType;

typedef enum CoordinateType
{
    baseCoordinate = 0,         ///<基坐标系
    jointCoordinate = 1,        ///<关节坐标系
    PointCoordinate = 2,        ///<点坐标系
    LineCoordinate = 3,         ///<线坐标系
    PlaneCoordinate = 4,        ///<面坐标系
    ToolBaseCoordinate = 5,     ///<工具坐标系
    visionCoordinate = 6,       ///<视角坐标系
    pathCoordinate = 7,         ///<轨迹坐标系
    worldCoordinate = 8,        ///<世界坐标系
    workCoordinate = 9          ///<工件坐标系
}CoordinateType;

typedef enum PointTransType
{
    pointTransStop = 0,             ///<过渡点停止
    pointTransConstantSpeed = 1,    ///<匀速过渡
    pointTransSmooth = 2            ///<柔顺过渡（不匀速，不停止）
}PointTransType;

typedef enum PoseTranType  ///预留参数
{
    poseTranMoveToTargetPose = 0,
    poseTranKeepStartPose = 1
}PoseTranType;

typedef enum MotiontriggerMode
{
    MovetriggerbySafetySingnal = 0,     ///<安全同步信号触发
    MovetriggerbyOnlyRpc = 1,            ///<指令直接触发
    MovetriggerbyMoveCache = 2          ///<移动指令会进入缓存区
}MotiontriggerMode;

typedef struct TCPMsg
{
    char tcpName[20];                  ///tcp名字
    double tcpOffset[ROB_AXIS_NUM];    ///<tcp偏移量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
}TCPMsg;

typedef struct PayLoad
{
    char payloadName[20];                 ///负载名称
    double toolPayload;                   ///负载重量:kg
    double centerOfGravity[3];            ///负载中心:CX，CY，CZ（单位：mm）
}PayLoad;

typedef struct  PointControlPara
{
    double pose[ROB_AXIS_NUM];                  ///<坐标点向量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double jointpos[ROB_AXIS_NUM];              ///<关节角度:关节1~6（单位：°）
    double tcpOffset[ROB_AXIS_NUM];             ///<tcp偏移量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    int  tcpID;                                 ///<tcp的ID号,当tcpID为非负数时，使用tcpID对应的TCP偏移量作为输入；当tcpID为负数时，使用tcpoffset数据作为TCP偏移量输入；
    enum CoordinateType  coordinateType;        ///<坐标系类型
    double coordinatePose[ROB_AXIS_NUM];        ///<参考坐标系位姿:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    ///<speed可以是tcp速度或者是关节速度,当coordinateType设置为jointCoordinate时，该速度为关节速度，其它情况是tcp速度
    ///<作为tcp速度时，单位为：mm/s; 作为关节速度时，单位为：°/s
    double speed[ROB_AXIS_NUM];
    ///<acc可以是tcp加速度或者是关节加速度,当coordinateType设置为jointCoordinate时，该速度为关节加速度，其它情况是tcp加速度
    ///<作为tcp加速度时，单位为：mm/s²; 作为关节加速度时，单位为：°/s²
    double acc[ROB_AXIS_NUM];
    double jerk[ROB_AXIS_NUM];                  ///<加加速度，预留参数
    enum PointTransType pointTransType;         ///<位置过渡方式
    double pointTransRadius;                    ///<过渡半径，预留参数
    enum PoseTranType poseTranType;             ///<姿态变换方式，预留参数
    enum MotiontriggerMode motiontriggerMode;   ///<运动轨迹触发方式
}PointControlPara;

///< 圆弧运动参数
typedef struct  PointControlParaList
{
  PointControlPara pointcontrolpara[MOVECPOINT_NUM];  ///<过渡点和终点
  int fixedrot;  ///<方向模式 0-无约束 1-固定
  double centralangle; ///<圆弧角，单位：°
}PointControlParaList;

typedef struct  PointControlParaSimple
{
    double pose[ROB_AXIS_NUM];                  ///<坐标点向量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double jointpos[ROB_AXIS_NUM];              ///<关节角度:关节1~6（单位：°）
    double tcpOffset[ROB_AXIS_NUM];             ///<tcp偏移量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    enum CoordinateType  coordinateType;        ///<坐标系类型
    double coordinatePose[ROB_AXIS_NUM];        ///<参考坐标系位姿:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    ///<speed可以是tcp速度或者是关节速度,当coordinateType设置为jointCoordinate时，该速度为关节速度，其它情况是tcp速度
    ///<作为tcp速度时，单位为：mm/s; 作为关节速度时，单位为：°/s
    double speed[ROB_AXIS_NUM];                 ///<速度
    ///<acc可以是tcp加速度或者是关节加速度,当coordinateType设置为jointCoordinate时，该速度为关节加速度，其它情况是tcp加速度
    ///<作为tcp加速度时，单位为：mm/s²; 作为关节加速度时，单位为：°/s²
    double acc[ROB_AXIS_NUM];                   ///<加速度
    enum PointTransType pointTransType;         ///<位置过渡方式
    enum MotiontriggerMode motiontriggerMode;   ///<运动轨迹触发方式
}PointControlParaSimple;

typedef enum MoveType
{
    DecStop = 0,    ///<当前运动减速停止
    ImdStop = 1,	///<当前运动立即停止
    Run = 2  		///<运动
}MoveType;

typedef enum JointModes
{
    JOINT_MODE_POWER_OFF = 1,					///<断电
    JOINT_MODE_BOOTING = 2,			  			///<机械臂上电至空闲状态下
    JOINT_MODE_IDLE = 3,						///<上电完成,空闲状态（ethercat——>op）
    JOINT_MODE_BACK_DRIVE = 4,					///<反向驱动状态
    JOINT_MODE_RELEASE_BRAKE = 5,				///<机械臂使能，松抱闸过程中
    JOINT_MODE_CSP_STOP = 7,					///<关节使能静止(位置模式)
    JOINT_MODE_CSP_MOVING = 8,					///<关节使能运动(位置模式)
    JOINT_MODE_CST_STOP = 9,					///<关节使能静止(转矩模式)
    JOINT_MODE_CST_MOVING = 10,					///<关节使能运动(转矩模式)
    JOINT_MODE_FAULT = 11,			    		///<错误状态
    JOINT_MODE_READY_FOR_POWER_OFF = 12 		///<准备断电
}JointModes;

typedef enum RequestState
{
    EC_INT_REQUEST_INIT = 0,     /// Init
    EC_INT_REQUEST_QUEUED = 1,   /// Queued
    EC_INT_REQUEST_BUSY = 2,     /// Busy
    EC_INT_REQUEST_SUCCESS = 3,  /// success
    EC_INT_REQUEST_FAILURE = 4   /// failure
}RequestState;

typedef enum RobotModes
{
    ///初始化
    Closed = 0,                           ///< 关闭
    Disconnect = 1,                       ///< 断开连接状态
    ConfirmSafty = 2,                     ///< 安全确认状态
    Booting = 3,                          ///< 控制柜正在启动
    ControlerIdle = 4,                    ///< 控制柜启动完成，空闲状态
    ControlerUpdataFirmWare = 5,          ///< 固件更新
    JointPowerOff = 6,                    ///< 控制柜启动完成，本体未上电
    JointPowerOn = 7,                     ///< 机器人上电过程
    JointIdle = 8,                        ///< 机器人上电完成，空闲
    BackDrive = 9,                        ///< 反向驱动
    ReleaseBrake = 10,                    ///< 机器人使能、松抱闸中
    Enable = 11,                          ///< 机器人使能完成，并已完全松开抱闸
    CloseBrake = 12,                      ///< 机器人断使能、关抱闸中
    ///运动
    Jog = 100,                            ///< 点动
    Teach = 101,                          ///< 拖动示教
    SDK_Moving = 102,                     ///< SDK运动
    ProgramStop = 103,                    ///< 使能完成，程序停止
    ProgramPause = 104,                   ///< 使能完成，程序暂停
    ProgramStopping = 105,                ///< 使能完成，程序和运动正在停止
    ProgramPausing = 106,                 ///< 使能完成，程序和运动正在暂停
    ProgramRun_MotionStop = 107,          ///< 程序运行，运动停止
    ProgramRun_MotionReducing = 108,      ///< 程序运行，运动减速中
    ProgramRun_MotionMoving  = 109,       ///< 程序运行
    ProgramRun_MotionCanBlend = 110,      ///< 程序运行，机器人进入到可交融区域
    Imdstop = 111,                        ///< 紧急停止
    ProtectiveStop = 112                  ///< 防护停止
}RobotModes;

typedef enum ControlModes
{
    CONTROL_MODE_POSITION = 0,					///< 控制柜启动后的各种RobotModes
    CONTROL_MODE_TEACH = 1,						///< ROBOT_MODE_RUNNING下，按下示教按钮或者执行teach_mode()，在此模式下手动使机器人来回移动
    CONTROL_MODE_FORCE = 2,						///< 力控命令内
    CONTROL_MODE_TORQUE = 3						///< 自由驱动模式
}ControlModes;

typedef enum SafetyModes
{
    SAFETY_MODE_UNDEFINED_SAFETY_MODE = 1,  ///< 未定义
    SAFETY_MODE_VALIDATE_JOINT_ID = 2,		///< 验证关节的ID
    SAFETY_MODE_FAULT = 3,                  ///< 安全错误，其他错误出现一段时间后，最终保持的错误状态
    SAFETY_MODE_VIOLATION = 4,              ///< 在安全平面处越界
    SAFETY_MODE_ROBOT_EMERGENCY_STOP = 5,	///< 急停按钮按下
    SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6,	///< 可配置IO输入任意两路设置为紧急停机，断开后出现的错误状态
    SAFETY_MODE_SAFEGUARD_STOP = 7,         ///< 安全IO断开
    SAFETY_MODE_RECOVERY = 8,               ///< 在违反安全限制后，机器人停止，重启后，进入恢复模式
    SAFETY_MODE_PROTECTIVE_STOP = 9,        ///< 保护性停止，运动过程中，外部强力推动机械手
    SAFETY_MODE_REDUCED	= 10,               ///< 限制模式
    SAFETY_MODE_NORMAL = 11                 ///< 正常运行模式
}SafetyModes;

///程序状态
typedef enum ProgramState
{
    PROGRAM_STATE_STOP = 0,		///< 程序停止
    PROGRAM_STATE_PAUSE = 1,        ///< 程序暂停
    PROGRAM_STATE_RUN = 2          ///< 程序运行
}ProgramState;

typedef struct ProgFileInfo    ///控制器程序信息
{
    int progfileIndex;     ///程序索引
    char progfileName[NAMELENTH];     ///程序名称
}ProgFileInfo;

typedef struct Version
{
    unsigned char deviceType[VERSION_MAX_LENGTH];       ///< 机器人类型
    unsigned char versionNo[VERSION_MAX_LENGTH];        ///< 版本信息
    int bugfix;                                         ///< bug修正
    long long buildDate;                                ///< 编译日期
    unsigned char hardwareID[VERSION_MAX_LENGTH];       ///< 硬件id
    unsigned char bootVersionNo[VERSION_MAX_LENGTH];    ///<boot版本号
}Version;

///脚本程序状态
typedef enum Lua_ScriptStatus
{
    lua_Script_NoneOP = 0,		///< 空
    lua_Script_stop = 1,		///< 脚本程序停止
    lua_Script_pause = 2,		///< 脚本程序暂停
    lua_Script_run = 3,			///< 脚本程序运行
    lua_Script_load = 4			///< 脚本程序正在加载
}Lua_ScriptStatus;

typedef enum VariableType
{
    LUA_TNIL = 0,                ///< 空
    LUA_TBOOLEAN = 1,            ///<布尔
    LUA_TLIGHTUSERDATA = 2,      ///<预留
    LUA_TNUMBER = 3,             ///<数字
    LUA_TSTRING = 4,             ///<字符串
    LUA_TTABLE = 5,              ///<表
    LUA_TFUNCTION = 6,           ///<函数
    LUA_TUSERDATA = 7,           ///<自定义类型
    LUA_TTHREAD = 8,             ///<线程
    LUA_NUMTAGS = 9              ///<预留
}VariableType;

///元素数据结构体
typedef struct  VarItemData
{
    enum VariableType varItemType;                 ///< 数据类型
    BOOL boolValue;                                ///< bool类型数据值
    double numberValue;                            ///< number类型数据值
    char stringValue[CR6_INSTALL_VAR_MAX_LENGTH];  ///< char类型字符串数据值
}VarItemData;

typedef struct VariableMsg
{
    char variableName[CR6_ISTALL_VAR_NAME_LENGTH];   ///< 变量名
    enum  VariableType variableType;                 ///< 数据类型
    int variableID;                                  ///< 变量id
    BOOL boolValue;                                  ///< bool类型数据值
    double numberValue;                              ///< number类型数据值
    char stringValue[CR6_INSTALL_VAR_MAX_LENGTH];    ///< char类型字符串数据值

    int tableValueCount;                             ///< table类型元素个数
    VarItemData tableValue[Script_MAX_TableElement]; ///< table类型元素值
    char tableValueStr[CR6_INSTALL_VAE_TABLE_MAX_LENGTH];///< table值
}VariableMsg;

///消息来源
typedef enum CommMessageSource
{
    Base = 0,               ///<基座关节（关节1）
    Shoulder = 1,           ///<肩部关节（关节2）
    Elbow = 2,              ///<肘部关节（关节3）
    Wrist1 = 3,             ///<腕关节1（关节4）
    Wrist2 = 4,             ///<腕关节2（关节5）
    Wrist3 = 5,             ///<腕关节3（关节6）
    Tool = 6,               ///<工具模块
    Controller = 100,       ///<主控制器
    SafetyProcessorB = 101, ///<协处理器
    TeachPendant = 200      ///<示教器
}CommMessageSource;

///警告等级
typedef enum WarningLevel
{
    MESSAGE_WARNING_LEVEL_INFO = 1,         ///<正常消息
    MESSAGE_WARNING_LEVEL_WARNING = 2,      ///<警告不停机
    MESSAGE_WARNING_LEVEL_VIOLATION = 3,    ///<违反限制停机
    MESSAGE_WARNING_LEVEL_FAULT = 4,        ///<系统故障，紧急停止
    MESSAGE_WARNING_LEVEL_USER = 5          ///<用户打印输出
}WarningLevel;

typedef struct  WorldTime
{
    unsigned int year;               /// 年
    unsigned int month;              /// 月
    unsigned int day;                /// 日
    unsigned int week;               /// 周
    unsigned int hour;               /// 时
    unsigned int minute;             /// 分
    unsigned int second;             /// 秒
    unsigned int milliSecond;        /// 毫秒
}WorldTime;

typedef struct RobotLogMsg
{
    unsigned int robotMessageCode;      ///<消息代码
    unsigned int robotMessageType;      ///<消息类型
    enum CommMessageSource source;      ///<消息来源
    char textMessage[400];              ///<消息信息
    WorldTime worldTime;                ///<时间戳
    enum WarningLevel warningLevel;     ///<警告等级
    RobotModes robotMode;               ///<机器人状态
    unsigned long long timeStamp_us; 	///<精确时间戳
    unsigned long long messageIndex;	///<消息序号
}RobotLogMsg;

///RS485/232的结构体内容
///波特率
typedef enum BaudRate
{
  BaudRate300 = 0,                ///波特率为300Baud
  BaudRate600 = 1,                ///波特率为600Baud
  BaudRate1200 = 2,               ///波特率为1200Baud
  BaudRate2400 = 3,               ///波特率为2400Baud
  BaudRate4800 = 4,               ///波特率为4800Baud
  BaudRate9600 = 5,               ///波特率为9600Baud
  BaudRate19200 = 6,              ///波特率为19200Baud
  BaudRate28800 = 7,              ///波特率为28800Baud
  BaudRate38400 = 8,              ///波特率为38400Baud
  BaudRate57600 = 9,              ///波特率为57600Baud
  BaudRate115200 = 10,            ///波特率为115200Baud
  BaudRate192000 = 11,            ///波特率为192000Baud
  BaudRate256000 = 12,            ///波特率为256000Baud
  BaudRate288000 = 13,            ///波特率为288000Baud
  BaudRate384000 = 14,            ///波特率为384000Baud
  BaudRate512000 = 15,            ///波特率为512000Baud
  BaudRate576000 = 16,            ///波特率为576000Baud
  BaudRate768000 = 17,            ///波特率为768000Baud
  BaudRate1000000 = 18,           ///波特率为1000000Baud
  BaudRate1200000 = 19,           ///波特率为1200000Baud
  BaudRate1500000 = 20,           ///波特率为1500000Baud
  BaudRate2400000 = 21,           ///波特率为2400000Baud
  BaudRate3000000 = 22,           ///波特率为3000000Baud
  BaudRate460800 = 23             ///波特率为460800Baud
}BaudRate;
typedef enum  ParityCheck
{
  no = 0,       ///无校验
  odd = 1,      ///奇校验
  even = 2      ///偶校验
}ParityCheck;
typedef enum SerialCommuType
{
     FreeComm = 0,       ///自由格式
     Modbus_RTU = 1,	 ///modbus RTU格式
     Modbus_ASCII = 2	 ///modbus ASCII格式
}SerialCommuType;

typedef enum SerialType
{
    rs485_tool_CommSettings = 0,   ///工具485
    rs485_1_CommSettings = 1,      ///485-1
    rs485_2_CommSettings = 2,	   ///485-2
    rs232_CommSettings=3,          ///rs232
    ethernet_CommSettings = 4      ///以太网
}SerialType;

///串口数据
typedef struct SerialCommSettings
{
    enum BaudRate baudRate;                  ///波特率
    enum ParityCheck parity;                 ///奇偶校验
    int dataBits;                            ///数据位
    double stopBits;                         ///停止位
    int retryNumber;                         ///重试次数
    int timeout;                             ///超时时间,单位：ms
    enum SerialCommuType serialCommType;     ///串口通信类型
    int modbusSlaveNo;                       ///modbus从站站号
}SerialCommSettings;

typedef struct SysVersion
{
     Version sjqVersion;                     ///示教器版本
     Version kzqVersion;                     ///控制器版本
     Version jpmVersion;                     ///本体放电板版本
     Version saftyCtrlVersion;               ///安全板版本
     Version jointVersion[ROB_AXIS_NUM];     ///关节版本
     Version etfVersion;                     ///工具法兰版本
}SysVersion;

typedef enum VarDigitialIOType   ///IO类型
{
    Invaild = 0,                ///无效
    Digitial = 1,               ///标准数字量
    ConfigureableDigitial = 2,  ///可配置数字量
    ToolDigitial = 3,           ///工具数字量
    Analog = 4,                 /// 模拟量
    ToolAnalog = 5              ///工具模拟量
}VarDigitialIOType;

typedef struct InstallAngle
{
    double baseAngle;	///基座旋转角度,单位：°
    double tiltAngle;	///基座倾斜角,单位：°
}InstallAngle;

///本体上电使能、回初始点、运行程序、暂停程序、停止程序
typedef enum InputAction
{
    InputAction_NONE = 0,                 ///无动作
    InputAction_RobotON = 1,              ///本体上电使能
    InputAction_MovetoHome = 2,           ///回初始点
    InputAction_RunProgram  = 3,          ///运行程序
    InputAction_SuspendedProgram = 4,     ///暂停程序
    InputAction_StopProgram = 5,          ///停止程序
    InputAction_FreeDrive = 6,            ///自由拖动
    InputAction_DownEnable = 7,           ///下使能
    InputAction_ClearFault = 8            ///清除故障
}InputAction;

///系统功能输出：使能完成、到达初始点、程序运行状态、运动状态
typedef enum OutputAction
{
    OutputAction_NONE = 0,                        ///无动作
    OutputAction_RobotON_HIGH = 1,                ///机器人使能时为高
    OutputAction_RobotON_LOW = 2,                 ///机器人使能时为低
    OutputAction_RobotPowerOn_HIGH = 3,           ///已上电时为高
    OutputAction_RobotPowerOn_LOW = 4,            ///已上电时为低
    OutputAction_PowerOn_HIGH = 5,                ///已开机时为高
    OutputAction_PowerOn_LOW = 6,                 ///已开机时为低
    OutputAction_WarnErr_HIGH = 7,                ///有系统错误时为高
    OutputAction_WarnErr_LOW = 8,                 ///有系统错误时为低
    OutputAction_ProgramStatus_Stop_HIGH = 10,    ///程序停止时为高
    OutputAction_ProgramStatus_Stop_LOW = 11,     ///程序停止时为低
    OutputAction_Fault_Stop_HIGH = 12,            ///异常停止时为高
    OutputAction_Fault_Stop_LOW = 13,             ///异常停止时为低
    OutputAction_ProgramStatus_Pause_HIGH = 14,   ///程序暂停输出为高
    OutputAction_ProgramStatus_Pause_LOW = 15,    ///程序暂停输出为低
    OutputAction_MotionStatus_Moving_HIGH = 20,   ///运动时为高
    OutputAction_MotionStatus_Moving_LOW = 21,    ///运动时为低
    OutputAction_MovetoHome_HIGH = 22             ///回初始位到位信号，到位时为高
}OutputAction;

typedef enum OutputOptions   ///IO选项卡输出操作管理
{
    OutputOptions_Enable = 0,   ///启用
    OutputOptions_Manual = 1,   ///仅手动
    OutputOptions_Disable = 2   ///禁用
}OutputOptions;

typedef enum OutputRule   //输出信号状态（自动复位）
{
    OutputRule_Disable = 0,         ///禁用
    OutputRule_Enable = 1           ///启用
}OutputRule;

typedef struct IOInputConfiguration
{
    int inputFilteringTime;    ///输入信号滤波时间,单位:ms
    char inputName[NAMELENTH];          ///输入信号名称
    InputAction inputActions;  	///输入信号操作
}IOInputConfiguration;

typedef struct IOOutputConfiguration
{
    char outputName[NAMELENTH];		///输出信号名称
    OutputAction outputActions;		///输出信号条件
    OutputOptions outputOptions;	///输出信号操作控制选项
    OutputRule outputRule;          ///输出信号状态（自动复位）
}IOOutputConfiguration;

typedef enum StaticCollisionProtectionMode
{
    StaticCollisionProtection_Off = 0,                    ///关闭静止时碰撞保护和快速切回
    StaticCollisionProtection_Enable = 1,                 ///仅开启静止时碰撞保护
    StaticCollisionProtection_EnableWithFastReturn = 2    ///开启静止时碰撞保护和快速切回
}StaticCollisionProtectionMode;

typedef enum  ExjLineDirection  ///扩展轴直线运动方向
{
    X_PLUS = 0,  /// X+
    X_MINUS = 1, /// X-
    Y_PLUS = 2,  /// Y+
    Y_MINUS = 3, /// Y-
    Z_PLUS = 4,  /// Z+
    Z_MINUS = 5  /// Z-
}ExjLineDirection;

typedef enum ToolIoType
{
    IO_OUTPUT = 0,        ///输出
    IO_INPUT = 1          ///输入
}ToolIoType;

typedef enum AnalogType
{
    Current = 0,         ///电流,对应值的单位:mA
    Voltage = 1          ///电压,对应值的单位:V
}AnalogType;

typedef struct AnalogInputConfiguration
{
    char analogInputName[NAMELENTH];	///模拟量输入通道名称
    AnalogType analogInputType;	///模拟量输入类型
}AnalogInputConfiguration;

typedef struct AnalogOutputConfiguration
{
    char analogOutputName[NAMELENTH];	 ///模拟量输出通道名称
    AnalogType analogOutputType; ///模拟量输出类型
}AnalogOutputConfiguration;

typedef enum ToolPower
{
    Power_invaild = 0, ///无效值
    Power_on = 1,      ///24V
    Power_off = 2,     ///0V
}ToolPower;

typedef struct EncoderConfiguration
{
    int eqepType;       ///< 类型
    int eqepCountMode;      ///< 模式
    int strobeInputMode;    ///< 探针
    int eqepEnabled;        ///< 控制
}EncoderConfiguration;

typedef struct BitRegisterConfiguration
{
    char GeneralPurposeBOOLeanRegisterName[NAMELENTH];    	///BOOL型寄存器名称
    InputAction GeneralPurposeBOOLeanRegisterInputActions;	 ///BOOL型寄存器置位操作
    OutputAction GeneralPurposeBOOLeanRegisterOutputActions;///BOOL型寄存器置位的条件
    OutputRule GeneralPurposeBOOLeanRegisterOutputRule; ///BOOL型寄存器自复位
}BitRegisterConfiguration;

typedef struct RegisterConfiguration
{
    char GeneralPurposeRegisterName[NAMELENTH];    	///寄存器名称
}RegisterConfiguration;

typedef struct PoseMessage
{
    double toolPosition[ROB_AXIS_NUM];   ///位姿:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double toolAxisAngle[ROB_AXIS_NUM];  ///关节角度:关节1~6（单位：°）
}PoseMessage;

typedef struct PointCSNode
{
    BOOL isValid;            ///是否有效
    int id;                  ///id号
    char name[NAMELENTH];    ///点坐标系名字
    PoseMessage point;       ///点坐标系数据
}PointCSNode;  ///点坐标系

typedef struct LineCSNode
{
    BOOL isValid;                              ///是否有效
    int id;                                    ///id号
    char name[NAMELENTH];                      ///线坐标系名字
    PointCSNode firstPoint;                    ///第一个点的数据相关信息
    PointCSNode secondPoint;                   ///第二个点的数据相关信息
    double coordinatePose[ROB_AXIS_NUM];       ///线坐标系位姿数据:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double coordinateJointPos[ROB_AXIS_NUM];   ///线坐标系关节数据:关节1~6（单位：°）
}LineCSNode; ///线坐标系

typedef struct PlaneCSNode
{
    BOOL isValid;                              ///是否有效
    int id;                                    ///id号
    char name[NAMELENTH];                      ///面坐标系名字
    PointCSNode firstPoint;                    ///第一个点的数据相关信息
    PointCSNode secondPoint;                   ///第二个点的数据相关信息
    PointCSNode thirdPoint;                    ///第三个点的数据相关信息
    double coordinatePose[ROB_AXIS_NUM];       ///面坐标系位姿数据:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double coordinateJointPos[ROB_AXIS_NUM];   ///面坐标系关节数据:关节1~6（单位：°）
}PlaneCSNode;  /// 面坐标系

typedef struct WCSConfig
{
    char name[NAMELENTH];           ///< 工件坐标系名字
    int id;                         ///< 工件坐标系id号
    int baselineID;                 ///< 基准id,-1为世界坐标系,非负为机械单元的id
    double root2Ref[ROB_AXIS_NUM];  ///< 工件坐标系相对于基准的位姿
} WCSConfig;  ///< 工件坐标系

typedef enum SafetyLimitsValuesType
{
    Userdefined = 0,        ///自定义
    LimitLevel_1 = 1,       ///等级1（限制最多）
    LimitLevel_2 = 2,       ///等级2（限制偏多）
    LimitLevel_3 = 3,       ///等级3（限制偏少）
    LimitLevel_4 = 4        ///等级4（限制最少）
}SafetyLimitsValuesType; ///<安全限制等级

typedef enum SafetyCollisionHandleMode
{
    Collision_EnterReboundMode = 0,     ///<进入反弹模式
    Collision_ProgramPause = 1,         ///<暂停
    Collision_ProgramStop = 2           ///<停止
}SafetyCollisionHandleMode;///<碰撞后处理模式

typedef enum LimitsBoundaryPlaneMode
{
    Disabled = 0,  	///禁用
    Normal = 1,	   	///正常
    Reduced = 2,	///缩减
    Both = 3,		///二者都是
    Trigger = 4  	///触发器缩减模式
}LimitsBoundaryPlaneMode;

typedef enum SafetyLimitsMode
{
    invaild = 0,     	///禁用
    mode_Normal = 1,	///正常
    mode_Reduced = 2	///缩减
}SafetyLimitsMode;

typedef struct SafetyLimitsValues
{
    double maxTcpSpeed;		    ///tcp最高速度,单位:mm/s
    double maxForce;			///tcp最大力,单位:N
    double maxElbowSpeed;		///肘部最高速度,单位:mm/s
    double maxElbowForce;		///肘部最大力,单位:N
    double maxStoppingDistance; ///最大停止距离,单位:mm
    double maxStoppingTime;  	///最大停止时间,单位:ms
    double maxPower;			///最高功率,单位:W
    double maxMomentum;		    ///最大动量,单位:kg m/s
    double maxTcpAcc;	    	///tcp最高加速度 预留
    double maxTcpJerk;	    	///tcp最高加加速度 预留
}SafetyLimitsValues;

typedef struct SafetyLimitsJointAngle
{
    double maxJointSpeed;		///最大角速度,单位:°/s
    double minJointPosition;	///最小位置,单位:°
    double maxJointPosition;	///最大位置,单位:°
}SafetyLimitsJointAngle;

typedef struct SafetyLimitsBoundaryPlane
{
    char name[NAMELENTH];                  ///安全平面名字
    int id; 			                   ///id号
    double displacement;                   ///z方向平移,单位:mm
    enum LimitsBoundaryPlaneMode mode;     ///限制模式
    double planeNormal;                    ///预留值 设为0
    double distanceToOrigin;               ///预留值 设为0
    BOOL elbowRestricted;                  ///是否限制第三关节
    CoordinateType sourceGeomFeatureType;  ///关联的坐标系类型
    int sourceGeomFeatureId;               ///关联的坐标系ID
}SafetyLimitsBoundaryPlane;  ///安全平面信息

typedef enum SafetyIOInput
{
    input_invaild = 0,    ///未分配
    emergencyStop = 1,    ///紧急停机
    reducedMode = 2,      ///缩减模式
    safeguardStop = 3,    ///安全停止
    safeguardReset = 6    ///安全重置
}SafetyIOInput;

typedef enum SafetyIOOutput
{
    output_invaild = 0,      ///未分配
    systemEmergencyStop =1,  ///紧急停止输出
    robotMoving =2,          ///机器人运动
    robotNotStopping =3      ///机器人未停止
}SafetyIOOutput;

typedef struct IPConfig
{
    int ipAddr[4];         ///IP地址
    int subnetMask[4];     ///子网掩码
}IPConfig;
typedef enum ModbusMasterFunctionNo
{
    ReadBool = 0,               /// (01)读位寄存器
    WriteBool = 1,              /// (15)写位寄存器
    ReadInt16 = 2,              /// (03)读16位整数寄存器
    WriteInt16 = 3,             /// (16)写16位整数寄存器
    ReadInt32 = 4,              /// (03)读32位整数寄存器
    WriteInt32 = 5,             /// (16)写32位整数寄存器
    ReadFloat = 6,              /// (03)读浮点寄存器
    WriteFloat = 7,             /// (16)写浮点寄存器
    ReadDiscreteBool = 8,       /// (02)读位寄存器
    ReadDiscreteInt16 = 9,      /// (04)读16位整数寄存器
    ReadDiscreteInt32 = 10,     /// (04)读32位整数寄存器
    ReadDiscreteFloat = 11,     /// (04)读浮点寄存器
    WriteSingleBool = 12,       /// (05)写位寄存器
    WriteSingleInt16 = 13      /// (06)写16位整数寄存器
}ModbusMasterFunctionNo;  ///modbusMaster函数功能码


typedef struct ModbusAddrMap
{
    ModbusMasterFunctionNo functionNo;     ///modbusMaster函数功能码
    int slaveStartAddr;                    ///从站寄存器首地址
    int masterOffsetNo;                    ///主站寄存器个数
    int masterStartAddr;                   ///主站寄存器索引
    CommuVarType masterRegType;            ///寄存器类型
}ModbusAddrMap;

typedef enum ModbusMasterOperate
{
    Start = 0,   ///开始连接
    Stop = 1     ///断开连接
}ModbusMasterOperate;  ///modbusMaster操作功能码

typedef struct ModbusMasterConfig
{
    int id;                                        ///从站id号
    int slaveNo; 		                           ///从站站号
    int slaveIP[4];		                           ///modbus-tcp时，从站ip
    int scanTime; 		                           ///扫描周期,单位:ms
    ModbusMasterOperate modbusMasterOperate;       ///modbusMaster操作功能码
    int autoConnect;	                           ///自动连接,1-自动连接 0-非自动连接
}ModbusMasterConfig; ///modbusMaster配置

typedef struct CustomAddrMap
{
    CommuVarType requestRegType;    ///< 请求寄存器类型
    int requestStartAddr;           ///< 请求寄存器首地址
    int requestOffsetNo;            ///< 请求寄存器个数
    int responseType;               ///< 应答寄存器类型  0-无应答 1-bool类型  2-16bit整数类型 3-32bit整数类型 4-32bit浮点类型
    int responseStartAddr;
}CustomAddrMap;

typedef struct CustomMasterConfig
{
    int scanTime;                       ///< 扫描周期
    ModbusMasterOperate modbusMasterOperate;    ///< 操作功能码
    int autoConnect;                    ///< 自动连接,1-自动连接 0-非自动连接
}CustomMasterConfig;

typedef struct PopUpMsg
{
    int popupType;				///弹窗类型，0-输出弹窗，1-输入弹窗
    char var_data[1024]; 		///弹窗显示的字符串，字符串类型
    char var_title[128];		///弹窗标题，字符串类型
    BOOL iswarning;				///是否是警告信息，bool类型
    BOOL iserror;				///是否是报警信息，bool类型
    BOOL isblocking;			///是否暂停程序运行，bool类型
    enum VariableType inVarType; 	///输入弹窗时，变量类型
    double inVarMinValue; 		///输入弹窗时，变量数值范围最小值
    double inVarMaxValue; 		///输入弹窗时，变量数值范围最大值
}PopUpMsg;
typedef struct PathPoint
{
    double pose[ROB_AXIS_NUM];                  ///<坐标点向量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double jointpos[ROB_AXIS_NUM];              ///<关节角度：关节1~6（单位：°）
    double exjData[ROB_AXIS_NUM];               ///<扩展轴数据；扩展轴1~6，只在透传时有效
}PathPoint;

typedef struct PathPara  ///<轨迹参数
{
    int index;          ///<轨迹索引 上传时：-1~49，下载时：0~49
    int moveType;       ///<运动轨迹类型，0-透传，1-样条，3-movex, 4-关节再现
    double speed; 		///<速度  moveType为3时有效
    double acc; 		///<加速度 moveType为3时有效
    double blendRadius; ///<过渡半径 moveType为3时有效
}PathPara;

typedef struct PathData     ///<轨迹数据
{
    PathPoint *pathPoints;  ///<轨迹序列数据
    int pathPointsNum;      ///<轨迹数量
    int moveTime;           ///<运动时间  moveType为1时有效,2~10000,单位:ms
    int exjNum;             ///<实际扩展轴数量，只有透传时有效，非透传时需要设置为0
}PathData;

typedef struct PathDownloadData  ///<下载轨迹数据
{
    PathData pathData;     ///< 轨迹数据
    PathPara pathPara;     ///<轨迹参数
}PathDownloadData;

typedef struct RecordPathPara  ///< 轨迹记录参数
{
    int sampleTime;     ///<采样时间,recordControl为1时有效,单位:ms
    int recordControl;  ///<记录控制，0-停止，1-启动，2-暂停
}RecordPathPara;

typedef struct PathRecordStatus  ///<轨迹记录状态
{
    int recordStatus;    ///<0-记录完成，1-记录中，2-暂停记录
    int waypointNumber;  ///<已记录点数
}PathRecordStatus;

typedef struct PathRunMsg  ///<轨迹运行状态
{
    int pathrunstatus;  ///< 0或10001-轨迹控制停止 1-轨迹运行中
    int pointIndex;    ///<  当前运行轨迹索引
}PathRunMsg;

typedef enum Ect_DataType  ///<参考ETG1020
{
    Ect_BOOL = 1,    /// bool
    Ect_SINT = 2,    /// sint
    Ect_INT = 3,     /// int
    Ect_DINT = 4,    /// dint
    Ect_USINT = 5,   /// usint
    Ect_UINT = 6,    /// uint
    Ect_UDINT = 7,   /// unint
    Ect_REAL = 8,    /// real
    Ect_LREAL = 17,  /// lreal
    Ect_LINT = 21,   /// lint
    Ect_ULINT = 27,  /// ulint
    Ect_BYTE = 30,   /// byte
    Ect_WORD = 31,   /// word
    Ect_DWORD = 32   /// dword
}Ect_DataType;

///pdo类型
typedef enum Ect_PdoType
{
    Ect_TxPdo = 1,  /// TxPdo
    Ect_RxPdo = 2   /// RxPdo
}Ect_PdoType;

typedef enum EscState
{
    ECT_STATE_INIT = 1,     /// Init
    ECT_STATE_PREOP = 2,    /// Pre-OP
    ECT_STATE_BOOT = 3,     /// Bootstrap
    ECT_STATE_SAFEOP = 4,   /// Safe-OP
    ECT_STATE_OP = 8        /// OP
}EscState;

typedef struct PdoTypeData
{
    int offAddress;  ///映射地址
    unsigned char pdoData[8]; ///读出的数据
    int pdoDataLen;   ///读出的数据字节长度
    Ect_DataType pdoDataType; ///数据类型
    Ect_PdoType pdoType; ///pdo类型
}PdoTypeData;

///ethercat从站信息
typedef struct EtherCATSlaveData
{
    int index;
    int venderID;
    int productCode;
    int revisionNo;
}EtherCATSlaveData;

///
typedef struct ForceSetting  ///力控设置
{
    int forceBaseType;		///力控类型:  0-基于TCP力传感器,1-基于动力学,2-基于底座力传感器
    int forceCtrlType;	    ///力控用法:  0-运动力控,1-力控拖动
    int flexibleAxis[6];    ///力控轴
}ForceSetting;

typedef struct ForceTCPMsg ///力控传感器tcp
{
    char forceTcpName[20];   ///力控传感器tcp的名称
    int forceTcpId;         ///力控传感器tcp的ID
    double forceTcpOffset[ROB_AXIS_NUM];             ///力控传感器tcp偏移量
}ForceTCPMsg;

typedef struct ForcePayLoad  ///力控传感器负载
{
    char forcePayloadName[20];                 ///力控传感器负载名称
    int forcePayloadId;                        ///力控传感器负载ID
    double forceToolPayload;                   ///力控传感器负载重量
    double forceCenterOfGravity[3];            ///负载中心（CX，CY，CZ）
}ForcePayLoad;
typedef struct SensorMessage ///力控传感器信息
{
     int venderNo;             ///力传感器厂家号：0-CGX,1-蓝点,2-鑫精诚
     int productNo;            ///力传感器产品号
     int sequenceNo;           ///力传感器通讯拓扑顺序号
}SensorMessage;
typedef struct CommuConfig  ///通讯配置
{
    int  autoConnect;	     					///自动连接  不处理-0 连接-1 关闭-2
    int  mannaulOperate;	     			    ///手动连接  不处理-0 连接-1 关闭-2
    int  paraBuf;                               ///预留参数
}CommuConfig;
typedef struct  ForceToolSetting    ///力控传感器工具设置
{
    char activeForceTCP_name[20];				///激活的力控tcp名称
    char activeForcePayload_name[20];	      ///激活的力控负载名称
    ForceTCPMsg availableForceTCP[20];		///可以使用的力控TCP
    int availableForceTCPLen;                ///可以使用的力控TCP数量
    ForcePayLoad availableForcePayLoad[20];   ///可以使用的力控负载
    int availableForcePayLoadLen;  ///可以使用的力控负载数量
}ForceToolSetting;

typedef struct ForceConfig  ///力控配置
{
    SensorMessage sensorMessage;  ///传感器信息
    ForceToolSetting forceCtrlTcpImpl; ///力控工具设置
    CommuConfig commuConfig;  ///力控通讯配置
}ForceConfig;

typedef struct ForceCtlPara   ///力控控制参数
{
    double taskFrame[6];                             ///力控坐标系
    double wrench[6];                                ///力给定

    double limits[6];                               ///力控保护限制
    int limitsLen;    ///力控保护限制长度0或6

    double mass[6];                                  ///力控虚拟质量
    int massLen; ///力控虚拟质量长度 0、1或6

    double damping[6];                               ///<力控虚拟阻尼
    int dampingLen; ///力控虚拟阻尼长度 0、1或6

    double stiffness[6];                             ///力控虚拟刚度
    int stiffnessLen; ///力控虚拟刚度长度 0、1或6
}ForceCtlPara;

typedef struct ForceData   ///力控数据
{
    double ftRaw[6];                                 ///力传感器原始值
    double ftCtrl[6];                                ///力传感器控制值
    ControlModes controlMode;                        ///力控模式
}ForceData;

typedef struct RobotStateData
{
    double tcpActualPose[ROB_AXIS_NUM]; //实际tcp位置
    double jointActualPos[ROB_AXIS_NUM]; //实际关节位置
    RobotModes robotMode; //机械臂当前状态
    BOOL robotMoveStatus; //机械臂运动状态
    unsigned int robotSpeedPercent; //机械臂速度百分比
    int configurableDigitalOutput[CR6_CO_NUM]; //控制柜可配置数字输出
}RobotStateData;

//机械臂和扩展轴数据
typedef struct RobotExjData
{
    double actualExjointPos[EXTERNAL_MAX_NUM];  ///< 扩展轴实际位置
    double actualJointPos[ROB_AXIS_NUM];        ///< 实际关节角度
    double actualTcpVector[ROB_AXIS_NUM];       ///< 实际TCP位姿
}RobotExjData;

typedef struct MoveExjPara
{
    double pose[ROB_EXJ_AXIS_NUM];              ///< 前六个代表关节姿态（x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）），后六个代表扩展轴位置
    double jointPos[ROB_EXJ_AXIS_NUM];          ///< 前六个代表关节1~6关节角度，后六个代表扩展轴位置
    double speed[ROB_EXJ_AXIS_NUM];             ///< 前六个代表机械臂速度，当坐标系类型为关节坐标系时，分别代表一到六关节的速度 °/s，其他时候代表tcp速度 mm/s，第七个代表扩展轴直线速度 mm/s，第八个代表扩展轴旋转速度 °/s
    double acc[ROB_EXJ_AXIS_NUM];               ///< 前六个代表关节加速度，后六个无效
                                                ///< acc可以是tcp加速度或者是关节加速度,当coordinateType设置为jointCoordinate时，该速度为关节加速度，其它情况是tcp加速度
                                                ///< 作为tcp加速度时，单位为：mm/s²; 作为关节加速度时，单位为：°/s²
    double tcpOffset[ROB_AXIS_NUM];             ///< tcp偏移量:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    int tcpId;                                  ///< tcp的ID号
    enum CoordinateType coordinateType;         ///< 坐标系类型
    double coordinatePose[ROB_AXIS_NUM];        ///< 参考坐标系位姿:x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double pointTransRadius;                    ///< 过渡半径
    enum MotiontriggerMode motiontriggerMode;   ///< 运动轨迹触发方式
}MoveExjPara;

//扩展轴点动运动参数
typedef struct MoveJogExjPara
{
    double jointPos[EXTERNAL_MAX_NUM];          ///< 扩展轴位置
    double speed[EXTERNAL_MAX_NUM];             ///< 扩展轴速度
    double acc[EXTERNAL_MAX_NUM];               ///< 扩展轴加速度
}MoveJogExjPara;

//扩展轴运动参数
typedef struct ExjMovePara
{
    int exjCount;                                   ///< 扩展轴数量
    int index[EXTERNAL_MAX_NUM];                    ///< 扩展轴索引
    double speed[EXTERNAL_MAX_NUM];                 ///< 扩展轴速度
    double position[EXTERNAL_MAX_NUM];              ///< 扩展轴位置
    double acctime;                                 ///< 扩展轴加速时间
    enum MotiontriggerMode motiontriggerMode;       ///< 运动轨迹触发方式
}ExjMovePara;

//扩展轴类型
typedef enum ExternalJointType
{
    line_externalJoint = 0,  //直线
    rotation_externalJoint = 1 //旋转
}ExternalJointType;

//扩展轴配置
typedef struct ExjConfig
{
    char name[NAMELENTH];                   ///< 扩展轴名称
    enum ExternalJointType type;            ///< 扩展轴类型
    double ratio;                           ///< 传动比
    double minlimit;                        ///< 最小值
    double maxlimit;                        ///< 最大值
    long long zeroPosition;                 ///< 零位(pulse)
    double exjToRef[ROB_AXIS_NUM];          ///< 扩展轴相对于参考系的关系
    char masterExjName[NAMELENTH];          ///< 主轴名称
    double masterSlaveRate;                 ///< 主从比
}ExjConfig;

typedef struct RefExjConfig
{
    int exjIndex;                           ///< 扩展轴索引
    double exj2Ref[ROB_AXIS_NUM];           ///< 扩展轴相对于上一组的参考
}RefExjConfig;
typedef struct MechanicalUnitConfig
{
    char mechanicalUnitName[NAMELENTH];                  ///< 机械单元名称
    int baselineId;                                      ///< 基准id, -1为世界坐标系,其他为机械单元的id 不存在id为0的基准
    double root2Ref[ROB_AXIS_NUM];                       ///< 机械单元相对于基准的位姿
    int refExjCount;                                     ///< 机械单元中扩展轴数量,最多为6个
    RefExjConfig refExjConfig[REF_EXJ_CONFIG_MAX_NUM];   ///< 机械单元中扩展轴配置
}MechanicalUnitConfig;

/// 机械臂报警时锁存的数据
typedef struct AlarmData
{
    int alarmCode;  ///< 报警代码
    double tcpPosition[ROB_AXIS_NUM];    ///< 报警时位姿数据：x,y,z（单位：mm）;Rx,Ry,Rz（单位：°）
    double jointPosition[ROB_EXJOG_AXIS_NUM]; ///< 报警时关节及10个扩展轴数据：关节1~6（单位：°）, e0,e1...e9（单位：mm或°）
}AlarmData;

enum CoPdoType
{
    CO_TX_PDO = 1,
    CO_RX_PDO = 2
};

enum CoDataType
{
    EDS_DT_BOOL = 1,   //0x0001;
    EDS_DT_S8 = 2,     //0x0002;
    EDS_DT_S16 = 3,    //0x0003;
    EDS_DT_S32 = 4,    //0x0004;
    EDS_DT_U8 = 5,     //0x0005;
    EDS_DT_U16 = 6,    //0x0006;
    EDS_DT_U32 = 7,    //0x0007;
    EDS_DT_FLOAT = 8,  //0x0008;
    EDS_DT_STRING = 9, //0x0009;  一个字符占8位
    EDS_DT_OCTET_STRING = 0xA,   //0x000A; 一个字符占8位
    EDS_DT_UNICODE_STRING = 0xB, //0x000B; 一个字符占16位
    EDS_DT_DOMAIN = 0xF,  //0x000F
    EDS_DT_DOUBLE = 0x11, //0x0011
    EDS_DT_INT64 = 0x15,  //0x0015
    EDS_DT_UINT64 = 0x1B  //0x001B
};

#endif /// BASESTRUCT_H

