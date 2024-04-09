#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "key.h"

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotor_Instance *friction_l, *friction_r, *loader, *friction_limit; // 拨盘电机
KEY_Instance *loader_key;                                                    // 拨盘电机限位开关

// dwt定时,计算冷却用
static float hibernate_time = 0,
             dead_time      = 0;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp                = 1, // 20
                .Ki                = 0, // 1
                .Kd                = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve           = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit     = 10000,
                .MaxOut            = 15000,
            },
            .current_PID = {
                .Kp            = 1.5, // 0.7
                .Ki            = 0,   // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l                            = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id                             = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r                                                        = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id      = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp     = 10, // 10
                .Ki     = 0,
                .Kd     = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp            = 10, // 10
                .Ki            = 1,  // 1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut        = 5000,
            },
            .current_PID = {
                .Kp            = 0.7, // 0.7
                .Ki            = 0,   // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut        = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type    = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type    = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    loader_config.can_init_config.tx_id                             = 4; // 摩擦轮限位电机
    loader_config.controller_setting_init_config.outer_loop_type    = SPEED_LOOP;
    loader_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_limit                                                  = DJIMotorInit(&loader_config);

    KEY_Config_s key_config = {
        .gpio_config = {
            .GPIOx     = GPIOA,
            .GPIO_Pin  = GPIO_PIN_0,
            .exti_mode = GPIO_EXTI_MODE_RISING_FALLING,
        },
    };
    loader_key = KEYRegister(&key_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    // if (shoot_cmd_recv.shoot_mode == SHOOT_OFF) {
    //     DJIMotorStop(friction_l);
    //     DJIMotorStop(friction_r);
    //     DJIMotorStop(loader);
    //     DJIMotorStop(friction_limit);
    // } else // 恢复运行
    // {
    //     DJIMotorEnable(friction_l);
    //     DJIMotorEnable(friction_r);
    //     DJIMotorEnable(loader);
    //     DJIMotorEnable(friction_limit);
    // }

    // 微动开关控制限位电机,当微动开关触发时,限位电机启动,否则关闭
    if (!loader_key->state) {
        DJIMotorSetRef(friction_limit, 1000);
    } else {
        // DJIMotorSetRef(friction_limit, 0);
        DJIMotorOuterLoop(friction_limit, ANGLE_LOOP);
        DJIMotorSetRef(friction_limit, -1000);
    }
    DJIMotorSetRef(friction_l, 30000);
    DJIMotorSetRef(friction_r, 30000);
}