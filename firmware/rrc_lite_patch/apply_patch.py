"""Apply the WaroTrans patch to a FW-25 source tree (RosRobotControllerLite_ros_250811 @ 9396c71).

    python3 apply_patch.py <fw25-source-dir>

Exact-match edits (CRLF-safe); fails loudly if the source does not match. warotrans_rrc_lite_fw25.patch is the
same change as a reviewable diff (items 1-7); item 8 (motors initialised at boot) is only applied here.
"""
import os
import sys

B = sys.argv[1]


def edit(path, pairs):
    path = os.path.join(B, path)
    raw = open(path, "rb").read()
    crlf = b"\r\n" in raw
    s = raw.decode("utf-8").replace("\r\n", "\n")
    for old, new in pairs:
        assert s.count(old) == 1, (path, old[:70], s.count(old))
        s = s.replace(old, new)
    if crlf:
        s = s.replace("\n", "\r\n")
    open(path, "wb").write(s.encode("utf-8"))


# 1. packet.h: new function id ULTRASONIC (NONE moves 12 -> 13)
edit("Hiwonder/Misc/packet.h", [(
    "    PACKET_FUNC_RGB,\n    \n    PACKET_FUNC_NONE,",
    "    PACKET_FUNC_RGB,\n    PACKET_FUNC_ULTRASONIC,   /* WAROTRANS PATCH: board -> host, uint16 distance_mm */\n\n    PACKET_FUNC_NONE,")])

# 2. packet_reports.h: report structs
edit("Hiwonder/Portings/packet_reports.h", [(
    "typedef struct  {\n    uint8_t key_id;",
    """/* WAROTRANS PATCH: encoder feedback, MOTOR sub 0x80, board -> host, 50 Hz */
typedef struct {
    uint8_t sub_cmd;      /* 0x80 */
    uint32_t tick_ms;     /* osKernelGetTickCount() */
    int32_t counts[4];    /* cumulative encoder counts, index = motor port M1..M4 */
    float rps[4];         /* firmware-filtered output-shaft rev/s */
} PacketReportMotorFeedbackTypeDef;

/* WAROTRANS PATCH: version reply, SYS sub 0x10 */
typedef struct {
    uint8_t sub_cmd;      /* 0x10 */
    uint8_t major;
    uint8_t minor;
    uint16_t flags;       /* bit0 MCU watchdog, bit1 encoder feedback, bit2 ultrasonic */
    uint32_t build;       /* yyyymmdd */
} PacketReportVersionTypeDef;

typedef struct  {
    uint8_t key_id;""")])

# 3. packet_handle.c
edit("Hiwonder/System/packet_handle.c", [
    ("static uint8_t motor_init_flag = 0;\nvoid motors_init(void);",
     """static uint8_t motor_init_flag = 0;
void motors_init(void);

/* WAROTRANS PATCH: host command watchdog. Updated on every MOTOR packet, checked in app.c. */
volatile uint32_t motor_last_cmd_tick = 0;
uint8_t motors_initialised(void) { return motor_init_flag; }

/* WAROTRANS PATCH item 8: drive the motor outputs from boot (setpoint 0, JGB37 preset with positive PID gains).
 * Stock FW-25 initialised the motors lazily on the first MOTOR packet; until then the RBT-002 rear wheels
 * could run by themselves. */
void motors_init_at_boot(void)
{
    if(motor_init_flag == 0) {
        motor_init_flag = 1;
        motors_init();
    }
    for(int i = 0; i < 4; ++i) {
        set_motor_type(motors[i], MOTOR_TYPE_JGB37);
        motors[i]->pid_controller.set_point = 0.0f;
    }
    motor_last_cmd_tick = osKernelGetTickCount();
}

static void motor_set_checked(uint8_t id, float rps)
{
    if(id >= 4) return;                         /* stock firmware indexed motors[id] without a check */
    if(!(rps == rps)) rps = 0.0f;               /* NaN -> stop */
    encoder_motor_set_speed(motors[id], rps);   /* applies rps_limit (stock serial path bypassed it) */
}"""),
    ("""    if(motor_init_flag == 0)
    {
        motor_init_flag = 1;
        motors_init();
    }
    switch(frame->data_and_checksum[0]) {
        case 0: {
            MotorSingalCtrlCommandTypeDef *mscc = (MotorSingalCtrlCommandTypeDef *)frame->data_and_checksum;
            motors[mscc->motor_id]->pid_controller.set_point = mscc->speed;
            break;
        }
        case 1: {
            MotorMutilCtrlCommandTypeDef *mmcc = NULL;
            mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            for(int i = 0; i < mmcc->motor_num; ++i) {
                motors[mmcc->element[i].motor_id]->pid_controller.set_point = mmcc->element[i].speed;
            }
            break;
        }
        case 2:  {
            MotorSingalStopCommandTypeDef *mssc = (MotorSingalStopCommandTypeDef *)frame->data_and_checksum;
            motors[mssc->motor_id]->pid_controller.set_point = 0;
            break;
        }""",
     """    if(motor_init_flag == 0)
    {
        motor_init_flag = 1;
        motors_init();
    }
    motor_last_cmd_tick = osKernelGetTickCount();
    switch(frame->data_and_checksum[0]) {
        case 0: {
            MotorSingalCtrlCommandTypeDef *mscc = (MotorSingalCtrlCommandTypeDef *)frame->data_and_checksum;
            if(frame->data_length != sizeof(MotorSingalCtrlCommandTypeDef)) break;
            motor_set_checked(mscc->motor_id, mscc->speed);
            break;
        }
        case 1: {
            MotorMutilCtrlCommandTypeDef *mmcc = NULL;
            mmcc = (MotorMutilCtrlCommandTypeDef *)frame->data_and_checksum;
            if(mmcc->motor_num > 4 || frame->data_length != 2 + 5 * mmcc->motor_num) break;
            for(int i = 0; i < mmcc->motor_num; ++i) {
                motor_set_checked(mmcc->element[i].motor_id, mmcc->element[i].speed);
            }
            break;
        }
        case 2:  {
            MotorSingalStopCommandTypeDef *mssc = (MotorSingalStopCommandTypeDef *)frame->data_and_checksum;
            if(mssc->motor_id < 4) motors[mssc->motor_id]->pid_controller.set_point = 0;
            break;
        }"""),
    ("""            for(int i = 0 ; i < 4 ; i++){
                set_motor_type(motors[i], type);
            }
            break;
        }
        default:
            break;
    }
}""",
     """            for(int i = 0 ; i < 4 ; i++){
                set_motor_type(motors[i], type);
            }
            break;
        }
        case 6: { /* WAROTRANS PATCH: custom encoder counts per output rev + rps limit (keeps PID gains) */
            #pragma pack(1)
            typedef struct { uint8_t cmd; float ticks_per_circle; float rps_limit; } MotorCprTypeDef;
            #pragma pack()
            MotorCprTypeDef *c = (MotorCprTypeDef *)frame->data_and_checksum;
            if(frame->data_length != sizeof(MotorCprTypeDef)) break;
            if(c->ticks_per_circle > 100.0f && c->ticks_per_circle < 100000.0f && c->rps_limit > 0.0f && c->rps_limit < 10.0f) {
                for(int i = 0; i < 4; i++) {
                    motors[i]->ticks_per_circle = c->ticks_per_circle;
                    motors[i]->rps_limit = c->rps_limit;
                }
            }
            break;
        }
        default:
            break;
    }
}"""),
    ("""        case 1: {
            change_battery_limit(cmd->limit);
        }break;""",
     """        case 1: {
            change_battery_limit(cmd->limit);
        }break;
        case 0x10: { /* WAROTRANS PATCH: version query */
            PacketReportVersionTypeDef v = { .sub_cmd = 0x10, .major = 1, .minor = 0,
                                             .flags = 0x0007, .build = 20261115 };
            packet_transmit(&packet_controller, PACKET_FUNC_SYS, &v, sizeof(v));
        }break;"""),
])

# 4. app.c: watchdog, encoder feedback 50 Hz, ultrasonic 10 Hz
edit("Hiwonder/System/app.c", [
    ("void button_event_callback(ButtonObjectTypeDef *button,  ButtonEventIDEnum event)",
     """/* ---------------- WAROTRANS PATCH ---------------- */
#define WT_MOTOR_WATCHDOG_MS   250u   /* no MOTOR packet for this long -> all setpoints 0 */
#define WT_FEEDBACK_PERIOD_MS  20u
#define WT_SONAR_PERIOD_MS     100u
#define WT_SONAR_ADDR          0x77u  /* Hiwonder I2C sonar; I2C1 PB6/PB7 assumed = expansion port (VERIFY schematic) */
extern volatile uint32_t motor_last_cmd_tick;
uint8_t motors_initialised(void);
void motors_init_at_boot(void);
extern I2C_HandleTypeDef hi2c1;

static void wt_motor_watchdog(uint32_t now)
{
    if(!motors_initialised()) return;
    if(now - motor_last_cmd_tick > WT_MOTOR_WATCHDOG_MS) {
        for(int i = 0; i < 4; ++i) {
            motors[i]->pid_controller.set_point = 0.0f;
        }
    }
}

static void wt_motor_feedback(uint32_t now)
{
    static uint32_t last = 0;
    if(!motors_initialised() || now - last < WT_FEEDBACK_PERIOD_MS) return;
    last = now;
    PacketReportMotorFeedbackTypeDef r;
    r.sub_cmd = 0x80;
    r.tick_ms = now;
    for(int i = 0; i < 4; ++i) {
        r.counts[i] = (int32_t)motors[i]->counter;
        r.rps[i] = motors[i]->rps;
    }
    packet_transmit(&packet_controller, PACKET_FUNC_MOTOR, &r, sizeof(r));
}

static void wt_sonar(uint32_t now)
{
    static uint32_t last = 0;
    if(now - last < WT_SONAR_PERIOD_MS) return;
    last = now;
    uint8_t buf[2] = {0, 0};
    uint16_t mm = 0;
    /* 3 ms timeout: the IWDG window is ~20 ms and this loop refreshes it every 10 ms */
    if(HAL_I2C_Mem_Read(&hi2c1, WT_SONAR_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT, buf, 2, 3) == HAL_OK) {
        mm = (uint16_t)(buf[0] | (buf[1] << 8));
        if(mm > 5000) mm = 5000;
    }
    packet_transmit(&packet_controller, PACKET_FUNC_ULTRASONIC, &mm, sizeof(mm));
}
/* ---------------- END WAROTRANS PATCH ---------------- */

void button_event_callback(ButtonObjectTypeDef *button,  ButtonEventIDEnum event)"""),
    ("""        HAL_IWDG_Refresh(&hiwdg);
        ATOMIC_SET_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE));
        osDelay(10);""",
     """        HAL_IWDG_Refresh(&hiwdg);
        ATOMIC_SET_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE));
        {   /* WAROTRANS PATCH */
            uint32_t now = osKernelGetTickCount();
            wt_motor_watchdog(now);
            wt_motor_feedback(now);
            wt_sonar(now);
        }
        osDelay(10);"""),
])

# 4b. app.c: call the boot init right after the LEDs
edit("Hiwonder/System/app.c", [(
    "    leds_init();\n",
    "    leds_init();\n    motors_init_at_boot();  /* WAROTRANS PATCH item 8 */\n")])

# 5. motor_porting.c: start the M4 encoder timer
edit("Hiwonder/Portings/motor_porting.c", [(
    "    __HAL_TIM_ENABLE(&htim3);\n    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);",
    "    __HAL_TIM_ENABLE(&htim3);\n    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* WAROTRANS PATCH: was htim4 (M3's timer) */")])
print("ok")
