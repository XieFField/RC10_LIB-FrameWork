#include "Module_lora.h"
#include "usart.h"
#include "main.h"
#include <cmath>
namespace {
constexpr uint32_t LORA_LINK_TIMEOUT_MS = 200U;

static inline float NormalizeJoystick(uint16_t raw, float center, float span, float deadzone = 0.05f, bool invert = false)
{
    float value = (static_cast<float>(raw) - center) / span;
    if (invert) {
        value = -value;
    }
    if (std::fabs(value) < deadzone) {
        value = 0.0f;
    }
    return value;
}

// 三位拨杆解码（SWA/SWF）：raw编码 00=中,01=下,10=上 → 映射 0=上,1=中,2=下
static inline uint8_t DecodeSwitch3Pos(uint16_t raw, uint8_t shift)
{
    uint8_t value = static_cast<uint8_t>((raw >> shift) & 0x03U);
    // 映射表：raw[0]=00→1(中), raw[1]=01→2(下), raw[2]=10→0(上)
    // static const uint8_t map[] = {1, 2, 0};
    static const uint8_t map[] = {1, 0, 2};  //修改逻辑，这个才匹配最上为0，最下为2的需求
    return map[value > 2 ? 0 : value];
}

// 二档拨杆解码（SWB/SWC/SWD/SWE）：raw编码 0=下,1=上 → 映射 0=上,1=下（取反）
static inline uint8_t DecodeSwitch2Pos(uint16_t raw, uint8_t shift)
{
    return static_cast<uint8_t>(1U - ((raw >> shift) & 0x01U));
}

static inline uint16_t PackSigned16(float value, float scale)
{
    int32_t scaled = static_cast<int32_t>(std::round(value * scale));
    if (scaled > INT16_MAX) {
        scaled = INT16_MAX;
    } else if (scaled < INT16_MIN) {
        scaled = INT16_MIN;
    }
    return static_cast<uint16_t>(static_cast<int16_t>(scaled));
}
}  // namespace (anonymous)



namespace communication {

/* ========== 静态成员定义 
========== */
Lora_communication* Lora_communication::s_instance = nullptr;

/* ========== 单例获取 =======
=== */
Lora_communication* Lora_communication::GetInstance()
{
    static Lora_communication instance(&huart4, &huart6,
                                       Lora_IO1_GPIO_Port, Lora_IO1_Pin,
                                       Lora_IO2_GPIO_Port, Lora_IO2_Pin,
                                       nullptr);
    if (s_instance == nullptr) {
        s_instance = &instance;
    }
    return &instance;
}

/* ========== 构造 / 析构 ========== */
Lora_communication::Lora_communication(UART_HandleTypeDef* tx_huart, UART_HandleTypeDef* rx_huart,
     GPIO_TypeDef* tx_aux_gpio_port, uint16_t tx_aux_gpio_pin,
      GPIO_TypeDef* rx_aux_gpio_port, uint16_t rx_aux_gpio_pin,
       tim::Tim* timer)
    : Communication(tx_huart, rx_huart,
        tx_ring_buffer, tx_dma_buffer, rx_ring_buffer, rx_dma_buffer,
        tx_aux_gpio_port, tx_aux_gpio_pin, rx_aux_gpio_port, rx_aux_gpio_pin),
      GpioExti(tx_aux_gpio_pin),
      bsp_rx(DMA_BUF_SIZE, rx_dma_buffer, rx_huart),
      attached_timer(timer),
      airjoy_data()
{
    lora_tx_huart = tx_huart;
    lora_rx_huart = rx_huart;
    lora_aux_port = tx_aux_gpio_port;
    lora_aux_pin = tx_aux_gpio_pin;
    
    s_instance = this;
    // 初始化 KFS 结构体为 0
    kfs_data = {};
    send_x = 0; send_y = 0; send_z = 0;
    send_gripper_status = 0;
    send_suction_cup_status = 0;
    send_automatic_status = 0;
    send_mode = 0;
    send_chosen_command = 0; send_chosen_command_cnt = 0;
    send_kfs_want_place1 = 0; send_kfs_want_place2 = 0;
    send_spear = 0;
    send_kfs_keepplace = 0;
    timer_tick_count=0;
    last_joystick_rx_tick = HAL_GetTick();
    last_joystick_frame_count = 0;
    link_lost = true;
    airjoy_data.link_lost = 1;
}

Lora_communication::~Lora_communication() {
}

/* ========== 初始化 ========== */
void Lora_communication::Init() {

    bsp_rx.SetCallback(RxCallback);
    bsp_rx.UART_Init();
}


/* ========== 虚函数实现 ========== */
HAL_StatusTypeDef Lora_communication::Comm_TxUseTxDMA(UART_HandleTypeDef* huart, uint8_t* data, uint16_t size) {
#if LORA_TEST_FORCE_TX_DMA_FAIL_ONCE
    static uint8_t force_fail_once = 1U;
    if (force_fail_once) {
        force_fail_once = 0U;
        return HAL_BUSY;
    }
#endif
    if (huart != nullptr && huart == lora_tx_huart) {
        return HAL_UART_Transmit_DMA(huart, data, size);
    }
    return HAL_ERROR;
}

void Lora_communication::Task_Process() {
    if (Comm_Task_Loop()) {
        // SetSendAxisData(Axis_x, Axis_y, Axis_yaw);
        // SetSendWantKFSData(KFS_want1, KFS_want2);
        // SetSendSpearData(spear);

        uint16_t joystick[4];
        // 新的 Communication 接口：分别获取摇杆和按键/设置数据
        GetRecvJoystickData(joystick);
        uint16_t key = GetRecvAllKeyData();

        kfs_data.color = GetColor();

        kfs_data.r1_kfs[0] = GetRecvFKFS1Data(1);
        kfs_data.r1_kfs[1] = GetRecvFKFS1Data(2);
        kfs_data.r1_kfs[2] = GetRecvFKFS1Data(3);

        kfs_data.r2_kfs[0] = GetRecvFKFS2Data(1);
        kfs_data.r2_kfs[1] = GetRecvFKFS2Data(2);
        kfs_data.r2_kfs[2] = GetRecvFKFS2Data(3);
        kfs_data.r2_kfs[3] = GetRecvFKFS2Data(4);

        kfs_data.fake_kfs = GetRecvFKFSfData(1);

        // 读取串口屏转发的命令帧数据
        uint8_t _;
        GetRecvCommandFrameData(send_chosen_command, send_chosen_command_cnt, _);
        airjoy_data.recv_command_command = send_chosen_command;
        airjoy_data.recv_command_cnt       = GetRecvCommandCnt(send_chosen_command);  // 当前单个命令的累计次数
        airjoy_data.recv_command_total_cnt = GetRecvCommandTotalCnt();                // 0~8号命令的总和

        airjoy_data.page = GetPage();

        // 左摇杆：极性反转（4096=左/下, 0=右/上），映射后右=+1, 上=+1
        airjoy_data.left_x  = NormalizeJoystick(joystick[0], 2048.0f, 2048.0f, 0.05f, true);
        airjoy_data.left_y  = NormalizeJoystick(joystick[1], 2048.0f, 2048.0f, 0.05f, true);
        // 右摇杆：极性正常（4096=右/上, 0=左/下），映射后右=+1, 上=+1
        airjoy_data.right_x = NormalizeJoystick(joystick[2], 2048.0f, 2048.0f);
        airjoy_data.right_y = NormalizeJoystick(joystick[3], 2048.0f, 2048.0f);

        airjoy_data.SWA = DecodeSwitch3Pos(
            (GetRecvKeyData(3) ? 2U : 0U) | (GetRecvKeyData(2) ? 1U : 0U), 0);
        airjoy_data.SWB = GetRecvKeyData(7) ? 0U : 1U;
        airjoy_data.SWC = GetRecvKeyData(6) ? 0U : 1U;
        airjoy_data.SWD = GetRecvKeyData(5) ? 0U : 1U;
        airjoy_data.SWE = GetRecvKeyData(4) ? 0U : 1U;
        airjoy_data.SWF = DecodeSwitch3Pos(
            (GetRecvKeyData(1) ? 2U : 0U) | (GetRecvKeyData(0) ? 1U : 0U), 0);

        // 十字键：b11=上, b8=下, b10=左, b9=右
        airjoy_data.d_pad_up    = GetRecvKeyData(11) ? 1U : 0U;
        airjoy_data.d_pad_down  = GetRecvKeyData(8)  ? 1U : 0U;
        airjoy_data.d_pad_left  = GetRecvKeyData(10) ? 1U : 0U;
        airjoy_data.d_pad_right = GetRecvKeyData(9)  ? 1U : 0U;

        // 肩键：LB=左后(b15), LT=左前(b14), RT=右前(b13), RB=右后(b12)
        airjoy_data.LT = GetRecvKeyData(15) ? 1U : 0U;
        airjoy_data.LB = GetRecvKeyData(14) ? 1U : 0U;
        airjoy_data.RB = GetRecvKeyData(13) ? 1U : 0U;
        airjoy_data.RT = GetRecvKeyData(12) ? 1U : 0U;  //这里原本T和B是和现在相反的，但因为和我想要的逻辑反了，所以我就自己改了

        uint16_t key_status = key;
        airjoy_data.key_pressed_count = 0;
        for (uint8_t i = 0; i < 16; ++i) {
            if (key_status & (1U << i)) {
                airjoy_data.key_pressed_count++;
            }
        }

        uint16_t rising_edges = static_cast<uint16_t>(key_status & (~airjoy_data.key_last_status));
        for (uint8_t i = 0; i < 16; ++i) {
            if (rising_edges & (1U << i)) {
                airjoy_data.key_down_count++;
            }
        }
        airjoy_data.key_last_status = key_status;

        // ====== 同步全部数据到 airjoy_data，方便 debug 查看 rc_data ======
        airjoy_data.joystick1 = joystick[0];
        airjoy_data.joystick2 = joystick[1];
        airjoy_data.joystick3 = joystick[2];
        airjoy_data.joystick4 = joystick[3];
        airjoy_data.key  = key;
        airjoy_data.page = airjoy_data.page; // 上面已赋值

        airjoy_data.KFS1_1 = kfs_data.r1_kfs[0];
        airjoy_data.KFS1_2 = kfs_data.r1_kfs[1];
        airjoy_data.KFS1_3 = kfs_data.r1_kfs[2];
        airjoy_data.KFS2_1 = kfs_data.r2_kfs[0];
        airjoy_data.KFS2_2 = kfs_data.r2_kfs[1];
        airjoy_data.KFS2_3 = kfs_data.r2_kfs[2];
        airjoy_data.KFS2_4 = kfs_data.r2_kfs[3];
        airjoy_data.KFSf_1 = kfs_data.fake_kfs;
        airjoy_data.color  = kfs_data.color;
    }

    uint32_t joystick_frame_count = GetRecvJoystickFrameCount();
    if (joystick_frame_count != last_joystick_frame_count) {
        last_joystick_frame_count = joystick_frame_count;
        last_joystick_rx_tick = HAL_GetTick();
        link_lost = false;
    } else if ((HAL_GetTick() - last_joystick_rx_tick) > LORA_LINK_TIMEOUT_MS) {
        link_lost = true;
        airjoy_data.left_x = 0.0f;
        airjoy_data.left_y = 0.0f;
        airjoy_data.right_x = 0.0f;
        airjoy_data.right_y = 0.0f;
//        airjoy_data.key = 0;
//        airjoy_data.key_pressed_count = 0;
//        airjoy_data.SWA = 1;
//        airjoy_data.SWB = 1;
//        airjoy_data.SWC = 1;
//        airjoy_data.SWD = 1;
//        airjoy_data.SWE = 1;
//        airjoy_data.SWF = 1;
//        airjoy_data.LB = 0;
//        airjoy_data.RB = 0;
//        airjoy_data.LT = 0;
//        airjoy_data.RT = 0;
//        airjoy_data.d_pad_up = 0;
//        airjoy_data.d_pad_down = 0;
//        airjoy_data.d_pad_left = 0;
//        airjoy_data.d_pad_right = 0;
    }
    airjoy_data.link_lost = link_lost ? 1U : 0U;

    // 实时快照通信统计至 airjoy_data
    airjoy_data.rx_drop_cnt      = GetRxDropCnt();
    airjoy_data.tx_drop_cnt      = GetTxDropCnt();
    airjoy_data.tx_error_cnt     = GetTxErrorCnt();
    airjoy_data.rx_crc_error_cnt = GetRxCrcErrorCnt();
}

void Lora_communication::send_robot_pos(float x, float y, float yaw)
{
    send_x = PackSigned16(x, 100.0f);
    send_y = PackSigned16(y, 100.0f);
    send_z = PackSigned16(yaw, 10.0f);
}

void Lora_communication::send_claw_status(bool claw1, bool claw2, bool claw3)
{
    send_gripper_status = static_cast<uint8_t>((claw1 ? 0x01U : 0x00U) |
                                               (claw2 ? 0x02U : 0x00U) |
                                               (claw3 ? 0x04U : 0x00U));
}

void Lora_communication::send_sucker_status(bool sucker)
{
    send_suction_cup_status = static_cast<uint8_t>((sucker ? 0x01U : 0x00U));
}

// void Lora_communication::send_auto_status(bool auto_status)
// {
//     send_automatic_status = auto_status;
// }
    
void Lora_communication::set_robot_KFS_want_place(uint8_t want1, uint8_t want2,uint8_t want3)
{
    send_kfs_want_place1 = want1|(want2<<4);
    send_kfs_want_place2 = want3;
}

void Lora_communication::update_airjoy_data(RC10_AirJoy_Data_S * data)
{
    if(!data) return;

    data->joystick1 = airjoy_data.joystick1;
    data->joystick2 = airjoy_data.joystick2;
    data->joystick3 = airjoy_data.joystick3;
    data->joystick4 = airjoy_data.joystick4;

    data->key  = airjoy_data.key;
    data->page = airjoy_data.page;

    data->left_x = airjoy_data.left_x;
    data->left_y = airjoy_data.left_y;
    data->right_y = airjoy_data.right_x;
    data->right_x = airjoy_data.right_y;

    data->SWA = airjoy_data.SWA;
    data->SWB = airjoy_data.SWB;
    data->SWC = airjoy_data.SWC;
    data->SWD = airjoy_data.SWD;
    data->SWE = airjoy_data.SWE;
    data->SWF = airjoy_data.SWF;

    data->LB = airjoy_data.LB;
    data->RB = airjoy_data.RB;
    data->LT = airjoy_data.LT;
    data->RT = airjoy_data.RT;

    data->d_pad_up = airjoy_data.d_pad_up;
    data->d_pad_down = airjoy_data.d_pad_down;
    data->d_pad_left = airjoy_data.d_pad_left;
    data->d_pad_right = airjoy_data.d_pad_right;

    data->key_pressed_count = airjoy_data.key_pressed_count;
    data->key_down_count    = airjoy_data.key_down_count;
    data->key_last_status   = airjoy_data.key_last_status;

    data->KFS1_1 = airjoy_data.KFS1_1;
    data->KFS1_2 = airjoy_data.KFS1_2;
    data->KFS1_3 = airjoy_data.KFS1_3;
    data->KFS2_1 = airjoy_data.KFS2_1;
    data->KFS2_2 = airjoy_data.KFS2_2;
    data->KFS2_3 = airjoy_data.KFS2_3;
    data->KFS2_4 = airjoy_data.KFS2_4;
    data->KFSf_1 = airjoy_data.KFSf_1;
    data->color  = airjoy_data.color;

    data->recv_command_command = airjoy_data.recv_command_command;
    data->recv_command_cnt       = airjoy_data.recv_command_cnt;
    data->recv_command_total_cnt = airjoy_data.recv_command_total_cnt;
    data->link_lost = airjoy_data.link_lost;

    data->rx_drop_cnt      = airjoy_data.rx_drop_cnt;
    data->tx_drop_cnt      = airjoy_data.tx_drop_cnt;
    data->tx_error_cnt     = airjoy_data.tx_error_cnt;
    data->rx_crc_error_cnt = airjoy_data.rx_crc_error_cnt;

}

/* ========== 定时器中断 ========== */
void Lora_communication::Tim_It_Process() {
    timer_tick_count++;
    if (timer_tick_count >= 15)
    { // 计数达到 1ms 
        timer_tick_count = 0;
        Comm_SendAxisDataToTxBuffer(send_x, send_y, send_z,
        send_gripper_status, send_suction_cup_status, send_automatic_status,
        send_mode, send_chosen_command, send_chosen_command_cnt,
        send_kfs_want_place1, send_kfs_want_place2, send_spear, send_kfs_keepplace);
    }
}

/* ========== GPIO 中断处理 ========== */
void Lora_communication::EXTI_Prosess() {
    Comm_TxBufferToTxDMA(lora_tx_huart);
}

/* ========== 静态回调 ========== */
void Lora_communication::RxCallback(uint8_t* buf, uint16_t len) {
    (void)buf;
    if (s_instance) {
        s_instance->Comm_RxDMAToRxBuffer(s_instance->lora_rx_huart, len);
    }
}
} // namespace communication
