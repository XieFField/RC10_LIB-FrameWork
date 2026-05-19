//#include "rc_link.h"
//#include "RC_communication.h"
//#include "BSP_USB_UART_Driver.h"
//#include "usart.h"
//#include "tim.h"
//#include "stm32h7xx_hal.h"

///* ========================= 引脚定义 ========================= */
//#define TX_AUX_PORT     GPIOA
//#define TX_AUX_PIN      GPIO_PIN_3      // PA3 = 发送流控 (EXTI3)
//#define RX_AUX_PIN      GPIO_PIN_2      // PA2 = 接收流控 (EXTI2)

///* ================= 桥接类 ================= */
//class RC_Link : public communication::Communication {
//public:
//    RC_Link(UART_HandleTypeDef *txhuart, UART_HandleTypeDef *rxhuart,
//            // uint8_t *tx_ring_buf, uint8_t *tx_dma_buf,
//            // uint8_t *rx_ring_buf, uint8_t *rx_dma_buf,
//            GPIO_TypeDef *tx_aux_port, uint16_t tx_aux_pin,
//            GPIO_TypeDef *rx_aux_port, uint16_t rx_aux_pin)
//        : Communication(txhuart, rxhuart, s_tx_ring, s_tx_dma,
//                        s_rx_ring, s_uart4_dma,
//                        tx_aux_port, tx_aux_pin, rx_aux_port, rx_aux_pin),
//          my_txhuart_(txhuart)
//    {
//    }

//        virtual void Comm_TxUseTxDMA(UART_HandleTypeDef *huart, uint8_t* data, uint16_t size) override {
//        HAL_UART_Transmit_DMA(huart, data, size);
//    }

//    // void OnUartTxCplt(UART_HandleTypeDef *huart) {
//    //     // 按 Communication 注释：Comm_TxBufferToTxDMA 应该写在发送流控 GPIO 的外部中断中，
//    //     // 不应在 UART 发送完成回调中直接调用。
//    //     (void)huart;
//    // }

//     void OnTxAuxRising(void) {
//        // HAL_UART_StateTypeDef state = HAL_UART_GetState(my_txhuart_);
//        // if (state == HAL_UART_STATE_BUSY_TX || state == HAL_UART_STATE_BUSY_TX_RX) {
//        //     return;
//        // }
//        Comm_TxBufferToTxDMA(my_txhuart_);
//    }

// bool ProcessRx(uint16_t joy[4], uint16_t *key) {
//        if (!Comm_Task_Loop()) {
//            return false;
//        }
//        GetRecvData(joy, *key);
//        return true;
//    }
//private:
//    UART_HandleTypeDef *my_txhuart_;
//     uint8_t s_tx_ring[256];
//     uint8_t s_tx_dma[64];
//     uint8_t s_rx_ring[256];
//     uint8_t s_uart4_dma[64];
//};

///* ================= 静态实例（必须在回调函数上面！）================= */
//static uint8_t s_tx_ring[256];
//static uint8_t s_tx_dma[64];
//static uint8_t s_rx_ring[256];
//static uint8_t s_uart4_dma[64];

//static RC_Link s_link(          //	构造实例
//    &huart5, &huart4,
//    s_tx_ring, s_tx_dma,
//    s_rx_ring, s_uart4_dma,
//    TX_AUX_PORT, TX_AUX_PIN,
//    TX_AUX_PORT, RX_AUX_PIN
//);

///* BSP 接收实例：管理 UART4 */
//static UART_ s_bsp_rx(64, s_uart4_dma, &huart4);       //构造实例

///* ================= 回调函数（在 s_link 和 s_bsp_rx 定义之后）================= */

///* BSP 接收回调 */
//static void BSP_RxCB(uint8_t* buf, uint16_t len) {
//    // 走 RC_communication 基类接口：把 DMA 接收数据交给通信层处理。
//    // 该接口内部会处理数据缓存和 DMA 重启。
//    s_link.Comm_RxDMAToRxBuffer(&huart4, len);
//}

///* ================= 对外 C 接口 ================= */
//extern "C" void RC_Init(void) {
//    /* 发送流控脚 PA3 初始置高（允许发送） */
//    HAL_GPIO_WritePin(TX_AUX_PORT, TX_AUX_PIN, GPIO_PIN_SET);

//    /* 启动 BSP 接收：自动注册实例、启动 HAL_UARTEx_ReceiveToIdle_DMA */
//    s_bsp_rx.SetCallback(BSP_RxCB);
//    s_bsp_rx.UART_Init();

//    /* 启动 TIM3 中断 */
//    HAL_TIM_Base_Start_IT(&htim3);
//}

//extern "C" bool RC_Process(uint16_t joy[4], uint16_t* key) {
//    return s_link.ProcessRx(joy, key);
//}

//extern "C" void RC_Send(uint16_t x, uint16_t y, uint16_t z,
//                        uint8_t gripper, uint8_t suction, uint8_t auto_mode,
//                        uint8_t mode, uint8_t cmd1, uint8_t cmd2) {
//    s_link.Comm_SendAxisDataToTxBuffer(x, y, z, gripper, suction, auto_mode, mode, cmd1, cmd2);
//}

///* 供外部已有 HAL 回调调用的接口（非 HAL 回调，不会重复定义） */
//extern "C" void RC_OnUartTxCplt(UART_HandleTypeDef *huart) {
//    if (huart == &huart5) {
//        s_link.OnUartTxCplt(huart);
//    }
//}

///* 发送流控 GPIO 上升沿回调：真正触发 Comm_TxBufferToTxDMA 的位置 */
//extern "C" void RC_OnTxAuxRising(void) {
//    s_link.OnTxAuxRising();
//}

//extern "C" void RC_OnTimPeriodElapsed(TIM_HandleTypeDef *htim) {
//    if (htim == &htim3) {
//        s_link.Comm_SendAxisDataToTxBuffer(0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
//    }
//}