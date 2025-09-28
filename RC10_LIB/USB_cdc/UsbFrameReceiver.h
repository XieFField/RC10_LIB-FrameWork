#ifndef USB_FRAME_RECEIVER_H
#define USB_FRAME_RECEIVER_H

#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
// C++ 专属代码
extern "C" {
#endif

// 前向声明
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[];

// C 接口函数声明
extern void USB_DataReceivedCallback(uint8_t* buf, uint32_t len);

#ifdef __cplusplus
} // extern "C"

// C++ 专属代码开始
#ifndef USE_STL
    #define MAX_FRAME_DATA_SIZE 16
    typedef void (*FrameCallbackFunc)(uint8_t id, const uint8_t* data, uint8_t length);
    typedef void (*ErrorCallbackFunc)(const char* error);
#else
    #include <functional>
    #include <vector>
#endif

/**
 * @brief USB帧接收器类，用于解析特定格式的数据帧
 * 帧格式: 帧头(0x21 0x40) + ID(1字节) + 数据长度(1字节) + 数据内容(最大16字节) + 帧尾(0x26 0x2A)
 */
class UsbFrameReceiver {
public:
    // 根据编译器支持选择回调类型
#ifdef USE_STL
    using FrameCallback = std::function<void(uint8_t id, const std::vector<uint8_t>& data)>;
    using ErrorCallback = std::function<void(const char* error)>;
#else
    using FrameCallback = FrameCallbackFunc;
    using ErrorCallback = ErrorCallbackFunc;
#endif

    /**
     * @brief 构造函数
     * @param usbHandle USB设备句柄
     * @param frameCallback 帧数据接收回调
     * @param errorCallback 错误回调(可选)
     */
    UsbFrameReceiver(USBD_HandleTypeDef* usbHandle, 
                    FrameCallback frameCallback, 
                    ErrorCallback errorCallback = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~UsbFrameReceiver();

    /**
     * @brief 初始化USB帧接收器
     * @return true-成功, false-失败
     */
    bool init();

    /**
     * @brief 反初始化
     */
    void deinit();

    /**
     * @brief 发送数据帧
     * @param id 帧ID
     * @param data 数据内容
     * @param length 数据长度
     * @return true-成功, false-失败
     */
    bool sendFrame(uint8_t id, const uint8_t* data, uint8_t length);

#ifdef USE_STL
    /**
     * @brief 发送数据帧(STL版本)
     * @param id 帧ID
     * @param data 数据内容
     * @return true-成功, false-失败
     */
    bool sendFrame(uint8_t id, const std::vector<uint8_t>& data);
#endif

    /**
     * @brief 处理接收到的原始数据（供外部调用）
     * @param data 接收到的数据
     * @param length 数据长度
     */
    void processData(const uint8_t* data, uint32_t length);

    /**
     * @brief 获取接收统计信息
     * @return 包含接收统计信息的结构体
     */
    struct Statistics {
        uint32_t totalFramesReceived;
        uint32_t totalBytesReceived;
        uint32_t errorCount;
        uint32_t lastErrorCode;
    };
    
    Statistics getStatistics() const;

    /**
     * @brief 设置全局实例（用于C回调函数）
     * @param instance USB帧接收器实例指针
     */
    static void setInstance(UsbFrameReceiver* instance);

    /**
     * @brief USB数据接收回调（供C代码调用）
     * @param buf 接收缓冲区
     * @param len 数据长度
     */
    static void usbDataReceivedCallback(uint8_t* buf, uint32_t len);

private:
    // 帧解析状态
    enum class ParseState {
        WAIT_FOR_HEADER1,
        WAIT_FOR_HEADER2,
        WAIT_FOR_ID,
        WAIT_FOR_LENGTH,
        WAIT_FOR_DATA,
        WAIT_FOR_FOOTER1,
        WAIT_FOR_FOOTER2
    };

    // 帧头帧尾定义
    static const uint8_t FRAME_HEADER1 = 0x21;
    static const uint8_t FRAME_HEADER2 = 0x40;
    static const uint8_t FRAME_FOOTER1 = 0x26;
    static const uint8_t FRAME_FOOTER2 = 0x2A;
    static const uint8_t MAX_DATA_SIZE = 16;

    // 成员变量
    USBD_HandleTypeDef* usbHandle_;
    FrameCallback frameCallback_;
    ErrorCallback errorCallback_;
    ParseState currentState_;
    uint8_t receiveBuffer_[MAX_DATA_SIZE];
    uint8_t currentId_;
    uint8_t expectedLength_;
    uint8_t dataIndex_;
    uint8_t dataReceived_;
    
    // 统计信息
    Statistics stats_;

    // 静态实例指针
    static UsbFrameReceiver* instance_;

    /**
     * @brief 解析单个字节
     * @param byte 要解析的字节
     */
    void parseByte(uint8_t byte);

    /**
     * @brief 重置解析状态
     */
    void resetParser();

    /**
     * @brief 报告错误
     * @param errorMsg 错误信息
     * @param errorCode 错误代码
     */
    void reportError(const char* errorMsg, uint32_t errorCode = 0);

    /**
     * @brief 调用帧回调函数
     * @param id 帧ID
     * @param data 数据指针
     * @param length 数据长度
     */
    void invokeFrameCallback(uint8_t id, const uint8_t* data, uint8_t length);
};

#endif // __cplusplus

#endif // USB_FRAME_RECEIVER_H