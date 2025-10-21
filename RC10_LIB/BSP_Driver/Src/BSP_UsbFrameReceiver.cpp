#include "BSP_UsbFrameReceiver.h"
#include "usbd_cdc_if.h"

// 静态成员初始化
UsbFrameReceiver* UsbFrameReceiver::instance_ = nullptr;

UsbFrameReceiver::UsbFrameReceiver(USBD_HandleTypeDef* usbHandle, 
                                 FrameCallback frameCallback, 
                                 ErrorCallback errorCallback)
    : usbHandle_(usbHandle)
    , frameCallback_(frameCallback)
    , errorCallback_(errorCallback)
    , currentState_(ParseState::WAIT_FOR_HEADER1)
    , currentId_(0)
    , expectedLength_(0)
    , dataIndex_(0)
    , dataReceived_(0) {
    
    // 初始化统计信息
    stats_.totalFramesReceived = 0;
    stats_.totalBytesReceived = 0;
    stats_.errorCount = 0;
    stats_.lastErrorCode = 0;
    
    // 初始化接收缓冲区
    memset(receiveBuffer_, 0, sizeof(receiveBuffer_));
}

UsbFrameReceiver::~UsbFrameReceiver() {
    deinit();
    // 如果这是当前实例，清除实例指针
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

bool UsbFrameReceiver::init() {
    if (usbHandle_ == nullptr) {
        reportError("USB device handle is null", 1);
        return false;
    }
    
    // 设置当前实例
    setInstance(this);
    
    resetParser();
    return true;
}

void UsbFrameReceiver::deinit() {
    resetParser();
}

bool UsbFrameReceiver::sendFrame(uint8_t id, const uint8_t* data, uint8_t length) {
    if (usbHandle_ == nullptr) {
        reportError("USB device not initialized", 2);
        return false;
    }
    
    if (length > MAX_DATA_SIZE) {
        reportError("Data length exceeds maximum limit", 3);
        return false;
    }
    
    if (data == nullptr && length > 0) {
        reportError("Data pointer is null but length > 0", 4);
        return false;
    }
    
    // 构建帧数据 (头2 + ID1 + 长度1 + 数据 + 尾2)
    uint8_t frame[6 + MAX_DATA_SIZE];
    uint8_t frameLength = 0;
    
    // 添加帧头
    frame[frameLength++] = FRAME_HEADER1;
    frame[frameLength++] = FRAME_HEADER2;
    
    // 添加ID
    frame[frameLength++] = id;
    
    // 添加数据长度
    frame[frameLength++] = length;
    
    // 添加数据内容
    if (length > 0 && data != nullptr) {
        memcpy(&frame[frameLength], data, length);
        frameLength += length;
    }
    
    // 添加帧尾
    frame[frameLength++] = FRAME_FOOTER1;
    frame[frameLength++] = FRAME_FOOTER2;
    
    // 通过USB发送
    if (CDC_Transmit_HS(frame, frameLength) == USBD_OK) {
        return true;
    } else {
        reportError("USB transmission failed", 5);
        return false;
    }
}

#ifdef USE_STL
bool UsbFrameReceiver::sendFrame(uint8_t id, const std::vector<uint8_t>& data) {
    return sendFrame(id, data.data(), static_cast<uint8_t>(data.size()));
}
#endif

void UsbFrameReceiver::processData(const uint8_t* data, uint32_t length) {
    stats_.totalBytesReceived += length;
    
    for (uint32_t i = 0; i < length; ++i) {
        parseByte(data[i]);
    }
}

UsbFrameReceiver::Statistics UsbFrameReceiver::getStatistics() const {
    return stats_;
}

void UsbFrameReceiver::setInstance(UsbFrameReceiver* instance) {
    instance_ = instance;
}

void UsbFrameReceiver::usbDataReceivedCallback(uint8_t* buf, uint32_t len) {
    if (instance_ != nullptr) {
        instance_->processData(buf, len);
    }
}
void UsbFrameReceiver::parseByte(uint8_t byte) {
    switch (currentState_) {
        case ParseState::WAIT_FOR_HEADER1:
            if (byte == FRAME_HEADER1) {
                currentState_ = ParseState::WAIT_FOR_HEADER2;
            }
            // 如果不是期待的帧头，保持等待状态
            break;
            
        case ParseState::WAIT_FOR_HEADER2:
            if (byte == FRAME_HEADER2) {
                currentState_ = ParseState::WAIT_FOR_ID;
            } else {
                // 如果第二个字节不是期待的帧头，回到第一个帧头等待状态
                currentState_ = ParseState::WAIT_FOR_HEADER1;
                // 重新检查当前字节，可能是一个新的帧头
                if (byte == FRAME_HEADER1) {
                    currentState_ = ParseState::WAIT_FOR_HEADER2;
                }
            }
            break;
            
        case ParseState::WAIT_FOR_ID:
            currentId_ = byte;
            currentState_ = ParseState::WAIT_FOR_LENGTH;
            break;
            
        case ParseState::WAIT_FOR_LENGTH:
            if (byte <= MAX_DATA_SIZE) {
                expectedLength_ = byte;
                dataIndex_ = 0;
                dataReceived_ = 0;
                if (expectedLength_ > 0) {
                    currentState_ = ParseState::WAIT_FOR_DATA;
                } else {
                    // 如果数据长度为0，直接等待帧尾
                    currentState_ = ParseState::WAIT_FOR_FOOTER1;
                }
            } else {
                reportError("Invalid data length", 6);
                resetParser();
            }
            break;
            
        case ParseState::WAIT_FOR_DATA:
            if (dataIndex_ < MAX_DATA_SIZE) {
                receiveBuffer_[dataIndex_++] = byte;
                dataReceived_++;
                if (dataReceived_ >= expectedLength_) {
                    currentState_ = ParseState::WAIT_FOR_FOOTER1;
                }
            } else {
                reportError("Data buffer overflow", 7);
                resetParser();
            }
            break;
            
        case ParseState::WAIT_FOR_FOOTER1:
            if (byte == FRAME_FOOTER1) {
                currentState_ = ParseState::WAIT_FOR_FOOTER2;
            } else {
                reportError("Frame footer 1 mismatch", 8);
                resetParser();
            }
            break;
            
        case ParseState::WAIT_FOR_FOOTER2:
            if (byte == FRAME_FOOTER2) {
                // 成功接收完整帧
                stats_.totalFramesReceived++;
                invokeFrameCallback(currentId_, receiveBuffer_, expectedLength_);
            } else {
                reportError("Frame footer 2 mismatch", 9);
            }
            resetParser();
            break;
    }
}

void UsbFrameReceiver::resetParser() {
    currentState_ = ParseState::WAIT_FOR_HEADER1;
    memset(receiveBuffer_, 0, sizeof(receiveBuffer_));
    currentId_ = 0;
    expectedLength_ = 0;
    dataIndex_ = 0;
    dataReceived_ = 0;
}

void UsbFrameReceiver::reportError(const char* errorMsg, uint32_t errorCode) {
    stats_.errorCount++;
    stats_.lastErrorCode = errorCode;
    
    if (errorCallback_ != nullptr) {
        errorCallback_(errorMsg);
    }
}

void UsbFrameReceiver::invokeFrameCallback(uint8_t id, const uint8_t* data, uint8_t length) {
    if (frameCallback_ != nullptr) {
#ifdef USE_STL
        // STL版本：创建vector并调用回调
        std::vector<uint8_t> vecData(data, data + length);
        frameCallback_(id, vecData);
#else
        // 非STL版本：直接传递指针和长度
        frameCallback_(id, data, length);
#endif
    }
}

// C接口函数定义
extern "C" void USB_DataReceivedCallback(uint8_t* buf, uint32_t len) {
    UsbFrameReceiver::usbDataReceivedCallback(buf, len);
}