#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <arm_math.h> // 包含 DSP 库，用于数学计算

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// 定义队列的最大容量
#define QUEUE_CAPACITY 3

/**
 * @class Vector2D
 * @brief 表示二维向量的类
 *
 * 提供了向量的基本操作，包括加法、减法、点乘、叉积、
 * 标量乘法、单位化、投影等。
 */
class Vector2D
{
public:
    float32_t x; ///< 向量的 x 分量
    float32_t y; ///< 向量的 y 分量

    /**
     * @brief 默认构造函数
     * 初始化向量的 x 和 y 分量为 0
     */
    Vector2D();

    /**
     * @brief 带参数构造函数
     * @param x_ 向量的 x 分量
     * @param y_ 向量的 y 分量
     */
    Vector2D(float32_t x_, float32_t y_);

    /**
     * @brief 重载赋值运算符
     * @param other 另一个 Vector2D 对象
     * @return Vector2D& 当前对象的引用
     */
    Vector2D &operator=(const Vector2D &other);

    /**
     * @brief 向量加法
     * @param other 另一个 Vector2D 对象
     * @return Vector2D 加法结果
     */
    Vector2D operator+(const Vector2D &other) const;

    /**
     * @brief 向量减法
     * @param other 另一个 Vector2D 对象
     * @return Vector2D 减法结果
     */
    Vector2D operator-(const Vector2D &other) const;

    /**
     * @brief 重载负号运算符
     * @return Vector2D 取反后的向量
     */
    Vector2D operator-() const;

    /**
     * @brief 向量点乘
     * @param other 另一个 Vector2D 对象
     * @return float32_t 点乘结果
     */
    float32_t operator*(const Vector2D &other) const;

    /**
     * @brief 计算向量的叉积
     * @param other 另一个 Vector2D 对象
     * @return float 叉积的标量结果
     */
    float cross(const Vector2D &other) const;

    /**
     * @brief 向量乘以标量
     * @param scalar 标量值
     * @return Vector2D 乘法结果
     */
    Vector2D operator*(float32_t scalar) const;

    /**
     * @brief 标量乘以向量（标量在左侧）
     * @param scalar 标量值
     * @param vec 向量对象
     * @return Vector2D 乘法结果
     */
    friend Vector2D operator*(float32_t scalar, const Vector2D &vec);

    /**
     * @brief 计算向量的模（长度）
     * @return float32_t 向量的模
     */
    float32_t magnitude() const;

    /**
     * @brief 单位化向量
     * @return Vector2D 单位向量
     */
    Vector2D normalize() const;

    /**
     * @brief 向量投影
     * @param other 投影到的向量
     * @return Vector2D 投影结果
     */
    Vector2D project_onto(const Vector2D &other) const;

    /**
     * @brief 计算两点之间的距离平方
     * @param a 第一个点
     * @param b 第二个点
     * @return float 两点之间的距离平方
     */
    static float distanceSquared(const Vector2D &a, const Vector2D &b);

    /**
     * @brief 线性插值
     * @param a 起始向量
     * @param b 结束向量
     * @param t 插值参数，范围 [0, 1]
     * @return Vector2D 插值结果
     */
    static Vector2D lerp(const Vector2D &a, const Vector2D &b, float t);

    /**
     * @brief 通过三个点计算曲率
     * @param p0 第一个点
     * @param p1 第二个点
     * @param p2 第三个点
     * @return float 曲率值
     */
    static float curvatureFromThreePoints(const Vector2D &p0, const Vector2D &p1, const Vector2D &p2);

private:
    /**
     * @brief 辅助函数：检查标量是否接近零
     * @param scalar 标量值
     * @return true 如果标量接近零
     * @return false 如果标量不接近零
     */
    static bool isZero(float scalar)
    {
        return (scalar < 0 ? -scalar : scalar) < 1e-6f;
    }
};

/**
 * @class Vector2DQueue
 * @brief 表示一个固定容量的 Vector2D 队列
 *
 * 提供了队列的基本操作，包括入队、出队、查看队首元素等。
 */
class Vector2DQueue
{
private:
    Vector2D data[QUEUE_CAPACITY]; ///< 用于存储队列元素的数组
    int front;                     ///< 队首索引
    int rear;                      ///< 队尾索引
    int size;                      ///< 当前队列大小

public:
    /**
     * @brief 默认构造函数
     * 初始化队列为空
     */
    Vector2DQueue();

    /**
     * @brief 检查队列是否为空
     * @return true 队列为空
     * @return false 队列不为空
     */
    bool isEmpty() const;

    /**
     * @brief 检查队列是否已满
     * @return true 队列已满
     * @return false 队列未满
     */
    bool isFull() const;

    /**
     * @brief 返回队列中的元素数量
     * @return int 队列中的元素数量
     */
    int queueSize() const;

    /**
     * @brief 入队操作
     * @param vec 要入队的 Vector2D 对象
     * @return true 入队成功
     * @return false 入队失败（队列已满）
     */
    bool enqueue(const Vector2D &vec);

    /**
     * @brief 强制入队操作
     * @param vec 要入队的 Vector2D 对象
     * 如果队列已满，将覆盖队尾元素。
     */
    void forceEnqueue(const Vector2D &vec);

    /**
     * @brief 出队操作
     * @param vec 用于存储出队的 Vector2D 对象
     * @return true 出队成功
     * @return false 出队失败（队列为空）
     */
    bool dequeue(Vector2D &vec);

    /**
     * @brief 查看队首元素
     * @return Vector2D 队首元素
     */
    Vector2D peek() const;

    /**
     * @brief 将一个数组压入队列
     * @param arr 要压入的 Vector2D 数组
     * @param length 数组长度
     * @return true 压入成功
     * @return false 压入失败
     */
    bool enqueueArray(const Vector2D arr[], int length);

    /**
     * @brief 强制将一个数组压入队列
     * @param arr 要压入的 Vector2D 数组
     * @param length 数组长度
     */
    void forceEnqueueArray(const Vector2D arr[], int length);

    /**
     * @brief 清空队列
     */
    void clear();

    /**
     * @brief 计算队列中所有元素的总距离
     * @return float 总距离
     */
    float totalDistance() const;
};

#endif // VECTOR2D_H
#endif
