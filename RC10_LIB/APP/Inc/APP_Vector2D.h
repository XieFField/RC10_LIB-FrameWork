// 包含 Vector2D 类的定义
#include "APP_Vector2D.h"

/**
 * @brief 默�?构造函�?
 * 初�?化向量的 x �? y 分量�? 0
 */
Vector2D::Vector2D() : x(0.0f), y(0.0f) {}

/**
 * @brief 带参数构造函�?
 * @param x_ 向量�? x 分量
 * @param y_ 向量�? y 分量
 */
Vector2D::Vector2D(float32_t x_, float32_t y_) : x(x_), y(y_) {}

/**
 * @brief 重载赋值运算�?
 * @param other 另一�? Vector2D 对象
 * @return Vector2D& 当前对象的引�?
 */
Vector2D &Vector2D::operator=(const Vector2D &other)
{
    if (this != &other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    return *this;
}

/**
 * @brief 向量加法
 * @param other 另一�? Vector2D 对象
 * @return Vector2D 加法结果
 */
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(this->x + other.x, this->y + other.y);
}

/**
 * @brief 向量减法
 * @param other 另一�? Vector2D 对象
 * @return Vector2D 减法结果
 */
Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(this->x - other.x, this->y - other.y);
}

/**
 * @brief 向量点乘
 * @param other 另一�? Vector2D 对象
 * @return float32_t 点乘结果
 */
float32_t Vector2D::operator*(const Vector2D &other) const
{
    return this->x * other.x + this->y * other.y;
}

/**
 * @brief 向量乘以标量
 * @param scalar 标量�?
 * @return Vector2D 乘法结果
 */
Vector2D Vector2D::operator*(float32_t scalar) const
{
    return Vector2D(this->x * scalar, this->y * scalar);
}

/**
 * @brief 标量乘以向量（标量在左侧�?
 * @param scalar 标量�?
 * @param vec 向量对象
 * @return Vector2D 乘法结果
 */
Vector2D operator*(float32_t scalar, const Vector2D &vec)
{
    return Vector2D(vec.x * scalar, vec.y * scalar);
}

/**
 * @brief 计算向量的模（长度）
 * @return float32_t 向量的模
 */
float32_t Vector2D::magnitude() const
{
    return sqrt(this->x * this->x + this->y * this->y);
}

/**
 * @brief 单位化向�?
 * @return Vector2D 单位向量
 */
Vector2D Vector2D::normalize() const
{
    Vector2D result;
    float32_t mag = this->magnitude();
    if (mag > 0)
    {
        result.x = this->x / mag;
        result.y = this->y / mag;
    }
    return result;
}

/**
 * @brief 向量投影
 * @param other 投影到的向量
 * @return Vector2D 投影结果
 */
Vector2D Vector2D::project_onto(const Vector2D &other) const
{
    float32_t dotProduct = (*this) * other;
    float32_t magOtherSquared = other.x * other.x + other.y * other.y;
    float32_t scalar = dotProduct / magOtherSquared;
    return other * scalar;
}

/**
 * @brief 重载负号运算�?
 * @return Vector2D 取反后的向量
 */
Vector2D Vector2D::operator-() const
{
    return Vector2D(-this->x, -this->y);
}

/**
 * @brief 计算向量的叉�?
 * @param other 另一�? Vector2D 对象
 * @return float 向量的叉�?���?
 */
float Vector2D::cross(const Vector2D &other) const
{
    return (this->x * other.y) - (this->y * other.x);
}

/**
 * @brief 线性插�?
 * @param a 起�?向量
 * @param b 结束向量
 * @param t 插值参数，范围 [0, 1]
 * @return Vector2D 插值结�?
 */
Vector2D Vector2D::lerp(const Vector2D &a, const Vector2D &b, float t)
{
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return a + (b - a) * t;
}

/**
 * @brief 计算两点之间的距离平�?
 * @param a �?���?��
 * @param b �?���?��
 * @return float 两点之间的距离平�?
 */
float Vector2D::distanceSquared(const Vector2D &a, const Vector2D &b)
{
    return (b - a) * (b - a);
}

/**
 * @brief 通过三个点�?算曲�?
 * @param p0 �?���?��
 * @param p1 �?���?��
 * @param p2 �?���?��
 * @return float 曲率�?
 */
float Vector2D::curvatureFromThreePoints(const Vector2D &p0, const Vector2D &p1, const Vector2D &p2)
{
    const Vector2D v0 = p1 - p0; // �? p0 �? p1 的向�?
    const Vector2D v1 = p2 - p1; // �? p1 �? p2 的向�?

    const float cross = v0.cross(v1);           // 计算叉积
    const float numerator = fabsf(2.f * cross); // 分子：三角形面积的两�?

    if (isZero(numerator)) // 若叉�?�� 0，三点共�?
    {
        return 0.f;
    }

    const float len0 = v0.magnitude();
    const float len1 = v1.magnitude();
    const float len2 = (v0 + v1).magnitude();

    if (isZero(len0) || isZero(len1) || isZero(len2)) // 避免分母�? 0
    {
        return 0.f;
    }

    const float denominator = len0 * len1 * len2; // 分母：三边长度乘�?

    float curvature = numerator / denominator; // 曲率 = 分子 / 分母

    if (curvature > 1e6f)
        curvature = 1e6f;
    if (curvature < 0.f)
        curvature = 0.f;

    return curvature;
}

/**
 * @brief  获取垂直法向量（逆时�?90度）
 */
Vector2D Vector2D::perpendicular() const
{
    return Vector2D(-this->y, this->x);
}

/**
 * @brief Vector2DQueue 默�?构造函�?
 */
Vector2DQueue::Vector2DQueue() : front(0), rear(-1), size(0) {}

/**
 * @brief 检查队列是否为�?
 * @return true 队列为空
 * @return false 队列不为�?
 */
bool Vector2DQueue::isEmpty() const
{
    return size == 0;
}

/**
 * @brief 检查队列是否已�?
 * @return true 队列已满
 * @return false 队列�?��
 */
bool Vector2DQueue::isFull() const
{
    return size == QUEUE_CAPACITY;
}

/**
 * @brief 返回队列�?��元素数量
 * @return int 当前队列大小
 */
int Vector2DQueue::queueSize() const
{
    return size;
}

/**
 * @brief 入队操作
 * @param vec 要入队的 Vector2D 对象
 * @return true 入队成功
 * @return false 入队失败（队列已满）
 */
bool Vector2DQueue::enqueue(const Vector2D &vec)
{
    if (isFull())
    {
        return false; // 队列已满，入队失�?
    }
    rear = (rear + 1) % QUEUE_CAPACITY; // �?��队列
    data[rear] = vec;                   // 插入元素
    size++;
    return true; // 入队成功
}

/**
 * @brief 出队操作
 * @param vec 用于接收出队元素�? Vector2D 对象
 * @return true 出队成功
 * @return false 出队失败（队列为空）
 */
bool Vector2DQueue::dequeue(Vector2D &vec)
{
    if (isEmpty())
    {
        return false; // 队列为空，出队失�?
    }
    vec = data[front];                    // 获取队�?元素
    front = (front + 1) % QUEUE_CAPACITY; // 更新队�?索引
    size--;
    return true; // 出队成功
}

/**
 * @brief 查看队�?元素
 * @return Vector2D 队�?元素的副�?
 */
Vector2D Vector2DQueue::peek() const
{
    if (isEmpty())
    {
        return Vector2D(0.0f, 0.0f); // 队列为空，查看失�?
    }

    return data[front]; // 查看成功
}

/**
 * @brief 强制入队操作（�?盖队尾元素）
 * @param vec 要入队的 Vector2D 对象
 */
void Vector2DQueue::forceEnqueue(const Vector2D &vec)
{
    if (isFull())
    {
        front = (front + 1) % QUEUE_CAPACITY; // 移动队�?索引以�?盖最早插入的元素
        size--;                               // 因�?盖，队列大小减少
    }
    rear = (rear + 1) % QUEUE_CAPACITY; // 插入新元�?
    data[rear] = vec;
    size++; // 更新队列大小
}

/**
 * @brief 将一�?��组压入队列，数组索引小的元素先压�?
 * @param arr 要压入队列的数组
 * @param length 数组的长�?
 * @return true 数组全部元素成功入队
 * @return false 如果有元素无法入队，返回失败
 */
bool Vector2DQueue::enqueueArray(const Vector2D arr[], int length)
{
    if (length <= 0)
    {
        return false; // 无效长度，直接返回失�?
    }

    for (int i = 0; i < length; ++i)
    {
        // 检查是否能够�?常入�?
        if (!enqueue(arr[i]))
        {
            return false; // 如果有元素无法入队，返回失败
        }
    }
    return true; // 数组全部成功入队
}

/**
 * @brief 将一�?��组压入队列，数组索引小的元素先压�?
 * @param arr 要压入队列的数组
 * @param length 数组的长�?
 */
void Vector2DQueue::forceEnqueueArray(const Vector2D arr[], int length)
{
    for (int i = 0; i < length; ++i)
    {
        forceEnqueue(arr[i]); // 强制入队覆盖队尾元素
    }
}

/**
 * @brief 清空队列
 */
void Vector2DQueue::clear()
{
    front = 0;
    rear = -1;
    size = 0;
}

/**
 * @brief 计算队列�?��有向量的总距�?
 * @return float 总距�?
 */
float Vector2DQueue::totalDistance() const
{
    float total = 0.0f;
    if (size < 2)
        return total;
    int idx = front;
    Vector2D prev = data[idx];
    for (int i = 1; i < size; i++)
    {
        idx = (front + i) % QUEUE_CAPACITY;
        total += (data[idx] - prev).magnitude();
        prev = data[idx];
    }
    return total;
}
