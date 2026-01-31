#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <arm_math.h> // åŒ…å« DSP åº“ï¼Œç”¨äºŽæ•°å?è®¡ç®—

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// å®šä¹‰é˜Ÿåˆ—çš„æœ€å¤§å?é‡?
#define QUEUE_CAPACITY 3

/**
 * @class Vector2D
 * @brief è¡¨ç¤ºäºŒç»´å‘é‡çš„ç±»
 *
 * æä¾›äº†å‘é‡çš„åŸºæœ¬æ“ä½œï¼ŒåŒ…æ‹?Š æ³•ã€å‡æ³•ã€ç‚¹ä¹˜ã€å‰ç§?€?
 * æ ‡é‡ä¹˜æ³•ã€å•ä½åŒ–ã€æŠ•å½±ç­‰ã€?
 */
class Vector2D
{
public:
    float32_t x; ///< å‘é‡çš? x åˆ†é‡
    float32_t y; ///< å‘é‡çš? y åˆ†é‡

    /**
     * @brief é»˜è?æž„é€ å‡½æ•?
     * åˆå?åŒ–å‘é‡çš„ x å’? y åˆ†é‡ä¸? 0
     */
    Vector2D();

    /**
     * @brief å¸¦å‚æ•°æž„é€ å‡½æ•?
     * @param x_ å‘é‡çš? x åˆ†é‡
     * @param y_ å‘é‡çš? y åˆ†é‡
     */
    Vector2D(float32_t x_, float32_t y_);

    /**
     * @brief é‡è½½èµ‹å€¼è¿ç®—ç?
     * @param other å¦ä¸€ä¸? Vector2D å¯¹è±¡
     * @return Vector2D& å½“å‰å¯¹è±¡çš„å¼•ç”?
     */
    Vector2D &operator=(const Vector2D &other);

    /**
     * @brief å‘é‡åŠ æ³•
     * @param other å¦ä¸€ä¸? Vector2D å¯¹è±¡
     * @return Vector2D åŠ æ³•ç»“æžœ
     */
    Vector2D operator+(const Vector2D &other) const;

    /**
     * @brief å‘é‡å‡æ³•
     * @param other å¦ä¸€ä¸? Vector2D å¯¹è±¡
     * @return Vector2D å‡æ³•ç»“æžœ
     */
    Vector2D operator-(const Vector2D &other) const;

    /**
     * @brief é‡è½½è´Ÿå·è¿ç®—ç¬?
     * @return Vector2D å–ååŽçš„å‘é‡
     */
    Vector2D operator-() const;

    /**
     * @brief å‘é‡ç‚¹ä¹˜
     * @param other å¦ä¸€ä¸? Vector2D å¯¹è±¡
     * @return float32_t ç‚¹ä¹˜ç»“æžœ
     */
    float32_t operator*(const Vector2D &other) const;

    /**
     * @brief è®¡ç®—å‘é‡çš„å‰ç§?
     * @param other å¦ä¸€ä¸? Vector2D å¯¹è±¡
     * @return float å‰ç§¯çš„æ ‡é‡ç»“æž?
     */
    float cross(const Vector2D &other) const;

    /**
     * @brief å‘é‡ä¹˜ä»¥æ ‡é‡
     * @param scalar æ ‡é‡å€?
     * @return Vector2D ä¹˜æ³•ç»“æžœ
     */
    Vector2D operator*(float32_t scalar) const;

    /**
     * @brief æ ‡é‡ä¹˜ä»¥å‘é‡ï¼ˆæ ‡é‡åœ¨å·¦ä¾§ï¼?
     * @param scalar æ ‡é‡å€?
     * @param vec å‘é‡å¯¹è±¡
     * @return Vector2D ä¹˜æ³•ç»“æžœ
     */
    friend Vector2D operator*(float32_t scalar, const Vector2D &vec);

    /**
     * @brief è®¡ç®—å‘é‡çš„æ¨¡ï¼ˆé•¿åº¦ï¼‰
     * @return float32_t å‘é‡çš„æ¨¡
     */
    float32_t magnitude() const;

    /**
     * @brief å•ä½åŒ–å‘é‡?
     * @return Vector2D å•ä½å‘é‡
     */
    Vector2D normalize() const;

    /**
     * @brief å‘é‡æŠ•å½±
     * @param other æŠ•å½±åˆ°çš„å‘é‡
     * @return Vector2D æŠ•å½±ç»“æžœ
     */
    Vector2D project_onto(const Vector2D &other) const;

    /**
     * @brief è®¡ç®—ä¸¤ç‚¹ä¹‹é—´çš„è·ç¦»å¹³æ–?
     * @param a ç¬?¸€ä¸?‚¹
     * @param b ç¬?ºŒä¸?‚¹
     * @return float ä¸¤ç‚¹ä¹‹é—´çš„è·ç¦»å¹³æ–?
     */
    static float distanceSquared(const Vector2D &a, const Vector2D &b);

    /**
     * @brief çº¿æ€§æ’å€?
     * @param a èµ·å?å‘é‡
     * @param b ç»“æŸå‘é‡
     * @param t æ’å€¼å‚æ•°ï¼ŒèŒƒå›´ [0, 1]
     * @return Vector2D æ’å€¼ç»“æž?
     */
    static Vector2D lerp(const Vector2D &a, const Vector2D &b, float t);

    /**
     * @brief é€šè¿‡ä¸‰ä¸ªç‚¹è?ç®—æ›²çŽ?
     * @param p0 ç¬?¸€ä¸?‚¹
     * @param p1 ç¬?ºŒä¸?‚¹
     * @param p2 ç¬?¸‰ä¸?‚¹
     * @return float æ›²çŽ‡å€?
     */
    static float curvatureFromThreePoints(const Vector2D &p0, const Vector2D &p1, const Vector2D &p2);

    /**
     * @brief èŽ·å–åž‚ç›´æ³•å‘é‡ï¼ˆé€†æ—¶é’?90åº¦ï¼‰
     * @return Vector2D åž‚ç›´æ³•å‘é‡?
     */
    Vector2D perpendicular() const;

private:
    /**
     * @brief è¾…åŠ©å‡½æ•°ï¼šæ?æŸ¥æ ‡é‡æ˜¯å¦æŽ¥è¿‘é›¶
     * @param scalar æ ‡é‡å€?
     * @return true å¦‚æžœæ ‡é‡æŽ¥è¿‘é›?
     * @return false å¦‚æžœæ ‡é‡ä¸æŽ¥è¿‘é›¶
     */
    static bool isZero(float scalar)
    {
        return (scalar < 0 ? -scalar : scalar) < 1e-6f;
    }
};

/**
 * @class Vector2DQueue
 * @brief è¡¨ç¤ºä¸€ä¸?›ºå®šå?é‡çš„ Vector2D é˜Ÿåˆ—
 *
 * æä¾›äº†é˜Ÿåˆ—çš„åŸºæœ¬æ“ä½œï¼ŒåŒ…æ‹?…¥é˜Ÿã€å‡ºé˜Ÿã€æŸ¥çœ‹é˜Ÿé¦–å…ƒç´ ç­‰ã€?
 */
class Vector2DQueue
{
private:
    Vector2D data[QUEUE_CAPACITY]; ///< ç”¨äºŽå­˜å‚¨é˜Ÿåˆ—å…ƒç´ çš„æ•°ç»?
    int front;                     ///< é˜Ÿé?ç´¢å¼•
    int rear;                      ///< é˜Ÿå°¾ç´¢å¼•
    int size;                      ///< å½“å‰é˜Ÿåˆ—å¤§å°

public:
    /**
     * @brief é»˜è?æž„é€ å‡½æ•?
     * åˆå?åŒ–é˜Ÿåˆ—ä¸ºç©?
     */
    Vector2DQueue();

    /**
     * @brief æ£€æŸ¥é˜Ÿåˆ—æ˜¯å¦ä¸ºç©?
     * @return true é˜Ÿåˆ—ä¸ºç©º
     * @return false é˜Ÿåˆ—ä¸ä¸ºç©?
     */
    bool isEmpty() const;

    /**
     * @brief æ£€æŸ¥é˜Ÿåˆ—æ˜¯å¦å·²æ»?
     * @return true é˜Ÿåˆ—å·²æ»¡
     * @return false é˜Ÿåˆ—æœ?»¡
     */
    bool isFull() const;

    /**
     * @brief è¿”å›žé˜Ÿåˆ—ä¸?š„å…ƒç´ æ•°é‡
     * @return int é˜Ÿåˆ—ä¸?š„å…ƒç´ æ•°é‡
     */
    int queueSize() const;

    /**
     * @brief å…¥é˜Ÿæ“ä½œ
     * @param vec è¦å…¥é˜Ÿçš„ Vector2D å¯¹è±¡
     * @return true å…¥é˜ŸæˆåŠŸ
     * @return false å…¥é˜Ÿå¤±è´¥ï¼ˆé˜Ÿåˆ—å·²æ»¡ï¼‰
     */
    bool enqueue(const Vector2D &vec);

    /**
     * @brief å¼ºåˆ¶å…¥é˜Ÿæ“ä½œ
     * @param vec è¦å…¥é˜Ÿçš„ Vector2D å¯¹è±¡
     * å¦‚æžœé˜Ÿåˆ—å·²æ»¡ï¼Œå°†è¦†ç›–é˜Ÿå°¾å…ƒç´ ã€?
     */
    void forceEnqueue(const Vector2D &vec);

    /**
     * @brief å‡ºé˜Ÿæ“ä½œ
     * @param vec ç”¨äºŽå­˜å‚¨å‡ºé˜Ÿçš? Vector2D å¯¹è±¡
     * @return true å‡ºé˜ŸæˆåŠŸ
     * @return false å‡ºé˜Ÿå¤±è´¥ï¼ˆé˜Ÿåˆ—ä¸ºç©ºï¼‰
     */
    bool dequeue(Vector2D &vec);

    /**
     * @brief æŸ¥çœ‹é˜Ÿé?å…ƒç´ 
     * @return Vector2D é˜Ÿé?å…ƒç´ 
     */
    Vector2D peek() const;

    /**
     * @brief å°†ä¸€ä¸?•°ç»„åŽ‹å…¥é˜Ÿåˆ?
     * @param arr è¦åŽ‹å…¥çš„ Vector2D æ•°ç»„
     * @param length æ•°ç»„é•¿åº¦
     * @return true åŽ‹å…¥æˆåŠŸ
     * @return false åŽ‹å…¥å¤±è´¥
     */
    bool enqueueArray(const Vector2D arr[], int length);

    /**
     * @brief å¼ºåˆ¶å°†ä¸€ä¸?•°ç»„åŽ‹å…¥é˜Ÿåˆ?
     * @param arr è¦åŽ‹å…¥çš„ Vector2D æ•°ç»„
     * @param length æ•°ç»„é•¿åº¦
     */
    void forceEnqueueArray(const Vector2D arr[], int length);

    /**
     * @brief æ¸…ç©ºé˜Ÿåˆ—
     */
    void clear();

    /**
     * @brief è®¡ç®—é˜Ÿåˆ—ä¸?‰€æœ‰å…ƒç´ çš„æ€»è·ç¦?
     * @return float æ€»è·ç¦?
     */
    float totalDistance() const;
};

#endif // VECTOR2D_H
#endif
    fk_speed.y = this->getWorldSpeed().vy;

    SpeedFK_Queue.send(fk_speed);
}

/////////////////////////////////    Â·¾¶¾ÀÆ«´úÂë   //////////////////////////////////////////////

/**
 * @brief ÕûºÏÒÑÓÐ½Ó¿Ú£¬»ñÈ¡¡°×î½üµã×ø±ê¡±ºÍ¡°¶ÔÓ¦µÄtÖµ¡±
 * @param robotPos ÊäÈë£º»úÆ÷ÈËµ±Ç°Êµ¼ÊÎ»ÖÃ£¨±Õ»·ºËÐÄÊäÈë£©
 * @param tNearest Êä³ö£º×î½üµã¶ÔÓ¦µÄÇúÏß²ÎÊýt£¨0~1£©£¬¸øºóÐøÕÒÇ°ÊÓµãÓÃ
 * @return Vector2D Êä³ö£º×î½üµãµÄ×ø±ê£¨¸øºóÐøËãºáÏòÆ«²îÓÃ£©
 */
Vector2D OmniChassis_Setup::GetPathNearestPoint(BezierCurve &path_, const Vector2D &robotPos, float &tNearest)
{
    // µÚÒ»²½£ºµ÷ÓÃÄãµÄGet_Nearest_Distance£¬ÄÃµ½tNearest£¨×î½üµã¶ÔÓ¦µÄtÖµ£©
    // ÖØµã£ºµÚ¶þ¸ö²ÎÊý´« &tNearest£¨tNearestµÄµØÖ·£©£¬ÒòÎªÄãµÄº¯ÊýÊÇ¡°Êä³ö²ÎÊý¡±£¨Í¨¹ýÖ¸Õë¸³Öµ£©
    path_.Get_Nearest_Distance(robotPos, &tNearest);

    // µÚ¶þ²½£ºÓÃµÚÒ»²½ÄÃµ½µÄtNearest£¬µ÷ÓÃÄãµÄGet_Point£¬ÄÃµ½×î½üµã×ø±ê
    Vector2D nearestPt = path_.Get_Point(tNearest);

    // µÚÈý²½£º·µ»Ø×î½üµã×ø±ê£¬¸øºóÐø¡°ËãºáÏòÆ«²î¡±ÓÃ
    return nearestPt;
}

// º¯Êý×÷ÓÃ£ºÊäÈë×î½üµãµÄ±àºÅtNearest£¬Êä³öÇ°ÊÓµã×ø±êºÍËüµÄ±àºÅtLookahead
Vector2D OmniChassis_Setup::FindLookaheadPoint(BezierCurve &path_, float tNearest, float &tLookahead)
{
    // -------------- ¶ÔÓ¦µÚ1²½£º³õÊ¼»¯£¬´Ó×î½üµã¿ªÊ¼ --------------
    tLookahead = tNearest;        // Ç°ÊÓµãµÄ±àºÅ£¬ÏÈ´Ó×î½üµãµÄ±àºÅ¿ªÊ¼£¨±ÈÈçt=0.3£©
    float accumulatedDist = 0.0f; // ÀÛ¼ÆÅ²ÁË¶àÉÙ¾àÀë£¨¸Õ¿ªÊ¼ÊÇ0£©
    float step = 0.01f;           // Ã¿´ÎÅ²µÄ¡°Ð¡²½×Ó¡±

    // ÄÃµ½×î½üµãµÄ×ø±ê£¨±ÈÈç(5.2, 6.1)£©£¬×÷Îª¡°Å²²½¡±µÄÆðµã
    Vector2D lastPt = path_.Get_Point(tLookahead);

    // -------------- ¶ÔÓ¦µÚ2²½£ºÐ¡²½ÂýÅ²£¬Ö±µ½ÀÛ¼Æ¾àÀë¹»Ç°ÊÓ¾àÀë --------------
    // Ìõ¼þ£º1. ±àºÅtÃ»µ½ÖÕµã£¨<1.0£©£»2. ÀÛ¼Æ¾àÀë»¹Ã»µ½Ç°ÊÓ¾àÀë£¨<0.4m£©
    while (tLookahead < 1.0f && accumulatedDist < m_lookaheadDist)
    {
        // 1. ÍùÇ°Å²Ò»Ð¡²½£ºtÔö¼Ó0.005£¨±ÈÈç0.3¡ú0.305£©
        float nextT = tLookahead + step;
        // ·ÀÖ¹Å²³¬ÖÕµã£ºÈç¹ûnextT>1.0£¬¾Í¸Ä³É1.0£¨²»ÄÜ³¬³öÇúÏß£©
        if (nextT > 1.0f)
        {
            nextT = 1.0f;
        }

        // 2. ÄÃµ½ÕâÒ»²½Å²µ½µÄµãµÄ×ø±ê£¨±ÈÈçt=0.305¶ÔÓ¦µÄÇúÏßµã(5.22, 6.11)£©
        Vector2D nextPt = path_.Get_Point(nextT);

        // 3. ¼ÆËãÕâÒ»²½×ßÁË¶àÔ¶£¨±ÈÈç´Ó(5.2,6.1)µ½(5.22,6.11)£¬¾àÀë¡Ö0.022m£©
        float distStep = (nextPt - lastPt).magnitude();

        // 4. ÀÛ¼Æ¾àÀë£º°ÑÕâÒ»²½µÄ¾àÀë¼Ó½øÈ¥£¨±ÈÈç0+0.022=0.022m£©
        accumulatedDist += distStep;

        // 5. ¸üÐÂ£º×¼±¸ÏÂÒ»²½Å²²½£¨°Ñµ±Ç°µãµ±Æðµã£¬µ±Ç°tµ±ÏÂÒ»²½µÄ»ù´¡£©
        tLookahead = nextT; // ±àºÅ¸üÐÂ
        lastPt = nextPt;    // Æðµã¸üÐÂÎª(5.22,6.11)
    }

    // -------------- ¶ÔÓ¦µÚ3²½£ºÈç¹ûµ½ÖÕµãÁË£¬Ö±½ÓÓÃÖÕµãµ±Ç°ÊÓµã --------------
    if (tLookahead >= 1.0f)
    {
        lastPt = path_.Get_Point(1.0f); // ÄÃÇúÏßÖÕµã×ø±ê
    }

    // -------------- ·µ»ØÇ°ÊÓµã×ø±ê --------------
    return lastPt;
}

/**
 * @brief ¼ÆËãºáÏòÆ«²î£¨´ø·½Ïò£ºÕý=Æ«×ó£¬¸º=Æ«ÓÒ£¬µ¥Î»£ºm£©
 * @param robotPos »úÆ÷ÈËµ±Ç°Î»ÖÃ£¨Ö÷º¯ÊýµÄm_robotPos£©
 * @param nearestPt ÇúÏß×î½üµã£¨Ö÷º¯ÊýµÄnearestPt£¬¼´P(t')£©
 * @param tLookahead Ç°ÊÓµãtÖµ£¨Ö÷º¯ÊýµÄtLookahead£¬ÓÃÓÚ»ñÈ¡ÎÈ¶¨ÇÐÏòÁ¿£©
 * @return float ´¿ºáÏòÆ«²î£¨ÎÞÇ°ºó¸ÉÈÅ£¬Ö±½Ó¸øPIDÓÃ£©
 */
float OmniChassis_Setup::CalculateLateralError(BezierCurve &path_, const Vector2D &robotPos, const Vector2D &nearestPt, float tLookahead)
{
    // ²½Öè1£º¼ÆËãÔ­Ê¼Æ«²îÏòÁ¿ ¦¤p = »úÆ÷ÈËÎ»ÖÃ - ×î½üµã£¨ÄãµÄ¶¨Òå£º¦¤p = p - p(t')£©
    Vector2D delta_p = robotPos - nearestPt;

    // ²½Öè2£º»ñÈ¡Ç°ÊÓµãµÄÇÐÏòÁ¿£¨ºÍÖ÷º¯ÊýÒ»ÖÂ£¬È·±£Ç°½ø·½Ïò»ù×¼Í³Ò»£©
    // Ö÷º¯ÊýÀïÒÑ¾­µ÷ÓÃ¹ýÒ»´Î£¬µ«ÕâÀïÔÙµ÷ÓÃÒ»´Î£¬±£Ö¤Æ«²î¼ÆËãºÍÇ°½ø·½ÏòÍêÈ«Í¬²½
    lookaheadTangent = path_.Get_Tangent_Vector(tLookahead);

    // ²½Öè4£º¶¨Òå¡°ºáÏò·½Ïò¡±£º´¹Ö±ÓÚÇ°ÊÓµãÇÐÏòÁ¿£¨×ó×ª90¶È£¬ºÍÖ÷º¯ÊýcorrDir·½ÏòÒ»ÖÂ£©
    // Ö÷º¯Êý¾ÀÆ«·½ÏòÊÇ corrDir = (-lookaheadTangent.y, lookaheadTangent.x)£¬ÕâÀïºáÏò·½ÏòºÍËü±£³ÖÒ»ÖÂ
    Vector2D lateral_dir = Vector2D(-lookaheadTangent.y, lookaheadTangent.x);
    // ºáÏò·½ÏòÒ²¹éÒ»»¯£ºÈ·±£µã»ý¼ÆËãµÄÆ«²îµ¥Î»ÊÇ¡°Ã×¡±£¨ÎÞËõ·Å¸ÉÈÅ£©
    lateral_dir.normalize();

    // ²½Öè5£ººËÐÄ£º¼ÆËãÔ­Ê¼Æ«²î¦¤pÔÚ¡°ºáÏò·½Ïò¡±µÄÍ¶Ó° ¡ú ´¿ºáÏòÆ«²î
    // µã»ý¹«Ê½£ºdelta_p ¡¤ lateral_dir = |delta_p| * cos¦È£¨¦ÈÊÇ¦¤pºÍºáÏò·½ÏòµÄ¼Ð½Ç£©
    // ×÷ÓÃ£º¹ýÂËÇ°ºó·½Ïò¸ÉÈÅ£¨Ç°ºó·½ÏòÓëºáÏò´¹Ö±£¬cos90¡ã=0£©£¬Ö»Áô×óÓÒÆ«²î
    float lateral_err = delta_p * lateral_dir;

    // £¨¿ÉÑ¡£©µ÷ÊÔÓÃ£ºÈç¹û·¢ÏÖ¾ÀÆ«·½Ïò·´ÁË£¬°ÑÆ«²î³Ë-1¼´¿É
    // lateral_err *= -1;

    return lateral_err;
}
void OmniChassis_Setup::KFS_Selection_Planning(void)
{
    int cho = 0;

    Point2D temp;
    temp.x = robot_pos_.x;
    temp.y = robot_pos_.y;
    if (robot_pos_.x < -0.5f || robot_pos_.x > 5.2f || robot_pos_.y < -0.5f || robot_pos_.y > 8.3f)
    {
        flag_run = 0;
        cho = 0;
    }
    else if (robot_pos_.y < 2.0f)
    {
        cho = 1;
    }
    else
    {
        //cho = 2;
    }

    if (cho == 1)
    {
        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(temp, KFS, 0, 26);
        int point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.entranceMap, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 )
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        int KFS_next_index=index;
        
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);

        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // ÐÞ¸Ä³µ×Ó³¯Ïò
        if (abs(path_key_point_[KFS_next_index] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
    else if (cho == 2)
    {

        KFS_result_ = MF_AutoCtrler::PathNodeResult_calc(Point2D{temp.x, 1.2f, 0.0f}, KFS, 0, 26);
        temp.x += 0.5f;
        temp.y += 0.5f;
        point_map = MF_AutoCtrler::GetMapNumFromPos(temp);
        // ½«³µ×Ó¿ªµ½¿Õ¿õµØ´øµÄÂ·¿Ú
        int point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        int index = 0;
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }
        // ½«³µ×Ó¿ªµ½kfsÇ°
        //        point_sum = MF_AutoCtrler::BFS_GetPath(point_map, KFS_result_.bestBMF1, path_point_, 20);
        //        for (int i = 0; i < point_sum; i++)
        //        {
        //            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.bestBMF1)
        //            {
        //                path_key_point_[index] = path_point_[i];
        //                index++;
        //            }
        //        }
        // ½«³µ×Ó¿ªµ½Ð±ÆÂÇ°
        point_sum = MF_AutoCtrler::BFS_GetPath(KFS_result_.bestBMF1, KFS_result_.exitMap, path_point_, 20);
        for (int i = 0; i < point_sum; i++)
        {
            if (path_point_[i] == 1 || path_point_[i] == 5 || path_point_[i] == 30 || path_point_[i] == 26 || path_point_[i] == KFS_result_.exitMap)
            {
                path_key_point_[index] = path_point_[i];
                index++;
            }
        }

        // ÐÞ¸Ä³µ×Ó³¯Ïò
        if (abs(path_key_point_[0] - KFS_result_.bestBMF1) < 5.0f)
        {
            if (KFS_result_.bestBMF1 < 10.0f)
            {
                target_yaw_ = 90.0f;
            }
            else
            {
                target_yaw_ = -90.0f;
            }
        }
        else
        {
            if (KFS_result_.bestBMF1 == 21 || KFS_result_.bestBMF1 == 16 || KFS_result_.bestBMF1 == 11 || KFS_result_.bestBMF1 == 6)
            {
                target_yaw_ = 180.0f;
            }
            else
            {
                target_yaw_ = 0.0f;
            }
        }

        path_line_.plan_reset();
        path_line_.Reset();
        path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_);
        for (int i = 0; i < index; i++)
        {
            if (i == (index - 1))
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_End_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
            else
            {
                Vector2D temp_vector = MF_AutoCtrler::MapCenterWorld_Vector2D(path_key_point_[i]);
                path_line_.Add_Point(Vector2D{temp_vector.x - 0.5f, temp_vector.y - 0.5f});
            }
        }
    }
}

void OmniChassis_Setup::Path_correction(void)
{
    // »ñÈ¡ÇúÏß£¨´ø±£»¤£©
    BezierCurve &curve = path_line_.get_bezier_curve();

    pathEnd = curve.Get_Point(1.0f);
    // 1. ÕÒ×î½üµã+tÖµ£º»ñÈ¡Â·¾¶ÉÏ¾àÀëµ±Ç°Î»ÖÃ×î½üµÄµã¼°Æä²ÎÊý tNearest
    nearestPt = GetPathNearestPoint(curve, robot_pos_, tNearest);

    // ÖÕµã¾ÀÆ«²¹¶¡£ºÈç¹û·Ç³£½Ó½üÖÕµã£¬Ö±½ÓÊ¹ÓÃÖÕµãÎ»ÖÃÎü¸½
    // tNearest > 0.99 ±íÊ¾»ù±¾µ½ÁËÖÕµã£¬»òÕß Is_End()==false ±íÊ¾¹æ»®ÒÑ½áÊø
    if (tNearest > 0.99f || path_line_.Is_End() == false)
    {
        Vector2D endPt = curve.Get_End_point();
        // Èç¹ûÇúÏßÎ´³õÊ¼»¯£¨ÀýÈç¿ÕÇúÏß£©£¬²»½øÐÐ²Ù×÷
        if (endPt.magnitude() < 0.0001f && curve.Get_Start_point().magnitude() < 0.0001f)
        {
            speed = planspeed; // ±£³ÖÔ­ÓÐËÙ¶È£¨Í¨³£ÊÇ0£©
            return;
        }

        Vector2D errorVec = endPt - robot_pos_;

        // ÖÕµãÎü¸½ÔöÒæ£¬¿ÉÒÔ¸ù¾ÝÐèÒªµ÷Õû£¬Ïàµ±ÓÚÎ»ÖÃ»· P ²ÎÊý
        float final_kp = 2.0f;
        corrVelocity = errorVec * final_kp;

        // ÏÞÖÆ×î´ó¾ÀÆ«ËÙ¶È£¬·ÀÖ¹ÖÕµã¶¶¶¯
        float max_corr = 0.5f;
        if (corrVelocity.magnitude() > max_corr)
        {
            corrVelocity = corrVelocity.normalize() * max_corr;
        }

        speed = planspeed + corrVelocity; // µþ¼Óµ½¹æ»®ËÙ¶ÈÉÏ
        return;
    }

    // 2. ÕÒÇ°ÊÓµã+Ç°½ø·½Ïò£º¸ù¾Ý×î½üµãºÍÇ°ÊÓ¾àÀë£¬Ñ°ÕÒÇ°ÊÓµã¼°Æä²ÎÊý tLookahead
    lookaheadPt = FindLookaheadPoint(curve, tNearest, tLookahead);
    lookaheadTangent = curve.Get_Tangent_Vector(tLookahead);
    // 3. ¼ÆËãºáÏòÆ«²î£º¼ÆËã»úÆ÷ÈËµ±Ç°Î»ÖÃµ½Â·¾¶ÇÐÏßµÄ´¹Ö±¾àÀë
    lateralError = CalculateLateralError(curve, robot_pos_, nearestPt, tLookahead);
    // 4. ºáÏòÆ«²îPID¿ØÖÆ£º¼ÆËãºáÏò¾ÀÆ«ËÙ¶È´óÐ¡
    correctspeed = pid_track.pid_calc(0.0f, lateralError);
    Vector2D corrDir(-lookaheadTangent.y, lookaheadTangent.x); // ¾ÀÆ«·½Ïò£¨´¹Ö±Ç°½ø·½Ïò£¬×óÓÒ¾ÀÆ«£©
    corrVelocity = corrDir * correctspeed;                     // ºÏ³É¾ÀÆ«ËÙ¶È£¨·½Ïò+´óÐ¡£©

    // baseVelocity = lookaheadTangent * planspeed.magnitude();
    speed = planspeed + corrVelocity; // ×îÖÕËÙ¶È = ¹æ»®µÄÇ°½øËÙ¶È + ºáÏò¾ÀÆ«ËÙ¶È
}

void OmniChassis_Setup::Clamping_Bar_Selection_Planning(void)
{
    target_yaw_ = 0.0f;
    path_line_.plan_reset();
    path_line_.Reset();
    path_line_.Add_Start_Point(Vector2D{robot_pos_.x, robot_pos_.y}, path_param_1);
    path_line_.Add_Point(Vector2D{1.7f, 0.6f});
    path_line_.Add_End_Point(Clamping_Bar_Selection_pos_);
}
