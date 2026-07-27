#include "MotionPlanner.h"
#include <math.h>       // sqrtf, fabsf
#include "cmsis_os2.h"  // osKernelGetTickCount()

/*
  ==============================================================================
  本文件实现“动作序列”模块的所有状态、数据结构和逻辑，包含：
    - MotorProfile_t：描述单个电机在一个动作步骤内的匀速+距离
    - MotionStep_t：描述一帧动作步骤，包含 6 路电机各自的 MotorProfile_t
    - MotionLibrary_t：将所有步骤打包成一个动作库
    - 状态机：根据 posMea 计算出当前步的 spdRef，自动切换到下一步，循环往复
  ==============================================================================
*/

/** ----------------------------------------------------------------------------
 *  MotorProfile_t：
 *
 *  描述“一个动作步骤”里，单一路电机要做的匀速运动规则。
 *
 *  two_segments = false 时，本电机只跑一段匀速 v1，跑完 dist1 后停。
 *  two_segments = true  时，本电机会先跑第一段（v1, dist1），
 *      然后再跑第二段（v2, dist2），总距离 = dist1 + dist2。
 *
 *  - v1, v2: 赋予 signed float，可正可负，表示匀速的大小与方向
 *  - dist1, dist2: 表示各段要跑的绝对距离（单位与 posMea/posRef 一致）
 *
 *  例如： two_segments = false, v1 = -180, dist1 = 290
 *        表示这一路电机沿负方向以 180 的速度跑 290 单位距离，然后停。
 * ----------------------------------------------------------------------------
 */
typedef struct {
    bool    two_segments;
    float   v1;     ///< 第一段匀速（signed），正负代表方向
    float   dist1;  ///< 第一段要跑的绝对距离（>0）
    float   v2;     ///< 第二段匀速（仅当 two_segments==true 才有效）
    float   dist2;  ///< 第二段要跑的绝对距离（仅当 two_segments==true 才有效）
} MotorProfile_t;

/** ----------------------------------------------------------------------------
 *  MotionStep_t：
 *
 *  描述“动作库”里的一个步骤（step）。每个步骤包含：
 *    - max_duration:       本步骤最长期限（秒），超过后强制停止本步骤
 *    - motors[MOTOR_COUNT]: 每路电机在本步骤里的速度 + 距离规则
 * ----------------------------------------------------------------------------
 */
typedef struct {
    float          max_duration;       ///< 本步骤最多允许时长（秒），超时强制切换下一步
    MotorProfile_t motors[MOTOR_COUNT];///< 本步骤里 6 路电机的规则
} MotionStep_t;

/** ----------------------------------------------------------------------------
 *  MotionLibrary_t：
 *
 *  一套动作由若干个 MotionStep_t 组成，本例只定义“握拳”这一个步骤，
 *  如果后续要做更多动作（例如松开 → 再握拳），就可以追加更多 MotionStep_t。
 * ----------------------------------------------------------------------------
 */
typedef struct {
    MotionStep_t *steps;   ///< 指向所有步骤的数组
    size_t        count;   ///< 数组长度（步骤数量）
} MotionLibrary_t;

/** ----------------------------------------------------------------------------
 *  下面我们先只实现“握拳”这个动作（motion_step_0）：
 *
 *  motion_step_0 中每一路电机都只跑一段匀速（two_segments=false），
 *  大拇指①： v1 = -62,  dist1 = 100
 *  大拇指②： v1 = -100, dist1 = 160
 *  食指/中指/无名指/小拇指： v1 = -180, dist1 = 290
 *  max_duration = 1.62 秒（略大于实际 1.61s，留出容差）
 *
 *  如果以后想追加“松手”步骤，只需再定义一个 motion_step_1，并把它
 *  追加到 motion_steps[] 数组中即可。
 * ----------------------------------------------------------------------------
 */

/** “握拳”步骤 */
static MotionStep_t motion_step_0 = {
    .max_duration = 1.62f,  ///< 本步骤不超过 1.62s，否则强制跳到下一步

    .motors = {
        // i=0: 大拇指①号电机 (弯曲)
        { .two_segments = false, .v1 = -62.0f,  .dist1 = 100.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        // i=1: 大拇指②号电机 (摆动)
        { .two_segments = false, .v1 = -100.0f, .dist1 = 160.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        // i=2: 食指③号电机
        { .two_segments = false, .v1 = -180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        // i=3: 中指④号电机
        { .two_segments = false, .v1 = -180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        // i=4: 无名指⑤号电机
        { .two_segments = false, .v1 = -180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        // i=5: 小拇指⑥号电机
        { .two_segments = false, .v1 = -180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
    }
};

/** 如果想在“握拳”之后再做“松手”步骤，可按如下格式再定义 motion_step_1：
static MotionStep_t motion_step_1 = {
    .max_duration = 1.50f,
    .motors = {
        { .two_segments = false, .v1 = +62.0f,  .dist1 = 100.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        { .two_segments = false, .v1 = +100.0f, .dist1 = 160.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        { .two_segments = false, .v1 = +180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        { .two_segments = false, .v1 = +180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        { .two_segments = false, .v1 = +180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
        { .two_segments = false, .v1 = +180.0f, .dist1 = 290.0f, .v2 = 0.0f,    .dist2 = 0.0f   },
    }
};
*/

/** 组合所有步骤到一个数组中，务必在元素之间用逗号分隔——否则会出现编译错误！ */
static MotionStep_t motion_steps[] = {
    motion_step_0,
    // 如果需要“握拳→松手”，就在此处添加一行 “, motion_step_1”
    // motion_step_1
};

/** 整个动作库 */
static MotionLibrary_t gMotionLibrary = {
    .steps = motion_steps,
    .count = sizeof(motion_steps) / sizeof(motion_steps[0])
};

/*
  ==============================================================================
  以下为内部状态机所需的全局静态变量，Routine 組織逻辑已注释在函数体内。
  ==============================================================================
*/

/** 当前正在执行的步骤索引（0 开始） */
static uint32_t currentStepIdx;

/** 本步骤开始时，每一路电机的测量位置 */
static float stepStartPos[MOTOR_COUNT];

/** 本步骤开始时的时刻（秒） */
static float stepStartTime;

/** 标志：当前帧是否已经“进入”了某个步骤 */
static bool inStep;

/** ----------------------------------------------------------------------------
 *  获取“当前时刻”，单位为秒（Float）
 *  假设 osKernelGetTickCount() 返回的是 ms 级别滴答数。
 * ----------------------------------------------------------------------------
 */
static float GetNowSeconds(void)
{
    uint32_t ms = osKernelGetTickCount();
    return ((float)ms) * 0.001f;
}

/** ----------------------------------------------------------------------------
 *  进入下一步动作：
 *    - currentStepIdx++（如果超过 gMotionLibrary.count-1，则回到 0）
 *    - stepStartTime = 现在时刻
 *    - stepStartPos[i] = 当前 posMea[i]
 *    - inStep = true
 * ----------------------------------------------------------------------------
 */
static void StartNextMotionStep(const float posMea[MOTOR_COUNT])
{
    currentStepIdx++;
    if (currentStepIdx >= gMotionLibrary.count)
    {
        currentStepIdx = 0;
    }

    stepStartTime = GetNowSeconds();
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        stepStartPos[i] = posMea[i];
    }
    inStep = true;
}

/** ----------------------------------------------------------------------------
 *  MotionPlanner_Init：
 *    在系统初始化时只调用一次，让状态机达到初始就绪状态。
 * ----------------------------------------------------------------------------
 */
void MotionPlanner_Init(void)
{
    currentStepIdx = 0;
    stepStartTime  = 0.0f;
    inStep         = false;
    // stepStartPos 可以不用初始化
}

/** ----------------------------------------------------------------------------
 *  MotionPlanner_Update：
 *
 *  @param posMea[MOTOR_COUNT] : 当前 6 路电机的位置测量值
 *  @param spdRef[MOTOR_COUNT] : 本帧要输出的 6 路电机速度参考（signed float）
 *
 *  @return true  = 当前“动作库”中所有 step[s] 已完成一轮，并已回到第 0 步
 *          false = 当前仍在某一个步骤中，或刚进入下一步尚未“跑完一轮”
 *
 *  核心思路：
 *    1. 如果 inStep == false，说明上一帧不在任何步骤里，那么马上 StartNextMotionStep()
 *    2. 取出当前步骤 step = &gMotionLibrary.steps[currentStepIdx]
 *    3. 计算 t_pass = now - stepStartTime
 *    4. 对于每一路电机：
 *         - 计算 traveled = |posMea[i] - stepStartPos[i]|
 *         - 若 two_segments == false：
 *             如果 traveled ≥ dist1 或 t_pass ≥ max_duration → spdRef[i] = 0
 *             否则 spdRef[i] = v1
 *           否则（two_segments == true）：
 *             如果 traveled < dist1：
 *                 如果 t_pass ≥ max_duration → spdRef[i] = 0
 *                 否则 spdRef[i] = v1
 *             否则（跑到第二段）：
 *                 traveled2 = traveled - dist1
 *                 如果 traveled2 ≥ dist2 或 t_pass ≥ max_duration → spdRef[i] = 0
 *                 否则 spdRef[i] = v2
 *    5. 判断本步骤是否结束：只要【t_pass ≥ max_duration】或【每一路电机的 traveled ≥ 各自总距离】，
 *       则认定本步骤完成，inStep = false。
 *    6. 如果本步骤恰好是最后一个步骤（currentStepIdx == count-1）且结束，本函数返回 true，
 *       否则返回 false。
 * ----------------------------------------------------------------------------
 */
bool MotionPlanner_Update(const float posMea[MOTOR_COUNT], float spdRef[MOTOR_COUNT])
{
    // 1) 如果上一帧没有在任何步骤内，就立即进入下一步骤
    if (!inStep)
    {
        StartNextMotionStep(posMea);
    }

    // 2) 获取当前步骤
    MotionStep_t *step = &gMotionLibrary.steps[currentStepIdx];

    // 3) 计算已经过去的时间 t_pass
    float now    = GetNowSeconds();
    float t_pass = now - stepStartTime;
    if (t_pass < 0.0f)
    {
        t_pass = 0.0f;
    }

    // 4) 遍历 6 路电机，根据 MotorProfile_t 决定本帧 spdRef[i]
    bool allDone = true;  // 用于判断“本步骤是否所有通道都完成各自距离”
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        MotorProfile_t *mp = &step->motors[i];
        float traveled = fabsf(posMea[i] - stepStartPos[i]);

        if (!mp->two_segments)
        {
            // —— 单段匀速 v1 —— //
            if ((traveled >= mp->dist1) || (t_pass >= step->max_duration))
            {
                // 跑超距离或超时 → 速度设为 0
                spdRef[i] = 0.0f;
            }
            else
            {
                spdRef[i] = mp->v1;
                allDone = false;
            }
        }
        else
        {
            // —— 两段匀速 (v1, dist1) 然后 (v2, dist2) —— //
            if (traveled < mp->dist1)
            {
                // 还在第一段
                if (t_pass >= step->max_duration)
                {
                    spdRef[i] = 0.0f;
                }
                else
                {
                    spdRef[i] = mp->v1;
                    allDone = false;
                }
            }
            else
            {
                // 已完成第一段，进入第二段
                float traveled2 = traveled - mp->dist1;
                if ((traveled2 >= mp->dist2) || (t_pass >= step->max_duration))
                {
                    spdRef[i] = 0.0f;
                }
                else
                {
                    spdRef[i] = mp->v2;
                    allDone = false;
                }
            }
        }
    }

    // 5) 判断本步骤是否结束：只要“超时” 或 “所有电机通道的 traveled 都 >= 它们应跑的总距离”
    if ((t_pass >= step->max_duration) || allDone)
    {
        // 标记为“本步骤结束”
        inStep = false;

        // 如果这个步骤恰好是动作库最后一个步骤，则视为“一轮动作跑完”
        if ((currentStepIdx == (gMotionLibrary.count - 1)) && (allDone || (t_pass >= step->max_duration)))
        {
            // 下一次再 StartNextMotionStep 时会自动回到 0
            return true;
        }
    }

    return false;
}
