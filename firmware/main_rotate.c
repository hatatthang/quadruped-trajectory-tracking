/* USER CODE BEGIN Header */
/*
  newotatever0.txt - DUNG / XOAY TAI CHO TRAI-PHAI, BU GOC VONG HO, 17/09/2026.
  Copy toan bo vao main.c cua Servoan / STM32F446RE; giu main.h, pca9685.c/.h, HAL va MSP.
  Don vi mm, s, deg. IK, mapping, trim, PCA va ngoai vi giu tu newcircle.txt.

  RUN_MODE: MODE_STAND, MODE_LEFT hoac MODE_RIGHT. Chi doi khi build/nap lai.
  Ca hai chieu bat dau voi dau robot huong +X, khong doi vai tro cac chan.
  ANGLE_DEG la goc muon dat; LEFT quay duong, RIGHT quay am tu huong +X.
  Goc lenh = ANGLE_DEG * CAL_COMMAND_DEG / CAL_MEASURED_DEG, toi da 270 do.
  CAL_COMMAND_DEG la goc LENH SAU BU cua luot do truoc; CAL_MEASURED_DEG
  la do lon goc THAN quay thuc te cung luot/cung chieu, khong la goc mui chan.
  Mac dinh 90/90 chi la he so 1 (chua bu), khong phai so lieu da do.
  Khao sat thu cong: giu CAL_* = 90/90, doi ANGLE_DEG trong (0,270].
  Vi du: lenh 90 do, than do duoc 60 do -> CAL_COMMAND_DEG=90,
  CAL_MEASURED_DEG=60; ANGLE_DEG=90 se tao lenh 135 do. Khong nhap 60 neu chua do.
  Hai chieu can do rieng; khi doi LEFT/RIGHT, nhap cap CAL_* cua chieu do.
  Khong tu cat lenh vuot 270 do; bao loi tham so de sua lai bo so.
  Tam tham chieu Pc giu tai (0,0); than van dich X/Y de chuyen tai khi nhac chan.

  N_CYCLES=0: tu tinh N=max(4, 2+ceil(goc lenh SAU BU/11.25)).
  N_CYCLES>0: dung N tu nhap, nhung khong duoc nho hon N tu tinh; loi 1 neu thieu.
  Lenh 90/150/175/200/270 do -> N10/16/18/20/26 khi de N_CYCLES=0.
  T12s: 200 do/N20 mat 240s; 270 do/N26 mat 312s, chua gom HOME/POSE/READY.
  LIFT30mm, SHIFT8/17mm; giu mapping 90+q2 / 90-q2 va trim goc.
  Hai bo 200/N20 va 270/N26 da kiem COM ly tuong, IK/goc/toc do hai chieu.
  N la tong so chu ky buoc; doi N khong doi tong goc lenh sau bu.
  Gioi han 11.25 do/chu ky giu buoc vua; khong la phep do COM hay goc than that.
  Yaw tang/giam toc cosine trong moi dau 1T, giua giu toc do quay deu.
  Doan yaw: 0.5T .. (N-0.5)T; hai dau giu de dat du bon chan.
  Dau/cuoi moi luot: bon chan o stance trung tinh, body shift bang 0.
  Ket thuc tai N*T thi giu tu the, khong tu lap va khong quay tiep.
  Chi bu he so goc tong; giu cong thuc tham chieu va neo chan cua SLX rotate.
  Khong cong bu yaw tung chu ky cua line. Bu nay khong la hoi tiep camera.

  HOME4s -> POSE6s -> READY12s -> ROTATE N*T -> giu.
  Ke than khi HOME/POSE; dat xuong trong READY. Reset se chay lai.
  Giu pre-shift / swing / post-return, nang sin^2, swing cycloid giua hai neo.
  RIGHT: FL -> FR -> RR -> RL (giu nguyen); LEFT: FR -> FL -> RL -> RR.
  LEFT doi xung lich chan cua RIGHT qua truc doc than; khong doi mapping/trim.
  Stance neo he dat, than/hong/chan cung mot yaw.
  Pc la tham chieu; can bang mo phong dung khoi luong gia dinh, khong la do COM that.

  WPG: FL/FR/RR/RL; PWM: FL0/RL1/RR2/FR3. dbg_* chi la lenh.
  Kiem IK/goc/toc do toan luot truoc PWM; khong cat goc de ep quy dao.
  Loi lenh giu xung cu; loi I2C/PCA yeu cau OFF. Do than khi xu ly loi.
  Khong halt MCU khi robot tu dung; dung debugger khong tat PWM.
*/
/* USER CODE END Header */
#include "main.h"
#include "pca9685.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#if !defined(STM32F446xx)
#error "Can project STM32F446RE; khong dung project STM32F407"
#endif

I2C_HandleTypeDef hi2c1;

// =====================================================================
// 1. KHAI BAO, HINH HOC, THONG SO GAIT VA HIEU CHUAN
// =====================================================================
#define MODE_STAND       0U
#define MODE_LEFT        1U
#define MODE_RIGHT       2U
#ifndef RUN_MODE
#define RUN_MODE         MODE_STAND
#endif
#if RUN_MODE != MODE_STAND && RUN_MODE != MODE_LEFT && RUN_MODE != MODE_RIGHT
#error "RUN_MODE chi nhan MODE_STAND, MODE_LEFT hoac MODE_RIGHT"
#endif
#if RUN_MODE == MODE_RIGHT
#define ROTATE_DIR       (-1)
#else
#define ROTATE_DIR       1
#endif

#define LEG_FL           0U
#define LEG_RL           1U
#define LEG_RR           2U
#define LEG_FR           3U
#define L1              39.0f
#define L2              72.0f
#define L3             106.0f
#define HIP_X           46.5f
#define HIP_Y           44.5f
#define STANCE_DX       53.0f
#define STANCE_DY       53.0f
#define BODY_HEIGHT     88.0f

#define GAIT_T           12.0f
#define ANGLE_DEG        250.0f
// Goc lenh toi da cua THAN; khong thay gioi han goc tung servo.
#define ANGLE_LIMIT_DEG 270.0f
// Cap so cung luot/cung chieu. 90/90 = he so 1, chua ap bu thuc nghiem.
#define CAL_COMMAND_DEG  90.0f
#define CAL_MEASURED_DEG 90.0f
#define LIFT_MM         30.0f
#define SHIFT_X_MM       8.0f
#define SHIFT_Y_MM      17.0f
// Goc tang moi chu ky o doan quay deu; moc 90 do / (10-2) chu ky.
#define MAX_YAW_PER_CYCLE_DEG 11.25f
#ifndef N_CYCLES
#define N_CYCLES          0U // 0: tu tinh theo goc lenh sau bu; >0: N thu cong
#endif
#define BETA_PREP        0.75f
#define BETA_SWING       0.80f
#define BETA_TOUCH       0.92f

#define HOME_T           4.0f
#define POSE_T           6.0f
#define READY_T         12.0f
#define TICK_MS         20U
// Chan loi lenh, KHONG la rate limiter; nguong giu tu Ver0.
#define MAX_RATE        80.0f
#define PRESCALE_FIXED 121U
#define PI_F             3.14159265358979323846f

_Static_assert(SERVO_MIN == 110 && SERVO_MAX == 500, "Can count 110..500 nhu ve base");
_Static_assert(PCA9685_ADDRESS == 0x80, "Can PCA address 0x80 theo HAL");

// Bang theo PHAN CUNG: FL, RL, RR, FR; giu trim cua Ver0fix da chay.
static const float HIP[4][2] = {
    { HIP_X,  HIP_Y},
    {-HIP_X,  HIP_Y},
    {-HIP_X, -HIP_Y},
    { HIP_X, -HIP_Y}
};
// 03/10/2026: JOINT_TRIM theo ban hieu chuan moi; FL femur chuyen sang kenh PCA 3.
static const uint8_t CH[4][3] = {
    {0, 3, 2},
    {4, 5, 6},
    {8, 9, 10},
    {12, 13, 14}
};
static const float JOINT_TRIM[4][3] = {
    { -4.0f,  -3.0f,  8.0f},
    { -9.5f,  -2.5f, -8.5f},
    {-10.5f, -10.0f,  8.0f},
    { -5.0f,  -1.0f,  0.0f}
};
static const float FOOT_Z_TRIM[4] = {1.0f, 0.0f, -1.0f, 0.0f};
// Coxa 10..170 deg: command envelope checked with current trims; verify mechanical clearance before floor test.
static const float LO[3] = {10.0f, 30.0f, 30.0f};
static const float HI[3] = {170.0f, 150.0f, 150.0f};

// Bang theo MODEL: FL, FR, RR, RL;
static const uint8_t MODEL_TO_HW[4] = {LEG_FL, LEG_FR, LEG_RR, LEG_RL};
#if RUN_MODE == MODE_LEFT
// Model FL/FR/RR/RL: FR -> FL -> RL -> RR; dich than theo chan active.
static const float PHI_LEG[4] = {0.50f, 0.75f, 0.00f, 0.25f};
#else
// RIGHT va STAND giu nguyen lich pha cu.
static const float PHI_LEG[4] = {0.75f, 0.50f, 0.25f, 0.00f};
#endif
static const float DIR_X[4] = {1.0f, 1.0f, -1.0f, -1.0f};
static const float DIR_Y[4] = {1.0f, -1.0f, -1.0f, 1.0f};

typedef struct {
    float t1;
    float t2;
    float t3;
} Joint;

enum {
    ST_HOME = 0,
    ST_POSE = 1,
    ST_READY = 2,
    ST_PRE_SHIFT = 3,
    ST_SWING = 4,
    ST_POST_RETURN = 5,
    ST_DONE = 6,
    ST_STAND = 7
};

// Loi: 1param, 3IK, 4angle, 6rate, 7late_tick, 8emergency_stop;
// 101..113 I2C/PCA. Loi latch den khi reset; khong tu bo qua/bo pha.
volatile uint32_t emergency_stop = 0U;
volatile uint32_t dbg_build_id = 202609173U;
volatile uint32_t dbg_mode = RUN_MODE;
volatile uint32_t dbg_alive = 0U;
volatile uint32_t dbg_stage = ST_HOME;
volatile uint32_t dbg_fault = 0U;
volatile uint32_t dbg_ready = 0U;
volatile uint32_t dbg_preflight = 0U;
volatile uint32_t dbg_output_on = 0U;
volatile uint32_t dbg_prescale = 0U;
volatile uint32_t dbg_mode1 = 0U;
volatile uint32_t dbg_verify_count = 0U;
volatile int8_t dbg_swing = -1;       // -1: bon chan tru; 0/1/2/3: FL/RL/RR/FR
volatile uint32_t dbg_active_leg = LEG_FL;
volatile uint32_t dbg_contact_mask = 15U; // bit 0/1/2/3: FL/RL/RR/FR, chi la lich
volatile float dbg_gait_t = 0.0f;
volatile int8_t dbg_direction = ROTATE_DIR;
volatile float dbg_goal_deg = ROTATE_DIR * ANGLE_DEG; // goc than MUON dat
volatile float dbg_yaw_gain = 1.0f;
volatile float dbg_target_deg = ROTATE_DIR * ANGLE_DEG; // lenh SAU BU, cap nhat o Prepare
volatile uint32_t dbg_cycles = 0U; // N thuc su sau khi Prepare tinh/kiem tra
volatile float dbg_cycle_yaw_deg = 0.0f; // do lon goc moi chu ky o doan deu
volatile float dbg_duration_s = 0.0f; // thoi gian buoc, chua gom HOME/POSE/READY
volatile float dbg_yaw_ref_deg = 0.0f; // yaw LENH, khong phai goc thuc do duoc
volatile float dbg_body_x = 0.0f;     // dich them trong HE THAN, khong gom tien/quay
volatile float dbg_body_y = 0.0f;
volatile float dbg_lift_mm = 0.0f;
volatile float dbg_foot_z[4] = {0.0f}; // lenh tuong doi than, da cong FOOT_Z_TRIM
volatile float dbg_servo[4][3] = {{0.0f}};
volatile uint16_t dbg_ticks[4][3] = {{0}};

// Pc, Phip_r, Pfoot_r la he dat LY TUONG, don vi mm.
static float Pc[3];
static float Body_Yaw = 0.0f; // rad, dung chung cho than/hong/chan/IK
static float Rotate_Angle_Deg = ANGLE_DEG; // do lon lenh sau bu, Prepare kiem truoc PWM
static uint32_t Rotate_Cycles = 10U; // N da giai quyet, dung chung cho quy dao va thoi gian
static float Pc_body[3];
static float Phip_r[4][3];
static float Pfoot_r[4][3];
static uint8_t contact[4];
static float home_angle[4][3];
static float pose_angle[4][3];
static float last_angle[4][3];
static uint8_t last_frame[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void Error_Handler(void);

// =====================================================================
// 2. DONG HOC NGHICH - KIEM MIEN TRUOC KHI TINH GOC
// =====================================================================
static Joint Leg_IK(float local_x, float local_y, float local_z, uint8_t *ok)
{
    Joint q = {0.0f, 0.0f, 0.0f};
    *ok = 0U;

    float radius = sqrtf(local_x * local_x + local_y * local_y);
    float planar_x = radius - L1;
    float distance = sqrtf(planar_x * planar_x + local_z * local_z);

    if (!isfinite(radius) || !isfinite(distance) || radius <= L1)
        return q;

    if (distance <= fabsf(L2 - L3) + 1.0f || distance >= L2 + L3 - 1.0f)
        return q;

    float c3 = (distance * distance - L2 * L2 - L3 * L3) / (2.0f * L2 * L3);

    if (!isfinite(c3) || c3 < -1.0f || c3 > 1.0f)
        return q;

    float s3 = -sqrtf(1.0f - c3 * c3);
    q.t1 = atan2f(local_y, local_x) * 180.0f / PI_F;
    q.t2 = (atan2f(local_z, planar_x) - atan2f(L3 * s3, L2 + L3 * c3)) * 180.0f / PI_F;
    q.t3 = atan2f(s3, c3) * 180.0f / PI_F;
    *ok = 1U;

    return q;
}

static Joint Body_IK(const float foot[3], int leg, uint8_t *ok)
{
    float dx = foot[0] - HIP[leg][0];
    float dy = foot[1] - HIP[leg][1];
    float local_x = (HIP[leg][0] > 0.0f) ? dx : -dx;
    float local_y = (HIP[leg][1] > 0.0f) ? dy : -dy;

    return Leg_IK(local_x, local_y, foot[2], ok);
}

// =====================================================================
// 3. MAPPING KHOP -> SERVO - KHONG CAT GOC DE EP QUY DAO
// =====================================================================
static void Map_Servo(int leg, Joint q, float out[3])
{
    q.t1 += JOINT_TRIM[leg][0];
    q.t2 += JOINT_TRIM[leg][1];
    q.t3 += JOINT_TRIM[leg][2];

    if (leg == LEG_FL || leg == LEG_RR)
    {
        out[0] = q.t1;
        out[1] = 90.0f + q.t2;
        out[2] = -q.t3;
    }
    else
    {
        out[0] = 180.0f - q.t1;
        out[1] = 90.0f - q.t2;
        out[2] = 180.0f + q.t3;
    }
}

static uint32_t Check_Angles(const float angles[4][3])
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (!isfinite(angles[leg][j]) || angles[leg][j] < LO[j] || angles[leg][j] > HI[j])
                return 4U;
        }
    }

    return 0U;
}

static uint32_t IK_Spider_4Leg(float out[4][3])
{
    float c = cosf(Body_Yaw);
    float s = sinf(Body_Yaw);
    for (int i = 0; i < 4; i++)
    {
        int leg = MODEL_TO_HW[i];
        // R(yaw)^T * (chan_dat - than_dat), roi Body_IK tru hong/mirror.
        float dx = Pfoot_r[i][0] - Pc_body[0];
        float dy = Pfoot_r[i][1] - Pc_body[1];
        float foot[3];
        foot[0] = c * dx + s * dy;
        foot[1] = -s * dx + c * dy;
        foot[2] = Pfoot_r[i][2] - Pc_body[2] + FOOT_Z_TRIM[leg];
        uint8_t ok = 0U;
        Joint q = Body_IK(foot, leg, &ok);

        if (!ok)
            return 3U;

        Map_Servo(leg, q, out[leg]);
        dbg_foot_z[leg] = foot[2];
    }
    return Check_Angles(out);
}

static uint32_t Check_Rate(const float current[4][3], const float previous[4][3], float dt)
{
    for (int leg = 0; leg < 4; leg++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (fabsf(current[leg][j] - previous[leg][j]) > MAX_RATE * dt + 0.001f)
                return 6U;
        }
    }

    return 0U;
}

// =====================================================================
// 4. THAM CHIEU XOAY TAI CHO VA PHA CHAN - CUNG CONG THUC TRONG SLX
// =====================================================================
static float Q5(float u)
{
    if (u <= 0.0f)
        return 0.0f;
    if (u >= 1.0f)
        return 1.0f;
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

static void Reference_Pose(float t, float p[3], float *yaw)
{
    // Hai dau giu nua chu ky: neo stance dau/cuoi cung la tu the trung tinh.
    p[0] = 0.0f;
    p[1] = 0.0f;
    p[2] = BODY_HEIGHT;
    float z = t / GAIT_T - 0.5f;
    float span = Rotate_Cycles - 1.0f;
    float target = ROTATE_DIR * Rotate_Angle_Deg * PI_F / 180.0f;
    if (z <= 0.0f)
    {
        *yaw = 0.0f;
        return;
    }
    if (z >= span)
    {
        *yaw = target;
        return;
    }

    // Tich phan van toc: nua-cosine 1 chu ky, doan deu, roi giam toc 1 chu ky.
    float progress;
    if (z < 1.0f)
        progress = 0.5f * (z - sinf(PI_F * z) / PI_F);
    else if (z <= span - 1.0f)
        progress = z - 0.5f;
    else
    {
        float remaining = span - z;
        progress = span - 1.0f - 0.5f * (remaining - sinf(PI_F * remaining) / PI_F);
    }
    *yaw = target * progress / (span - 1.0f);
}

static void Update_COM_Rotate(float t)
{
    // Pc la tham chieu than, KHONG la COM hay vi tri do duoc.
    Reference_Pose(t, Pc, &Body_Yaw);
    dbg_yaw_ref_deg = Body_Yaw * 180.0f / PI_F;
}

static void Foot_Anchor(int leg, float mid_t, float xy[2])
{
    // Neo chan theo tu the than tai GIUA stance; Reference_Pose giu hai dau.
    float p[3];
    float yaw;
    Reference_Pose(mid_t, p, &yaw);
    float c = cosf(yaw);
    float s = sinf(yaw);
    float x = HIP[leg][0] + ((HIP[leg][0] > 0.0f) ? STANCE_DX : -STANCE_DX);
    float y = HIP[leg][1] + ((HIP[leg][1] > 0.0f) ? STANCE_DY : -STANCE_DY);
    xy[0] = p[0] + c * x - s * y;
    xy[1] = p[1] + s * x + c * y;
}

static float Leg_Phase(float t, float T, int i)
{
    float s = t / T + PHI_LEG[i];
    s -= floorf(s);
    const float edges[5] = {0.0f, BETA_PREP, BETA_SWING, BETA_TOUCH, 1.0f};
    // SLX dung double voi 1e-12; C float can dung sai nho hon mot tick.
    for (int j = 0; j < 5; j++)
    {
        if (fabsf(s - edges[j]) < 0.000001f)
            s = edges[j];
    }
    if (s >= 1.0f)
        s = 0.0f;
    return s;
}

// =====================================================================
// 5. WPG_ROTATE_VER0 - GIU NGUYEN PRE-SHIFT / SWING / POST-RETURN
// =====================================================================
static void WPG_Rotate_Ver0(float T, float t)
{
    float beta = 1.0f - (BETA_TOUCH - BETA_SWING);
    float phase[4];
    for (int i = 0; i < 4; i++)
        phase[i] = Leg_Phase(t, T, i);

    int active = 0;
    uint32_t stage = ST_PRE_SHIFT;
    float shift_val = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float s = phase[i];
        if (s >= BETA_PREP)
        {
            active = i;
            if (s < BETA_SWING)
            {
                float u = (s - BETA_PREP) / (BETA_SWING - BETA_PREP);
                shift_val = 0.5f - 0.5f * cosf(PI_F * u);
                stage = ST_PRE_SHIFT;
            }
            else if (s < BETA_TOUCH)
            {
                shift_val = 1.0f;
                stage = ST_SWING;
            }
            else
            {
                float u = (s - BETA_TOUCH) / (1.0f - BETA_TOUCH);
                shift_val = 0.5f + 0.5f * cosf(PI_F * u);
                stage = ST_POST_RETURN;
            }
            break;
        }
    }

    float offset_x = DIR_X[active] * SHIFT_X_MM * shift_val;
    float offset_y = DIR_Y[active] * SHIFT_Y_MM * shift_val;
    float c = cosf(Body_Yaw);
    float sn = sinf(Body_Yaw);
    // Luong don giu nguyen trong he than; xoay cung yaw sang he dat.
    Pc_body[0] = Pc[0] - c * offset_x + sn * offset_y;
    Pc_body[1] = Pc[1] - sn * offset_x - c * offset_y;
    Pc_body[2] = Pc[2];
    dbg_gait_t = t;
    dbg_stage = stage;
    dbg_active_leg = MODEL_TO_HW[active];
    dbg_swing = -1;
    dbg_contact_mask = 0U;
    dbg_body_x = -offset_x;
    dbg_body_y = -offset_y;
    dbg_lift_mm = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        int leg = MODEL_TO_HW[i];
        float s = phase[i];
        float xy[2];
        float zs = 0.0f;
        if (s >= BETA_SWING && s < BETA_TOUCH)
        {
            float u = (s - BETA_SWING) / (BETA_TOUCH - BETA_SWING);
            float progress = u - sinf(2.0f * PI_F * u) / (2.0f * PI_F);
            float swing_T = (BETA_TOUCH - BETA_SWING) * T;
            float touchdown_t = t + (1.0f - u) * swing_T;
            float next_mid_t = touchdown_t + beta * T / 2.0f;
            float start_xy[2];
            float end_xy[2];
            Foot_Anchor(leg, next_mid_t - T, start_xy);
            Foot_Anchor(leg, next_mid_t, end_xy);
            xy[0] = start_xy[0] + (end_xy[0] - start_xy[0]) * progress;
            xy[1] = start_xy[1] + (end_xy[1] - start_xy[1]) * progress;
            float z_wave = sinf(PI_F * u);
            zs = LIFT_MM * z_wave * z_wave;
            contact[i] = 0U;
            dbg_swing = (int8_t)leg;
            dbg_lift_mm = zs;
        }
        else
        {
            float tau = (s >= BETA_TOUCH) ? s - BETA_TOUCH : 1.0f - BETA_TOUCH + s;
            float mid_t = t - tau * T + beta * T / 2.0f;
            Foot_Anchor(leg, mid_t, xy);
            contact[i] = 1U;
            dbg_contact_mask |= 1U << leg;
        }

        // Hong theo than DA don va quay. Neo stance CO DINH trong he dat.
        // Trong he than, chan tru van doi XY theo cung mot chuyen dong than.
        // LEFT doi dau yaw va doi xung lich pha; cong thuc neo giu nguyen.
        Phip_r[i][0] = Pc_body[0] + c * HIP[leg][0] - sn * HIP[leg][1];
        Phip_r[i][1] = Pc_body[1] + sn * HIP[leg][0] + c * HIP[leg][1];
        Phip_r[i][2] = Pc_body[2];
        Pfoot_r[i][0] = xy[0];
        Pfoot_r[i][1] = xy[1];
        Pfoot_r[i][2] = zs;
    }
}

// =====================================================================
// 6. HOME -> TU THE BAT DAU -> CHO DAT -> DI -> GIU, KIEM TRA TRUOC PWM
// =====================================================================
static uint32_t Motion(float t, float out[4][3])
{
    dbg_swing = -1;
    dbg_lift_mm = 0.0f;
    if (t < HOME_T)
    {
        dbg_stage = ST_HOME;
        float u = Q5(t / HOME_T);
        for (int leg = 0; leg < 4; leg++)
        {
            for (int j = 0; j < 3; j++)
                out[leg][j] = 90.0f + (home_angle[leg][j] - 90.0f) * u;
        }
    }
    else if (t < HOME_T + POSE_T)
    {
        dbg_stage = ST_POSE;
        float u = Q5((t - HOME_T) / POSE_T);
        for (int leg = 0; leg < 4; leg++)
        {
            for (int j = 0; j < 3; j++)
                out[leg][j] = home_angle[leg][j] + (pose_angle[leg][j] - home_angle[leg][j]) * u;
        }
    }
    else if (RUN_MODE == MODE_STAND)
    {
        dbg_stage = ST_STAND;
        memcpy(out, pose_angle, sizeof(pose_angle));
    }
    else if (t < HOME_T + POSE_T + READY_T)
    {
        dbg_stage = ST_READY;
        memcpy(out, pose_angle, sizeof(pose_angle));
    }
    else
    {
        float gait_t = t - HOME_T - POSE_T - READY_T;
        float duration = Rotate_Cycles * GAIT_T;
        if (gait_t > duration)
            gait_t = duration;
        Update_COM_Rotate(gait_t);
        WPG_Rotate_Ver0(GAIT_T, gait_t);
        if (gait_t >= duration)
            dbg_stage = ST_DONE;
        return IK_Spider_4Leg(out);
    }
    return Check_Angles(out);
}

static uint32_t Prepare(void)
{
    dbg_preflight = 0U;
    // Duty co dinh; lich pha LEFT doi xung RIGHT theo PHI_LEG o tren.
    if (!isfinite(GAIT_T) || GAIT_T <= 0.0f || (N_CYCLES != 0U && N_CYCLES < 4U) || N_CYCLES > 200U ||
        !isfinite(ANGLE_DEG) || ANGLE_DEG <= 0.0f || ANGLE_DEG > ANGLE_LIMIT_DEG ||
        !isfinite(CAL_COMMAND_DEG) || CAL_COMMAND_DEG <= 0.0f || CAL_COMMAND_DEG > ANGLE_LIMIT_DEG ||
        !isfinite(CAL_MEASURED_DEG) || CAL_MEASURED_DEG <= 0.0f ||
        !isfinite(MAX_YAW_PER_CYCLE_DEG) || MAX_YAW_PER_CYCLE_DEG <= 0.0f ||
        !isfinite(LIFT_MM) || LIFT_MM <= 0.0f ||
        !isfinite(SHIFT_X_MM) || SHIFT_X_MM < 0.0f || !isfinite(SHIFT_Y_MM) || SHIFT_Y_MM < 0.0f ||
        BETA_PREP != 0.75f || BETA_SWING != 0.80f || BETA_TOUCH != 0.92f)
        return 1U;

    // Bu vong ho: dung goc LENH thuc su cua luot do, khong nhan bu lap lai.
    float gain = CAL_COMMAND_DEG / CAL_MEASURED_DEG;
    float command_deg = ANGLE_DEG * gain;
    if (!isfinite(gain) || !isfinite(command_deg) || command_deg <= 0.0f || command_deg > ANGLE_LIMIT_DEG)
        return 1U;

    // Chia tong goc thanh cac buoc vua: cung cong thuc ramp co mau so N-2.
    float required = ceilf(command_deg / MAX_YAW_PER_CYCLE_DEG) + 2.0f;
    if (!isfinite(required) || required > 200.0f)
        return 1U;
    if (required < 4.0f)
        required = 4.0f;
    uint32_t minimum_cycles = (uint32_t)required;
    uint32_t cycles = (N_CYCLES == 0U) ? minimum_cycles : N_CYCLES;
    if (cycles < minimum_cycles)
        return 1U;

    Rotate_Angle_Deg = command_deg;
    Rotate_Cycles = cycles;
    dbg_goal_deg = ROTATE_DIR * ANGLE_DEG;
    dbg_yaw_gain = gain;
    dbg_target_deg = ROTATE_DIR * command_deg;
    dbg_cycles = cycles;
    dbg_cycle_yaw_deg = command_deg / (cycles - 2.0f);
    dbg_duration_s = cycles * GAIT_T;

    for (int leg = 0; leg < 4; leg++)
    {
        Joint home = {90.0f, 0.0f, -90.0f};
        Map_Servo(leg, home, home_angle[leg]);
    }
    uint32_t error = Check_Angles(home_angle);
    if (error)
        return error;

    Update_COM_Rotate(0.0f);
    WPG_Rotate_Ver0(GAIT_T, 0.0f);
    error = IK_Spider_4Leg(pose_angle);
    if (error)
        return error;

    float previous[4][3];
    float current[4][3];
    error = Motion(0.0f, previous);
    if (error)
        return error;
    float duration = HOME_T + POSE_T + READY_T + Rotate_Cycles * GAIT_T + 1.0f;
    int samples = (int)ceilf(duration * 1000.0f / TICK_MS);
    for (int k = 1; k <= samples; k++)
    {
        error = Motion(k * TICK_MS * 0.001f, current);
        if (error)
            return error;
        error = Check_Rate(current, previous, TICK_MS * 0.001f);
        if (error)
            return error;
        memcpy(previous, current, sizeof(previous));
    }
    // Tra ca debug ve t=0 sau khi quet, truoc khi bat xung dau tien.
    Update_COM_Rotate(0.0f);
    WPG_Rotate_Ver0(GAIT_T, 0.0f);
    error = IK_Spider_4Leg(current);
    if (error)
        return error;
    dbg_preflight = 1U;
    return Motion(0.0f, current);
}

// =====================================================================
// 7. PCA9685 - CUNG PWM/TOAN BO KHUNG XUNG, KIEM TRA LOI I2C
// =====================================================================
/* HARDWARE: host verification extracts the mathematical core above. */
static HAL_StatusTypeDef Reg_Write(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, 50U);
}

static void Fatal(uint32_t code)
{
    dbg_fault = code;
    dbg_ready = 0U;

    // FULL_OFF tung kenh khong can auto-increment; bus hong co the
    // khong nhan duoc lenh OFF. Can do than/ngat nguon servo bang tay.
    for (int channel = 0; channel < 16; channel++)
        (void)Reg_Write((uint8_t)(PCA9685_LED0_OFF_H + 4 * channel), 0x10U);

    dbg_output_on = 0U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100U);
    }
}

static void All_Off(void)
{
    uint8_t frame[64] = {0};

    for (int channel = 0; channel < 16; channel++)
        frame[4 * channel + 3] = 0x10U;

    if (HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_LED0_ON_L, 1U, frame, 64U, 50U) != HAL_OK)
        Fatal(102U);

    memcpy(last_frame, frame, sizeof(frame));
    dbg_output_on = 0U;
}

static void Verify_PCA(void)
{
    uint8_t prescale = 0U;
    uint8_t mode1 = 0U;
    uint8_t mode2 = 0U;
    uint8_t frame[64];

    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_PRESCALE, 1U, &prescale, 1U, 50U) != HAL_OK)
        Fatal(110U);

    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_MODE1, 1U, &mode1, 1U, 50U) != HAL_OK)
        Fatal(110U);

    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, 0x01U, 1U, &mode2, 1U, 50U) != HAL_OK)
        Fatal(110U);

    dbg_prescale = prescale;
    dbg_mode1 = mode1;

    if (prescale != PRESCALE_FIXED || (mode1 & 0x70U) != 0x20U || mode2 != 0x04U)
        Fatal(111U);

    if (HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, PCA9685_LED0_ON_L, 1U, frame, 64U, 50U) != HAL_OK)
        Fatal(110U);

    if (memcmp(frame, last_frame, sizeof(frame)) != 0)
        Fatal(113U);

    dbg_verify_count++;
    // Khong ghi ready=1 o day: doc PCA thanh cong khong xoa loi gait.
}

static void Configure_PCA(void)
{
    if (Reg_Write(PCA9685_MODE1, 0x30U) != HAL_OK)
        Fatal(101U);

    All_Off();

    if (Reg_Write(PCA9685_PRESCALE, PRESCALE_FIXED) != HAL_OK)
        Fatal(103U);

    if (Reg_Write(0x01U, 0x04U) != HAL_OK)
        Fatal(104U);

    if (Reg_Write(PCA9685_MODE1, 0x20U) != HAL_OK)
        Fatal(105U);

    HAL_Delay(1U);

    if (Reg_Write(PCA9685_MODE1, 0xA0U) != HAL_OK)
        Fatal(106U);

    HAL_Delay(10U);
    Verify_PCA();
}

static void Write_Frame(const float angles[4][3])
{
    uint8_t frame[64] = {0};
    uint16_t ticks[4][3];

    for (int channel = 0; channel < 16; channel++)
        frame[4 * channel + 3] = 0x10U;

    for (int leg = 0; leg < 4; leg++)
    {
        for (int j = 0; j < 3; j++)
        {
            float count = (angles[leg][j] - MIN_ANGLE) * (SERVO_MAX - SERVO_MIN)
                        / (MAX_ANGLE - MIN_ANGLE) + SERVO_MIN;
            ticks[leg][j] = (uint16_t)count;
            int index = 4 * CH[leg][j];
            frame[index + 2] = (uint8_t)ticks[leg][j];
            frame[index + 3] = (uint8_t)(ticks[leg][j] >> 8);
        }
    }

    // Cung mot I2C frame cho 12 khop; MODE2 cap nhat khi STOP.
    if (memcmp(frame, last_frame, sizeof(frame)) != 0)
    {
        if (HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_LED0_ON_L, 1U, frame, 64U, 50U) != HAL_OK)
            Fatal(112U);

        memcpy(last_frame, frame, sizeof(frame));
    }

    for (int leg = 0; leg < 4; leg++)
    {
        for (int j = 0; j < 3; j++)
        {
            dbg_servo[leg][j] = angles[leg][j];
            dbg_ticks[leg][j] = ticks[leg][j];
        }
    }

    dbg_output_on = 1U;
}

static void Update_LEDs(uint32_t now)
{
    uint8_t blink = (now % 1000U < 500U);
    uint8_t done = (dbg_stage == ST_DONE || dbg_stage == ST_STAND);
    uint8_t on = (done || blink);

    if (dbg_fault)
        on = (now % 200U < 100U);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// =====================================================================
// 8. MAIN VA NGOAI VI STM32F446RE - PROJECT SERVOAN / SYSTICK
// =====================================================================
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    Configure_PCA();

    dbg_fault = Prepare();
    dbg_ready = (dbg_fault == 0U);

    uint32_t start = HAL_GetTick();
    uint32_t last = start;
    uint32_t verified = start;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - last;

        if (elapsed < TICK_MS)
            continue;

        last = now;
        dbg_alive++;

        if (emergency_stop && !dbg_fault)
            dbg_fault = 8U;

        if (elapsed > 2U * TICK_MS && !dbg_fault)
            dbg_fault = 7U;

        if (!dbg_fault)
        {
            float angles[4][3];
            dbg_fault = Motion((now - start) * 0.001f, angles);

            if (!dbg_fault && dbg_output_on)
                dbg_fault = Check_Rate(angles, last_angle, elapsed * 0.001f);

            if (!dbg_fault)
            {
                Write_Frame(angles);
                memcpy(last_angle, angles, sizeof(angles));
            }
        }

        if (dbg_fault)
            dbg_ready = 0U;

        if (now - verified >= 1000U)
        {
            Verify_PCA();
            verified = HAL_GetTick();
        }

        Update_LEDs(now);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    osc.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
        Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
}

// HAL tick 1ms do SysTick_Handler cua project F446 cap nhat.

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
