/* USER CODE BEGIN Header */
/*
  VER0 UART - DUNG / DI, NHAN TRIM TAI RANH GIOI CHU KY.
  Copy toan bo vao main.c cua Servoan2 (NUCLEO-F446RE).
  Giu main.h, HAL/MSP va pca9685.c/.h; don vi mm, s, deg.

  HOME 4s -> POSE 6s -> READY 12s -> WAIT_START.
  MODE_WALK chi di khi nhan G,1; N=4, STEP40, T12, trim nen +3.
  MODE_STAND chi dung, khong nhan G,1. Ke than khi HOME/POSE.
  Chu ky dau dung trim nen; trim moi vao tu chu ky hai, doi toi da 2 deg.
  Noi cung theo tu the cuoi doan; chan tru giu neo trong he dat ly tuong.
  G,0: het nhip chan hien tai (toi da GAIT_T/4) thi giu tu the, khong tat PWM.
  STOPPED nhan G,1 de tiep tuc phan con lai; DONE can reset cho luot moi.
  Loi IK/goc/rate/I2C van xu ly nhu cu, khong phai dung mem theo pha.

  UART2 PA2/PA3, 115200 8N1 qua ST-LINK. Can bat USART2 + ngat NVIC.
  Khung: $seq,LOAI,gia_tri*CS; CS XOR phan giua $ va *, hai ky tu hex.
  T: trim phan nghin do/chu ky [-5000,10000]; P/Q: gia tri 0.
  G: 1 bat dau khi WAIT_START, 0 yeu cau dung. G,1 lap khong reset gait.
  T la muc tieu tuyet doi, khong cong don; seq dung doi chieu ACK.
  ACK: 0 chap nhan; 1 cu phap; 2 checksum; 3 lenh la; 4 ngoai mien;
  5 dang loi; 6 trang thai khong cho phep/khong con chu ky ap trim.
  T duoc chap nhan khong co nghia da ap ngay; xem trim tren telemetry.
  D: gait_t,x,y,yaw,trim,stage,fault,alive,build,N,target; trim la gia tri dang ap.
  RX/ACK hang doi 7 dong; main xu ly mot dong/tick; TX IT, ACK uu tien.
  PC gui tung lenh doi ACK, toi da 10 lenh/s; RX loi gui CR/LF roi thu lai.

  Mat link HOAC 3s khong nhan T hop le trong WALK: yeu cau dung theo pha.
  P/Q chi nuoi link, KHONG nuoi tuoi lenh T. G,1 cho 3s gui T dau tien.
  Qua han -> STOPPING -> STOPPED; can G,1 chu dong de tiep tuc sau khi dung.
  Camera do PC quan ly: mat thoang qua gui lai trim DANG AP DUNG;
  qua nguong mat anh gui G,0/ngung T; khong gui lai hieu chinh tu anh cu.
  Console thu tay gia han T moi giay; khong dung co che nay cho camera.
  Timeout la 3s de PHAT HIEN; robot con hoan tat nhip chan toi da 3s.

  Giu IK, hieu chuan servo, gioi han goc/rate va preflight.
  WPG MODEL FL/FR/RR/RL; PWM HW FL/RL/RR/FR. dbg_* la LENH, khong phai cam bien.
  Servo nguon ngoai; USB nuoi bo/logic. Loi I2C yeu cau OFF, can do than.
  Theo doi dbg_tick_exec_max_ms / dbg_tick_gap_max_ms khi thu tren bo.
*/
/* USER CODE END Header */
#include "main.h"
#include "pca9685.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if !defined(STM32F446xx)
#error "Can project STM32F446RE; khong dung project STM32F407"
#endif

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// =====================================================================
// 1. KHAI BAO, HINH HOC, THONG SO GAIT VA HIEU CHUAN
// =====================================================================
#define MODE_STAND       0U
#define MODE_WALK        1U
#ifndef RUN_MODE
#define RUN_MODE         MODE_WALK
#endif
#if RUN_MODE != MODE_STAND && RUN_MODE != MODE_WALK
#error "RUN_MODE chi nhan MODE_STAND hoac MODE_WALK"
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
#define STEP_MM         40.0f
#define GAIT_V          (STEP_MM / GAIT_T)
#define LIFT_MM         30.0f
#define SHIFT_X_MM       5.0f
#define SHIFT_Y_MM      20.0f
#define N_CYCLES         4U
// Bu quay CHUNG moi chu ky: duong = trai (+Y), am = phai (-Y).
// Khong doi tham so nay bang Live Expressions khi robot dang di.
#ifndef YAW_TRIM_DEG_PER_CYCLE
#define YAW_TRIM_DEG_PER_CYCLE  3.0f
#endif
#define BETA_PREP        0.75f
#define BETA_SWING       0.80f
#define BETA_TOUCH       0.92f

#define HOME_T           4.0f
#define POSE_T           6.0f
#define READY_T         12.0f
#define TICK_MS         20U
// Chan loi lenh, KHONG la rate limiter; Ver0 can khoang 65 deg/s.
#define MAX_RATE        80.0f
#define PRESCALE_FIXED 121U
#define PI_F             3.14159265358979323846f

// UART: khung toi da 96 ky tu; telemetry 5 tick mot lan = 100ms.
#define UART_LINE_MAX   96
#define UART_QUEUE_SIZE  8U
#define TELE_EVERY       5U
#define LINK_TIMEOUT_MS 3000U
#define CONTROL_TIMEOUT_MS 3000U
// Mien lenh trim; COXA 15..170 da mo phong, can kiem chung co khi thuc.
#define TRIM_LIMIT      10.0f
#define TRIM_MIN        -5.0f
// trim ap dung doi toi da bao nhieu do/chu ky tai moi ranh gioi.
#define TRIM_STEP_MAX    3.0f

_Static_assert(SERVO_MIN == 110 && SERVO_MAX == 500, "Can count 110..500 nhu ve base");
_Static_assert(PCA9685_ADDRESS == 0x80, "Can PCA address 0x80 theo HAL");

// Bang theo PHAN CUNG: FL, RL, RR, FR; ban walking 8 chu ky da doi chieu.
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
static const float LO[3] = {15.0f, 30.0f, 30.0f};
static const float HI[3] = {170.0f, 150.0f, 150.0f};

// Bang theo MODEL: FL, FR, RR, RL;
static const uint8_t MODEL_TO_HW[4] = {LEG_FL, LEG_FR, LEG_RR, LEG_RL};
static const float PHI_LEG[4] = {0.75f, 0.50f, 0.25f, 0.00f};
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
    ST_STAND = 7,
    ST_WAIT_START = 8,
    ST_STOPPING = 9,
    ST_STOPPED = 10
};

// Loi: 1param, 3IK, 4angle, 6rate, 7late_tick, 8emergency_stop;
// 101..113 I2C/PCA. Loi latch den khi reset; khong tu bo qua/bo pha.
volatile uint32_t emergency_stop = 0U;
volatile uint32_t dbg_build_id = 202609230U;
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
volatile float dbg_yaw_trim_deg_per_cycle = YAW_TRIM_DEG_PER_CYCLE; // trim dang ap dung
volatile float dbg_yaw_ref_deg = 0.0f; // yaw LENH, khong phai goc thuc do duoc
volatile float dbg_body_x = 0.0f;     // dich them trong HE THAN, khong gom tien/quay
volatile float dbg_body_y = 0.0f;
volatile float dbg_lift_mm = 0.0f;
volatile float dbg_foot_z[4] = {0.0f}; // lenh tuong doi than, da cong FOOT_Z_TRIM
volatile float dbg_servo[4][3] = {{0.0f}};
volatile uint16_t dbg_ticks[4][3] = {{0}};

// Trang thai UART. dbg_trim_cmd la trim may tinh gui xuong, cho ranh gioi
// chu ky; dbg_trim_applied dem so lan trim moi da vao gait.
volatile float dbg_trim_cmd = YAW_TRIM_DEG_PER_CYCLE;
volatile uint32_t dbg_trim_applied = 0U;
volatile uint32_t dbg_rx_ok = 0U;
volatile uint32_t dbg_rx_bad = 0U;
volatile uint32_t dbg_link_up = 0U;
volatile uint32_t dbg_link_lost = 0U;
volatile uint32_t dbg_ack_sent = 0U;
volatile uint32_t dbg_tx_error = 0U;
volatile uint32_t dbg_tele_skip = 0U;
volatile uint32_t dbg_rx_queue_full = 0U;
volatile uint32_t dbg_rx_discard = 0U;
volatile uint32_t dbg_uart_error = 0U;
volatile uint32_t dbg_uart_last_error = 0U;
volatile uint32_t dbg_rx_arm_error = 0U;
volatile uint32_t dbg_rx_recovered = 0U;
volatile uint32_t dbg_tick_exec_max_ms = 0U;
volatile uint32_t dbg_tick_gap_max_ms = 0U;

// Pc, Phip_r, Pfoot_r la he dat LY TUONG, don vi mm.
static float Pc[3];
static float Body_Yaw = 0.0f; // rad, dung chung cho than/hong/chan/IK
static float Pc_body[3];
static float Phip_r[4][3];
static float Pfoot_r[4][3];
static uint8_t contact[4];
// doan tham chieu hien tai bat dau o Seg_T0 voi tu the Seg_P (x,y,yaw).
// Seg_W: yaw rad/chu ky cua doan; Seg_Next: ranh gioi chu ky ke tiep, s gait.
static float Seg_T0 = 0.0f;
static float Seg_P[3] = {0.0f, 0.0f, 0.0f};
static float Seg_W = YAW_TRIM_DEG_PER_CYCLE * PI_F / 180.0f;
static float Seg_Next = GAIT_T;
// Neo chan da chot trong he dat va moc giua stance cua neo do (MODEL i).
static float Anchor[4][2];
static float Anchor_Mid[4] = {-1.0e9f, -1.0e9f, -1.0e9f, -1.0e9f};
// Dung/tiep tuc giu pha va neo chan; DONE can reset cho luot moi.
static uint8_t start_requested = 0U, walk_started = 0U, walk_finished = 0U;
static uint8_t stop_requested = 0U, resume_requested = 0U;
static float walk_start_t = 0.0f, walk_stop_t = -1.0f;
static uint32_t control_last = 0U;
volatile uint32_t dbg_stop_reason = 0U; // 1 lenh dung, 2 mat link, 3 lenh T qua han
static float home_angle[4][3];
static float pose_angle[4][3];
static float last_angle[4][3];
static uint8_t last_frame[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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
// 4. THAM CHIEU THAN, BU HUONG CHUNG VA PHA CHAN
// =====================================================================
static float Q5(float u)
{
    if (u <= 0.0f)
        return 0.0f;
    if (u >= 1.0f)
        return 1.0f;
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

static float Sinc(float a)
{
    // sin(a)/a; tranh chia 0 va mat chinh xac khi bu huong rat nho.
    if (fabsf(a) < 0.01f)
    {
        float a2 = a * a;
        return 1.0f - a2 / 6.0f + a2 * a2 / 120.0f;
    }
    return sinf(a) / a;
}

static void Reference_Pose(float v, float T, float t, float p[3], float *yaw)
{
    // Cung tron cua doan hien tai, noi tiep tu the Seg_P tai Seg_T0.
    // t truoc Seg_T0 hay sau Seg_Next deu ngoai suy cung do cong doan nay.
    float dt = t - Seg_T0;
    float a = Seg_W * (dt / T);
    float half_sinc = Sinc(0.5f * a);
    // Tuong duong v/w * [sin(wt), 1-cos(wt)], on dinh ca khi w=0.
    float fx = v * dt * Sinc(a);
    float fy = v * dt * (0.5f * a) * half_sinc * half_sinc;
    float c = cosf(Seg_P[2]);
    float s = sinf(Seg_P[2]);
    p[0] = Seg_P[0] + c * fx - s * fy;
    p[1] = Seg_P[1] + s * fx + c * fy;
    p[2] = BODY_HEIGHT;
    *yaw = Seg_P[2] + a;
}

static void Gait_Reset(void)
{
    // Ve doan dau: goc toa do, trim bien dich; xoa moi neo da chot.
    Seg_T0 = 0.0f;
    Seg_P[0] = 0.0f;
    Seg_P[1] = 0.0f;
    Seg_P[2] = 0.0f;
    Seg_W = YAW_TRIM_DEG_PER_CYCLE * PI_F / 180.0f;
    Seg_Next = GAIT_T;
    dbg_yaw_trim_deg_per_cycle = YAW_TRIM_DEG_PER_CYCLE;
    for (int i = 0; i < 4; i++)
        Anchor_Mid[i] = -1.0e9f;
}

static void Trim_Commit(float boundary)
{
    // Chi goi tai ranh gioi chu ky: khong chan nao swing, neo tru da chot.
    float now = dbg_yaw_trim_deg_per_cycle;
    float step = dbg_trim_cmd - now;
    if (step > TRIM_STEP_MAX)
        step = TRIM_STEP_MAX;
    if (step < -TRIM_STEP_MAX)
        step = -TRIM_STEP_MAX;
    if (fabsf(step) < 0.0005f)
        return; // Khong doi: giu doan cu.
    float p[3];
    float yaw;
    Reference_Pose(GAIT_V, GAIT_T, boundary, p, &yaw);
    Seg_T0 = boundary;
    Seg_P[0] = p[0];
    Seg_P[1] = p[1];
    Seg_P[2] = yaw;
    Seg_W = (now + step) * PI_F / 180.0f;
    dbg_yaw_trim_deg_per_cycle = now + step;
    dbg_trim_applied++;
}

static void Update_COM_Line(float v, float t)
{
    // Pc la tham chieu than, KHONG la COM hay vi tri do duoc.
    Reference_Pose(v, GAIT_T, t, Pc, &Body_Yaw);
    dbg_yaw_ref_deg = Body_Yaw * 180.0f / PI_F;
}

static void Foot_Anchor(int leg, float v, float T, float mid_t, float xy[2])
{
    // Neo chan theo tu the than tai GIUA stance. mid_t co the am o t=0.
    // Day la moc toan hoc; khong clamp ve 0 hay ve thoi diem ket thuc.
    float p[3];
    float yaw;
    Reference_Pose(v, T, mid_t, p, &yaw);
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
// 5. WPG_LINE_VER0 - GIU NGUYEN PRE-SHIFT / SWING / POST-RETURN
// =====================================================================
static void WPG_Line_Ver0(float v, float T, float t)
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
            // Diem nhac = neo da chot cua stance vua xong; diem dat tinh theo
            // doan hien tai, se duoc chot o tick cham dat dau tien.
            if (fabsf(Anchor_Mid[i] - (next_mid_t - T)) < 0.01f)
                memcpy(start_xy, Anchor[i], sizeof(start_xy));
            else
                Foot_Anchor(leg, v, T, next_mid_t - T, start_xy);
            Foot_Anchor(leg, v, T, next_mid_t, end_xy);
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
            // mid_t khong doi suot mot stance; stance moi thi chot neo moi.
            if (fabsf(Anchor_Mid[i] - mid_t) >= 0.01f)
            {
                Foot_Anchor(leg, v, T, mid_t, Anchor[i]);
                Anchor_Mid[i] = mid_t;
            }
            xy[0] = Anchor[i][0];
            xy[1] = Anchor[i][1];
            contact[i] = 1U;
            dbg_contact_mask |= 1U << leg;
        }

        // Hong theo than DA don va quay. Neo stance CO DINH trong he dat.
        // Trong he than, chan tru van doi XY theo cung mot chuyen dong than.
        // Khi yaw trim=0, cac neo va cycloid nay tro ve dung Ver0 cu.
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
static uint32_t Motion_Path(float t, float out[4][3])
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
        float duration = N_CYCLES * GAIT_T;
        if (gait_t > duration)
            gait_t = duration;
        // qua ranh gioi chu ky thi nhan trim moi truoc khi tinh tham chieu.
        while (gait_t >= Seg_Next && Seg_Next < duration)
        {
            if (!stop_requested)
                Trim_Commit(Seg_Next);
            Seg_Next += GAIT_T;
        }
        Update_COM_Line(GAIT_V, gait_t);
        WPG_Line_Ver0(GAIT_V, GAIT_T, gait_t);
        if (gait_t >= duration)
            dbg_stage = ST_DONE;
        return IK_Spider_4Leg(out);
    }
    return Check_Angles(out);
}

static void Motion_RequestStop(uint32_t reason)
{
    start_requested = 0U;
    resume_requested = 0U;
    if (!walk_started || walk_finished || stop_requested)
        return;
    stop_requested = 1U;
    dbg_stop_reason = reason;
    dbg_trim_cmd = dbg_yaw_trim_deg_per_cycle; // Bo muc tieu dang cho/tang dan.
}

static uint32_t Motion(float t, float out[4][3])
{
    float ready_end = HOME_T + POSE_T + READY_T;
    if (RUN_MODE == MODE_STAND || t < ready_end)
        return Motion_Path(t, out);
    if (!walk_started)
    {
        if (!start_requested)
        {
            dbg_stage = ST_WAIT_START;
            memcpy(out, pose_angle, sizeof(pose_angle));
            return Check_Angles(out);
        }
        start_requested = 0U;
        walk_started = 1U;
        walk_start_t = t; // Thoi gian doi lenh khong tinh vao gait.
    }
    if (resume_requested)
    {
        resume_requested = 0U;
        walk_start_t = t - walk_stop_t; // Bo thoi gian dung, giu nguyen pha/neo.
        stop_requested = 0U;
        walk_finished = 0U;
        walk_stop_t = -1.0f;
        dbg_stop_reason = 0U;
        dbg_trim_cmd = dbg_yaw_trim_deg_per_cycle;
    }
    float gait_t = t - walk_start_t;
    if (stop_requested && walk_stop_t < 0.0f)
    {
        // Tinh tu thoi gian tick HIEN TAI, khong tu dbg_gait_t cua tick truoc.
        float leg_T = GAIT_T / 4.0f;
        walk_stop_t = ceilf(gait_t / leg_T) * leg_T;
        if (walk_stop_t > N_CYCLES * GAIT_T)
            walk_stop_t = N_CYCLES * GAIT_T;
    }
    if (stop_requested && gait_t >= walk_stop_t)
        gait_t = walk_stop_t;
    uint32_t error = Motion_Path(ready_end + gait_t, out);
    if (gait_t >= N_CYCLES * GAIT_T)
    {
        dbg_stage = ST_DONE;
        walk_finished = 1U;
    }
    else if (stop_requested)
    {
        dbg_stage = (gait_t >= walk_stop_t) ? ST_STOPPED : ST_STOPPING;
        if (dbg_stage == ST_STOPPED)
            walk_finished = 1U;
    }
    else if (dbg_stage == ST_DONE)
        walk_finished = 1U;
    return error;
}

static uint32_t Prepare(void)
{
    dbg_preflight = 0U;
    // Pha nay duoc port co dinh tu Ver0, khong ho tro thay doi duty/order ngam.
    if (!isfinite(GAIT_T) || GAIT_T <= 0.0f || N_CYCLES < 1U || N_CYCLES > 40U ||
        !isfinite(STEP_MM) || STEP_MM <= 0.0f || !isfinite(LIFT_MM) || LIFT_MM <= 0.0f ||
        !isfinite(SHIFT_X_MM) || SHIFT_X_MM < 0.0f || !isfinite(SHIFT_Y_MM) || SHIFT_Y_MM < 0.0f ||
        !isfinite(YAW_TRIM_DEG_PER_CYCLE) || (YAW_TRIM_DEG_PER_CYCLE < TRIM_MIN || YAW_TRIM_DEG_PER_CYCLE > TRIM_LIMIT) ||
        BETA_PREP != 0.75f || BETA_SWING != 0.80f || BETA_TOUCH != 0.92f)
        return 1U;

    for (int leg = 0; leg < 4; leg++)
    {
        Joint home = {90.0f, 0.0f, -90.0f};
        Map_Servo(leg, home, home_angle[leg]);
    }
    uint32_t error = Check_Angles(home_angle);
    if (error)
        return error;

    Gait_Reset();
    Update_COM_Line(GAIT_V, 0.0f);
    WPG_Line_Ver0(GAIT_V, GAIT_T, 0.0f);
    error = IK_Spider_4Leg(pose_angle);
    if (error)
        return error;

    float previous[4][3];
    float current[4][3];
    error = Motion_Path(0.0f, previous);
    if (error)
        return error;
    float duration = HOME_T + POSE_T + READY_T + N_CYCLES * GAIT_T + 1.0f;
    int samples = (int)ceilf(duration * 1000.0f / TICK_MS);
    for (int k = 1; k <= samples; k++)
    {
        error = Motion_Path(k * TICK_MS * 0.001f, current);
        if (error)
            return error;
        error = Check_Rate(current, previous, TICK_MS * 0.001f);
        if (error)
            return error;
        memcpy(previous, current, sizeof(previous));
    }
    // Tra ca debug ve t=0 sau khi quet, truoc khi bat xung dau tien.
    // Quet chi kiem trim bien dich; trim UART chi vao tu chu ky thu hai.
    Gait_Reset();
    Update_COM_Line(GAIT_V, 0.0f);
    WPG_Line_Ver0(GAIT_V, GAIT_T, 0.0f);
    error = IK_Spider_4Leg(current);
    if (error)
        return error;
    dbg_preflight = 1U;
    return Motion_Path(0.0f, current);
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
// 8. UART2 - KHUNG LENH, ACK VA TELEMETRY; KHONG CHAN VONG LAP
// =====================================================================
static uint8_t rx_byte;
static char rx_buf[UART_LINE_MAX];
static uint16_t rx_len = 0U;
static uint8_t rx_discard = 0U;
// RX: ngat ghi head, main doc tail. De trong mot o de phan biet day/rong.
static char rx_queue[UART_QUEUE_SIZE][UART_LINE_MAX];
static volatile uint8_t rx_head = 0U, rx_tail = 0U;
static volatile uint8_t rx_recover = 0U;
// ACK: main ghi head, ngat TX hoan tat moi bo phan tu tai tail.
static char ack_queue[UART_QUEUE_SIZE][UART_LINE_MAX];
static volatile uint8_t ack_head = 0U, ack_tail = 0U;
static char tx_buf[UART_LINE_MAX];
static char tx_body[UART_LINE_MAX];
static volatile uint8_t tx_busy = 0U, tx_is_ack = 0U;
static uint32_t link_last = 0U;
static uint32_t tele_count = 0U;

static uint8_t Uart_Next(uint8_t index)
{
    return (uint8_t)((index + 1U) % UART_QUEUE_SIZE);
}

static uint8_t Checksum(const char *s)
{
    uint8_t c = 0U;
    while (*s)
        c ^= (uint8_t)(*s++);
    return c;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2 || !tx_busy)
        return;
    if (tx_is_ack)
    {
        ack_tail = Uart_Next(ack_tail);
        dbg_ack_sent++;
    }
    tx_is_ack = 0U;
    __DMB();
    tx_busy = 0U;
}

static uint8_t Uart_Send(const char *body, uint8_t is_ack)
{
    if (tx_busy)
        return 0U;
    int n = snprintf(tx_buf, sizeof(tx_buf), "$%s*%02X\r\n", body, Checksum(body));
    if (n <= 0 || n >= (int)sizeof(tx_buf))
    {
        dbg_tx_error++;
        return 0U;
    }
    tx_is_ack = is_ack;
    tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx_buf, (uint16_t)n) != HAL_OK)
    {
        tx_busy = 0U;
        tx_is_ack = 0U;
        dbg_tx_error++;
        return 0U; // ACK van nam trong hang doi, thu lai o tick sau.
    }
    return 1U;
}

static void Uart_Ack(long seq, int code)
{
    // Chi goi tu Poll khi da kiem tra con o trong; moi lenh tao mot ACK.
    snprintf(ack_queue[ack_head], UART_LINE_MAX, "%ld,A,%d", seq, code);
    __DMB();
    ack_head = Uart_Next(ack_head);
}

static void Uart_StartRx(void)
{
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK)
    {
        dbg_rx_arm_error++;
        rx_recover = 1U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;
    if (huart->ErrorCode != HAL_UART_ERROR_NONE)
    {
        rx_recover = 1U;
        return; // Khong dua byte dang bao loi vao hang doi lenh.
    }
    char c = (char)rx_byte;
    if (c == '\n' || c == '\r')
    {
        if (rx_len > 0U && !rx_discard)
        {
            uint8_t next = Uart_Next(rx_head);
            if (next != rx_tail)
            {
                memcpy(rx_queue[rx_head], rx_buf, rx_len);
                rx_queue[rx_head][rx_len] = '\0';
                __DMB();
                rx_head = next;
            }
            else
                dbg_rx_queue_full++;
        }
        rx_len = 0U;
        rx_discard = 0U;
    }
    else if (!rx_discard)
    {
        if (c == '\0' || rx_len >= UART_LINE_MAX - 1U)
        {
            rx_discard = 1U; // Bo HET dong, khong nhan duoi dong nhu lenh moi.
            dbg_rx_discard++;
        }
        else
            rx_buf[rx_len++] = c;
    }
    if (!rx_recover)
        Uart_StartRx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        dbg_uart_error++;
        dbg_uart_last_error = huart->ErrorCode;
        rx_recover = 1U; // ORE/FE/NE/PE: phuc hoi RX trong main, giu TX/ACK.
    }
}

static void Uart_Service(void)
{
    if (rx_recover)
    {
        // RX chi dung IT, khong DMA. AbortReceive khong doi du lieu den.
        // Khoa ngan de callback khong chen vao luc reset bo dem RX.
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        HAL_StatusTypeDef status = HAL_UART_AbortReceive(&huart2);
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        rx_len = 0U;
        rx_discard = 1U; // Sau loi, gui CR/LF roi gui lai nguyen khung.
        if (status == HAL_OK)
        {
            rx_recover = 0U;
            Uart_StartRx();
            if (!rx_recover)
                dbg_rx_recovered++;
        }
        else
            dbg_rx_arm_error++;
        __set_PRIMASK(mask);
    }
    if (!tx_busy && ack_tail != ack_head)
    {
        __DMB();
        (void)Uart_Send(ack_queue[ack_tail], 1U);
    }
}

// Doc so thap phan CHINH XAC, khong khoang trang, khong hau to/rac.
// Gioi han +/-2147483647 de giong nhau tren MCU 32 bit va host 64 bit.
static uint8_t Uart_Integer(char **cursor, char end, uint8_t sign_ok, long *value)
{
    char *p = *cursor;
    uint8_t negative = 0U;
    if (sign_ok && (*p == '-' || *p == '+'))
        negative = (uint8_t)(*p++ == '-');
    if (*p < '0' || *p > '9')
        return 0U;
    uint32_t n = 0U;
    while (*p >= '0' && *p <= '9')
    {
        uint32_t digit = (uint32_t)(*p++ - '0');
        if (n > (2147483647U - digit) / 10U)
            return 0U;
        n = n * 10U + digit;
    }
    if (*p != end)
        return 0U;
    *value = negative ? -(long)n : (long)n;
    *cursor = p;
    return 1U;
}

static int Uart_Hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void Uart_Apply(long seq, char type, long value, uint32_t now)
{
    if (type != 'T' && type != 'P' && type != 'Q' && type != 'G')
    {
        Uart_Ack(seq, 3);
        return;
    }
    float trim = (float)value * 0.001f;
    if ((type == 'T' && (trim > TRIM_LIMIT || trim < TRIM_MIN)) ||
        (type == 'G' && value != 0L && value != 1L))
    {
        Uart_Ack(seq, 4);
        return;
    }
    if ((type == 'P' || type == 'Q') && value != 0L)
    {
        Uart_Ack(seq, 1);
        return;
    }
    if (dbg_fault)
    {
        Uart_Ack(seq, 5);
        return;
    }
    if (type == 'G' && value == 1L)
    {
        if (RUN_MODE == MODE_WALK && dbg_stage == ST_STOPPED &&
            walk_stop_t >= 0.0f && walk_stop_t < N_CYCLES * GAIT_T)
        {
            resume_requested = 1U;
            control_last = now;
            link_last = now;
            dbg_link_up = 1U;
            Uart_Ack(seq, 0);
            return;
        }
        if (RUN_MODE != MODE_WALK || stop_requested || walk_finished ||
            (!walk_started && dbg_stage != ST_WAIT_START))
        {
            Uart_Ack(seq, 6);
            return;
        }
        if (!walk_started && !start_requested)
        {
            start_requested = 1U;
            dbg_trim_cmd = YAW_TRIM_DEG_PER_CYCLE;
            control_last = now;
        }
    }
    else if (type == 'G')
        Motion_RequestStop(1U);
    else if (type == 'T')
    {
        if (RUN_MODE != MODE_WALK || !walk_started || stop_requested || walk_finished)
        {
            Uart_Ack(seq, 6);
            return;
        }
        // T con hop le van gia han dieu khien o chu ky cuoi, du khong doi trim.
        control_last = now;
        dbg_trim_cmd = (Seg_Next < N_CYCLES * GAIT_T) ? trim : dbg_yaw_trim_deg_per_cycle;
    }
    link_last = now;
    dbg_link_up = 1U;
    Uart_Ack(seq, 0); // Chu ky cuoi chi gia han lien lac, giu trim dang ap dung.
}

static void Uart_Poll(uint32_t now)
{
    // Mot dong/tick de gioi han CPU; ACK day thi cho, khong bo ACK.
    if (rx_tail == rx_head || Uart_Next(ack_head) == ack_tail)
        return;
    char line[UART_LINE_MAX];
    __DMB();
    memcpy(line, rx_queue[rx_tail], sizeof(line));
    __DMB();
    rx_tail = Uart_Next(rx_tail);
    char *star = strchr(line, '*');
    if (line[0] != '$' || star == NULL || strlen(star + 1) != 2U)
        goto bad_frame;
    int high = Uart_Hex(star[1]), low = Uart_Hex(star[2]);
    if (high < 0 || low < 0)
        goto bad_frame;
    *star = '\0';
    if (Checksum(line + 1) != (uint8_t)(high * 16 + low))
    {
        dbg_rx_bad++;
        Uart_Ack(-1L, 2); // Khong tin seq trong khung sai checksum.
        return;
    }
    char *p = line + 1;
    long seq, value;
    if (!Uart_Integer(&p, ',', 0U, &seq))
        goto bad_frame;
    p++;
    char type = *p;
    if (type == '\0' || p[1] != ',')
        goto bad_frame;
    p += 2;
    if (!Uart_Integer(&p, '\0', 1U, &value))
        goto bad_frame;
    dbg_rx_ok++;
    Uart_Apply(seq, type, value, now);
    return;
bad_frame:
    dbg_rx_bad++;
    Uart_Ack(-1L, 1);
}

static void Uart_Watchdog(uint32_t now)
{
    if (dbg_link_up && (now - link_last) >= LINK_TIMEOUT_MS)
    {
        dbg_link_up = 0U;
        dbg_link_lost++;
        Motion_RequestStop(2U);
    }
    if (walk_started && !walk_finished && !stop_requested &&
        (now - control_last) >= CONTROL_TIMEOUT_MS)
        Motion_RequestStop(3U);
}

static void Uart_Telemetry(void)
{
    if (tx_busy || ack_tail != ack_head)
    {
        dbg_tele_skip++;
        return; // Chi telemetry duoc bo; ACK uu tien.
    }
    int n = snprintf(tx_body, sizeof(tx_body), "%lu,D,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%u,%ld",
             (unsigned long)tele_count,
             (long)(dbg_gait_t * 100.0f),
             (long)(Pc[0] * 10.0f),
             (long)(Pc[1] * 10.0f),
             (long)(dbg_yaw_ref_deg * 100.0f),
             (long)(dbg_yaw_trim_deg_per_cycle * 1000.0f),
             (unsigned long)dbg_stage,
             (unsigned long)dbg_fault,
             (unsigned long)dbg_alive,
             (unsigned long)dbg_build_id, N_CYCLES,
             (long)(dbg_trim_cmd * 1000.0f));
    if (n > 0 && n < (int)sizeof(tx_body))
        (void)Uart_Send(tx_body, 0U);
    else
        dbg_tx_error++;
    tele_count++;
}

// =====================================================================
// 9. MAIN VA NGOAI VI STM32F446RE - PROJECT SERVOAN / SYSTICK
// =====================================================================
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    Configure_PCA();

    Uart_StartRx();

    dbg_fault = Prepare();
    dbg_ready = (dbg_fault == 0U);

    uint32_t start = HAL_GetTick();
    uint32_t last = start;
    uint32_t verified = start;
    uint32_t ticks = 0U;
    link_last = start;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - last;

        if (elapsed < TICK_MS)
            continue;

        last = now;
        dbg_alive++;
        ticks++;

        if (emergency_stop && !dbg_fault)
            dbg_fault = 8U;

        if (elapsed > 2U * TICK_MS && !dbg_fault)
            dbg_fault = 7U;

        if (elapsed > dbg_tick_gap_max_ms)
            dbg_tick_gap_max_ms = elapsed;
        Uart_Watchdog(now); // Qua han da latch; lenh den muon khong huy dung.
        Uart_Service();
        Uart_Poll(now);
        Uart_Service();

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

        if (ticks % TELE_EVERY == 0U)
            Uart_Telemetry();

        if (now - verified >= 1000U)
        {
            Verify_PCA();
            verified = HAL_GetTick();
        }

        Update_LEDs(now);
        uint32_t used = HAL_GetTick() - now;
        if (used > dbg_tick_exec_max_ms)
            dbg_tick_exec_max_ms = used;
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

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
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
