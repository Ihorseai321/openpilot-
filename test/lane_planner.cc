#include "lane_planner.h"
#include <cmath>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"


// mat1(4, 4) * mat2(4, 50) => result(4, 50)
static void matrix_mul_4_450(float mat1[4][4], float mat2[4][50], float result[4][50])
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 50; ++j) {
            for (int k = 0; k < 4; ++k) {
                result[i][j] += (mat1[i][k] * mat2[k][j]);
            }
        }
    }
}

// mat1(4, 50) * mat2(50, 50) => result(4, 50)
static void matrix_mul_450_50(float mat1[4][50], float mat2[50][50], float result[4][50])
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 50; ++j) {
            for (int k = 0; k < 50; ++k) {
                result[i][j] += (mat1[i][k] * mat2[k][j]);
            }
        }
    }
}

int transpose_4(float src[4][4], float dst[4][4])
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            dst[j][i] = src[i][j];
        }
    }
    return 0;
}

int transpose_50(float src[50][50], float dst[50][50])
{
    for (int i = 0; i < 50; ++i) {
        for (int j = 0; j < 50; ++j) {
            dst[j][i] = src[i][j];
        }
    }
    return 0;
}

int transpose_50_4(float src[50][4], float dst[4][50])
{
    for (int i = 0; i < 50; ++i) {
        for (int j = 0; j < 4; ++j) {
            dst[j][i] = src[i][j];
        }
    }
    return 0;
}

static inline float hypot_(float a, float b)
{
    a = std::fabs(a);
    b = std::fabs(b);
    if (a > b) {
        b /= a;
        return a * std::sqrt(1 + b*b);
    }
    if (b > 0) {
        a /= b;
        return b * std::sqrt(1 + a*a);
    }
    return 0;
}

static void JacobiSVD(float At[50][50], float _W[4][1], float Vt[4][4])
{
    float minval = FLT_MIN;
    float eps = (FLT_EPSILON * 2);
    const int m = 50;
    const int n = 4;
    const int n1 = 50; // urows
    float W[4] = {0, 0, 0, 0};

    for (int i = 0; i < n; i++) {
        float sd = 0.0;
        for (int k = 0; k < m; k++) {
            float t = At[i][k];
            sd += t * t;
        }
        W[i] = sd;

        for (int k = 0; k < n; k++)
            Vt[i][k] = 0;
        Vt[i][i] = 1;
    }

    for (int iter = 0; iter < 50; iter++) {
        bool changed = false;
        float c, s;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                float *Ai = At[i], *Aj = At[j];
                float a = W[i], p = 0, b = W[j];

                for (int k = 0; k < m; k++)
                    p += Ai[k] * Aj[k];

                if (std::abs(p) <= eps * std::sqrt(a*b))
                    continue;

                p *= 2;
                float beta = a - b, gamma = hypot_(p, beta);
                if (beta < 0) {
                    float delta = (gamma - beta)*0.5;
                    s = std::sqrt(delta / gamma);
                    c = (p / (gamma*s * 2));
                }
                else {
                    c = std::sqrt((gamma + beta) / (gamma * 2));
                    s = (p / (gamma*c * 2));
                }

                a = b = 0;
                for (int k = 0; k < m; k++) {
                    float t0 = c * Ai[k] + s * Aj[k];
                    float t1 = -s * Ai[k] + c * Aj[k];
                    Ai[k] = t0;
                    Aj[k] = t1;

                    a += t0 * t0;
                    b += t1 * t1;
                }
                W[i] = a; W[j] = b;

                changed = true;

                float *Vi = Vt[i], *Vj = Vt[j];

                for (int k = 0; k < n; k++) {
                    float t0 = c * Vi[k] + s * Vj[k];
                    float t1 = -s * Vi[k] + c * Vj[k];
                    Vi[k] = t0;
                    Vj[k] = t1;
                }
            }
        }

        if (!changed)
            break;
    }

    for (int i = 0; i < n; i++) {
        float sd = 0.0;
        for (int k = 0; k < m; k++) {
            float t = At[i][k];
            sd += t * t;
        }
        W[i] = std::sqrt(sd);
    }

    for (int i = 0; i < n - 1; i++) {
        int j = i;
        for (int k = i + 1; k < n; k++) {
            if (W[j] < W[k])
                j = k;
        }
        if (i != j) {
            std::swap(W[i], W[j]);

            for (int k = 0; k < m; k++)
                std::swap(At[i][k], At[j][k]);

            for (int k = 0; k < n; k++)
                std::swap(Vt[i][k], Vt[j][k]);
        }
    }

    for (int i = 0; i < n; i++)
        _W[i][0] = W[i];

    srand(time(NULL));

    for (int i = 0; i < n1; i++) {
        float sd = i < n ? W[i] : 0;

        for (int ii = 0; ii < 100 && sd <= minval; ii++) {
            const float val0 = (1. / m);
            for (int k = 0; k < m; k++) {
                unsigned int rng = rand() % 4294967295; // 2^32 - 1
                float val = (rng & 256) != 0 ? val0 : -val0;
                At[i][k] = val;
            }
            for (int iter = 0; iter < 2; iter++) {
                for (int j = 0; j < i; j++) {
                    sd = 0;
                    for (int k = 0; k < m; k++)
                        sd += At[i][k] * At[j][k];
                    float asum = 0;
                    for (int k = 0; k < m; k++) {
                        float t = (At[i][k] - sd*At[j][k]);
                        At[i][k] = t;
                        asum += std::abs(t);
                    }
                    asum = asum > eps * 100 ? 1 / asum : 0;
                    for (int k = 0; k < m; k++)
                        At[i][k] *= asum;
                }
            }

            sd = 0;
            for (int k = 0; k < m; k++) {
                float t = At[i][k];
                sd += t*t;
            }
            sd = std::sqrt(sd);
        }

        float s = (sd > minval ? 1 / sd : 0.);
        for (int k = 0; k < m; k++)
            At[i][k] *= s;
    }
}

// matSrc为原始矩阵，支持非方阵，matD存放奇异值，matU存放左奇异向量，matVt存放转置的右奇异向量
int svd(float matSrc[50][4], float matD[4][1], float matU[50][50], float matVt[4][4])
{
    for(int i = 0; i < 4; ++i){
        matD[i][0] = 0;
    }

    for(int i = 0; i < 50; ++i){
        for(int j = 0; j < 50; ++j){
            matU[i][j] = 0;
        }
    }

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            matVt[i][j] = 0;
        }
    }

    float tmp_u[50][50];

    for(int i = 0; i < 50; ++i){
        for(int j = 0; j < 50; ++j){
            tmp_u[i][j] = matU[i][j];
        }
    }

    float tmp_v[4][4];

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            tmp_v[i][j] = matVt[i][j];
        }
    }

    float tmp_a[4][50], tmp_a_[50][50];
    
    transpose_50_4(matSrc, tmp_a);

    for (int i = 0; i < 50; ++i) {
        for(int j = 0; j < 50; ++j){
            tmp_a_[i][j] = 0;
        }
    }

    for (int i = 0; i < 4; ++i) {
        for(int j = 0; j < 50; ++j){
            tmp_a_[i][j] = tmp_a[i][j];
        }
    }
    
    JacobiSVD(tmp_a_, matD, tmp_v);

    transpose_50(tmp_a_, matU);

    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 4; ++j){
            matVt[i][j] = tmp_v[i][j];
        }
    }

    return 0;
}

int pinv(float src[50][4], float dst[4][50], float tolerance = 1.e-6)
{
    for(int i = 0; i < 4; ++i){
        for(int j = 0; j < 50; ++j){
            dst[i][j] = 0.0;
        }
    }
    float D[4][1], U[50][50], Vt[4][4];
    if (svd(src, D, U, Vt) != 0) {
        fprintf(stderr, "singular value decomposition fail\n");
        return -1;
    }

    float Drecip[4][50], Ut[50][50], V[4][4];

    transpose_4(Vt, V);
    transpose_50(U, Ut);

    for (int i = 0; i < 4; ++i) {
        for(int j = 0; j < 50; ++ j){
            Drecip[i][j] = 0;

            if (D[i][0] > tolerance)
                Drecip[i][i] = 1.0f / D[i][0];
        }
    }

    float tmp[4][50];
    matrix_mul_4_450(V, Drecip, tmp);
    matrix_mul_450_50(tmp, Ut, dst);

    return 0;
}

int compute_path_pinv(float pinv1[4][50])
{
    float src[50][4];
    for(int i=0; i<4; ++i){
        for(int j=0; j<50; ++j){
            src[j][i] = pow(j, 3-i) * 1.0;
        }
    }
    
    if (pinv(src, pinv1) != 0) {
        fprintf(stderr, "C++ implement pseudoinverse fail\n");
        return -1;
    }

    return 0; 
}

float interp(float x, float xp[], float fp[], int n)
{
    int hi = 0;

    while(hi < n && x > xp[hi]){
        hi += 1;
    }

    int low = hi -1;
    if(hi == n && x > xp[low]){
      return fp[n-1];
    }else if (hi == 0)
    {
      return fp[0];
    }else{
      return (x - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low];
    }  
}

void model_polyfit(float points[N], float path_pinv[M][N], float ret[M])
{
    float sum;
    for(int i = 0; i < M; ++i){
        sum = 0.0;
        for(int j = 0; j < N; ++j){
            sum += path_pinv[i][j] * points[j];
        }
        ret[i] = sum;
    }
}

void calc_d_poly(float l_poly[], float r_poly[], float p_poly[], float l_prob, float r_prob, float lane_width, float d_poly[])
{
    float xp[2] = {2.0, 2.5};
    float fp[2] = {1.0, 0.0};
    float _lane_width_ = lane_width < 4.0 ? lane_width : 4.0;
    float _l_prob_ = l_prob * interp(fabs(l_poly[3]), xp, fp, 2);
    float _r_prob_ = r_prob * interp(fabs(r_poly[3]), xp, fp, 2);
    
    float lr_prob = _l_prob_ + _r_prob_ - _l_prob_ * _r_prob_;

    float path_from_left_lane = 0.0;
    float path_from_right_lane = 0.0;
    float d_poly_line = 0.0;

    for(int i = 0; i < 4; ++i){
        if(i == 3){
            path_from_left_lane = l_poly[i] - _lane_width_ / 2.0;
            path_from_right_lane = r_poly[i] + _lane_width_ / 2.0;
        }
        else{
            path_from_left_lane = l_poly[i];
            path_from_right_lane = r_poly[i];
        }

        d_poly_line = (_l_prob_ * path_from_left_lane + _r_prob_ * path_from_right_lane) / (_l_prob_ + _r_prob_ + 0.001);
        d_poly[i] = lr_prob *  d_poly_line + (1.0 - lr_prob) * p_poly[i];
    }
}

LanePlanner::LanePlanner()
{
    for(int i = 0;i < 4; ++i){
        l_poly[i] = 0.0;
        r_poly[i] = 0.0;
        p_poly[i] = 0.0;
        d_poly[i] = 0.0;
    }

    lane_width_estimate = 3.7;
    lane_width_certainty = 1.0;
    lane_width = 3.7;

    l_prob = 0.0;
    r_prob = 0.0;

    l_lane_change_prob = 0.0;
    r_lane_change_prob = 0.0;

    // for(int i = 0; i < 4; ++i){
    //     for(int j = 0; j < 50; ++j){
    //         _path_pinv[i][j] = 0.0;
    //     }
    // }
    compute_path_pinv(_path_pinv);
    
    for(int i = 0; i < 50; ++i){
        x_points[i] = 0.0;
    }
}

LanePlanner::~LanePlanner()
{
}

void LanePlanner::parse_model(Handler handler)
{
    if(handler.l_has_poly){
        for(int i = 0; i < 4; ++i){
            l_poly[i] = handler.l_poly[i];
            r_poly[i] = handler.r_poly[i];
            p_poly[i] = handler.p_poly[i];
        }
    }
    else{
        model_polyfit(handler.l_points, _path_pinv, l_poly);
        model_polyfit(handler.r_points, _path_pinv, r_poly);
        model_polyfit(handler.p_points, _path_pinv, p_poly);
    }

    l_prob = handler.l_prob;
    r_prob = handler.r_prob;

    if(handler.has_meta_desirePrediction){
        l_lane_change_prob = meta_desirePrediction[2];
        r_lane_change_prob = meta_desirePrediction[3];
    }
}

void LanePlanner::update_d_poly(float v_ego)
{   
    l_poly[3] += CAMERA_OFFSET;
    r_poly[3] += CAMERA_OFFSET;

    // Find current lanewidth
    lane_width_certainty += 0.05 * (l_prob * r_prob - lane_width_certainty);
    float current_lane_width = fabs(l_poly[3] - r_poly[3]);
    lane_width_estimate += 0.005 * (current_lane_width - lane_width_estimate);
    float speed_range[2] = {0.0, 31.0};
    float lanewidth_range[2] = {2.8, 3.5};
    float speed_lane_width = interp(v_ego, speed_range, lanewidth_range, 2);
    lane_width = lane_width_certainty * lane_width_estimate + (1 - lane_width_certainty) * speed_lane_width;
    
    calc_d_poly(l_poly, r_poly, p_poly, l_prob, r_prob, lane_width, d_poly);
}

void LanePlanner::update(Handler handler, float v_ego)
{
    parse_model(handler);
    update_d_poly(v_ego);
}