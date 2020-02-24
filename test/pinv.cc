#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <float.h>

//selfdrive/controls/lib/lane_planner.py--compute_path_pinv()


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

// int main(int argc, char *argv[])
// {
//     float pinv1[4][50];
//     compute_path_pinv(pinv1);

//     for(int i = 0; i < 4; ++i){
//         std::cout << "[";
//         for(int j = 0; j < 50; ++j){
//             std::cout << pinv1[i][j] << ",\t";
//             if((j+1)%4 == 0){
//                 std::cout << std::endl;
//             }
//         }
//         std::cout << "]" << std::endl;
//     }
//     return 0;
// }