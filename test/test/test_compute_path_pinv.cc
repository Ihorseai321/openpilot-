#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <float.h>

//selfdrive/controls/lib/lane_planner.py--compute_path_pinv()


// ================================= 输出矩阵值 =================================
template<typename _Tp>
void print_matrix(const std::vector<std::vector<_Tp> >& mat)
{
    int rows = mat.size();
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < mat[y].size(); ++x) {
            fprintf(stderr, "  %f  ", mat[y][x]);
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
}

template<typename _Tp>
void print_matrix(const std::vector<_Tp>& vec)
{
    for (int i = 0; i < vec.size(); ++i) {
        fprintf(stderr, "  %f  ", vec[i]);
    }
    fprintf(stderr, "\n");
}

template<typename _Tp> // mat1(m, n) * mat2(n, p) => result(m, p)
static std::vector<std::vector<_Tp> > matrix_mul(const std::vector<std::vector<_Tp> >& mat1, const std::vector<std::vector<_Tp> >& mat2)
{
    std::vector<std::vector<_Tp> > result;
    int m1 = mat1.size(), n1 = mat1[0].size();
    int m2 = mat2.size(), n2 = mat2[0].size();
    if (n1 != m2) {
        fprintf(stderr, "mat dimension dismatch\n");
        return result;
    }

    result.resize(m1);
    for (int i = 0; i < m1; ++i) {
        result[i].resize(n2, (_Tp)0);
    }

    for (int y = 0; y < m1; ++y) {
        for (int x = 0; x < n2; ++x) {
            for (int t = 0; t < n1; ++t) {
                result[y][x] += mat1[y][t] * mat2[t][x];
            }
        }
    }

    return result;
}

// ================================= 矩阵转置 =================================
// Blog: http://blog.csdn.net/fengbingchun/article/details/71514010
template<typename _Tp>
int transpose(const std::vector<std::vector<_Tp> >& src, std::vector<std::vector<_Tp> >& dst)
{
    int m = src.size();
    int n = src[0].size();

    dst.resize(n);
    for (int i = 0; i < n; ++i) {
        dst[i].resize(m);
    }

    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < m; ++x) {
            dst[y][x] = src[x][y];
        }
    }

    return 0;
}

// ================================= 矩阵奇异值分解 =================================
// Blog: http://blog.csdn.net/fengbingchun/article/details/72853757
template<typename _Tp>
static inline _Tp hypot_(_Tp a, _Tp b)
{
    a = std::abs(a);
    b = std::abs(b);
    if (a > b) {
        b /= a;
        return a*std::sqrt(1 + b*b);
    }
    if (b > 0) {
        a /= b;
        return b*std::sqrt(1 + a*a);
    }
    return 0;
}

template<typename _Tp>
static void JacobiSVD(std::vector<std::vector<_Tp> >& At,
    std::vector<std::vector<_Tp> >& _W, std::vector<std::vector<_Tp> >& Vt)
{
    double minval = FLT_MIN;
    _Tp eps = (_Tp)(FLT_EPSILON * 2);
    const int m = At[0].size();
    const int n = _W.size();
    const int n1 = m; // urows
    std::vector<double> W(n, 0.);

    for (int i = 0; i < n; i++) {
        double sd{ 0. };
        for (int k = 0; k < m; k++) {
            _Tp t = At[i][k];
            sd += (double)t*t;
        }
        W[i] = sd;

        for (int k = 0; k < n; k++)
            Vt[i][k] = 0;
        Vt[i][i] = 1;
    }

    int max_iter = std::max(m, 30);
    for (int iter = 0; iter < max_iter; iter++) {
        bool changed = false;
        _Tp c, s;

        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                _Tp *Ai = At[i].data(), *Aj = At[j].data();
                double a = W[i], p = 0, b = W[j];

                for (int k = 0; k < m; k++)
                    p += (double)Ai[k] * Aj[k];

                if (std::abs(p) <= eps * std::sqrt((double)a*b))
                    continue;

                p *= 2;
                double beta = a - b, gamma = hypot_((double)p, beta);
                if (beta < 0) {
                    double delta = (gamma - beta)*0.5;
                    s = (_Tp)std::sqrt(delta / gamma);
                    c = (_Tp)(p / (gamma*s * 2));
                }
                else {
                    c = (_Tp)std::sqrt((gamma + beta) / (gamma * 2));
                    s = (_Tp)(p / (gamma*c * 2));
                }

                a = b = 0;
                for (int k = 0; k < m; k++) {
                    _Tp t0 = c*Ai[k] + s*Aj[k];
                    _Tp t1 = -s*Ai[k] + c*Aj[k];
                    Ai[k] = t0; Aj[k] = t1;

                    a += (double)t0*t0; b += (double)t1*t1;
                }
                W[i] = a; W[j] = b;

                changed = true;

                _Tp *Vi = Vt[i].data(), *Vj = Vt[j].data();

                for (int k = 0; k < n; k++) {
                    _Tp t0 = c*Vi[k] + s*Vj[k];
                    _Tp t1 = -s*Vi[k] + c*Vj[k];
                    Vi[k] = t0; Vj[k] = t1;
                }
            }
        }

        if (!changed)
            break;
    }

    for (int i = 0; i < n; i++) {
        double sd{ 0. };
        for (int k = 0; k < m; k++) {
            _Tp t = At[i][k];
            sd += (double)t*t;
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
        _W[i][0] = (_Tp)W[i];

    srand(time(nullptr));

    for (int i = 0; i < n1; i++) {
        double sd = i < n ? W[i] : 0;

        for (int ii = 0; ii < 100 && sd <= minval; ii++) {
            // if we got a zero singular value, then in order to get the corresponding left singular vector
            // we generate a random vector, project it to the previously computed left singular vectors,
            // subtract the projection and normalize the difference.
            const _Tp val0 = (_Tp)(1. / m);
            for (int k = 0; k < m; k++) {
                unsigned int rng = rand() % 4294967295; // 2^32 - 1
                _Tp val = (rng & 256) != 0 ? val0 : -val0;
                At[i][k] = val;
            }
            for (int iter = 0; iter < 2; iter++) {
                for (int j = 0; j < i; j++) {
                    sd = 0;
                    for (int k = 0; k < m; k++)
                        sd += At[i][k] * At[j][k];
                    _Tp asum = 0;
                    for (int k = 0; k < m; k++) {
                        _Tp t = (_Tp)(At[i][k] - sd*At[j][k]);
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
                _Tp t = At[i][k];
                sd += (double)t*t;
            }
            sd = std::sqrt(sd);
        }

        _Tp s = (_Tp)(sd > minval ? 1 / sd : 0.);
        for (int k = 0; k < m; k++)
            At[i][k] *= s;
    }
}

// matSrc为原始矩阵，支持非方阵，matD存放奇异值，matU存放左奇异向量，matVt存放转置的右奇异向量
template<typename _Tp>
int svd(const std::vector<std::vector<_Tp> >& matSrc,
    std::vector<std::vector<_Tp> >& matD, std::vector<std::vector<_Tp> >& matU, std::vector<std::vector<_Tp> >& matVt)
{
    int m = matSrc.size();
    int n = matSrc[0].size();
    for (const auto& sz : matSrc) {
        if (n != sz.size()) {
            fprintf(stderr, "matrix dimension dismatch\n");
            return -1;
        }
    }

    bool at = false;
    if (m < n) {
        std::swap(m, n);
        at = true;
    }

    matD.resize(n);
    for (int i = 0; i < n; ++i) {
        matD[i].resize(1, (_Tp)0);
    }
    matU.resize(m);
    for (int i = 0; i < m; ++i) {
        matU[i].resize(m, (_Tp)0);
    }
    matVt.resize(n);
    for (int i = 0; i < n; ++i) {
        matVt[i].resize(n, (_Tp)0);
    }
    std::vector<std::vector<_Tp> > tmp_u = matU, tmp_v = matVt;

    std::vector<std::vector<_Tp> > tmp_a, tmp_a_;
    if (!at)
        transpose(matSrc, tmp_a);
    else
        tmp_a = matSrc;

    if (m == n) {
        tmp_a_ = tmp_a;
    }
    else {
        tmp_a_.resize(m);
        for (int i = 0; i < m; ++i) {
            tmp_a_[i].resize(m, (_Tp)0);
        }
        for (int i = 0; i < n; ++i) {
            tmp_a_[i].assign(tmp_a[i].begin(), tmp_a[i].end());
        }
    }

    std::cout << "tmp_a size: " << tmp_a.size() << ", " << tmp_a[0].size() << std::endl;
    print_matrix(vecm);
    std::cout << "----------------" << std::endl;
    print_matrix(vecm);
    std::cout << "tmp_a_ size: " << tmp_a_.size() << ", " << tmp_a_[0].size() << std::endl;

    JacobiSVD(tmp_a_, matD, tmp_v);

    if (!at) {
        transpose(tmp_a_, matU);
        matVt = tmp_v;
    }
    else {
        transpose(tmp_v, matU);
        matVt = tmp_a_;
    }

    return 0;
}

template<typename _Tp>
int pinv(const std::vector<std::vector<_Tp> >& src, std::vector<std::vector<_Tp> >& dst, _Tp tolerance)
{
    std::vector<std::vector<_Tp> > D, U, Vt;
    if (svd(src, D, U, Vt) != 0) {
        fprintf(stderr, "singular value decomposition fail\n");
        return -1;
    }

    int m = src.size();
    int n = src[0].size();

    std::vector<std::vector<_Tp> > Drecip, DrecipT, Ut, V;

    transpose(Vt, V);
    transpose(U, Ut);

    if (m < n)
        std::swap(m, n);

    Drecip.resize(n);
    for (int i = 0; i < n; ++i) {
        Drecip[i].resize(m, (_Tp)0);

        if (D[i][0] > tolerance)
            Drecip[i][i] = 1.0f / D[i][0];
    }

    if (src.size() < src[0].size())
        transpose(Drecip, DrecipT);
    else
        DrecipT = Drecip;

    std::vector<std::vector<_Tp> > tmp = matrix_mul(V, DrecipT);
    dst = matrix_mul(tmp, Ut);

    std::cout << "src.size: " << src.size() << ", " << src[0].size() << std::endl;
    std::cout << "D.size: " << D.size() << ", " << D[0].size() << std::endl;
    std::cout << "U.size: " << U.size() << ", " << U[0].size() << std::endl;
    std::cout << "Vt.size: " << Vt.size() << ", " << Vt[0].size() << std::endl;
    std::cout << "Drecip.size: " << Drecip.size() << ", " << Drecip[0].size() << std::endl;
    std::cout << "DrecipT.size: " << DrecipT.size() << ", " << DrecipT[0].size() << std::endl;
    std::cout << "Ut.size: " << Ut.size() << ", " << Ut[0].size() << std::endl;
    std::cout << "V.size: " << V.size() << ", " << V[0].size() << std::endl;
    return 0;
}

int test_pseudoinverse()
{
    //std::vector<std::vector<float> > vec{ { 0.68f, 0.597f },
    //                { -0.211f, 0.823f },
    //                { 0.566f, -0.605f } };
    //const int rows{ 3 }, cols{ 2 };

    std::vector<std::vector<float> > vecm;
    std::vector<float> vec;
    float arr[200];
    for(int i=0; i<4; ++i){
        for(int j=0; j<50; ++j){
            arr[j*4+i] = pow(j, 3-i);
        }
    }
    
    for(int m=0; m<200; ++m){
        vec.push_back(arr[m]);
        if((m+1)%4 == 0){
            vecm.push_back(vec);
            vec.clear();
        }
    }
    
    // vec.push_back(0.597f);
    // vec.push_back(-0.211f);
    
    // vec.push_back(0.823f);
    // vec.push_back(0.566f);
    // vec.push_back(-0.605f);
    // vecm.push_back(vec);
    // vec.clear();

    // { { 0.68f, 0.597f, -0.211f },
                        // { 0.823f, 0.566f, -0.605f } };
    // const int rows{ 2 }, cols{ 3 };

    fprintf(stderr, "source matrix:\n");
    print_matrix(vecm);

    fprintf(stderr, "\nc++ implement pseudoinverse:\n");
    std::vector<std::vector<float> > pinv1;
    float  pinvtoler = 1.e-6;
    if (pinv(vecm, pinv1, pinvtoler) != 0) {
        fprintf(stderr, "C++ implement pseudoinverse fail\n");
        return -1;
    }
    print_matrix(pinv1);

    return 0; 
}

int main(int argc, char *argv[])
{
    test_pseudoinverse();
    return 0;
}