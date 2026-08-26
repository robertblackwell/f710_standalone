
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <valarray>
#include <unittest.h>
#include <iobuffer.h>
#include <fbcontrol/custom.h>

using namespace ros2_bridge;

int test_01()
{
    auto x = MotorSide::left;
    printf("XXXXXXXXXXXHello world from buffer test \n");
    UT_EQUAL_INT(1, 1);
    return 0;
}
int test_02()
{
    auto fgf_left = make_first_guess_function(MotorSide::left);
    {
        auto y = fgf_left(6999.999);
        UT_TRUE((y == 95.0));
    }
    {
        auto pwm_left = get_pwm(MotorSide::left);
        auto rpm = get_rpm();
        std::vector<double> r; r.resize(pwm_left.size());
        std::transform(rpm.begin(), rpm.end(), r.begin(), [&](double x){return (double)fgf_left(x);});
        r.pop_back();
        for(int i = 0; i < pwm_left.size()-1; i++) {
            auto a = pwm_left[i];
            auto b = r[i];
            UT_TRUE((a == b))
        }
    }
    {
        auto pwm_left = get_pwm(MotorSide::left);
        auto rpm = get_rpm();
        std::vector<double> ar = {
                fgf_left(6250),
                fgf_left(5750),
                fgf_left(5250),
                fgf_left(4750),
                fgf_left(4250),
                fgf_left(3750),
                fgf_left(3250),
                fgf_left(2750),
                fgf_left(2250),
        };
        for (int i = 0; i < rpm.size()-1; i++) {
            auto xx = (rpm[i] + rpm[i+1]) / 2.0;
            auto a = fgf_left(xx);
            auto x1 = rpm[i+1];
            auto x0 = rpm[i];
            auto y0 = fgf_left(x0);
            auto y1 = fgf_left(x1);
            auto y = ((y1 - y0)/(x1 - x0))*(xx - x0) + y0;

            auto b = fgf_left(rpm[i]) + fgf_left(rpm[i+1]) / 2.0;
            printf("%f  %f\n", a, y);
            bool z = (a == y);
            UT_TRUE((z));
        }
    }
    printf("XXXXXXXXXXXHello world from buffer test \n");
    UT_EQUAL_INT(1, 1);
    return 0;
}

int main()
{
    UT_ADD(test_01);
    UT_ADD(test_02);
    // UT_ADD(test_expansion);
    // UT_ADD(test_big_expansion);
    // UT_ADD(test_buffer_move);
    // UT_ADD(test_iobuffer_consume_commit_01);
//    UT_ADD(test_chain_front);
//    UT_ADD(test_iobuffer_commit_consume_extra);
//    UT_ADD(test_make_buffer);
//    UT_ADD(test_expansion);
//    UT_ADD(test_big_expansion);
//    UT_ADD(test_cbuffer_clear);
//    UT_ADD(test_cbuffer_move);
//    UT_ADD(test_chain_make);
//    UT_ADD(test_chain_compact);
//    UT_ADD(test_iobuffer_make);
//    UT_ADD(test_iobuffer_make2);
    int rc = UT_RUN();
    return rc;
}