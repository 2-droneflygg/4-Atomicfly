/*********************************************************************************
 提供标准化输入输出相关API。
*********************************************************************************/
#include <math.h>
#include <stdint.h>

#include "platform.h"

// ---------------------------------------------------------解决gcc中缺乏优化的问题
float exp_cst1 = 2139095040.f;
float exp_cst2 = 0.f;

/**********************************************************************
函数名称：exp_approx
函数功能：指数函数 - 标准化输出的相对误差为1e-5
函数形参：val
函数返回值：返回无效的nan输入输出连续错误
函数描述：None
**********************************************************************/
float exp_approx(float val) {
  union { int32_t i; float f; } xu, xu2;
  float val2, val3, val4, b;
  int32_t val4i;
  val2 = 12102203.1615614f*val+1065353216.f;
  val3 = val2 < exp_cst1 ? val2 : exp_cst1;
  val4 = val3 > exp_cst2 ? val3 : exp_cst2;
  val4i = (int32_t) val4;
  // mask exponent  / round down to neareset 2^n (implicit mantisa bit)
  xu.i = val4i & 0x7F800000;       
  // force exponent to 0
  xu2.i = (val4i & 0x7FFFFF) | 0x3F800000;     
  b = xu2.f;

  /* Generated in Sollya with:
     > f=remez(1-x*exp(-(x-1)*log(2)),
               [|(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x|],
               [1.000001,1.999999], exp(-(x-1)*log(2)));
     > plot(exp((x-1)*log(2))/(f+x)-1, [1,2]);
     > f+x;
  */
  return
    xu.f * (0.509871020343597804469416f + b *
            (0.312146713032169896138863f + b *
             (0.166617139319965966118107f + b *
              (-2.19061993049215080032874e-3f + b *
               1.3555747234758484073940937e-2f))));
}

/**********************************************************************
函数名称：log_approx
函数功能：Log函数 - 标准化输入的绝对错误范围为1e-6
函数形参：val
函数返回值：为+inf输入返回一个有限的数字,当nan和<= 0输入时返回-inf连续错误
函数描述：None
**********************************************************************/
float log_approx(float val) {
  union { float f; int32_t i; } valu;
  float exp, addcst, x;
  valu.f = val;
  exp = valu.i >> 23;
  /* 89.970756366f = 127 * log(2) - constant term of polynomial */
  addcst = val > 0 ? -89.970756366f : -(float)INFINITY;
  valu.i = (valu.i & 0x7FFFFF) | 0x3F800000;
  x = valu.f;

  /* Generated in Sollya using:
    > f = remez(log(x)-(x-1)*log(2),
            [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x,
              (x-1)*(x-2)*x*x*x|], [1,2], 1, 1e-8);
    > plot(f+(x-1)*log(2)-log(x), [1,2]);
    > f+(x-1)*log(2)
 */
  return
    x * (3.529304993f + x * (-2.461222105f +
      x * (1.130626167f + x * (-0.288739945f +
        x * 3.110401639e-2f))))
    + (addcst + 0.69314718055995f*exp);
}

/**********************************************************************
函数名称：pow_approx
函数功能：计算指数表达式 - 标准化输入输出
函数形参：a,b
函数返回值：exp_approx
函数描述：
	用于performBaroCalibrationCycle(执行气压校准循环)计算baroGroundAltitude。
**********************************************************************/
float pow_approx(float a, float b)
{
    return exp_approx(b * log_approx(a));
}

