#include <math.h>
#include "rpn_stack.h"

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// ✅ CORRECT MACROS (each line ends with \)
#define BINOP(s, expr) do {                           \
    if (rpn_depth(s) < 2) return RPN_ERR_UNDERFLOW;   \
    double b = rpn_get(s, 0);                         \
    double a = rpn_get(s, 1);                         \
    rpn_drop(s);                                      \
    rpn_drop(s);                                      \
    if (rpn_push(s, (expr)) != RPN_OK)                \
        return RPN_ERR_OVERFLOW;                      \
    return RPN_OK;                                    \
} while (0)

#define UNOP(s, expr) do {                            \
    if (rpn_depth(s) < 1) return RPN_ERR_UNDERFLOW;   \
    double a = rpn_get(s, 0);                         \
    rpn_drop(s);                                      \
    if (rpn_push(s, (expr)) != RPN_OK)                \
        return RPN_ERR_OVERFLOW;                      \
    return RPN_OK;                                    \
} while (0)
// Aritmetica base
RPN_Error rpn_add (RPN_Stack *s) { BINOP(s, a + b); }
RPN_Error rpn_sub (RPN_Stack *s) { BINOP(s, a - b); }
RPN_Error rpn_mul (RPN_Stack *s) { BINOP(s, a * b); }
RPN_Error rpn_div (RPN_Stack *s) {
  double a, b;
  rpn_pop(s, &b);
  rpn_pop(s, &a);
  if (b == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
  return rpn_push(s, a / b);
}
RPN_Error rpn_mod (RPN_Stack *s) {
  double a, b;
  rpn_pop(s, &b);
  rpn_pop(s, &a);
  if (b == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
  return rpn_push(s, fmod(a, b));
}
RPN_Error rpn_pow (RPN_Stack *s) { BINOP(s, pow(a, b)); }
RPN_Error rpn_neg (RPN_Stack *s) { UNOP(s, -a); }
RPN_Error rpn_inv (RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
  return rpn_push(s, 1.0 / a);
}

// Radici e log
RPN_Error rpn_sqrt(RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a < 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, sqrt(a));
}
RPN_Error rpn_cbrt (RPN_Stack *s) { UNOP(s, cbrt(a)); }
RPN_Error rpn_sq   (RPN_Stack *s) { UNOP(s, a * a); }
RPN_Error rpn_cube (RPN_Stack *s) { UNOP(s, a * a * a); }
RPN_Error rpn_exp  (RPN_Stack *s) { UNOP(s, exp(a)); }
RPN_Error rpn_exp2 (RPN_Stack *s) { UNOP(s, exp2(a)); }
RPN_Error rpn_log  (RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, log(a));
}
RPN_Error rpn_log10(RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, log10(a));
}
RPN_Error rpn_log2 (RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, log2(a));
}

// Trigonometria (in gradi — internamente convertiamo da/verso radianti)
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

RPN_Error rpn_sin  (RPN_Stack *s) { UNOP(s, sin(a * DEG2RAD)); }
RPN_Error rpn_cos  (RPN_Stack *s) { UNOP(s, cos(a * DEG2RAD)); }
RPN_Error rpn_tan  (RPN_Stack *s) { UNOP(s, tan(a * DEG2RAD)); }
RPN_Error rpn_asin (RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a < -1.0 || a > 1.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, asin(a) * RAD2DEG);
}
RPN_Error rpn_acos (RPN_Stack *s) {
  double a; rpn_pop(s, &a);
  if (a < -1.0 || a > 1.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
  return rpn_push(s, acos(a) * RAD2DEG);
}
RPN_Error rpn_atan (RPN_Stack *s) { UNOP(s, atan(a) * RAD2DEG); }
RPN_Error rpn_atan2(RPN_Stack *s) { BINOP(s, atan2(a, b) * RAD2DEG); }

// Iperboliche
RPN_Error rpn_sinh (RPN_Stack *s) { UNOP(s, sinh(a)); }
RPN_Error rpn_cosh (RPN_Stack *s) { UNOP(s, cosh(a)); }
RPN_Error rpn_tanh (RPN_Stack *s) { UNOP(s, tanh(a)); }
