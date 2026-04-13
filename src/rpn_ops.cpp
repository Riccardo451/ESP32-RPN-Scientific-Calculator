// rpn_ops.cpp
#include "rpn_stack.h"
#include <math.h>

// Macro helper: pop due operandi, esegui op, push risultato
#define BINOP(s, expr) 
  do { 
    double a, b, r; 
    RPN_Error e;
    e = rpn_pop(s, &b); if (e) return e;
    e = rpn_pop(s, &a); if (e) { rpn_push(s, b); return e; } 
    r = (expr); 
    if (isnan(r))  { s->last_err = RPN_ERR_NAN;    return RPN_ERR_NAN; } 
    if (isinf(r))  { s->last_err = RPN_ERR_INF;    return RPN_ERR_INF; } 
    return rpn_push(s, r); 
  } while(0)

#define UNOP(s, expr) 
  do { 
    double a, r; 
    RPN_Error e = rpn_pop(s, &a); if (e) return e;
    r = (expr); 
    if (isnan(r)) { s->last_err = RPN_ERR_NAN;   return RPN_ERR_NAN; } 
    if (isinf(r)) { s->last_err = RPN_ERR_INF;   return RPN_ERR_INF; } 
    return rpn_push(s, r); 
  } while(0)

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
