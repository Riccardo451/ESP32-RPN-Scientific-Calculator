/*
 * main.c — RPN Stack Engine test standalone
 * Compilare con:  gcc -o rpn_test main.c -lm && ./rpn_test
 * Oppure incollare direttamente su onlinegdb.com / godbolt.org
 *
 * Tutto in un file unico per comodità di test online.
 * In produzione (Arduino/ESP32) questi blocchi vanno in file separati:
 *   rpn_stack.h / rpn_stack.cpp / rpn_ops.cpp
 */
 
#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
 
#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif
#ifndef M_E
#define M_E   2.71828182845904523536
#endif
 
/* ================================================================
   rpn_stack.h — tipi e interfaccia
   ================================================================ */
 
#define RPN_STACK_SIZE   16
#define RPN_LASTX_DEPTH   4
 
typedef enum {
    RPN_OK = 0,
    RPN_ERR_OVERFLOW,
    RPN_ERR_UNDERFLOW,
    RPN_ERR_DIV_ZERO,
    RPN_ERR_DOMAIN,
    RPN_ERR_NAN,
    RPN_ERR_INF,
} RPN_Error;
 
typedef struct {
    double   data[RPN_STACK_SIZE];
    uint8_t  top;
    double   lastx[RPN_LASTX_DEPTH];
    uint8_t  lastx_idx;
    double   reg_x, reg_y, reg_z, reg_t;
    RPN_Error last_err;
} RPN_Stack;
 
/* ================================================================
   rpn_stack.c — implementazione stack
   ================================================================ */
 
static void _rpn_sync_regs(RPN_Stack *s) {
    s->reg_x = (s->top > 0) ? s->data[s->top - 1] : 0.0;
    s->reg_y = (s->top > 1) ? s->data[s->top - 2] : 0.0;
    s->reg_z = (s->top > 2) ? s->data[s->top - 3] : 0.0;
    s->reg_t = (s->top > 3) ? s->data[s->top - 4] : 0.0;
}
 
void rpn_init(RPN_Stack *s) {
    memset(s, 0, sizeof(RPN_Stack));
}
 
void rpn_reset(RPN_Stack *s) {
    rpn_init(s);
}
 
RPN_Error rpn_push(RPN_Stack *s, double val) {
    if (s->top >= RPN_STACK_SIZE) {
        s->last_err = RPN_ERR_OVERFLOW;
        return RPN_ERR_OVERFLOW;
    }
    if (s->top > 0) {
        s->lastx[s->lastx_idx % RPN_LASTX_DEPTH] = s->data[s->top - 1];
        s->lastx_idx++;
    }
    s->data[s->top++] = val;
    _rpn_sync_regs(s);
    return RPN_OK;
}
 
RPN_Error rpn_pop(RPN_Stack *s, double *out) {
    if (s->top == 0) {
        s->last_err = RPN_ERR_UNDERFLOW;
        return RPN_ERR_UNDERFLOW;
    }
    *out = s->data[--s->top];
    _rpn_sync_regs(s);
    return RPN_OK;
}
 
RPN_Error rpn_peek(const RPN_Stack *s, double *out) {
    if (s->top == 0) return RPN_ERR_UNDERFLOW;
    *out = s->data[s->top - 1];
    return RPN_OK;
}
 
RPN_Error rpn_drop(RPN_Stack *s) {
    double dummy;
    return rpn_pop(s, &dummy);
}
 
RPN_Error rpn_swap(RPN_Stack *s) {
    if (s->top < 2) return RPN_ERR_UNDERFLOW;
    double tmp        = s->data[s->top - 1];
    s->data[s->top-1] = s->data[s->top - 2];
    s->data[s->top-2] = tmp;
    _rpn_sync_regs(s);
    return RPN_OK;
}
 
RPN_Error rpn_dup(RPN_Stack *s) {
    if (s->top == 0) return RPN_ERR_UNDERFLOW;
    return rpn_push(s, s->data[s->top - 1]);
}
 
RPN_Error rpn_roll_down(RPN_Stack *s) {
    if (s->top < 2) return RPN_OK;
    double bottom = s->data[0];
    for (uint8_t i = 0; i < s->top - 1; i++)
        s->data[i] = s->data[i + 1];
    s->data[s->top - 1] = bottom;
    _rpn_sync_regs(s);
    return RPN_OK;
}
 
RPN_Error rpn_clear(RPN_Stack *s) {
    s->top = 0;
    _rpn_sync_regs(s);
    return RPN_OK;
}
 
double rpn_lastx(const RPN_Stack *s) {
    if (s->lastx_idx == 0) return 0.0;
    return s->lastx[(s->lastx_idx - 1) % RPN_LASTX_DEPTH];
}
 
uint8_t rpn_depth(const RPN_Stack *s)    { return s->top; }
bool    rpn_is_empty(const RPN_Stack *s) { return s->top == 0; }
bool    rpn_is_full(const RPN_Stack *s)  { return s->top >= RPN_STACK_SIZE; }
 
double rpn_get(const RPN_Stack *s, uint8_t idx) {
    if (idx >= s->top) return 0.0;
    return s->data[s->top - 1 - idx];
}
 
const char *rpn_err_str(RPN_Error err) {
    switch (err) {
        case RPN_OK:            return "OK";
        case RPN_ERR_OVERFLOW:  return "Stack full";
        case RPN_ERR_UNDERFLOW: return "Too few args";
        case RPN_ERR_DIV_ZERO:  return "Div/0";
        case RPN_ERR_DOMAIN:    return "Domain error";
        case RPN_ERR_NAN:       return "Not a number";
        case RPN_ERR_INF:       return "Infinity";
        default:                return "Unknown error";
    }
}
 
/* ================================================================
   rpn_ops.c — operazioni aritmetiche e scientifiche
   ================================================================ */
 
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)
 
 
/* helper interni: evitano le macro a livello globale (incompatibili C99) */
static RPN_Error _binop_result(RPN_Stack *s, double a, double b, double r) {
    (void)a; (void)b;
    if (isnan(r)) { s->last_err = RPN_ERR_NAN; return RPN_ERR_NAN; }
    if (isinf(r)) { s->last_err = RPN_ERR_INF; return RPN_ERR_INF; }
    return rpn_push(s, r);
}
static RPN_Error _unop_result(RPN_Stack *s, double r) {
    if (isnan(r)) { s->last_err = RPN_ERR_NAN; return RPN_ERR_NAN; }
    if (isinf(r)) { s->last_err = RPN_ERR_INF; return RPN_ERR_INF; }
    return rpn_push(s, r);
}
static RPN_Error _pop2(RPN_Stack *s, double *a, double *b) {
    RPN_Error e = rpn_pop(s, b); if (e) return e;
    e = rpn_pop(s, a); if (e) { rpn_push(s, *b); return e; }
    return RPN_OK;
}
static RPN_Error _pop1(RPN_Stack *s, double *a) {
    return rpn_pop(s, a);
}
 
RPN_Error rpn_add(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    return _binop_result(s, a, b, a + b);
}
RPN_Error rpn_sub(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    return _binop_result(s, a, b, a - b);
}
RPN_Error rpn_mul(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    return _binop_result(s, a, b, a * b);
}
RPN_Error rpn_div(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    if (b == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
    return rpn_push(s, a / b);
}
RPN_Error rpn_mod(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    if (b == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
    return rpn_push(s, fmod(a, b));
}
RPN_Error rpn_pow(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    return _binop_result(s, a, b, pow(a, b));
}
RPN_Error rpn_neg(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, -a);
}
RPN_Error rpn_inv(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a == 0.0) { s->last_err = RPN_ERR_DIV_ZERO; return RPN_ERR_DIV_ZERO; }
    return rpn_push(s, 1.0 / a);
}
RPN_Error rpn_sqrt(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a < 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, sqrt(a));
}
RPN_Error rpn_cbrt(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, cbrt(a));
}
RPN_Error rpn_sq(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, a * a);
}
RPN_Error rpn_cube(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, a * a * a);
}
RPN_Error rpn_exp(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, exp(a));
}
RPN_Error rpn_exp2(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, exp2(a));
}
RPN_Error rpn_log(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, log(a));
}
RPN_Error rpn_log10(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, log10(a));
}
RPN_Error rpn_log2(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a <= 0.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, log2(a));
}
RPN_Error rpn_sin(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, sin(a * DEG2RAD));
}
RPN_Error rpn_cos(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, cos(a * DEG2RAD));
}
RPN_Error rpn_tan(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, tan(a * DEG2RAD));
}
RPN_Error rpn_asin(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a < -1.0 || a > 1.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, asin(a) * RAD2DEG);
}
RPN_Error rpn_acos(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    if (a < -1.0 || a > 1.0) { s->last_err = RPN_ERR_DOMAIN; return RPN_ERR_DOMAIN; }
    return rpn_push(s, acos(a) * RAD2DEG);
}
RPN_Error rpn_atan(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, atan(a) * RAD2DEG);
}
RPN_Error rpn_atan2(RPN_Stack *s) {
    double a, b; RPN_Error e = _pop2(s,&a,&b); if(e) return e;
    return _binop_result(s, a, b, atan2(a, b) * RAD2DEG);
}
RPN_Error rpn_sinh(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, sinh(a));
}
RPN_Error rpn_cosh(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, cosh(a));
}
RPN_Error rpn_tanh(RPN_Stack *s) {
    double a; RPN_Error e = _pop1(s,&a); if(e) return e;
    return _unop_result(s, tanh(a));
}
 
/* ================================================================
   main.c — infrastruttura di test e suite
   ================================================================ */
 
static uint32_t tests_run    = 0;
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;
 
#define EPSILON 1e-9
 
static void suite(const char *name) {
    printf("\n--- %s ---\n", name);
}
 
static void check(const char *label, RPN_Stack *s, RPN_Error err, double expected) {
    tests_run++;
    if (err != RPN_OK) {
        printf("  FAIL [%s]  err=%s\n", label, rpn_err_str(err));
        tests_failed++;
        return;
    }
    double got = rpn_get(s, 0);
    double tol = EPSILON * (1.0 + fabs(expected));
    if (fabs(got - expected) <= tol) {
        printf("  PASS [%s]  = %.10g\n", label, got);
        tests_passed++;
    } else {
        printf("  FAIL [%s]  got=%.10g  expected=%.10g\n", label, got, expected);
        tests_failed++;
    }
}
 
static void check_err(const char *label, RPN_Error got, RPN_Error expected) {
    tests_run++;
    if (got == expected) {
        printf("  PASS [%s]  error=%s\n", label, rpn_err_str(got));
        tests_passed++;
    } else {
        printf("  FAIL [%s]  got=%s  expected=%s\n",
               label, rpn_err_str(got), rpn_err_str(expected));
        tests_failed++;
    }
}
 
static void check_depth(const char *label, RPN_Stack *s, uint8_t expected) {
    tests_run++;
    uint8_t got = rpn_depth(s);
    if (got == expected) {
        printf("  PASS [%s]  depth=%d\n", label, got);
        tests_passed++;
    } else {
        printf("  FAIL [%s]  depth=%d  expected=%d\n", label, got, expected);
        tests_failed++;
    }
}
 
static void load1(RPN_Stack *s, double a) {
    rpn_reset(s); rpn_push(s, a);
}
 
static void load2(RPN_Stack *s, double a, double b) {
    rpn_reset(s); rpn_push(s, a); rpn_push(s, b);
}
 
/* ---- suite: stack operations ---- */
static void test_stack_ops(void) {
    RPN_Stack s;
    double v;
 
    suite("Stack operations");
 
    rpn_init(&s);
    rpn_push(&s, 1.0); rpn_push(&s, 2.0); rpn_push(&s, 3.0);
    check_depth("push x3", &s, 3);
 
    rpn_peek(&s, &v);
    tests_run++;
    if (v == 3.0) { printf("  PASS [peek top=3.0]\n"); tests_passed++; }
    else          { printf("  FAIL [peek top]\n");      tests_failed++; }
 
    rpn_pop(&s, &v);
    check_depth("pop", &s, 2);
 
    rpn_push(&s, 99.0); rpn_drop(&s);
    check_depth("drop", &s, 2);
 
    rpn_reset(&s);
    rpn_push(&s, 10.0); rpn_push(&s, 20.0);
    rpn_swap(&s);
    tests_run++;
    if (rpn_get(&s,0)==10.0 && rpn_get(&s,1)==20.0) {
        printf("  PASS [swap X=10 Y=20]\n"); tests_passed++;
    } else {
        printf("  FAIL [swap] X=%.1f Y=%.1f\n", rpn_get(&s,0), rpn_get(&s,1));
        tests_failed++;
    }
 
    rpn_reset(&s);
    rpn_push(&s, 7.0); rpn_dup(&s);
    check_depth("dup depth=2", &s, 2);
    tests_run++;
    if (rpn_get(&s,0)==7.0 && rpn_get(&s,1)==7.0) {
        printf("  PASS [dup X=Y=7]\n"); tests_passed++;
    } else {
        printf("  FAIL [dup]\n"); tests_failed++;
    }
 
    rpn_reset(&s);
    rpn_push(&s, 1.0); rpn_push(&s, 2.0); rpn_push(&s, 3.0);
    rpn_roll_down(&s);
    tests_run++;
    if (rpn_get(&s,0)==1.0 && rpn_get(&s,1)==3.0 && rpn_get(&s,2)==2.0) {
        printf("  PASS [roll_down]\n"); tests_passed++;
    } else {
        printf("  FAIL [roll_down] X=%.1f Y=%.1f Z=%.1f\n",
               rpn_get(&s,0), rpn_get(&s,1), rpn_get(&s,2));
        tests_failed++;
    }
 
    rpn_clear(&s);
    check_depth("clear", &s, 0);
 
    rpn_reset(&s);
    rpn_push(&s, 5.0); rpn_push(&s, 8.0); rpn_add(&s);
    tests_run++;
    /* lastx registra il valore di X *prima* dell'ultimo push (=5.0) */
    if (rpn_lastx(&s) == 5.0) {
        printf("  PASS [lastx=5.0 (X prima del push 8)]\n"); tests_passed++;
    } else {
        printf("  FAIL [lastx] got=%.1f\n", rpn_lastx(&s)); tests_failed++;
    }
}
 
/* ---- suite: gestione errori ---- */
static void test_errors(void) {
    RPN_Stack s;
    RPN_Error e;
    double dummy;
 
    suite("Error handling");
 
    rpn_init(&s);
    e = rpn_pop(&s, &dummy);
    check_err("pop vuoto → UNDERFLOW", e, RPN_ERR_UNDERFLOW);
 
    rpn_push(&s, 1.0);
    e = rpn_swap(&s);
    check_err("swap 1 elem → UNDERFLOW", e, RPN_ERR_UNDERFLOW);
 
    load2(&s, 5.0, 0.0);  e = rpn_div(&s);
    check_err("5/0 → DIV_ZERO", e, RPN_ERR_DIV_ZERO);
 
    load1(&s, -4.0);  e = rpn_sqrt(&s);
    check_err("sqrt(-4) → DOMAIN", e, RPN_ERR_DOMAIN);
 
    load1(&s, 0.0);   e = rpn_log(&s);
    check_err("log(0) → DOMAIN", e, RPN_ERR_DOMAIN);
 
    load1(&s, -1.0);  e = rpn_log(&s);
    check_err("log(-1) → DOMAIN", e, RPN_ERR_DOMAIN);
 
    load1(&s, 2.0);   e = rpn_asin(&s);
    check_err("asin(2) → DOMAIN", e, RPN_ERR_DOMAIN);
 
    rpn_reset(&s);
    for (int i = 0; i < RPN_STACK_SIZE; i++) rpn_push(&s, (double)i);
    e = rpn_push(&s, 999.0);
    check_err("push su stack pieno → OVERFLOW", e, RPN_ERR_OVERFLOW);
}
 
/* ---- suite: aritmetica ---- */
static void test_arithmetic(void) {
    RPN_Stack s;
 
    suite("Arithmetic");
 
    load2(&s, 3.0,  4.0);  check("3+4=7",          &s, rpn_add(&s),   7.0);
    load2(&s, 10.0, 3.0);  check("10-3=7",          &s, rpn_sub(&s),   7.0);
    load2(&s, 6.0,  7.0);  check("6×7=42",          &s, rpn_mul(&s),  42.0);
    load2(&s, 22.0, 7.0);  check("22/7",            &s, rpn_div(&s),  22.0/7.0);
    load2(&s, 10.0, 3.0);  check("10 mod 3=1",      &s, rpn_mod(&s),   1.0);
    load2(&s, 2.0, 10.0);  check("2^10=1024",       &s, rpn_pow(&s), 1024.0);
    load1(&s, 5.0);         check("neg(5)=-5",       &s, rpn_neg(&s),  -5.0);
    load1(&s, 4.0);         check("inv(4)=0.25",     &s, rpn_inv(&s),   0.25);
    load1(&s, 4.0);         check("sq(4)=16",        &s, rpn_sq(&s),   16.0);
    load1(&s, 3.0);         check("cube(3)=27",      &s, rpn_cube(&s), 27.0);
}
 
/* ---- suite: radici e logaritmi ---- */
static void test_roots_logs(void) {
    RPN_Stack s;
 
    suite("Roots & logarithms");
 
    load1(&s, 9.0);    check("sqrt(9)=3",      &s, rpn_sqrt(&s),  3.0);
    load1(&s, 27.0);   check("cbrt(27)=3",     &s, rpn_cbrt(&s),  3.0);
    load1(&s, 1.0);    check("exp(1)=e",       &s, rpn_exp(&s),   M_E);
    load1(&s, 1.0);    check("exp2(1)=2",      &s, rpn_exp2(&s),  2.0);
    load1(&s, M_E);    check("log(e)=1",       &s, rpn_log(&s),   1.0);
    load1(&s, 100.0);  check("log10(100)=2",   &s, rpn_log10(&s), 2.0);
    load1(&s, 8.0);    check("log2(8)=3",      &s, rpn_log2(&s),  3.0);
}
 
/* ---- suite: trigonometria (gradi) ---- */
static void test_trig(void) {
    RPN_Stack s;
 
    suite("Trigonometry (degrees)");
 
    load1(&s, 0.0);       check("sin(0°)=0",       &s, rpn_sin(&s),   0.0);
    load1(&s, 90.0);      check("sin(90°)=1",      &s, rpn_sin(&s),   1.0);
    load1(&s, 0.0);       check("cos(0°)=1",       &s, rpn_cos(&s),   1.0);
    load1(&s, 60.0);      check("cos(60°)=0.5",    &s, rpn_cos(&s),   0.5);
    load1(&s, 45.0);      check("tan(45°)=1",      &s, rpn_tan(&s),   1.0);
    load1(&s, 1.0);       check("asin(1)=90°",     &s, rpn_asin(&s), 90.0);
    load1(&s, 1.0);       check("acos(1)=0°",      &s, rpn_acos(&s),  0.0);
    load1(&s, 1.0);       check("atan(1)=45°",     &s, rpn_atan(&s), 45.0);
    load2(&s, 1.0, 1.0);  check("atan2(1,1)=45°",  &s, rpn_atan2(&s),45.0);
 
    /* identità sin²(x) + cos²(x) = 1 */
    double sinv, cosv;
    load1(&s, 37.0); rpn_sin(&s); rpn_sq(&s); rpn_peek(&s, &sinv);
    load1(&s, 37.0); rpn_cos(&s); rpn_sq(&s); rpn_peek(&s, &cosv);
    tests_run++;
    if (fabs(sinv + cosv - 1.0) < 1e-12) {
        printf("  PASS [sin²+cos²=1 @ 37°]\n"); tests_passed++;
    } else {
        printf("  FAIL [sin²+cos²=1] got=%.15g\n", sinv+cosv); tests_failed++;
    }
}
 
/* ---- suite: iperboliche ---- */
static void test_hyperbolic(void) {
    RPN_Stack s;
 
    suite("Hyperbolic");
 
    load1(&s, 0.0);  check("sinh(0)=0",        &s, rpn_sinh(&s), 0.0);
    load1(&s, 0.0);  check("cosh(0)=1",        &s, rpn_cosh(&s), 1.0);
    load1(&s, 0.0);  check("tanh(0)=0",        &s, rpn_tanh(&s), 0.0);
    load1(&s, 1.0);  check("sinh(1)",          &s, rpn_sinh(&s), sinh(1.0));
    load1(&s, 1.0);  check("cosh(1)",          &s, rpn_cosh(&s), cosh(1.0));
}
 
/* ---- suite: scenari RPN reali ---- */
static void test_rpn_scenarios(void) {
    RPN_Stack s;
 
    suite("RPN multi-step scenarios");
 
    /* area cerchio r=5: π × 5² */
    rpn_init(&s);
    rpn_push(&s, 5.0); rpn_sq(&s);
    rpn_push(&s, M_PI); rpn_mul(&s);
    check("area cerchio r=5", &s, RPN_OK, M_PI * 25.0);
 
    /* ipotenusa pitagora 3-4-5 */
    rpn_init(&s);
    rpn_push(&s, 3.0); rpn_sq(&s);
    rpn_push(&s, 4.0); rpn_sq(&s);
    rpn_add(&s); rpn_sqrt(&s);
    check("ipotenusa 3-4-5", &s, RPN_OK, 5.0);
 
    /* formula quadratica: x²-5x+6=0 → x1=3
       x1 = (-b + sqrt(b²-4ac)) / 2a   con a=1 b=-5 c=6  */
    rpn_init(&s);
    rpn_push(&s, 5.0);               /* -b */
    rpn_push(&s, -5.0); rpn_sq(&s);  /* b² = 25 */
    rpn_push(&s, 4.0);
    rpn_push(&s, 1.0); rpn_mul(&s);
    rpn_push(&s, 6.0); rpn_mul(&s);  /* 4ac = 24 */
    rpn_sub(&s);                      /* b²-4ac = 1 */
    rpn_sqrt(&s);                     /* 1 */
    rpn_add(&s);                      /* -b + 1 = 6 */
    rpn_push(&s, 2.0);
    rpn_push(&s, 1.0); rpn_mul(&s);  /* 2a = 2 */
    rpn_div(&s);                      /* x1 = 3 */
    check("quadratica x²-5x+6 → x1=3", &s, RPN_OK, 3.0);
 
    /* conversione dB → lineare: 10^(-6/20) ≈ 0.50119 */
    rpn_init(&s);
    rpn_push(&s, 10.0);
    rpn_push(&s, -6.0);
    rpn_push(&s, 20.0); rpn_div(&s);
    rpn_pow(&s);
    check("-6 dB lineare", &s, RPN_OK, pow(10.0, -6.0/20.0));
 
    /* energia cinetica: Ek = 0.5 × m × v²   m=2kg v=3m/s → 9J */
    rpn_init(&s);
    rpn_push(&s, 0.5);
    rpn_push(&s, 2.0); rpn_mul(&s);   /* 0.5 × m = 1 */
    rpn_push(&s, 3.0); rpn_sq(&s);    /* v² = 9 */
    rpn_mul(&s);                       /* Ek = 9 */
    check("Ek = 0.5·2·3² = 9 J", &s, RPN_OK, 9.0);
 
    /* legge di Ohm: R = V/I  V=12V I=0.5A → R=24Ω */
    rpn_init(&s);
    rpn_push(&s, 12.0);
    rpn_push(&s, 0.5);
    rpn_div(&s);
    check("Ohm R=V/I=24Ω", &s, RPN_OK, 24.0);
 
    /* normalizzazione: (x - mean) / std   x=7 mean=5 std=2 → z=1 */
    rpn_init(&s);
    rpn_push(&s, 7.0);
    rpn_push(&s, 5.0); rpn_sub(&s);
    rpn_push(&s, 2.0); rpn_div(&s);
    check("z-score (7-5)/2=1", &s, RPN_OK, 1.0);
}
 
/* ================================================================
   main
   ================================================================ */
int main(void) {
    printf("========================================\n");
    printf("  RPN Stack Engine — Test Suite\n");
    printf("========================================\n");
 
    test_stack_ops();
    test_errors();
    test_arithmetic();
    test_roots_logs();
    test_trig();
    test_hyperbolic();
    test_rpn_scenarios();
 
    printf("\n========================================\n");
    printf("  Risultati: %u/%u passati", tests_passed, tests_run);
    if (tests_failed > 0)
        printf("  (%u FALLITI)", tests_failed);
    printf("\n========================================\n");
 
    if (tests_failed == 0)
        printf("  Tutti i test superati.\n\n");
    else
        printf("  Alcuni test FALLITI — controllare output sopra.\n\n");
 
    return (tests_failed == 0) ? 0 : 1;
}
