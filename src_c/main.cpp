// main.cpp — portable test runner (PC + Arduino)

#include <math.h>
#include <stdint.h>

#ifdef ARDUINO
  #include <Arduino.h>
#endif

#include "rpn_stack.h"

// ----------------------------------------------------------------
// PRINT ABSTRACTION (Arduino + PC)
// ----------------------------------------------------------------
#ifdef ARDUINO
  #define PRINT(...)   Serial.print(__VA_ARGS__)
  #define PRINTLN(...) Serial.println(__VA_ARGS__)
  #define PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #include <stdio.h>
  #define PRINT(...)   printf(__VA_ARGS__)
  #define PRINTLN(...) do { printf(__VA_ARGS__); printf("\n"); } while(0)
  #define PRINTF(...)  printf(__VA_ARGS__)
#endif

// ----------------------------------------------------------------
// rpn_ops declarations
// ----------------------------------------------------------------
RPN_Error rpn_add  (RPN_Stack *s);
RPN_Error rpn_sub  (RPN_Stack *s);
RPN_Error rpn_mul  (RPN_Stack *s);
RPN_Error rpn_div  (RPN_Stack *s);
RPN_Error rpn_mod  (RPN_Stack *s);
RPN_Error rpn_pow  (RPN_Stack *s);
RPN_Error rpn_neg  (RPN_Stack *s);
RPN_Error rpn_inv  (RPN_Stack *s);
RPN_Error rpn_sqrt (RPN_Stack *s);
RPN_Error rpn_cbrt (RPN_Stack *s);
RPN_Error rpn_sq   (RPN_Stack *s);
RPN_Error rpn_cube (RPN_Stack *s);
RPN_Error rpn_exp  (RPN_Stack *s);
RPN_Error rpn_exp2 (RPN_Stack *s);
RPN_Error rpn_log  (RPN_Stack *s);
RPN_Error rpn_log10(RPN_Stack *s);
RPN_Error rpn_log2 (RPN_Stack *s);
RPN_Error rpn_sin  (RPN_Stack *s);
RPN_Error rpn_cos  (RPN_Stack *s);
RPN_Error rpn_tan  (RPN_Stack *s);
RPN_Error rpn_asin (RPN_Stack *s);
RPN_Error rpn_acos (RPN_Stack *s);
RPN_Error rpn_atan (RPN_Stack *s);
RPN_Error rpn_atan2(RPN_Stack *s);
RPN_Error rpn_sinh (RPN_Stack *s);
RPN_Error rpn_cosh (RPN_Stack *s);
RPN_Error rpn_tanh (RPN_Stack *s);

// ----------------------------------------------------------------
// TEST PROTOTYPES (FIX for C++)
// ----------------------------------------------------------------
void test_stack_ops();
void test_errors();
void test_arithmetic();
void test_roots_logs();
void test_trig();
void test_hyperbolic();
void test_rpn_scenarios();

// ----------------------------------------------------------------
// TEST INFRASTRUCTURE
// ----------------------------------------------------------------
static uint32_t tests_run    = 0;
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;

#define EPSILON 1e-9

void suite(const char *name) {
  PRINTLN("");
  PRINT("--- ");
  PRINT(name);
  PRINTLN(" ---");
}

void check(const char *label, RPN_Stack *s, RPN_Error err, double expected) {
  tests_run++;
  bool ok = true;

  if (err != RPN_OK) {
    ok = false;
    PRINTF("  FAIL [%s] err=%s\n", label, rpn_err_str(err));
  } else {
    double got = rpn_get(s, 0);
    if (fabs(got - expected) > EPSILON * (1.0 + fabs(expected))) {
      ok = false;
      PRINTF("  FAIL [%s] got=%.10g expected=%.10g\n", label, got, expected);
    } else {
      PRINTF("  PASS [%s] = %.10g\n", label, got);
    }
  }

  if (ok) tests_passed++;
  else    tests_failed++;
}

void check_err(const char *label, RPN_Error got, RPN_Error expected) {
  tests_run++;

  if (got == expected) {
    PRINTF("  PASS [%s] error=%s\n", label, rpn_err_str(got));
    tests_passed++;
  } else {
    PRINTF("  FAIL [%s] got=%s expected=%s\n",
           label, rpn_err_str(got), rpn_err_str(expected));
    tests_failed++;
  }
}

void check_depth(const char *label, RPN_Stack *s, uint8_t expected) {
  tests_run++;
  uint8_t got = rpn_depth(s);

  if (got == expected) {
    PRINTF("  PASS [%s] depth=%d\n", label, got);
    tests_passed++;
  } else {
    PRINTF("  FAIL [%s] depth=%d expected=%d\n", label, got, expected);
    tests_failed++;
  }
}

// helpers
void load1(RPN_Stack *s, double a) {
  rpn_reset(s);
  rpn_push(s, a);
}

void load2(RPN_Stack *s, double a, double b) {
  rpn_reset(s);
  rpn_push(s, a);
  rpn_push(s, b);
}

// ----------------------------------------------------------------
// TESTS (shortened but complete core)
// ----------------------------------------------------------------
void test_stack_ops() {
  RPN_Stack s;
  double v;

  suite("Stack operations");

  rpn_init(&s);
  rpn_push(&s, 1.0);
  rpn_push(&s, 2.0);
  rpn_push(&s, 3.0);

  check_depth("push x3 depth=3", &s, 3);

  rpn_peek(&s, &v);
  tests_run++;
  if (v == 3.0) { PRINTLN("  PASS [peek top=3.0]"); tests_passed++; }
  else          { PRINTLN("  FAIL [peek]"); tests_failed++; }
}

void test_errors() {
  RPN_Stack s;
  suite("Error handling");

  rpn_init(&s);
  double dummy;
  check_err("pop empty", rpn_pop(&s, &dummy), RPN_ERR_UNDERFLOW);
}

void test_arithmetic() {
  RPN_Stack s;
  suite("Arithmetic");

  load2(&s, 3.0, 4.0);
  check("3+4", &s, rpn_add(&s), 7.0);
}

void test_roots_logs() {
  RPN_Stack s;
  suite("Roots");

  load1(&s, 9.0);
  check("sqrt(9)", &s, rpn_sqrt(&s), 3.0);
}

void test_trig() {
  RPN_Stack s;
  suite("Trig");

  load1(&s, 90.0);
  check("sin(90)", &s, rpn_sin(&s), 1.0);
}

void test_hyperbolic() {
  RPN_Stack s;
  suite("Hyperbolic");

  load1(&s, 0.0);
  check("sinh(0)", &s, rpn_sinh(&s), 0.0);
}

void test_rpn_scenarios() {
  RPN_Stack s;
  suite("Scenarios");

  rpn_init(&s);
  rpn_push(&s, 5.0);
  rpn_sq(&s);
  rpn_push(&s, M_PI);
  rpn_mul(&s);

  check("circle area", &s, RPN_OK, M_PI * 25.0);
}

// ----------------------------------------------------------------
// MAIN / SETUP
// ----------------------------------------------------------------
#ifdef ARDUINO

void setup() {
  Serial.begin(115200);
  delay(500);

#else

int main() {

#endif

  PRINTLN("\n========================================");
  PRINTLN("  RPN Stack Engine — Test Suite");
  PRINTLN("========================================");

  test_stack_ops();
  test_errors();
  test_arithmetic();
  test_roots_logs();
  test_trig();
  test_hyperbolic();
  test_rpn_scenarios();

  PRINTLN("\n========================================");
  PRINTF("  Risultati: %lu/%lu passati", tests_passed, tests_run);
  if (tests_failed)
    PRINTF(" (%lu FALLITI)", tests_failed);
  PRINTLN("\n========================================");

#ifdef ARDUINO
}

void loop() {}
#else
  return 0;
}
#endif
