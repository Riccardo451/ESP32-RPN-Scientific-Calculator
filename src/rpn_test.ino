// rpn_test.ino
// Test runner per rpn_stack + rpn_ops
// Funziona su ESP32 (Serial Monitor) e su PC con gcc (stampa su stdout)
//
// Struttura cartella progetto:
//   rpn_calc/
//     rpn_test.ino       <- questo file
//     rpn_stack.h
//     rpn_stack.cpp
//     rpn_ops.cpp

#include "rpn_stack.h"
#include <math.h>

// ----------------------------------------------------------------
// Dichiarazioni funzioni da rpn_ops.cpp
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
// Infrastruttura di test
// ----------------------------------------------------------------
static uint32_t tests_run    = 0;
static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;
static const char *current_suite = "";

#define EPSILON 1e-9

void suite(const char *name) {
  current_suite = name;
  Serial.print("\n--- ");
  Serial.print(name);
  Serial.println(" ---");
}

// Controlla che err == RPN_OK e il top dello stack sia vicino ad expected
void check(const char *label, RPN_Stack *s, RPN_Error err, double expected) {
  tests_run++;
  bool ok = true;
  char buf[80];

  if (err != RPN_OK) {
    ok = false;
    snprintf(buf, sizeof(buf), "  FAIL [%s] err=%s", label, rpn_err_str(err));
  } else {
    double got = rpn_get(s, 0);
    if (fabs(got - expected) > EPSILON * (1.0 + fabs(expected))) {
      ok = false;
      snprintf(buf, sizeof(buf), "  FAIL [%s] got=%.10g expected=%.10g", label, got, expected);
    } else {
      snprintf(buf, sizeof(buf), "  PASS [%s] = %.10g", label, got);
    }
  }

  Serial.println(buf);
  if (ok) tests_passed++;
  else     tests_failed++;
}

// Controlla che err sia esattamente quello atteso (test di errore)
void check_err(const char *label, RPN_Error got, RPN_Error expected) {
  tests_run++;
  char buf[80];
  if (got == expected) {
    snprintf(buf, sizeof(buf), "  PASS [%s] error=%s (expected)", label, rpn_err_str(got));
    tests_passed++;
  } else {
    snprintf(buf, sizeof(buf), "  FAIL [%s] got=%s expected=%s", label, rpn_err_str(got), rpn_err_str(expected));
    tests_failed++;
  }
  Serial.println(buf);
}

// Controlla depth dello stack
void check_depth(const char *label, RPN_Stack *s, uint8_t expected) {
  tests_run++;
  uint8_t got = rpn_depth(s);
  char buf[80];
  if (got == expected) {
    snprintf(buf, sizeof(buf), "  PASS [%s] depth=%d", label, got);
    tests_passed++;
  } else {
    snprintf(buf, sizeof(buf), "  FAIL [%s] depth=%d expected=%d", label, got, expected);
    tests_failed++;
  }
  Serial.println(buf);
}

// Helper: reset + push rapido
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
// Suite: stack operations
// ----------------------------------------------------------------
void test_stack_ops() {
  RPN_Stack s;
  RPN_Error e;
  double v;

  suite("Stack operations");

  // push / depth / peek
  rpn_init(&s);
  rpn_push(&s, 1.0);
  rpn_push(&s, 2.0);
  rpn_push(&s, 3.0);
  check_depth("push x3 depth=3", &s, 3);
  rpn_peek(&s, &v);
  tests_run++;
  if (v == 3.0) { Serial.println("  PASS [peek top=3.0]"); tests_passed++; }
  else          { Serial.println("  FAIL [peek top]");      tests_failed++; }

  // pop
  rpn_pop(&s, &v);
  check_depth("pop depth=2", &s, 2);

  // drop
  rpn_push(&s, 99.0);
  rpn_drop(&s);
  check_depth("drop depth=2", &s, 2);

  // swap: Y=1, X=2 → Y=2, X=1
  rpn_reset(&s);
  rpn_push(&s, 10.0);
  rpn_push(&s, 20.0);
  rpn_swap(&s);
  tests_run++;
  if (rpn_get(&s, 0) == 10.0 && rpn_get(&s, 1) == 20.0) {
    Serial.println("  PASS [swap X=10 Y=20]"); tests_passed++;
  } else {
    Serial.println("  FAIL [swap]"); tests_failed++;
  }

  // dup
  rpn_reset(&s);
  rpn_push(&s, 7.0);
  rpn_dup(&s);
  check_depth("dup depth=2", &s, 2);
  tests_run++;
  if (rpn_get(&s, 0) == 7.0 && rpn_get(&s, 1) == 7.0) {
    Serial.println("  PASS [dup X=Y=7]"); tests_passed++;
  } else {
    Serial.println("  FAIL [dup]"); tests_failed++;
  }

  // roll_down: [1,2,3] → [2,3,1]  (bottom→top)
  rpn_reset(&s);
  rpn_push(&s, 1.0); rpn_push(&s, 2.0); rpn_push(&s, 3.0);
  rpn_roll_down(&s);
  tests_run++;
  if (rpn_get(&s, 0) == 1.0 && rpn_get(&s, 1) == 3.0 && rpn_get(&s, 2) == 2.0) {
    Serial.println("  PASS [roll_down]"); tests_passed++;
  } else {
    Serial.printf("  FAIL [roll_down] X=%.1f Y=%.1f Z=%.1f\n",
      rpn_get(&s,0), rpn_get(&s,1), rpn_get(&s,2));
    tests_failed++;
  }

  // clear
  rpn_clear(&s);
  check_depth("clear depth=0", &s, 0);

  // lastx: push 5, push 8, op → lastx deve essere 8
  rpn_reset(&s);
  rpn_push(&s, 5.0);
  rpn_push(&s, 8.0);
  rpn_add(&s);
  tests_run++;
  if (rpn_lastx(&s) == 8.0) {
    Serial.println("  PASS [lastx=8.0]"); tests_passed++;
  } else {
    Serial.printf("  FAIL [lastx] got=%.1f\n", rpn_lastx(&s)); tests_failed++;
  }
}

// ----------------------------------------------------------------
// Suite: errori attesi
// ----------------------------------------------------------------
void test_errors() {
  RPN_Stack s;
  RPN_Error e;

  suite("Error handling");

  rpn_init(&s);
  // underflow: pop su stack vuoto
  double dummy;
  e = rpn_pop(&s, &dummy);
  check_err("pop empty → UNDERFLOW", e, RPN_ERR_UNDERFLOW);

  // swap con <2 elementi
  rpn_push(&s, 1.0);
  e = rpn_swap(&s);
  check_err("swap 1 elem → UNDERFLOW", e, RPN_ERR_UNDERFLOW);

  // divisione per zero
  load2(&s, 5.0, 0.0);
  e = rpn_div(&s);
  check_err("5/0 → DIV_ZERO", e, RPN_ERR_DIV_ZERO);

  // sqrt di negativo
  load1(&s, -4.0);
  e = rpn_sqrt(&s);
  check_err("sqrt(-4) → DOMAIN", e, RPN_ERR_DOMAIN);

  // log di zero
  load1(&s, 0.0);
  e = rpn_log(&s);
  check_err("log(0) → DOMAIN", e, RPN_ERR_DOMAIN);

  // log di negativo
  load1(&s, -1.0);
  e = rpn_log(&s);
  check_err("log(-1) → DOMAIN", e, RPN_ERR_DOMAIN);

  // asin fuori dominio
  load1(&s, 2.0);
  e = rpn_asin(&s);
  check_err("asin(2) → DOMAIN", e, RPN_ERR_DOMAIN);

  // overflow stack
  rpn_reset(&s);
  for (int i = 0; i < RPN_STACK_SIZE; i++) rpn_push(&s, (double)i);
  e = rpn_push(&s, 999.0);
  check_err("push su stack pieno → OVERFLOW", e, RPN_ERR_OVERFLOW);
}

// ----------------------------------------------------------------
// Suite: aritmetica base
// ----------------------------------------------------------------
void test_arithmetic() {
  RPN_Stack s;

  suite("Arithmetic");

  load2(&s, 3.0, 4.0);   check("3+4=7",         &s, rpn_add(&s),   7.0);
  load2(&s, 10.0, 3.0);  check("10-3=7",         &s, rpn_sub(&s),   7.0);
  load2(&s, 6.0, 7.0);   check("6×7=42",         &s, rpn_mul(&s),  42.0);
  load2(&s, 22.0, 7.0);  check("22/7≈3.142857",  &s, rpn_div(&s),  22.0/7.0);
  load2(&s, 10.0, 3.0);  check("10 mod 3=1",     &s, rpn_mod(&s),   1.0);
  load2(&s, 2.0, 10.0);  check("2^10=1024",      &s, rpn_pow(&s),1024.0);
  load1(&s, 5.0);         check("neg(5)=-5",      &s, rpn_neg(&s),  -5.0);
  load1(&s, 4.0);         check("inv(4)=0.25",    &s, rpn_inv(&s),   0.25);
  load1(&s, 4.0);         check("sq(4)=16",       &s, rpn_sq(&s),   16.0);
  load1(&s, 3.0);         check("cube(3)=27",     &s, rpn_cube(&s), 27.0);
}

// ----------------------------------------------------------------
// Suite: radici e logaritmi
// ----------------------------------------------------------------
void test_roots_logs() {
  RPN_Stack s;

  suite("Roots & logarithms");

  load1(&s, 9.0);    check("sqrt(9)=3",       &s, rpn_sqrt(&s),  3.0);
  load1(&s, 27.0);   check("cbrt(27)=3",      &s, rpn_cbrt(&s),  3.0);
  load1(&s, 1.0);    check("exp(1)=e",        &s, rpn_exp(&s),   M_E);
  load1(&s, 1.0);    check("exp2(1)=2",       &s, rpn_exp2(&s),  2.0);
  load1(&s, M_E);    check("log(e)=1",        &s, rpn_log(&s),   1.0);
  load1(&s, 100.0);  check("log10(100)=2",    &s, rpn_log10(&s), 2.0);
  load1(&s, 8.0);    check("log2(8)=3",       &s, rpn_log2(&s),  3.0);
}

// ----------------------------------------------------------------
// Suite: trigonometria (input in gradi)
// ----------------------------------------------------------------
void test_trig() {
  RPN_Stack s;

  suite("Trigonometry (degrees)");

  load1(&s, 0.0);    check("sin(0°)=0",      &s, rpn_sin(&s),  0.0);
  load1(&s, 90.0);   check("sin(90°)=1",     &s, rpn_sin(&s),  1.0);
  load1(&s, 0.0);    check("cos(0°)=1",      &s, rpn_cos(&s),  1.0);
  load1(&s, 60.0);   check("cos(60°)=0.5",   &s, rpn_cos(&s),  0.5);
  load1(&s, 45.0);   check("tan(45°)=1",     &s, rpn_tan(&s),  1.0);
  load1(&s, 1.0);    check("asin(1)=90°",    &s, rpn_asin(&s), 90.0);
  load1(&s, 1.0);    check("acos(1)=0°",     &s, rpn_acos(&s), 0.0);
  load1(&s, 1.0);    check("atan(1)=45°",    &s, rpn_atan(&s), 45.0);
  load2(&s, 1.0, 1.0); check("atan2(1,1)=45°",&s, rpn_atan2(&s),45.0);

  // identità fondamentale: sin²(x) + cos²(x) = 1
  RPN_Stack s2;
  rpn_init(&s2);
  rpn_push(&s2, 37.0); rpn_dup(&s2);
  rpn_sin(&s2); rpn_sq(&s2);
  // ora swap, cos, sq, add
  double sinq = rpn_get(&s2, 0); rpn_drop(&s2);
  rpn_push(&s2, 37.0); rpn_cos(&s2); rpn_sq(&s2);
  double cosq = rpn_get(&s2, 0);
  tests_run++;
  if (fabs(sinq + cosq - 1.0) < 1e-12) {
    Serial.println("  PASS [sin²+cos²=1 @ 37°]"); tests_passed++;
  } else {
    Serial.printf("  FAIL [sin²+cos²=1] got=%.15g\n", sinq+cosq); tests_failed++;
  }
}

// ----------------------------------------------------------------
// Suite: funzioni iperboliche
// ----------------------------------------------------------------
void test_hyperbolic() {
  RPN_Stack s;

  suite("Hyperbolic");

  load1(&s, 0.0);  check("sinh(0)=0",   &s, rpn_sinh(&s), 0.0);
  load1(&s, 0.0);  check("cosh(0)=1",   &s, rpn_cosh(&s), 1.0);
  load1(&s, 0.0);  check("tanh(0)=0",   &s, rpn_tanh(&s), 0.0);
  load1(&s, 1.0);  check("sinh(1)=1.1752", &s, rpn_sinh(&s), sinh(1.0));
  load1(&s, 1.0);  check("cosh(1)=1.5430", &s, rpn_cosh(&s), cosh(1.0));
}

// ----------------------------------------------------------------
// Suite: scenari RPN reali (calcoli multi-step)
// ----------------------------------------------------------------
void test_rpn_scenarios() {
  RPN_Stack s;

  suite("RPN multi-step scenarios");

  // Scenario 1: area cerchio r=5 → π × 5² = 78.5398...
  rpn_init(&s);
  rpn_push(&s, 5.0);
  rpn_sq(&s);
  rpn_push(&s, M_PI);
  rpn_mul(&s);
  check("area cerchio r=5", &s, RPN_OK, M_PI * 25.0);

  // Scenario 2: ipotenusa pitagora a=3, b=4 → sqrt(3²+4²) = 5
  rpn_init(&s);
  rpn_push(&s, 3.0); rpn_sq(&s);
  rpn_push(&s, 4.0); rpn_sq(&s);
  rpn_add(&s);
  rpn_sqrt(&s);
  check("ipotenusa 3-4-5", &s, RPN_OK, 5.0);

  // Scenario 3: formula quadratica x = (-b + sqrt(b²-4ac)) / 2a
  // equazione: 1x² - 5x + 6 = 0  → radici 2 e 3
  // a=1, b=-5, c=6  →  x1 = (5 + sqrt(25-24)) / 2 = 3
  rpn_init(&s);
  double a=1.0, b=-5.0, c=6.0;
  rpn_push(&s, b); rpn_neg(&s);          // -b = 5
  rpn_push(&s, b); rpn_sq(&s);           // b² = 25
  rpn_push(&s, 4.0);
  rpn_push(&s, a); rpn_mul(&s);
  rpn_push(&s, c); rpn_mul(&s);          // 4ac = 24
  rpn_sub(&s);                           // b²-4ac = 1
  rpn_sqrt(&s);                          // sqrt(1) = 1
  rpn_add(&s);                           // -b + sqrt(...) = 6
  rpn_push(&s, 2.0);
  rpn_push(&s, a); rpn_mul(&s);          // 2a = 2
  rpn_div(&s);                           // x1 = 3
  check("quadratica x²-5x+6 → x1=3", &s, RPN_OK, 3.0);

  // Scenario 4: conversione dB → lineare  val = 10^(dB/20)
  // -6 dB ≈ 0.50119 (tensione)
  rpn_init(&s);
  rpn_push(&s, -6.0);
  rpn_push(&s, 20.0);
  rpn_div(&s);
  rpn_exp2(&s);  // 2^(dB/20) — alternativa: push(10), swap, pow
  // usiamo la formula esatta: 10^(-6/20)
  rpn_init(&s);
  rpn_push(&s, 10.0);
  rpn_push(&s, -6.0);
  rpn_push(&s, 20.0);
  rpn_div(&s);
  rpn_pow(&s);
  check("-6dB lineare ≈ 0.50119", &s, RPN_OK, pow(10.0, -6.0/20.0));
}

// ----------------------------------------------------------------
// setup / loop
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  RPN Stack Engine — Test Suite");
  Serial.println("========================================");

  test_stack_ops();
  test_errors();
  test_arithmetic();
  test_roots_logs();
  test_trig();
  test_hyperbolic();
  test_rpn_scenarios();

  Serial.println("\n========================================");
  Serial.printf("  Risultati: %lu/%lu passati", tests_passed, tests_run);
  if (tests_failed > 0)
    Serial.printf("  (%lu FALLITI)", tests_failed);
  Serial.println("\n========================================");

  if (tests_failed == 0)
    Serial.println("  Tutti i test superati.");
  else
    Serial.println("  Alcuni test FALLITI — controllare output sopra.");
}

void loop() {
  // nulla — i test girano una volta sola in setup()
}
