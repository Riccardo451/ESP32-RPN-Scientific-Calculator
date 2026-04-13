// rpn_stack.h
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RPN_STACK_SIZE   16      // slot disponibili (modificabile)
#define RPN_LASTX_DEPTH   4      // storico LastX per UNDO

typedef enum {
  RPN_OK = 0,
  RPN_ERR_OVERFLOW,    // push su stack pieno
  RPN_ERR_UNDERFLOW,   // pop su stack vuoto
  RPN_ERR_DIV_ZERO,
  RPN_ERR_DOMAIN,      // sqrt(-1), log(-1), ecc.
  RPN_ERR_NAN,
  RPN_ERR_INF,
} RPN_Error;

typedef struct {
  double   data[RPN_STACK_SIZE];
  uint8_t  top;                    // indice del prossimo slot libero
  double   lastx[RPN_LASTX_DEPTH]; // registro LastX (stile HP)
  uint8_t  lastx_idx;
  double   reg_x, reg_y, reg_z, reg_t; // alias HP: X Y Z T
  RPN_Error last_err;
} RPN_Stack;

// Ciclo di vita
void    rpn_init   (RPN_Stack *s);
void    rpn_reset  (RPN_Stack *s);

// Operazioni stack fondamentali
RPN_Error rpn_push     (RPN_Stack *s, double val);
RPN_Error rpn_pop      (RPN_Stack *s, double *out);
RPN_Error rpn_peek     (const RPN_Stack *s, double *out);
RPN_Error rpn_drop     (RPN_Stack *s);          // scarta X
RPN_Error rpn_swap     (RPN_Stack *s);          // swap X<->Y
RPN_Error rpn_dup      (RPN_Stack *s);          // duplica X
RPN_Error rpn_roll_down(RPN_Stack *s);          // ruota stack (HP R↓)
RPN_Error rpn_clear    (RPN_Stack *s);          // svuota tutto

// Registro LastX
double    rpn_lastx    (const RPN_Stack *s);

// Query
uint8_t   rpn_depth    (const RPN_Stack *s);
bool      rpn_is_empty (const RPN_Stack *s);
bool      rpn_is_full  (const RPN_Stack *s);
double    rpn_get      (const RPN_Stack *s, uint8_t idx); // 0=X, 1=Y, ...

// Diagnostica
const char* rpn_err_str(RPN_Error err);
