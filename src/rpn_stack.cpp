// rpn_stack.cpp
#include "rpn_stack.h"
#include <math.h>
#include <string.h>

void rpn_init(RPN_Stack *s) {
  memset(s, 0, sizeof(RPN_Stack));
}

void rpn_reset(RPN_Stack *s) {
  rpn_init(s);
}

// --- Stack core ---

RPN_Error rpn_push(RPN_Stack *s, double val) {
  if (s->top >= RPN_STACK_SIZE) {
    s->last_err = RPN_ERR_OVERFLOW;
    return RPN_ERR_OVERFLOW;
  }
  // Salva LastX prima di modificare X
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
  double tmp = s->data[s->top - 1];
  s->data[s->top - 1] = s->data[s->top - 2];
  s->data[s->top - 2] = tmp;
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

// --- Registri HP-style (X Y Z T) ---

void _rpn_sync_regs(RPN_Stack *s) {
  s->reg_x = (s->top > 0) ? s->data[s->top - 1] : 0.0;
  s->reg_y = (s->top > 1) ? s->data[s->top - 2] : 0.0;
  s->reg_z = (s->top > 2) ? s->data[s->top - 3] : 0.0;
  s->reg_t = (s->top > 3) ? s->data[s->top - 4] : 0.0;
}

// --- LastX ---

double rpn_lastx(const RPN_Stack *s) {
  if (s->lastx_idx == 0) return 0.0;
  return s->lastx[(s->lastx_idx - 1) % RPN_LASTX_DEPTH];
}

// --- Query ---

uint8_t rpn_depth(const RPN_Stack *s)    { return s->top; }
bool    rpn_is_empty(const RPN_Stack *s) { return s->top == 0; }
bool    rpn_is_full(const RPN_Stack *s)  { return s->top >= RPN_STACK_SIZE; }

double rpn_get(const RPN_Stack *s, uint8_t idx) {
  if (idx >= s->top) return 0.0;
  return s->data[s->top - 1 - idx]; // 0=X (cima), 1=Y, ...
}

// --- Diagnostica ---

const char* rpn_err_str(RPN_Error err) {
  switch (err) {
    case RPN_OK:           return "OK";
    case RPN_ERR_OVERFLOW: return "Stack full";
    case RPN_ERR_UNDERFLOW:return "Too few args";
    case RPN_ERR_DIV_ZERO: return "Div/0";
    case RPN_ERR_DOMAIN:   return "Domain error";
    case RPN_ERR_NAN:      return "Not a number";
    case RPN_ERR_INF:      return "Infinity";
    default:               return "Unknown error";
  }
}
