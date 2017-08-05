#ifndef FILE_DEFINE
#define FILE_DEFINE

#define A0 100
#define A1 101
#define A2 102
#define A3 103
#define A4 104
#define A5 105
#define A6 106
#define A7 107
#define A8 108
#define A9 109
#define A10 110
#define A11 111
#define A12 112
#define A13 113
#define A14 114
#define A15 115

#define B0 200
#define B1 201
#define B2 202
#define B3 203
#define B4 204
#define B5 205
#define B6 206
#define B7 207
#define B8 208
#define B9 209
#define B10 210
#define B11 211
#define B12 212
#define B13 213
#define B14 214
#define B15 215

#define C0 300
#define C1 301
#define C2 302
#define C3 303
#define C4 304
#define C5 305
#define C6 306
#define C7 307
#define C8 308
#define C9 309
#define C10 310
#define C11 311
#define C12 312
#define C13 313
#define C14 314
#define C15 315

#define D0 300
#define D1 401
#define D2 402
#define D3 403
#define D4 404
#define D5 405
#define D6 406
#define D7 407
#define D8 408
#define D9 409
#define D10 410
#define D11 411
#define D12 412
#define D13 413
#define D14 414
#define D15 415

#define E0 500
#define E1 501
#define E2 502
#define E3 503
#define E4 504
#define E5 505
#define E6 506
#define E7 507
#define E8 508
#define E9 509
#define E10 510
#define E11 511
#define E12 512
#define E13 513
#define E14 514
#define E15 515
#define H0 600
#define H1 601

typedef enum port{A, B, C, D, E, H} v_port;
typedef enum mode{PWM, Output, Input, Alt, Analog, Exti} mode;

#define bool uint8_t
#define true 1
#define false 0

#endif
