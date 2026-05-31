#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f4xx.h"

// globals for logic (extern for main.c access)
// globals for logic (extern for main.c access)
extern volatile int current_input;
extern volatile int is_negative;
extern volatile int system_ON;
extern volatile int pid_Mode;
extern volatile int final_target_angle;

// read keypress
char Keypad_Scan();

// process keypress logic
void Process_Keypad();

#endif /* KEYPAD_H */
