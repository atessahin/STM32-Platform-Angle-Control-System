#include "keypad.h"
#include "spi_communication.h"
// read keypress
char Keypad_Scan(void) {
    char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    for (int row = 0; row < 4; row++) {
        // set all rows high
        GPIOC->BSRR = (1<<0) | (1<<1) | (1<<2) | (1<<3);

        // set current row low
        GPIOC->BSRR = (1 << (row + 16));

        // wait to settle
        for(volatile int i=0; i<100; i++);

        // read cols
        uint16_t idr = GPIOC->IDR;

        if (!(idr & (1<<4))) {
            while(!(GPIOC->IDR & (1<<4))); // debounce
            return keys[row][0];
        }
        if (!(idr & (1<<5))) {
            while(!(GPIOC->IDR & (1<<5)));
            return keys[row][1];
        }
        if (!(idr & (1<<6))) {
            while(!(GPIOC->IDR & (1<<6)));
            return keys[row][2];
        }
        if (!(idr & (1<<7))) {
            while(!(GPIOC->IDR & (1<<7)));
            return keys[row][3];
        }
    }
    return '\0'; // no key pressed
}

// globals for logic
volatile int current_input = 0;   // accumulated num
volatile int is_negative = 0;     // is neg flag
volatile int system_ON = 0;       // on/off flag (A)
volatile int pid_Mode = 0;        // pid flag (B)
volatile int final_target_angle = 0; // target angle

void Process_Keypad(void) {
    char key = Keypad_Scan();
    if (key == '\0') return; // no key pressed

    // 1. numbers
    if (key >= '0' && key <= '9') {
        current_input = (current_input * 10) + (key - '0');
        int temp_val = is_negative ? -current_input : current_input;
        Display_Angle(temp_val); // show live input
    }

    // 2. negative (*)
    else if (key == '*') {
        is_negative = 1;
        Display_Angle(-current_input);
    }

    // 3. enter (#)
    else if (key == '#') {
        final_target_angle = is_negative ? -current_input : current_input;

        // clamp -60 to 60
        if (final_target_angle > 60) final_target_angle = 60;
        if (final_target_angle < -60) final_target_angle = -60;

        Display_Angle(final_target_angle); // stay on screen

        // reset vars for next input
        current_input = 0;
        is_negative = 0;
    }

    // 4. clear/reset (C) - new feature
    else if (key == 'C') {
        current_input = 0;
        is_negative = 0;
        Display_Angle(0); // clear screen
    }

    // 5. toggle flags
    else if (key == 'A') system_ON = !system_ON;
    else if (key == 'B') pid_Mode = !pid_Mode;
}
