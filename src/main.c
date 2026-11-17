#include "xc.h"
#include "drivers.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

// === Global Variables ===
parser_state pstate;
int speed = 0, yawrate = 0;
volatile int state = STATE_WAIT;
volatile unsigned int counter1ms = 0;
unsigned int emergencyCounter = 0;
volatile bool printingFlag = false;

CircularBuffer cb;        // TX buffer
CMDCircularBuffer cbCMD;  // Command RX buffer

// === UART TX ISR ===
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    IFS0bits.U1TXIF = 0;
    char ch;
    if (dequeue(&cb, &ch)) {
        U1TXREG = ch;
    }
}

// === UART RX ISR ===
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    IFS0bits.U1RXIF = 0;
    char rec = U1RXREG;

    if (parse_byte(&pstate, rec)) {
        if (CMDisFull(&cbCMD)) {
            enqueue_buffer(&cb, "$MACK,0*");  // FIFO full
        } else {
            CMDenqueue(&cbCMD, pstate);
            enqueue_buffer(&cb, "$MACK,1*");
        }
        printingFlag = true;
        IFS0bits.U1TXIF = 1;
    }
}

// === Debounce Timer ISR (50ms) ===
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt() {
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 0;
    T1CONbits.TON = 0;

    if (PORTEbits.RE8 == 1 && state != STATE_EMERGENCY) {
        state = (state == STATE_WAIT) ? STATE_MOVING : STATE_WAIT;
    }
}

// === External Button Press ISR (starts debounce) ===
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() {
    IFS1bits.INT1IF = 0;
    IEC0bits.T1IE = 1;
    tmr_setup_period(TIMER1, 50); // debounce period
}

// === Main ===
int main(void) {
    init_adc();
    initCircularBuffer(&cb);
    CMDinitCircularBuffer(&cbCMD);
    init_UART1(true, true);
    set_up_PWM_wheels();
    init_LED();
    init_SPI1();
    setup_mag();

    pstate.state = STATE_DOLLAR;

    // Setup button interrupt
    TRISEbits.TRISE8 = 1;
    RPINR0bits.INT1R = 88;
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;

    tmr_setup_period(TIMER2, 2); // 500Hz loop
    double battery, distance;

    while (1) {
        get_distance_and_battery(&distance, &battery);
        char msg[BUFFER_SIZE];

        // === UART Periodic Messages ===
        if (counter1ms % 50 == 0) {
            sprintf(msg, "$MDIST,%d*", (int)distance);
            enqueue_buffer(&cb, msg);

            sprintf(msg, "$MACC,%d,%d,%d*", mag_get_x(), mag_get_y(), mag_get_z());
            enqueue_buffer(&cb, msg);
            printingFlag = true;
        }

        if (counter1ms % 500 == 0) {
            sprintf(msg, "$MBATT,%.2f*", battery);
            enqueue_buffer(&cb, msg);
            LED = !LED;
            printingFlag = true;
        }

        if (!isEmpty(&cb) && printingFlag) {
            IFS0bits.U1TXIF = 1;
            printingFlag = false;
        }

        // === Command Processor ===
        parser_state cmd;
        if (CMDdequeue(&cbCMD, &cmd)) {
            if (strcmp(cmd.msg_type, "PCREF") == 0) {
                int i = 0;
                speed = extract_integer(&cmd.msg_payload[i]);
                i = next_value(cmd.msg_payload, i);
                yawrate = extract_integer(&cmd.msg_payload[i]);

                // Clamp to [-100, 100]
                if (speed > 100) speed = 100;
                if (speed < -100) speed = -100;
                if (yawrate > 100) yawrate = 100;
                if (yawrate < -100) yawrate = -100;

            } else if (strcmp(cmd.msg_type, "PCSTP") == 0) {
                if (state != STATE_EMERGENCY) state = STATE_WAIT;
            } else if (strcmp(cmd.msg_type, "PCSTT") == 0) {
                if (state != STATE_EMERGENCY) state = STATE_MOVING;
            }
        }

        // === FSM ===
        switch (state) {
            case STATE_WAIT:
                stop_moving();
                break;

            case STATE_MOVING:
                if (distance < 20) {
                    state = STATE_EMERGENCY;
                    stop_moving();
                    enqueue_buffer(&cb, "$MEMRG,1*");
                    printingFlag = true;
                    IFS0bits.U1TXIF = 1;
                    emergencyCounter = 0;
                } else {
                    int left = speed - yawrate;
                    int right = speed + yawrate;
                    int left_pwm = (PWMFREQUENCY * abs(left)) / 100;
                    int right_pwm = (PWMFREQUENCY * abs(right)) / 100;

                    apply_motor_pwm((left >= 0) ? left_pwm : -left_pwm,
                                    (right >= 0) ? right_pwm : -right_pwm);
                }
                break;

            case STATE_EMERGENCY:
                stop_moving();
                if (counter1ms % 500 == 0) {
                    LIGHTLEFT = !LIGHTLEFT;
                    LIGHTRIGHT = !LIGHTRIGHT;
                }
                if (distance >= 20) emergencyCounter++;
                else emergencyCounter = 0;

                if (emergencyCounter >= 2500) {
                    state = STATE_WAIT;
                    enqueue_buffer(&cb, "$MEMRG,0*");
                    LIGHTLEFT = 0;
                    LIGHTRIGHT = 0;
                    printingFlag = true;
                    IFS0bits.U1TXIF = 1;
                }
                break;
        }

        counter1ms++;
        tmr_wait_period(TIMER2);
    }
    return 0;
}
