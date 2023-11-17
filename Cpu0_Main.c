#include "main.h"
#include "asclin.h"
#include "Drivers/asclin.h"
#include "gpt12.h"
#include "Motor.h"
#include "IO/ToF.h"
#include "Tof.h"
#include "GPIO.h"
#include "Ultrasonic.h"

#define Switch1   &MODULE_P02,0
#define LED_BLUE  &MODULE_P10,2
#define Switch2   &MODULE_P02,1
#define LED_RED   &MODULE_P10,1
IfxCpu_syncEvent g_cpuSyncEvent = 0;

int core0_main (void)
{
    IfxCpu_enableInterrupts();

    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /* Module Initialize */
    _init_uart3();
    _init_uart1();
    Init_Ultrasonics();
    Init_GPIO();
    init_gpt2();
    Init_DCMotors();
    Init_Buzzer();

    unsigned char ch, save= ' ';
    // 움직임 코드 movChA(duty, 1);
    // 정지코드 stopChA();
    int distance, res, dir = 1, flag = 0;;
    int sp[2] = {0, 0}; // B A
    float Rdistance, Ldistance;
    char buf[100];
    //ReadLeftUltrasonic_Filt();
    while(1){
        // 키 입력 res = _poll_uart3(&ch);
        // 거리 입력 getTofDistance();
        distance = getTofDistance();
        res = _poll_uart3(&ch);
        Rdistance = ReadRearUltrasonic_Filt();

        // 키 입력 받으면 왼쪽 오른쪽 속도 제어
        if(res > 0){
            if(save != ch) flag = 0;
            // 직진
            if(ch == 'w'){
                sp[0] = 33;
                sp[1] = 33;
                dir = 1;
            }
            // 좌회전
            else if(ch == 'a'){
                sp[0] = 0;
                sp[1] = 40;
            }
            // 우회전
            else if(ch == 'd'){
                sp[0] = 40;
                sp[1] = 0;
            }
            // 후진
            else if(ch == 's'){
                sp[0] = 33;
                sp[1] = 33;
                dir = 0;
            }
            else if(ch == 'q'){
                sp[0] = 50;
                sp[1] = 50;
                dir = 1;
            }
            else{
                sp[0] = 0;
                sp[1] = 0;
            }
            save = ch;
        }

        // 레이저 거리 이하 상황 - 긴급 정지
        if(ch != 's' && distance > 0 && distance <= 180){
            if(distance < 100 && (sp[0] > 0 || sp[1] > 0)){
                sp[0] = 0;
                sp[1] = 0;
            }
        }

        if(ch == 's' && Rdistance < 30.0f){
            if(Rdistance > 20.0f){
                setBeepCycle(150);
                sp[0] = 25;
                sp[1] = 25;
            }
            else if(Rdistance > 10.0f){
                setBeepCycle(70);
            }
            else{
                setBeepCycle(1);
                sp[0] = 0;
                sp[1] = 0;

            }
        }
        else{
            setBeepCycle(0);
        }

        if(flag == 0 && (sp[0] > 0 || sp[1] > 0)){
            movChA_PWM(80, dir);
            movChB_PWM(80, dir);
            delay_ms(10);
            flag = 1;
        }

        movChA_PWM(sp[1], dir);
        movChB_PWM(sp[0], dir);
        my_printf("distance = %d, ch = %c B = %d, A = %d\n", distance, ch, sp[0], sp[1]);
        my_printf("Udistance = %f Ldistance = %f \n", Rdistance, Ldistance);
        delay_ms(20);
    }

    return 0;
}

