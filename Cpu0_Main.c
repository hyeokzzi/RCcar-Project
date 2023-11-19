#include "main.h"
#include "asclin.h"
#include "gpt12.h"
#include "Motor.h"
#include "Tof.h"
#include "GPIO.h"
#include "Ultrasonic.h"
#include "Bluetooth.h"
#include "my_stdio.h"

#define Switch1   &MODULE_P02,0
#define LED_BLUE  &MODULE_P10,2
#define Switch2   &MODULE_P02,1
#define LED_RED   &MODULE_P10,1
IfxCpu_syncEvent g_cpuSyncEvent = 0;

int distance, res, dir = 1, flag = 0;
int sp[2] = {0, 0}; // B A

void stop_motor ()
{
    sp[0] = sp[1] = 0;
    stopChA();
    stopChB();
}
void change_mode (char ch)
{
    // 직진
    if (ch == 'w')
    {
        sp[0] = 40;
        sp[1] = 40                                                                   ;
        dir = 1;
    }
    // 좌회전
    else if (ch == 'a')
    {
        sp[0] = 0;
        sp[1] = 55;
    }
    // 우회전
    else if (ch == 'd')
    {
        sp[0] = 55;
        sp[1] = 0;
    }
    // 후진
    else if (ch == 's')
    {
        sp[0] = 35;
        sp[1] = 35;
        dir = 0;
    }
    else if (ch == 'q')
    {
        sp[0] = 50;
        sp[1] = 50;
        dir = 1;
    }
    else if (ch == 'f')
    {
        sp[0] = 0;
        sp[1] = 0;
        stopChA();
        stopChB();
    }
}
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
    Init_Mystdio();
    Init_Bluetooth();
    _init_uart3();
    _init_uart1();
    Init_Ultrasonics();
    Init_GPIO();
    init_gpt2();
    Init_DCMotors();
    Init_Buzzer();

    unsigned char ch, save = ' ';
    // 움직임 코드 movChA(duty, 1);
    // 정지코드 stopChA();
    float Rdistance, Ldistance;
    int cnt = 0;
    //char buf[100];
    //ReadLeftUltrasonic_Filt();
    setBeepCycle(0);
    while (1)
    {
        // 기존 방식
        //res = _poll_uart3(&ch);
        distance = getTofDistance();
        Rdistance = ReadRearUltrasonic_Filt();
        ch = getBluetoothByte_nonBlocked();

        // 키 입력 시, 속도 제어
        if (ch > 0)
        {
            if (ch >= 'a' && ch <= 'z')
            {
                if (save != ch)
                    flag = 0;
                save = ch;
            }
            change_mode(ch);
        }

        // 레이저 거리에 따른 속도 변환 - ACC 구현
        if (((save >= 'a' && save <= 'z') && save != 's') && (distance > 0 && distance <= 300))
        {
            // 70~220 => 20 ~ 40
            if ((save == 'w' || save == 'q') && distance >= 180)
            {
                int speed = ((distance - 180) * (40 - 15)) / (300 - 180) + 15;
                if (sp[0] != 0)
                    sp[0] = speed;
                if (sp[1] != 0)
                    sp[1] = speed;
            }
            else if((save == 'w' || save =='q') && distance < 150)
            {
                stop_motor();
            }
        }

        // 후진 경고음 및 감속
        if (save == 's' && Rdistance < 30.0f)
        {
            if (Rdistance >= 10.0f)
            {
                int speed = (((int) Rdistance - 10) * (30 - 10)) / (35 - 10) + 20;
                int sound = (((int) Rdistance - 10) * (150 - 10)) / (35 - 10) + 20;
                if (sp[0] != 0)
                    sp[0] = speed;
                if (sp[1] != 0)
                    sp[1] = speed;
                setBeepCycle(sound);
            }
            else
            {
                setBeepCycle(1);
                stop_motor();
            }
        }
        else
        {
            setBeepCycle(0);
        }

        if (flag == 0 && (sp[0] > 0 || sp[1] > 0))
        {
            movChA_PWM(80, dir);
            movChB_PWM(80, dir);
            delay_ms(10);
            flag = 1;
        }
        if (sp[0] == 0 && sp[1] == 0)
        {
            stopChA();
            stopChB();
        }
        else
        {
            movChA_PWM(sp[1], dir);
            movChB_PWM(sp[0], dir);
        }

        bl_printf("distance = %d, save = %c B = %d, A = %d\n", distance, save, sp[0], sp[1]);
        bl_printf("Rdistance = %f Ldistance = %f \n", Rdistance, Ldistance);
        delay_ms(20);

    }

    return 0;
}
