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
        sp[1] = 40;
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
        sp[0] = 30;
        sp[1] = 30;
        dir = 0;
    }
    else if (ch == 'q')
    {
        sp[0] = 70;
        sp[1] = 70;
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
    int cnt = 0, stop_flag = 0, save_cnt = 0;;
    //char buf[100];
    //ReadLeftUltrasonic_Filt();
    setBeepCycle(0);

    int rotate_standard = 10;

    while (1)
    {
        // 기존 방식
        distance = getTofDistance();
        Rdistance = ReadRearUltrasonic_Filt();
        ch = getBluetoothByte_nonBlocked();
        Ldistance = ReadLeftUltrasonic_Filt();

        // 키 입력 시, 속도 제어
        if (ch >= 'a' && ch <= 'z')
        {
            if (save != ch)
                flag = 0;
            save = ch;
        }
        change_mode(ch);

        // 자율주차 모드
        if(stop_flag == 0 && save == 'p'){
            sp[0] = sp[1] = 30;
            dir = 1;
            if(Ldistance > 40.0f) cnt++;
            else{
                // 주차 가능 여부 확인
                // 1. 가능
                if(cnt > rotate_standard){
                    bl_printf("find parking space \n");

                    // 주차 공간 중앙으로 이동하기 위한 변수값 저장
                    save_cnt = cnt;
                    stop_flag = 1;
                    stop_motor();
                    delay_ms(1000);

                    // 후진 진행 부스터
                    movChA_PWM(80, 0);
                    movChB_PWM(80, 0);
                    delay_ms(70);
                    sp[0] = sp[1] = 25;
                    dir = 0;
                }
                else cnt = 0;
            }
        }
        else if(stop_flag == 1 && save == 'p'){

            // 주차 공간 중앙으로 이동하기 위해 cnt값의 절반까지 이동
            if(Ldistance > 40.0f){
                sp[0] = sp[1] = 25;
                cnt--;
            }
            // cnt 값의 절반 까지 이동 완료
            if(cnt == save_cnt / 2 + 1){
                // 임시 정지
                stop_motor();
                delay_ms(2000);

                // 90도 회전
                bl_printf("starting rotate\n");
                movChA_PWM(80, 0);
                movChB_PWM(80, 1);
                delay_ms(320);
                stop_motor();
                delay_ms(1000);

                // 후진을 위한 부스터 진행
                bl_printf("starting parking\n");
                movChA_PWM(80, 0);
                movChB_PWM(80, 0);
                delay_ms(30);

                // 속도값
                dir = 0;
                sp[0] = sp[1] = 25;

                // 자율주차 모드 변수 초기화
                stop_flag = 0;
                cnt = 0;

                // 후진 충돌 방지 모드 실행
                save = 's';
            }
        }
        bl_printf("current cnt = %d, mode = %d\n", cnt, stop_flag);


        // 레이저 거리에 따른 속도 변환 - ACC 구현
        if (((save >= 'a' && save <= 'z') && save != 's') && (distance > 0 && distance <= 300))
        {
            // 거리에 따른 속도 변화 ( w - 주행 / q - 터보 주행 )
            if (save == 'w')
            {
                if (distance >= 120)
                {
                    int speed = ((distance - 120) * (40 - 15)) / (300 - 120) + 15;
                    if (sp[0] != 0)
                        sp[0] = speed;
                    if (sp[1] != 0)
                        sp[1] = speed;
                }
                else
                {
                    movChA_PWM(90, 0);
                    movChB_PWM(90, 0);
                    delay_ms(30);
                    save = ' '; // 최초 1회만 역방향 걸리도록 쓰레기값 넣음
                    stop_motor();
                }
            }
            else if (save == 'q')
            {
                if (distance >= 230)
                {
                    int speed = ((distance - 230) * (70 - 5)) / (300 - 230) + 5;
                    sp[0] = sp[1] = speed;
                }
                else
                {
                    movChA_PWM(100, 0);
                    movChB_PWM(100, 0);
                    delay_ms(60);
                    save = ' ';
                    stop_motor();
                }
            }
        }

        // 후진 경고음 및 감속
        if ((save == 's' || dir == 0) && Rdistance < 40.0f)
        {
            int dist = (int) Rdistance;
            if (Rdistance >= 5.0f)
            {
                int speed = ((dist - 5) * (30 - 25)) / (40 - 5) + 25;
                if (sp[0] != 0) sp[0] = speed;
                if (sp[1] != 0) sp[1] = speed;
            }
            else stop_motor();

            // 거리에 따른 부저 설정
            if (dist > 30) setBeepCycle(150);
            else if (dist > 20) setBeepCycle(70);
            else if (dist > 10) setBeepCycle(30);
            else setBeepCycle(1);
        }
        else setBeepCycle(0);

        // 키 입력 변화 시, 1회 모터 가속
        if (flag == 0 && (sp[0] > 0 || sp[1] > 0))
        {
            movChA_PWM(80, dir);
            movChB_PWM(80, dir);
            delay_ms(10);
            flag = 1;
        }

        // 완전 정지
        if (sp[0] == 0 && sp[1] == 0)
        {
            stop_motor();
        }
        else
        {
            movChA_PWM(sp[1], dir);
            movChB_PWM(sp[0], dir);
        }

        //bl_printf("distance = %d, save = %c B = %d, A = %d\n", distance, save, sp[0], sp[1]);
        //bl_printf("Rdistance = %f Ldistance = %f \n", Rdistance, Ldistance);
        delay_ms(20);

    }

    return 0;
}
