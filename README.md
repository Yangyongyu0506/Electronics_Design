# Electronics_Design
This repository stores all the ESP32 projects used in Electronics Design curriculum.

HC-SR04:

    - TRIG->GPIO9
    - ECHO->GPIO10 (use a voltage divider or level shifter to 3.3V)

接线:左邊是外設，右邊是開發板

    TB6612:
    
        - STBY->3V3
        - PWMA->GPIO1
        - AIN2->GPIO2
        - AIN1->GPIO42
        - E1A->GPIO41
        - E1B->GPIO40
        - PWMB->GPIO4
        - BIN2->GPIO6
        - BIN1->GPIO5
        - E2A->GPIO8
        - E2B->GPIO3
        - ADC->GPIO0
        - PWMD->GPIO7
        - DIN2->GPIO15
        - DIN1->GPIO16
        - E4A->GPIO17
        - E4B->GPIO18

    LQ-R4CHVB循線傳感器:

        - CH1->GPIO11
        - CH2->GPIO12
        - CH3->GPIO13
        - CH4->GPIO14

    USB camera:

        - D+->GPIO20
        - D-->GPIO19

    小车右前方是A电机，左前方是D，后方是B。
    10顺，01逆。
    傳感器0為黑，1為白，從右至左為CH1到CH4。

    LQ_TFT18SPIV33彩屏：

        - D/C->GPIO39
        - SDI->GPIO47
        - SCK->GPIO21
        - CS->GND