#include "other_gpioCM4.h"

other_gpio::other_gpio() {
    pinMode(GPIO_INFRARE_OUT, INPUT);
    pinMode(GPIO_INFRARE_IN, OUTPUT);

    // shoot
    pinMode(GPIO_CHARGE, OUTPUT);
    pinMode(PWM0_SHOOT, PWM_OUTPUT);
    pwmSetClock(320);
}

uint8_t other_gpio::shoot_gpio(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    if (mcu_type == 1)
    {
        /* code */
    }else {
        pwmWrite(PWM0_SHOOT, 0);
    
        if(test_charge_count++ > 1000) {
            digitalWrite(GPIO_CHARGE, HIGH);    // start charge
            std::cout << "Robot_Is_Boot_charged" << std::endl;
            test_charge_count = 0;
            Robot_Is_Boot_charged = 1;          // enable shoot
        }else if(test_charge_count == 20) {
            digitalWrite(GPIO_CHARGE, LOW);    // stop charge
            pwmWrite(PWM0_SHOOT, 0);
        }
        
        if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
            digitalWrite(GPIO_CHARGE, LOW);    // stop charge
            pwmWrite(PWM0_SHOOT, Robot_Boot_Power*3);
            Robot_Is_Boot_charged = 0;
            test_charge_count = 0;
            std::cout << "shoot" << std::endl;
        }
    }
    return Robot_Is_Boot_charged;
}