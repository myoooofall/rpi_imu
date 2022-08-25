#include "other_gpioROCKS.h"

other_gpio::other_gpio() : shoot(PWM0_SHOOT),charge(GPIO_CHARGE),infrarein(GPIO_INFRARE_IN),infrareout(GPIO_INFRARE_OUT) {
    mcu_type = 1;
    shoot.enable(true);
    charge.dir(mraa::DIR_OUT);
    infrarein.dir(mraa::DIR_OUT);
    infrareout.dir(mraa::DIR_IN);
}

uint8_t other_gpio::shoot_gpio(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    shoot.write(0);
    if(test_charge_count++ > 1000) {
        charge.write(HIGH);    // start charge
        std::cout << "Robot_Is_Boot_charged" << std::endl;
        test_charge_count = 0;
        Robot_Is_Boot_charged = 1;          // enable shoot
    }else if(test_charge_count == 20) {
        charge.write(LOW);    // stop charge
        shoot.write(0);
    }
    
    if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
        charge.write(LOW);    // stop charge
        shoot.write((Robot_Boot_Power<225)?(Robot_Boot_Power/300):0.75);  // duty ratio
        Robot_Is_Boot_charged = 0;
        test_charge_count = 0;
        std::cout << "shoot" << std::endl;
    }
    
    return Robot_Is_Boot_charged;
}