#include "interface.h"

#define LOW     0
#define HIGH    1

#define GPIO_INFRARE_IN     4
#define GPIO_INFRARE_OUT    5
#define PWM0_SHOOT          26
#define GPIO_CHARGE         0

class other_gpio {
public:
    other_gpio();
    uint8_t shoot_gpio(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power);
    void infrare_gpio();

    void dribbler();

private:
    int mcu_type;
    int test_charge_count = 0;

    mraa::Pwm shoot;
    mraa::Gpio charge;
    mraa::Gpio infrarein;
    mraa::Gpio infrareout;
};