#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#define CAM_RATE 75
#define CONTROL_DEBUGGER 1

namespace config{

	constexpr double control_rate =500;
    constexpr double dt =1/control_rate; //s
	constexpr double a_max = 20000;
    constexpr double v_max = 3000;
    constexpr double d_max = 20000; //mm/s
}

#endif
