#include "controlal.h"
using namespace std;

int main(){
    
    controlal control(controlal::Mode::RealTime);
    rate Rate_command(CAM_RATE);
    while(1){
        if(control.control_done())control.sendCommand(3000,2000,0,0);
        Rate_command.sleep();
    }
    return 0 ; 
}
