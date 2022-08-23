#include "controlal.h"


using namespace std;

int main(){
    
    controlal test;
    test.control_init();
    test.x1 = 8000;
	test.v0_x = 3000;
	test.v1_x = 0;
    test.y1 = 3000; 
	test.v0_y = 500;
	test.v1_y = 0;
    pause();
	return 0;

}