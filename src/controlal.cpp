#include "controlal.h"

controlal::controlal() {
}


void controlal::control_init(){
    std::thread controlThread(&controlal::Localplanner, this);
    controlThread.detach();
}

void controlal::Localplanner(){ 

    while(1){

        compute_motion_2d(x1,v0_x,v1_x,y1,v0_y,v1_y,a_max,d_max,v_max,
        traj_time_x,traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,traj_time_y,traj_time_acc_y,
        traj_time_dec_y,traj_time_flat_y,RealTime,dt);
		
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  // TODO: modify delay

    }
}

//非零速到点,实时规划，只需要返回加速度a_max/-a_max/0/d_max/-d_max,速度通过其他函数积分
double controlal::compute_motion_1d(double x1, double v0, double v1,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
					   Mode mode,double dt,PlanType PT){
 
    if(mode == RealTime){            
		
		int dis_flag = 1;
		int vel_flag = 1;
		if(x1<0) dis_flag = -1;
		if(v1-v0<0) vel_flag = -1;
        //case1:已到点，迅速停下
        if(fabs(x1) <= 2*v_max*dt) {
			if(fabs(v0) <= a_max*dt) v0 = 0;
			return (v0 ==0 )?0:d_max;
			}
        //case2:速度与位移同向，讨论
        if(x1*v0 >= 0 ){


            double distance = fabs(x1);
            v0 = fabs((v0>v_max)?v_max:v0);
            double to_target_min_dis = fabs(((v1*v1)-(v0*v0))/(2*a_max));
            double min_acc_dis = fabs(((v_max*v_max)-(v0*v0))/(2*a_max));
            double min_dec_dis = fabs(((v_max*v_max)-(v1*v1))/(2*a_max));
			//距离够匀速
            if(min_acc_dis + min_dec_dis < distance ){
                if(v0 == v_max) return 0;
                else return a_max*dis_flag;
            } 
			
			//距离不够，直接根据末速度加减速
            if(to_target_min_dis >= distance){
				//std::cout<< PT <<endl;
				std::cout<<" to_target_min_dis: "<< to_target_min_dis<<std::endl;
				// std::cout<<" distance: "<< distance <<std::endl;
                // std::cout<<" can not achieve the position and velocity at the same time "<<std::endl;
                return a_max*vel_flag;
            }
			//距离够了,但不够匀速，根据位移加减速
            if(min_acc_dis + min_dec_dis >= distance && v0 != v_max){
                return a_max*dis_flag;
            }
            
           
            
        }
        //case3:速度与位移反向，先减速到0，然后进入同向讨论
        if((x1) * v0 < 0 ){
            int vel_flag = -1;
			if(fabs(v0) <= a_max*dt) {
				v0 = 0;
				return 0;
				}
            if(v0<0) vel_flag = 1;
            return d_max*vel_flag;
        }
    }

	else if (mode == Table)
	{

		double acc1_time_to_v1 = fabs(v1-v0)/a_max;
		double acc1_dist_to_v1 = fabs((v1+v0)/2.0) *acc1_time_to_v1;
		double dec1_time_to_v1 = fabs(v0-v1)/d_max;
		double dec1_dist_to_v1 = fabs((v1+v0)/2.0) *dec1_time_to_v1;
		double v_c = 0; //
		double t_max = 0; //v_max lasting time
		// (v_max+v0)(v_max-v0)/(2*a_max)+(v_max+v1)(v_max-v1)/(2*d_max)+v_max*t_max = x1
		double timecount = 0;
		if(v0*x1<0){
			double  t_d = v0/d_max;
			double  x_d = (v0*v0)/(2*d_max);
			v0 = fabs(v0);
			x1 = fabs(x1);
			while(v0 != 0){

			}
			compute_motion_1d((x1+x_d), v0, v1, a_max, d_max, v_max,
			traj_time, traj_time_acc,traj_time_dec,traj_time_flat,Table,dt,PT);
			
		}
		else{
			
			t_max = get_t_max(x1,v0,v1,a_max,d_max,v_max);
			if(t_max >= 0){
				
				traj_time_acc = (v_max-v0)/a_max;
				traj_time_dec = (v_max-v1)/d_max;
				traj_time_flat = t_max;
				traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
				
			}
			
			if(t_max < 0){
				
				if(fabs(v0)>fabs(v1) && dec1_dist_to_v1>fabs(x1)){
					
					traj_time_acc = 0;
					traj_time_dec = fabs(sqrt(v0*v0-2*x1*d_max) - v0)/d_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
				}
				else if(fabs(v0)<fabs(v1) && acc1_dist_to_v1>fabs(x1)){
					
					traj_time_dec = 0;
					traj_time_acc = fabs(sqrt(v0*v0+2*x1*a_max) - v0)/a_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
					
				}
				else{
					double v_top = 0;
					//(pow(v_top,2)-pow(v0,2))/(2*a_max)+(pow(v_top,2)-pow(v1,2))/(2*d_max) = x1;
					v_top = sqrt((2*a_max*d_max*x1+a_max*v1*v1+d_max*v0*v0)/(a_max+d_max));
					traj_time_dec = (v_top - v1)/d_max;
					traj_time_acc = (v_top - v0)/a_max;
					traj_time_flat =0;
					traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
					
				}
			}
		}
	}
    return 0;
}


double controlal::get_t_max(double x1, double v0, double v1, 
                 double a_max, double d_max, double v_max){
					 
	double x_max = x1-(pow(v_max,2)-pow(v0,2))/(2*a_max)-(pow(v_max,2)-pow(v0,2))/(2*d_max);
	double t = x_max/v_max;
	return t;
	
}


void controlal::compute_motion_2d(double x1, double v0_x, double v1_x, 
                       double y1, double v0_y, double v1_y,
                       const double a_max, const double d_max, const double v_max,
                       double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                       double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
					   Mode mode,double dt){
	
	if(mode == RealTime){

		a[0] = compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
				traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
				mode,0,X);
		
	    a[1] = compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
				traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
				mode,0,Y);
        //new velocity and send to function motionplanner and for next time compute_motion_2d
		v[0] = v0_x + a[0]*dt;
        v[1] = v0_y + a[1]*dt;
        v0_x = v[0];
        v0_y = v[1];
		//new position for next time compute_motion_2d
		pos[0] = pos[0] + (v[0]+v0_x)*dt/2;
		pos[1] = pos[1] + (v[1]+v0_y)*dt/2;
        x1 = x1 - pos[0];
        y1 = y1 - pos[1];

	}

	else if(mode == Table){

		compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
				traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
				mode,dt,X);
		
	    compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
				traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
				mode,dt,Y);

	}

	
}
