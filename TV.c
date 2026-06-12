#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#define MASS 270
#define MAX_TORQUE 29.1f
#define Lf 0.78132f  //1.532*0.51
#define Lr 0.75068  
#define WHEELBASE (Lf+Lr)
#define trackwidth 1.235
#define Cf 35238.459f
#define Cr 35033.582f
#define U 1.2f   //friction coeff
#define g 9.81
#define Radius 0.2286f
#define VEL_TAB_SIZE 7
#define gear_ratio 10.05

float Kp_table[7]={8028.7132, 10814.8386, 12115.0305, 12867.7731, 13358.6922, 13704.1539, 13960.4641};
float Ki_table[7]={75179.7600, 248127.9103, 334798.3685, 386709.9046, 421250.2931, 445880.6878, 464327.9132};
float Vx_table[7]={7.0f, 11.0f, 15.0f, 19.0f, 23.0f, 27.0f, 31.0f};



float Understeer_gradient_clac(void){
    return (((Lr*MASS)/(Cf*WHEELBASE))-((Lf*MASS)/(Cr*WHEELBASE)));
    

}

float Target_yaw_rate(float steer_angle, float Vx){
    float Ku=Understeer_gradient_clac();

    float yaw_target=(((steer_angle*Vx)/(WHEELBASE+(Ku*Vx*Vx))));
    if(yaw_target>((U*g)/Vx)){
        yaw_target=(U*g)/Vx;
    }
    if(yaw_target<(-(U*g)/Vx)){
        yaw_target=-(U*g)/Vx;
    }
    return yaw_target;


}
int binary_search(float arr[],int size,float x ){
    int low =0;
    int high=size-1;

    while((high-low)>1){

        int mid=(low+high)/2;
       
        if(x>=arr[mid]){
            low=mid;
        }
        else {
            high=mid;
        }
    }
    return low;
}


float get_kp(float Vx){
    float x1;
    float x2;
    float y1;
    float y2;
     if(Vx< Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx>Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

 int i=binary_search(Vx_table, VEL_TAB_SIZE,Vx);
 x1=Vx_table[i];
 x2=Vx_table[i+1];
 y1=Kp_table[i];
 y2=Kp_table[i+1];

 float kp=y1+((Vx-x1)*(y2-y1))/(x2-x1);
 return kp;
}

float get_ki(float Vx){
    float x1;
    float x2;
    float y1;
    float y2;
     if(Vx< Vx_table[0]){
        Vx=Vx_table[0];
    }
    if(Vx>Vx_table[VEL_TAB_SIZE-1]){
        Vx=Vx_table[VEL_TAB_SIZE-1];
    }

 int i=binary_search(Vx_table, VEL_TAB_SIZE,Vx);
 x1=Vx_table[i];
 x2=Vx_table[i+1];
 y1=Ki_table[i];
 y2=Ki_table[i+1];

 float ki=y1+((Vx-x1)*(y2-y1))/(x2-x1);
 return ki;
}

double pid_controller(double ref, double y, double Kp, double Ki)
{
    static double integral = 0.0;

    const double T = 0.001;

    double u_max = (MAX_TORQUE * 10.05 / 0.2286) * 1.235;
    double u_min = -u_max;

    double error = ref - y;

    double P = Kp * error;
    double I = Ki * integral;

    double u_unsat = P + I;
    double u;

    if (u_unsat > u_max)
    {
        u = u_max;
    }
    else if (u_unsat < u_min)
    {
        u = u_min;
    }
    else
    {
        u = u_unsat;

        /* Anti-windup */
        integral += 0.5 * T * error;
    }

    return u;
}

float torque_dist(float u){
    float delta_t=(u*Radius)/trackwidth;
    return delta_t;

}
float torque_outside(float u,float torque_current_left){
    float delta_t=torque_dist(u);
    return torque_current_left+(delta_t/10.05f);

}
float torque_inside(float u,float torque_current_right){
    float delta_t=torque_dist(u);
    return torque_current_right-(delta_t/10.05);

}







int main(){
    float velocity=10.0f;
    float steering_angle=0.185f;
    float current_torque_left=18.0f;
    float current_torque_right=18.0f;
    float current_yaw_rate= 1.0f;
    float under_steer_grad=Understeer_gradient_clac();
    float target_yaw=Target_yaw_rate(steering_angle,velocity);
    float ki=get_ki(velocity);
    float kp=get_kp(velocity);
    double u=pid_controller(target_yaw,current_yaw_rate,kp,ki);
    float tout=torque_outside(u,current_torque_left);
    float tin=torque_inside(u,current_torque_right);

    printf("under_steer_grad %f ",under_steer_grad);
    printf(" %f \n",target_yaw);
    printf(" %f \n",ki);
    printf(" %f \n",kp);
    printf(" %f \n",u);
    printf(" %f \n",tin);
    printf(" %f \n",tout);



}
