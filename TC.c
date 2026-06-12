#include<stdio.h>
#include<stdlib.h>
#include<math.h>

#define LAMBDA_DESIRED 0.08f
#define PI 3.1415926f
#define GEAR_RATIO 10.05f
#define RADIUS 0.2286f
#define VEL_TAB_SIZE 5
#define LAMBDA_TAB_SIZE 9


float velocity_table[5]={0.1f,2.5f,5.0f,7.5f,10.0f};

float lambda_table[9]={0.1f,0.13f,0.15f,0.2f,0.25f,0.4f,0.6f,0.8f,0.990f};


float Kp_table[5][9]={
    {0,0, 0, 0, 0, 0, 0, -27.349359727984901f, -16.0066270143361f},

    {-12.762924193853999f, -13.379408025363601f,-13.8128407190821f, -15.345193573942f,-20.357279305276101f, -28.198949446354199f, -32.052075584671499f, -34.566878493769799f, 0}
     ,
    {-22.9504856800482f, -24.700528926158601f, -26.420442641173398f, -32.7735727935966f, -37.1302685077357f, -47.340341183748301f, 0, 0, 0}
    ,
    {-28.7804511556023f, -29.824471630540199f, -32.138751342701099f, -37.855783576526598f, -42.107584251118702f, -55.059013875800197f, 0, 0, 0}
    ,
    {-39.576096959295903f, -49.687581205460198f, -60.003709338410196f, -77.405667412310194f, -87.476305087217199f, 0, 0, 0, 0}};


float Ki_table[5][9]={
{0, 0, 0, 0, 0, 0,0, -11670.790945277f, -11739.8305053856f}
, 
{-561.66596120840995f,-556.78905522505704f, -554.70793302180698f, -574.58960009903603f, -580.0f, -591.63143190700202f, -609.58566806772399f, -657.67272906434596f,0}
     
,{-457.10946338920201f,-437.465599128539f, -629.40613477025499f, -758.94918952921705f, -822.20916479232903f, -907.77754639936097f, 0, 0, 0}

,{-557.75879260435795f, -508.84249924102602f, -540.07337548326495f, -613.94832258016299f, -651.92334489266898f, -755.40834234298904f, 0, 0, 0}

,{-436.84711841410899f,-567.02082103832402f, -709.61936431326399f, -966.68353937852203f, -1165.13113408405f, 0, 0, 0, 0}
};

float lambda_cal(float omega, float velocity){
    if(fabs(omega)<0.00001f ){
        return 0.0f;
    }
    float lambda= ((omega*RADIUS)-velocity)/(omega*RADIUS);
    return lambda;

}

float omega_cal(int motor_rpm){
    float omega=(motor_rpm*2*PI)/(GEAR_RATIO*60);
    return omega;

}

float linear_interpoletion(float x0,float x1, float y0,float y1 , float x){
    return y0+((x-x0)*(y1-y0))/(x1-x0);

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




float get_Kp_val(float velocity,float lambda){
    if(velocity < velocity_table[0]){
        velocity=velocity_table[0];
    }
    if(velocity>velocity_table[VEL_TAB_SIZE-1]){
        velocity=velocity_table[VEL_TAB_SIZE-1];
    }
       if(lambda<lambda_table[0]){
        lambda=lambda_table[0];
    }
    if(lambda>lambda_table[LAMBDA_TAB_SIZE-1]){
        lambda=lambda_table[LAMBDA_TAB_SIZE-1];
    }
    int i=binary_search(velocity_table, VEL_TAB_SIZE,velocity);
    int j= binary_search(lambda_table,LAMBDA_TAB_SIZE,lambda);

//interpoletion table
    float Q11= Kp_table[i][j];
    float Q12= Kp_table[i+1][j];
    float Q21= Kp_table[i][j+1];
    float Q22= Kp_table[i+1][j+1];

    //along right side

    float R1= linear_interpoletion(velocity_table[i],velocity_table[i+1],Q11,Q21,velocity);

    //along left side 
    float R2= linear_interpoletion(velocity_table[i],velocity_table[i+1],Q12,Q22,velocity);

    //along lambda

    float P=linear_interpoletion(lambda_table[j],lambda_table[j+1],R1,R2,lambda);
    


return P;
 



}


float get_ki_val(float velocity,float lambda){
     if(velocity < velocity_table[0]){
        velocity=velocity_table[0];
    }
    if(velocity>velocity_table[VEL_TAB_SIZE-1]){
        velocity=velocity_table[VEL_TAB_SIZE-1];
    }
       if(lambda<lambda_table[0]){
        lambda=lambda_table[0];
    }
    if(lambda>lambda_table[LAMBDA_TAB_SIZE-1]){
        lambda=lambda_table[LAMBDA_TAB_SIZE-1];
    }
    int i=binary_search(velocity_table, VEL_TAB_SIZE,velocity);
    int j= binary_search(lambda_table,LAMBDA_TAB_SIZE,lambda);

//interpoletion table
     float Q11= Ki_table[i][j];
     float Q12= Ki_table[i+1][j];
     float Q21= Ki_table[i][j+1];
     float Q22= Ki_table[i+1][j+1];
    //along right side

    float R1= linear_interpoletion(velocity_table[i],velocity_table[i+1],Q11,Q21,velocity);

    //along left side 
    float R2= linear_interpoletion(velocity_table[i],velocity_table[i+1],Q12,Q22,velocity);

    //along lambda

    float I=linear_interpoletion(lambda_table[j],lambda_table[j+1],R1,R2,lambda);
    


return I;
 



}

float torque_command(float velocity,float lambda){
    float error_lambda=LAMBDA_DESIRED-lambda;
    float P=get_Kp_val(velocity,lambda);
    float I=get_ki_val(velocity,lambda);

    double torque_to_be_reduced=(error_lambda*P); //+(error_lambda*I); // integral

    return 29.1-torque_to_be_reduced;
    
}


int main(){
    float velocity=6.2f;
    int motor_rpm=3000;

    float omega=omega_cal(motor_rpm);
    printf("omega is %f \n",omega);
    float lambda=lambda_cal(omega,velocity);
    printf("lambda is %f \n",lambda);
    float P=get_Kp_val(velocity,lambda);
    printf("P is %f \n",P);
    float I=get_ki_val(velocity,lambda);
    printf("I is %f \n",I);
    float torque_comm=torque_command(velocity,lambda);
    printf("torque command given is %f \n",torque_comm);
}


