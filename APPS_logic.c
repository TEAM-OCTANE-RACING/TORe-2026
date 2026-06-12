
//APPS CHecklist
// APPS signals are SCS

// sensor 1 ADC channel 1

// S2 ADC CH 2

// for S1 if ADC_val = 0 voltage = 0
// ADC_val = 4095 voltage = 5 (3.3)

// for S2 if ADC_val = 4095 voltage = 0
// ADC_val = 0 voltage = 5 (3.3)

// continuously check if both are running
// "Any 2 must be running"

// if 10% deviation occurs betn both values
// i.e sum of voltages must be 5
// if it differs by 0.5 => 10%. i.e comes 4.5

// implausibility occurs if persists more than 100 ms

// trigger safe state
// i.e motor torque commands are '0'
//Run gcc -o apps_test ADS_1115_APPS_cl.c APPS_1.c -lm


#include<stdio.h>
#include<stdint.h>
#include<stdbool.h>
#include<math.h>
#include<time.h>

//config based on checklist
#define ADC_MAX_VAL 4095.0f
#define SYS_VOLTAGE 3.3f
#define MAX_DEV 0.33f
#define IMPLAUSIBILITY_TIME_MS 100

static bool fault_active =false;
static long implaus_start_time =0;

long get_time_ms(void){
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)(ts.tv_sec * 1000L + ts.tv_nsec / 1000000L);

}

float APPS_pedal_press(uint16_t adc1,uint16_t adc2,long current_time){
    if(fault_active)return 0.0f;
    float S1_V =(adc1/ADC_MAX_VAL)*SYS_VOLTAGE;
    float S2_V =(1.0f-adc2/ADC_MAX_VAL)*SYS_VOLTAGE;
    bool implausible=(fabsf(S1_V+S2_V-SYS_VOLTAGE)>MAX_DEV);
        if(implausible){
            if(implaus_start_time == 0){
                implaus_start_time=current_time;
            }
            else if((current_time-implaus_start_time)>=IMPLAUSIBILITY_TIME_MS){
            fault_active=true;
            implaus_start_time=0;
            return 0.0f;
        }
    }
         else{
            implaus_start_time=0;
        }

    
           //safety pedal press is betn 0-1
        float pedal_press=S1_V/SYS_VOLTAGE;
        if(pedal_press>1.0f)pedal_press=1.0f;
        if(pedal_press<0.0f)pedal_press=0.0f;

        return pedal_press;
    }

      
void APPS_reset(void){
    fault_active=false;
    implaus_start_time=0;
}
//return the fault status to R2D
bool r2d_apps_fault(void) { return fault_active; }


 

    


