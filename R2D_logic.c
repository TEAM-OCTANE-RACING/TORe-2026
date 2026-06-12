#include<stdint.h>
#include<stdbool.h>


 
//R2D basic checklist
//R2D sound min 1 and max 3 seconds
// car is r2D as soon as Motors respond to APPS
// for R2D SDC msut be clossed i.e TSMS LVMS and all interlocks
//-ve AIR charges first then precharge to 95% DC link and then the +ve AIR
//We press the brake pedal to 30% and then the starter button For R2D while all other conditions are true

#define ADC_MAX 4095
#define BPPS_R2D_THRESHOLD 1025 // APPROX 25% PEDAL
#define APPS_Deadband 205
#define MAX_TORQUE_NM 29
#define R2D_SOUND_time 1500

static bool vehicle_is_r2d =false;
static bool sound_active =false;
static bool starter_buton_pressed =false;
static long sound_start_time =0;
//adc[0] BPPS
//adc[1] APPS S1
//adc[2] APPS S2
//all passed in R2D update as uint16_t adc[3]
bool r2d_is_active(void)     { return vehicle_is_r2d; }
bool r2d_sound_active(void)  { return sound_active;   }


// calling APPS_pedal_press function
extern float APPS_pedal_press(uint16_t adc1,uint16_t adc2,long current_time);
extern bool r2d_apps_fault(void);
extern void APPS_reset(void);


float R2D_update(uint16_t adc[3],bool SDC_closed,bool starter_btn_released,long now_time){
   //shutdown circuit condition check
    if(!SDC_closed){
        vehicle_is_r2d=false;
        sound_active=false;
        APPS_reset();
        return 0;

    }
    //Apps fault check
    float pedal = APPS_pedal_press(adc[1],adc[2],now_time);
    if(r2d_apps_fault()){
        vehicle_is_r2d=false;
        return 0;
    }
    //starter button check is it released
    bool falling_edge=(starter_buton_pressed && !starter_btn_released);
    //update
    starter_buton_pressed=starter_btn_released;
    //is release of startter button occours during brake pedal is pressed
    if(falling_edge && adc[0]>BPPS_R2D_THRESHOLD && !vehicle_is_r2d){
        vehicle_is_r2d=true;
        sound_active=true;
        sound_start_time=now_time;
    }
    //sound for 1.5sec
    if(sound_active && now_time-sound_start_time>R2D_SOUND_time){
        sound_active=false;
    }
    if(!vehicle_is_r2d){
        return 0.0f;
    }
    if(adc[1]<APPS_Deadband){
        return 0.0f;
    }
    float torque= pedal*(float)MAX_TORQUE_NM;
    return 0;
}

//for reset R2D
void r2d_reset(void){
    vehicle_is_r2d=false;
    sound_active=false;
    sound_start_time=0;
    starter_buton_pressed=false;
    APPS_reset();
}

