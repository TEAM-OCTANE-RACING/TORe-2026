#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// extern r2d_is_active(void);
struct Control_Word
{
    unsigned int on : 1;
    unsigned int not_off2 : 1;
    unsigned int not_off3 : 1;
    unsigned int enable : 1;
    unsigned int not_hlg_reset : 1;
    unsigned int not_rfg_stop : 1;
    unsigned int setpoint_enable : 1;
    unsigned int reset : 1;
    unsigned int N : 1;
    unsigned int D : 1;
    unsigned int R : 1;
    unsigned int brake_SW : 1;
    unsigned int Hillhold : 1;
    unsigned int creep_disable : 1;
    unsigned int empty : 1;
    unsigned int toggle_bit : 1;
};

struct Status_Word
{
    unsigned int power_up_ready : 1;
    unsigned int ready : 1;
    unsigned int operation_enabled : 1;
    unsigned int fault : 1;
    unsigned int no_off2_SF : 1;
    unsigned int no_off3_SH : 1;
    unsigned int power_on_inhibit : 1;
    unsigned int warning : 1;
    unsigned int N_signal_applied : 1;
    unsigned int D_signal_applied : 1;
    unsigned int R_signal_applied : 1;
    unsigned int creep_active : 1;
    unsigned int Hillhold_active : 1;
    unsigned int current_limit_active : 1;
    unsigned int empty : 1;
    unsigned int toggle_bit : 1;
};

bool interlocking = false;
bool not_ready_state = false;
bool ready_to_switch_on = false;
bool ready_for_operation = false;
bool opeation = false;
int falut_detected = 0;
int r2d = 0;

void Status_parsing(struct Status_Word *status, uint8_t byte0, uint8_t byte1)
{
    status->power_up_ready = (byte0 >> 0) & 0x01;
    status->ready = (byte0 >> 1) & 0x01;
    status->operation_enabled = (byte0 >> 2) & 0x01;
    status->fault = (byte0 >> 3) & 0x01;
    status->no_off2_SF = (byte0 >> 4) & 0x01;
    status->no_off3_SH = (byte0 >> 5) & 0x01;
    status->power_on_inhibit = (byte0 >> 6) & 0x01;
    status->warning = (byte0 >> 7) & 0x01;
    status->N_signal_applied = (byte1 >> 0) & 0x01;
    status->D_signal_applied = (byte1 >> 1) & 0x01;
    status->R_signal_applied = (byte1 >> 2) & 0x01;
    status->creep_active = (byte1 >> 3) & 0x01;
    status->Hillhold_active = (byte1 >> 4) & 0x01;
    status->current_limit_active = (byte1 >> 5) & 0x01;
    status->empty = (byte1 >> 6) & 0x01;
    status->toggle_bit = (byte1 >> 7) & 0x01;
}

void Pack_Control_CAN (struct Control_Word * control,uint8_t *byte0,uint8_t *byte1){
     *byte0 = (control->on              << 0)
           | (control->not_off2        << 1)
           | (control->not_off3        << 2)
           | (control->enable          << 3)
           | (control->not_hlg_reset   << 4)
           | (control->not_rfg_stop    << 5)
           | (control->setpoint_enable << 6)
           | (control->reset           << 7);

    *byte1 = (control->N               << 0)
           | (control->D               << 1)  
           | (control->R               << 2)
           | (control->brake_SW        << 3)
           | (control->Hillhold        << 4)
           | (control->creep_disable   << 5)
           | (control->empty           << 6)
           | (control->toggle_bit      << 7);
}

void set_control_word(struct Control_Word * control,struct Status_Word * status){
    control->toggle_bit= !control->toggle_bit;
    
    if(falut_detected==1){
        control->reset=!control->reset;
        falut_detected=0;
        control->D=0;
        return;
    }
    if(status->fault==1){
        control->reset=!control->reset;
        falut_detected=1;
        control->on=0;
        control->not_off2=0;
control->not_off3=0;
control->enable=0;
        control->D=0;
        return;

    }

    if(status->power_on_inhibit==1
    && status->power_up_ready==0
&& status->ready==0
&& status->operation_enabled==0
&& status->fault==0){
    control->not_off2=1;
    control->not_off3=1;
    control->on=0;
    control->D=0;
    interlocking=true;
    return;
}

if(status->power_on_inhibit==0
&& status->power_up_ready==0
&& status->operation_enabled==0
&& status->ready ==0
&& status->fault ==0
&& status->no_off2_SF==1
&& status->no_off3_SH==1){
    interlocking=false;
    not_ready_state=true;
    control->not_off2=1;
    control->not_off3=1;
    control->on=0;
    control->D=0;
return;
}

if(status->power_on_inhibit==0
&& status->power_up_ready==1
&& status->ready==0
&& status->operation_enabled==0
&& status->fault==0
&& status->no_off2_SF==1
&& status->no_off3_SH==1){
    ready_to_switch_on=true;
    not_ready_state=false;
    control->not_off2=1;
    control->not_off3=1;

    if(r2d ==0){
        control->on=0;
        control->D=0;

    }
    else{
        control->on=1;
    }
    return;


}

if(status->power_on_inhibit==0
&& status->power_up_ready==1
&& status->ready==1
&& status->operation_enabled==0
&& status->fault==0
&& status->no_off2_SF==1
&& status->no_off3_SH==1){
    ready_to_switch_on=false;
    ready_for_operation=true;
    control->enable=1;
    control->on=1;
    control->not_off2=1;
    control->not_off3=1;
    control->D=1;
    return;
}

if(status->power_on_inhibit==0
&& status->power_up_ready==1
&& status->ready==1
&& status->operation_enabled==1
&& status->fault==0
&& status->no_off2_SF==1
&& status->no_off3_SH==1){
    opeation=true;
    ready_for_operation=false;
    control->enable=1;
    control->on=1;
    control->not_off2=1;
    control->not_off3=1;
    control->D=1;

}


}