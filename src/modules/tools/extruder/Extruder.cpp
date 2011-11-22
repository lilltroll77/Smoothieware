#include "mbed.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/robot/Block.h"
#include "modules/tools/extruder/Extruder.h"



Extruder* extruder_for_irq; // We need this global because ISRs can't be attached to an object
extern "C" void TIMER1_IRQHandler (void){
    if((LPC_TIM1->IR >> 1) & 1){
        LPC_TIM1->IR |= 1 << 1;
        extruder_for_irq->step_pin = 0;
    }
    if((LPC_TIM1->IR >> 0) & 1){
        LPC_TIM1->IR |= 1 << 0;
        extruder_for_irq->stepping_tick();
    }
}

Extruder::Extruder(PinName stppin) : step_pin(stppin){
    this->absolute_mode = true;
}

void Extruder::on_module_loaded() {

    if( this->kernel->config->value( extruder_module_enable_checksum )->by_default(false)->as_bool() == false ){ return; } 

    extruder_for_irq = this;

    // Settings
    this->on_config_reload(this);

    // We work on the same Block as Stepper, so we need to know when it gets a new one and drops one
    this->register_for_event(ON_BLOCK_BEGIN);
    this->register_for_event(ON_BLOCK_END);
    this->register_for_event(ON_GCODE_EXECUTE);

    // Configuration
    this->acceleration_ticker.attach_us(this, &Extruder::acceleration_tick, 1000000/this->kernel->stepper->acceleration_ticks_per_second);

    this->start_position = 0;
    this->target_position = 0;
    this->current_position = 0;
    this->current_block = NULL;

    // Start Timer1
    LPC_TIM1->MR0 = 10000; 
    LPC_TIM1->MR1 = 500;
    LPC_TIM1->MCR = 11;            // For MR0 and MR1, with no reset at MR1
    NVIC_EnableIRQ(TIMER1_IRQn);
    LPC_TIM1->TCR = 1;  
}

// Get config
void Extruder::on_config_reload(void* argument){
    this->microseconds_per_step_pulse = 5; //this->kernel->config->value(microseconds_per_step_pulse_ckeckusm)->by_default(5)->as_number();
}

// Computer extrusion speed based on parameters and gcode distance of travel
void Extruder::on_gcode_execute(void* argument){
    Gcode* gcode = static_cast<Gcode*>(argument);
    
    // Absolute/relative mode
    if( gcode->has_letter('M')){
        int code = gcode->get_value('M');
        if( code == 82 ){ this->absolute_mode == true; }
        if( code == 83 ){ this->absolute_mode == false; }
    } 
   
    if( gcode->has_letter('G') ){

        if( gcode->get_value('G') == 92 ){

            if( gcode->has_letter('E') ){
                this->current_position = gcode->get_value('E') * 1000;
                this->target_position  = this->current_position;
                this->start_position   = this->current_position; 
            }            

        }else{

            // Extrusion length 
            if( gcode->has_letter('E' )){
                double extrusion_distance = gcode->get_value('E');
                //this->kernel->serial->printf("extrusion_distance: %f, millimeters_of_travel: %f\r\n", extrusion_distance, gcode->millimeters_of_travel);
                if( fabs(gcode->millimeters_of_travel) < 0.0001 ){
                    this->solo_mode = true;
                    this->travel_distance = extrusion_distance;
                    //this->kernel->serial->printf("solo mode distance: %f, mm: %f \r\n", this->travel_distance, gcode->millimeters_of_travel );
                }else{
                    this->solo_mode = false;
                    this->travel_ratio = extrusion_distance / gcode->millimeters_of_travel; 
                    //this->kernel->serial->printf("follow mode ratio: %f, mm: %f \r\n", this->travel_ratio, gcode->millimeters_of_travel );
                } 
            // Else do not extrude
            }else{
                this->travel_ratio = 0.0;
                this->travel_distance = 0.0;
            }

        }
 
    }    
    
}



void Extruder::on_block_begin(void* argument){
    Block* block = static_cast<Block*>(argument);

    if( fabs(this->travel_distance) > 0.001 ){

        block->take(); // In solo mode we take the block so we can move even if the stepper has nothing to do
        this->current_block = block; 
        this->start_position = this->current_position;
        this->target_position = ( !this->absolute_mode ? this->start_position : 0 ) + ( this->travel_distance * 1000 ); //TODO : Get from config ( extruder_steps_per_mm )
        this->acceleration_tick();
        //this->kernel->serial->printf("distance %f %f\r\n", this->travel_distance, this->travel_ratio);
    }   
    if( fabs(this->travel_ratio) > 0.001 ){
        
        // In non-solo mode, we just follow the stepper module
        this->current_block = block; 
        this->start_position = this->current_position;
        this->target_position =  ( !this->absolute_mode ? this->start_position : 0 ) + ( this->current_block->millimeters * this->travel_ratio * 1000 ); //TODO : Get from config ( extruder_steps_per_mm )
        this->acceleration_tick();
        //this->kernel->serial->printf("ratio %f %f %d\r\n", this->travel_distance, this->travel_ratio, block->times_taken );

    } 
    //this->kernel->serial->printf("%d %d %d %f %f\r\n", this->start_position, this->current_position, this->target_position, this->travel_distance, this->travel_ratio); 
}

void Extruder::on_block_end(void* argument){
    Block* block = static_cast<Block*>(argument);
    this->current_block = NULL; 
}       

void Extruder::acceleration_tick(){
    
    // If we are currently taking care of a block            
    if( this->current_block ){

        if( fabs(this->travel_ratio) > 0.001 ){
            double steps_by_acceleration_tick = this->kernel->stepper->trapezoid_adjusted_rate / 60 / this->kernel->stepper->acceleration_ticks_per_second;                
            // Get position along the block at next tick
            double current_position_ratio = double( (this->kernel->stepper->step_events_completed>>16) ) / double(this->kernel->stepper->current_block->steps_event_count);
            double next_position_ratio = ( (this->kernel->stepper->step_events_completed>>16) + steps_by_acceleration_tick ) / this->kernel->stepper->current_block->steps_event_count;
            // Get wanted next position
            double next_absolute_position = ( this->target_position - this->start_position ) * next_position_ratio;
            // Get desired  speed in steps per minute to get to the next position by the next acceleration tick
            double desired_speed = ( ( next_absolute_position + this->start_position ) - this->current_position ) * double(this->kernel->stepper->acceleration_ticks_per_second); //TODO : Replace with the actual current_position

            if( desired_speed <= 0 ){ return; }

            // Set timer
            LPC_TIM1->MR0 = ((SystemCoreClock/4))/int(floor(desired_speed)); 

            // In case we are trying to set the timer to a limit it has already past by
            if( LPC_TIM1->TC >= LPC_TIM1->MR0 ){
                LPC_TIM1->TCR = 3; 
                LPC_TIM1->TCR = 1; 
            }

            // Update Timer1    
            LPC_TIM1->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse;

        }

        if( fabs(this->travel_distance) > 0.001 ){

            // Set timer
            LPC_TIM1->MR0 = ((SystemCoreClock/4))/int(floor(300)); 

            // In case we are trying to set the timer to a limit it has already past by
            if( LPC_TIM1->TC >= LPC_TIM1->MR0 ){
                LPC_TIM1->TCR = 3; 
                LPC_TIM1->TCR = 1; 
            }

            // Update Timer1    
            LPC_TIM1->MR1 = (( SystemCoreClock/4 ) / 1000000 ) * this->microseconds_per_step_pulse;

        }

    }
}

inline void Extruder::stepping_tick(){
    if( this->current_position < this->target_position ){
        this->current_position++;
        this->step_pin = 1;
    }else{
        // Move finished
        if( fabs(this->travel_distance) > 0.001 && this->current_block != NULL ){
            this->current_block->release();        
        } 
    }
}



