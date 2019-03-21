/* 
##########  parameters to adjust for line following ##########
full_speed - duty cycle of pwm period for motor drive
rate - PID controller refresh rate
period - while loop refresh time (motor drive changes at this rate
threshold - threshold at which nothing is done to adjust the course of the robot
*/


#include "mbed.h"
#include "PID.h"

#define full_speed_straight 0.45f
#define full_speed_turn 0.55f
#define rate 0.05f    // PID controller rate in seconds
#define period 5000 // main loop timing in microseconds
#define speaker_periods 40
#define min_periods 6
#define pwm_period 1000 // in microseconds for motor drive
#define blue_threshold 0.77f
#define red_threshold 0.96f

//PID stuff -  Kc, Ti, Td, interval
//PID controller(15000.0, 10000.0, 0.0, rate); // 3, 0, 0 was good
float Kc = 17.0;  // P
float Ki = 30.0;  // I
float Kd = 8.0;  // D
PID controller(Kc, Kc/Ki, Kd/Kc, rate);

// variables for driving motors
PwmOut right_pwm(PTD0); //for right side motors
DigitalOut right_direction(PTD5); // 0 - forward, 1 - backward
PwmOut left_pwm(PTC8); // for left side motors
DigitalOut left_direction(PTC9); // 0 - forward, 1 - backward
DigitalOut enable1(PTE21);
DigitalOut enable2(PTE22);

DigitalOut red_led(PTB18);
DigitalOut blue_led(PTD1);
DigitalOut green_led(PTB19);

//inputs from line sensors
DigitalIn line_sensor_1(PTC12);
DigitalIn line_sensor_2(PTC13);
DigitalIn line_sensor_3(PTC16);
DigitalIn line_sensor_4(PTC17);
DigitalIn line_sensor_5(PTA16);
DigitalIn line_sensor_6(PTA17);
DigitalIn line_sensor_7(PTD6);
DigitalIn line_sensor_8(PTD7);

// ISD1700 controls
DigitalOut play(PTB2);
DigitalOut skip(PTC2);
DigitalOut reset(PTB3);

int s1, s2, s3, s4, s5, s6, s7, s8;

// for timing
Timer always_on;
Ticker to, soundticker;
int started, next_time, now = 0;
int tpickup = 0, tdrop = 0, tfwd = 3500000, tmoving = 999999999;
int soundcounter = 5;
int loop_counter = 0; // needed to fake the ISR for the speaker

// inputs from colour sensors
AnalogIn red_sensor(PTB0);
AnalogIn blue_sensor(PTB1);

// solenoid control output
DigitalOut solenoid(PTC11);

//flags for ISR
int disc_colour = 0; // 0 - red, right, 1 - blue, left
int intersection_bar = 1; //0 - robot is at intersection; 1 - robot is not at intersection
int intersection_bar_flag = 1;
int colour = 0;
int colour_detection_periods = 0;
int after_intersection = 0;

// motor drive functions (called from main loop)
void forward() {
    right_direction = 0;
    left_direction = 0;
    right_pwm.write(full_speed_straight);
    left_pwm.write(full_speed_straight);
}

void turn_right(float variable_speed) {
    right_direction = 1;
    left_direction = 0;
    right_pwm.write(variable_speed);
    left_pwm.write(full_speed_turn);
}

void turn_left(float variable_speed) {
    right_direction = 0;
    left_direction = 1;
    right_pwm.write(full_speed_turn);
    left_pwm.write(variable_speed);
}

void stop() {
    right_direction = 1;
    left_direction = 1;
    right_pwm.write(0.7);
    left_pwm.write(0.7);
    wait(0.05);

    right_direction = 0;
    left_direction = 0;
    right_pwm.write(0);
    left_pwm.write(0);
}

// ISR function for colour detection
void detect_colour() {
    
    if((red_sensor > red_threshold) && (blue_sensor > blue_threshold)) {
        // detecting blue background
        blue_led = 0;
        red_led = 1;
        colour_detection_periods++;
        if(colour % 4 == 0) {
            if(colour_detection_periods >= min_periods) {
                colour++;
                disc_colour = 1;
            }
        } else if(colour % 4 == 2) {
            if(colour_detection_periods >= min_periods) {
                colour++;
            }
        }
        
    } else if((blue_sensor < blue_threshold) && (red_sensor < red_threshold)) {
        // detecting red background
        blue_led = 1;
        red_led = 0;
        colour_detection_periods++;
        
        if(colour % 4 == 0) {
            if(colour_detection_periods >= min_periods) {
                colour++;
                disc_colour = 0;
            }
        } else if(colour % 4 == 2) {
            if(colour_detection_periods >= min_periods) {
                if(after_intersection == 1) {
                    colour++;
                }
            }
        }
        
    } else {
        // white background
        blue_led = 1;
        red_led = 1;
        
        if(colour % 2 == 1) {
            colour++;
        }
        colour_detection_periods = 0;
    }
}

// function called from main while loop to detect intersection
int intersection(int se1, int se2, int se3, int se4, int se5, int se6, int se7, int se8) {    
    intersection_bar_flag = 1;
    green_led = 1;
    if (se6 + se7 + se8 < 3) {
        if (se1 + se2 + se3 < 3) {                
            intersection_bar_flag = 0;
        }
    }
    
    if((se6 + se7 + se8 == 0) || (se1 + se2 + se3 == 0)){
        intersection_bar_flag = 0;
    }
    
    int section_bar = 1;
    
    if(intersection_bar_flag == 0) {
        section_bar = 0;
        green_led = 0;
    } else {
        section_bar = 1;
        green_led = 1;
    }
    
    return section_bar;       
}

void reset_speaker_pins() {
        
    switch(soundcounter){
        case 0: 
            skip = 0;
            soundcounter++;
            break;
        case 1:
            skip = 1;
            soundcounter++;
            break;
        case 2:
            play = 0;
            soundcounter++;
            break;
        case 3:
            play = 1;
            soundcounter++;
            break;
        default:
            soundcounter++;
        }
}





int main() {

    // attach ISR function
    to.attach(&detect_colour, 0.07);
    
    blue_led = 1;
    red_led = 1;
    green_led = 1;
    
    enable1 = 1;
    enable2 = 1;
    
    play = 1;
    skip = 1;
    reset = 0;
    wait(0.2);
    reset = 1;
    play = 0;
    wait(0.2);
    play = 1;
    wait(3);
    
    // PID controler setup
    controller.setInputLimits(-1, 1);
    controller.setOutputLimits(-full_speed_turn, full_speed_turn);
    //If there's a bias.
    controller.setBias(0);
    controller.setMode(1); // 0 for manual, 1 for auto
    //desired input value
    controller.setSetPoint(0.0);

    // PWM setup for motor drive
    right_pwm.period_us(pwm_period);
    left_pwm.period_us(pwm_period);

    //timer and interrupt setup
    always_on.start();
    float pid_input, co;

    while (1) {
        
        started = always_on.read_us();
        next_time = started + period;
        
        // faking the ISR
        loop_counter++;
        if(loop_counter % speaker_periods == 0) {
            reset_speaker_pins();
        }
        
        // setup input for the pid controller
        
        //  driving direction  ↑↑↑↑↑↑
        //           line →    ||||||
        //    sensors  1  2  3  4  5  6  7  8  
        
        s1 = line_sensor_1.read();
        s2 = line_sensor_2.read();        
        s3 = line_sensor_3.read();        
        s4 = line_sensor_4.read();        
        s5 = line_sensor_5.read();        
        s6 = line_sensor_6.read();        
        s7 = line_sensor_7.read();        
        s8 = line_sensor_8.read();
        
        intersection_bar = intersection(s1, s2, s3, s4, s5, s6, s7, s8);
        
        if(intersection_bar == 1) { after_intersection = 1; }
        
        
        pid_input = (0.5*s1 + 0.25*s2 + 0.125*s3+ 0.0625*s4 - 0.0625*s5 - 0.125*s6 - 0.25*s7 - 0.5*s8) * intersection_bar + (0.5*s1 + 0.25*s2 + 0.125*s3+ 0.0625*s4) * (!disc_colour) * (!intersection_bar) - (0.5*s5 + 0.25*s6 + 0.125*s7+ 0.0625*s8) * disc_colour * (!intersection_bar); // negative - left sensor has lower value (is over the black line) and vice versa
        
        controller.setProcessValue(pid_input);
        //disect the output of the pid controller and turn it into outputs for the two sets of wheels
        co = controller.compute(); // range [-full_speed; full_speed]
        
        if (co > 0) {
            turn_left(1 - abs(co));
        }
        else if (co < 0) {
            turn_right(1 - abs(co));
        }
        else {
            forward();
        }
        
        
        if(colour % 4 == 1) {
            solenoid = 1;
            
            if(tpickup == 0) {
                tpickup = now;
                tmoving = tpickup + tfwd;
                soundcounter = 0;
            }
                            
        } else if(colour % 4 == 2) {
            tdrop = 0;
            
            if(now > tmoving) {
                soundcounter = 0;
                tmoving = 999999999;
            }
        
        } else if(colour % 4 == 3) {
            if(colour_detection_periods >= min_periods) {
                solenoid = 0;
                colour_detection_periods = 0;
                after_intersection = 0;
                
                if(tdrop == 0) {
                    tdrop = now;
                    tmoving = tdrop + tfwd;
                    soundcounter = 0;
                }
            }
        } else if(colour % 4 == 0) {
            tpickup = 0;
            
            if(now > tmoving) {
                soundcounter = 0;
                tmoving = 999999999;
            }
        
        }
                
        //to control the rate of update
        now = always_on.read_us();
        while (now < next_time) { now = always_on.read_us(); }
        
    }
}
