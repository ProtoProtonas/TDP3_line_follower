/* possibly useful stuff

https://os.mbed.com/users/aberk/code/PID/docs/tip/classPID.html#a9f407b18ba6235cb96fa95611c1ea3a4
https://os.mbed.com/media/uploads/rikabel/kl25z-pinout-revised.jpg
https://dewesoft.pro/online/course/pid-control/page/pid-tuning-methods  - conversion between Kc, Ti, Td and Kp, Ki and Kd
https://os.mbed.com/handbook/SPI

*/

/* 
##########  parameters to adjust for line following ##########
full_speed - duty cycle of pwm period for motor drive
rate - PID controller refresh rate
period - while loop refresh time (motor drive changes at this rate
threshold - threshold at which nothing is done to adjust the course of the robot
*/


#include "mbed.h"
#include "PID.h"

#define full_speed_straight 0.55f
#define full_speed_turn 0.60f
#define zero 0.0f
#define braking_time 0.2f
#define rate 0.05f    // PID controller rate in seconds
#define period 5000 // main loop timing in microseconds
#define pwm_period 1000 // in microseconds for motor drive

//PID stuff -  Kc, Ti, Td, interval
//PID controller(15000.0, 10000.0, 0.0, rate); // 3, 0, 0 was good
float Kc = 17.0;  // P
float Ki = 60.0;  // I
float Kd = 10.0;  // D
PID controller(Kc, Kc/Ki, Kd/Kc, rate);

// variables for driving motors
PwmOut right_pwm(PTD0); //for right side motors
DigitalOut right_direction(PTD5); // 0 - forward, 1 - backward
PwmOut left_pwm(PTC9); // for left side motors
DigitalOut left_direction(PTC8); // 0 - forward, 1 - backward
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

int s1, s2, s3, s4, s5, s6, s7, s8;


// for timing
Timer always_on;
Ticker to;
int started, next_time, now;

// inputs from colour sensors
AnalogIn red_sensor(PTB0);
AnalogIn blue_sensor(PTB1);

// solenoid control output
DigitalOut solenoid(PTC11);


//flags for ISR
int disc_colour = 1; // 0 - red, right, 1 - blue, left
int intersection_bar = 1; //0 - robot is at intersection; 1 - robot is not at intersection
int intersection_bar_flag = 1;
int dropping_point = 0; // tells whether the robot is in the dropping point after the first and before second intersection

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
    right_pwm.write(0.5);
    left_pwm.write(0.5);
    wait(0.05);

    right_direction = 0;
    left_direction = 0;
    right_pwm.write(0);
    left_pwm.write(0);
}

void intersection() {    
    intersection_bar_flag = 1;
    green_led = 1;
    if (s6 + s7 + s8 < 3) {
        if (s1 + s2 + s3 < 3) {                
            intersection_bar_flag = 0;
            green_led = 0;
        }
    }
    
    if((s6 + s7 + s8 == 0) || (s1 + s2 + s3 == 0)){
        intersection_bar_flag = 0;
        green_led = 0;
    }
    
    
    if(intersection_bar_flag == 0) {
        intersection_bar = 0;
        green_led = 0;
    } else {
        intersection_bar = 1;
        green_led = 1;
    }
        
}



int main() {

    // attach ISR functions
    to.attach(&intersection, 0.01);    
    
    blue_led = 1;
    red_led = 1;
    green_led = 1;
    
    enable1 = 1;
    enable2 = 1;
    
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
    int started, next_time, now;
    float pid_input, co;

    while (1) {
        
        started = always_on.read_us();
        next_time = started + period;

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
        
        pid_input = 0.5*s1 + 0.25*s2 + 0.125*s3+ 0.0625*s4 - 0.0625*s5 - 0.125*s6 - 0.25*s7 - 0.5*s8;
       
        controller.setProcessValue(pid_input);
        //disect the output of the pid controller and turn it into outputs for the two sets of wheels
        co = controller.compute(); // range [-full_speed; full_speed]
        
        if (intersection_bar == 1) {
            if (co > 0) {
                turn_left(1 - abs(co));
            }
            else if (co < 0) {
                turn_right(1 - abs(co));
            }
            else {
                forward();
            }
        }
        else {
            if (disc_colour == 1) {
                intersection_bar = 1;
                stop();
                wait(1);
                turn_left(0.5);
                wait(0.5);
                intersection_bar = 1;
                //intersection_time = always_on.read_us();
                //drop_time = intersection_time + wait_for_drop;
                                
            } else {
                
                intersection_bar = 1;
                stop();
                wait(1);
                turn_right(0.5);
                wait(0.5);
                //intersection_time = always_on.read_us();
                
            }
        }       
                
        //to control the rate of update
        now = always_on.read_us();
        while (now < next_time) { now = always_on.read_us(); }
        
    }
}

