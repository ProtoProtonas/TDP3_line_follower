/* possibly useful stuff

https://os.mbed.com/users/aberk/code/PID/docs/tip/classPID.html#a9f407b18ba6235cb96fa95611c1ea3a4
https://os.mbed.com/media/uploads/rikabel/kl25z-pinout-revised.jpg

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

#define full_speed 0.55f
#define zero 0.0f
#define braking_time 0.2
#define rate 0.02f    // PID controller rate in seconds
#define period 20000 //0.02s; main loop timing in microseconds
#define threshold 0.0f
#define pwm_period 1000 // in microseconds for motor drive
#define blue_threshold 0.78f
#define red_threshold 0.95f

//PID stuff -  Kc, Ti, Td, interval
PID controller(3, 0.1, 0.0, rate); // 3, 0, 0 was good

// variables for driving motors
PwmOut right_pwm(PTD5); //for right side motors
DigitalOut right_direction(PTD0); // 0 - forward, 1 - backward
PwmOut left_pwm(PTD2); // for left side motors
DigitalOut left_direction(PTD3); // 0 - forward, 1 - backward
DigitalOut status_led(PTB19);
DigitalOut blue_led (PTD1);



//inputs from line sensors
DigitalIn line_sensor_1(PTC12);
DigitalIn line_sensor_2(PTA13);
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

// inputs from colour sensors
AnalogIn red_sensor(PTB0);
AnalogIn blue_sensor(PTB1);

// solenoid control output
DigitalOut solenoid(PTC11);


//flags for ISR
int disc_taken = 0; // 0 - no disc on robot, 1 - disc is in the robot
int disc_colour = 0; // 0 - red, right, 1 - blue, left
int intersection_bar = 1; //0 - robot is at intersection; 1 - robot is not at intersection



// motor drive functions (called from main loop)
void forward() {
    right_direction = 0;
    left_direction = 0;
    right_pwm.write(full_speed);
    left_pwm.write(full_speed);
}

void turn_right(float variable_speed) {
    right_direction = 1;
    left_direction = 0;
    right_pwm.write(variable_speed);
    left_pwm.write(full_speed);
}

void turn_left(float variable_speed) {
    right_direction = 0;
    left_direction = 1;
    right_pwm.write(full_speed);
    left_pwm.write(variable_speed);
}

void stop() {
    right_direction = 0;
    left_direction = 0;
    right_pwm.write(0);
    left_pwm.write(0);
}



void detect_colour() {
    /*if ((red_sensor.read() == 1) && (blue_sensor.read() == 0)) {
        disc_colour = 0;
        disc_taken = 1; // raises flag
        solenoid = 1; // turns on solenoid
    }
    else if ((red_sensor.read() == 0) && (blue_sensor.read() == 1)) {
        disc_colour = 1;
        disc_taken = 1; // raises flag
        solenoid = 1; // turns on the solenoid
    }*/
    
    // blue threshold 0.778787
    // red threshold ~0.95
    if ((red_sensor < red_threshold) && (blue_sensor < blue_threshold)) {
        //red detected
        disc_colour = 0;
        disc_taken = 1; // raises flag
        solenoid = 1; // turns on solenoid
    } else if ((red_sensor > red_threshold) && (blue_sensor > blue_threshold)) {
        //red detected
        disc_colour = 0;
        disc_taken = 1; // raises flag
        solenoid = 1; // turns on solenoid
    }
}



// ISR funtion to detect intersection
void intersection() {    
    if ((line_sensor_6.read() == 0) || (line_sensor_7.read() == 0) || (line_sensor_8.read() == 0)) {
        if ((line_sensor_3.read() == 0) || (line_sensor_2.read() == 0) || (line_sensor_1.read() == 0)) {
            if ((line_sensor_4.read() + line_sensor_5.read()) == 2) {
                intersection_bar = 0;
            } else { intersection_bar = 1; }
        } else { intersection_bar = 1; }
    } else { intersection_bar = 1; }
}



int main() {

    // attach ISR functions
    to.attach(&intersection, 0.05);
    
    
    
    status_led = 1;
    // PID controler setup
    controller.setInputLimits(-1, 1);
    controller.setOutputLimits(-full_speed, full_speed);
    //If there's a bias.
    controller.setBias(0);
    controller.setMode(1); // 0 for manual, 1 for auto
    //desired input value
    controller.setSetPoint(0.0);

    // PWM setup for motor drive
    //float pwm_period = 0.02;
    right_pwm.period_us(pwm_period);
    left_pwm.period_us(pwm_period);

    //timer and interrupt setup
    always_on.start();
    int started, next_time, now;
    float pid_input, co;

    status_led = 0;
    while (1) {
        blue_led = 1;
        started = always_on.read_us();
        next_time = started + period;

        // setup input for the pid controller
        
        //  driving direction  ↑↑↑↑↑↑
        //           line →    ||||||
        //    sensors  1  2  3  4  5  6  7  8  
        
        /*pid_input = 0.5*line_sensor_1.read() + 0.25*line_sensor_2.read() + 0.125*line_sensor_3.read() + 0.0625*line_sensor_4.read() - 0.0625*line_sensor_5.read() - 0.125*line_sensor_6.read() - 0.25*line_sensor_7.read() - 0.5*line_sensor_8.read(); // negative - left sensor has lower value (is over the black line) and vice versa
        
        controller.setProcessValue(pid_input);
        //disect the output of the pid controller and turn it into outputs for the two sets of wheels
        co = controller.compute(); // range [-full_speed; full_speed]
        if (intersection_bar == 1) {
            if (co - threshold > 0) {
                turn_left(1 - abs(co));
            }
            else if (co + threshold < 0) {
                turn_right(1 - abs(co));
            }
            else {
                forward();
            }
        }
        else {
            if (disc_colour == 0) {
                turn_right(0.5);
                wait(0.3);
                intersection_bar = 1;
            } else {
                turn_left(0.5);
                wait(0.3);
                intersection_bar = 1;
            }
        }*/
        
        
        
        s1 = line_sensor_1.read();
        s2 = line_sensor_2.read();        
        s3 = line_sensor_3.read();        
        s4 = line_sensor_4.read();        
        s5 = line_sensor_5.read();        
        s6 = line_sensor_6.read();        
        s7 = line_sensor_7.read();        
        s8 = line_sensor_8.read();
        
        pid_input = (0.5*s1 + 0.25*s2 + 0.125*s3+ 0.0625*s4 - 0.0625*s5 - 0.125*s6 - 0.25*s7 - 0.5*s8) * intersection_bar + (-0.9375 + 0.5*s1 + 0.25*s2 + 0.125*s3+ 0.0625*s4) * disc_colour * (!intersection_bar) - (-0.9375 + 0.5*s5 + 0.25*s6 + 0.125*s7+ 0.0625*s8) * (!disc_colour) * (!intersection_bar); // negative - left sensor has lower value (is over the black line) and vice versa
        
        
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

        blue_led = 0;
        //to control the rate of update
        now = always_on.read_us();
        while (now < next_time) { now = always_on.read_us(); }
        
    }
}

