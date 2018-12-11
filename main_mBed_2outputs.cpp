#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define gas_max 0.79f   //4.04V --> must be less than 4.2V
#define gas_min 0.25f   //.83V --> must be greater than 0.6V
#define steering_max 0.95f  //4.7V --> must be less than 4.8V
#define steering_min 0.04f  //0.227V --> must be greater than 0.1V
#define increment 0.00001f

ros::NodeHandle nh;

float steering_val = 0.505f;        //make sure each driver starts at 2.5 V
float gas_val = 0.545f; //2.48V with micro USB input
float steering_twist_val, gas_twist_val;

AnalogOut  aout(p18);
PwmOut pout(p21);
PwmOut myled1(LED1);
PwmOut myled2(LED2);

void messageCb(const geometry_msgs::Twist& twist){
   
    steering_twist_val = (float)((twist.linear.x));       //twist.values will come in between -2 to +2
                    
    gas_twist_val = (float)((twist.linear.y));       //output vals are not manipulated (but could be later) 
    
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", &messageCb);

int main() {
    myled1.period_ms(10);      //100 Hz
    myled1.write(steering_val);
    myled2.period_ms(10);
    myled2.write(gas_val);   
    
    pout.period_us(15);     //66.6 kHz period; 1 MHz too fast, 100 kHz to 60 kHz responds better w/o stepping
    pout.write(gas_val); 
    aout.write(steering_val);   //center gas and steering  
         
    nh.initNode();             //initiate node
    nh.subscribe(sub);         //subscribe to turtle
    
    while (1) 
   {
        nh.spinOnce();
        if(steering_twist_val <= 0.1f && steering_twist_val >= -0.1f){}//DEADZONE
        else if(steering_twist_val > 0.1f && steering_twist_val <= 1.0f && steering_val < steering_max){steering_val += increment;}     //turn wheel slowly CW
        else if(steering_twist_val > 1.0f && steering_twist_val <= 1.8f && steering_val < steering_max){steering_val += 5 * increment;}     //turn wheel CW
        else if(steering_twist_val > 1.8f && steering_twist_val <= 2.0f && steering_val < steering_max){steering_val += 20 * increment;}     //turn wheel fast CW
        else if(steering_twist_val < -0.1f && steering_twist_val >= -1.0f && steering_val > steering_min){steering_val -= increment;}     //turn wheel slowly CCW
        else if(steering_twist_val < -1.0f && steering_twist_val >= -1.8f && steering_val > steering_min){steering_val -= 5 * increment;}     //turn wheel CCW
        else if(steering_twist_val < -1.8f && steering_twist_val >= -2.0f && steering_val > steering_min){steering_val -= 20 * increment;}     //turn wheel fast CCW
        
        if(gas_twist_val <= 0.1f && gas_twist_val >= -0.1f){}//DEADZONE
        else if(gas_twist_val > 0.1f && gas_twist_val <= 1.0f && gas_val < gas_max){gas_val += increment;}     //gas
        else if(gas_twist_val > 1.0f && gas_twist_val <= 1.8f && gas_val < gas_max){gas_val += 5 * increment;}     //more gas
        else if(gas_twist_val > 1.8f && gas_twist_val <= 2.0f && gas_val < gas_max){gas_val += 20 * increment;}     //most gas
        else if(gas_twist_val < -0.1f && gas_twist_val >= -1.0f && gas_val > gas_min){gas_val -= increment;}     //brake
        else if(gas_twist_val < -1.0f && gas_twist_val >= -1.8f && gas_val > gas_min){gas_val -= 5 * increment;}     //more brake
        else if(gas_twist_val < -1.8f && gas_twist_val >= -2.0f && gas_val > gas_min){gas_val -= 20 * increment;}     //most brake
        
        aout.write(steering_val);
        pout.write(gas_val);
        myled1.write(steering_val);
        myled2.write(gas_val);
        
        wait_ms(1);       //SLOW IT DOWN
    }
}