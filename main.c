#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#define ZUMO "Zumo102"

//GENERAL USE FUNCTIONS

//tank turn robot
void spin(uint8 speed, uint32 delay, bool dir)
{
	//Left
	if(dir == 1){
		MotorDirLeft_Write(1);      // set LeftMotor backward mode
		MotorDirRight_Write(0);     // set RightMotor forward mode
		PWM_WriteCompare1(speed); 
		PWM_WriteCompare2(speed);
		
	}
	
	//Right
	else{
		MotorDirLeft_Write(0);      // set LeftMotor forward mode
		MotorDirRight_Write(1);     // set RightMotor backward mode
		PWM_WriteCompare1(speed); 
		PWM_WriteCompare2(speed);
		
	}
    vTaskDelay(delay);
}

//move forward with motor control
void motor_f(uint8 speed,uint32 delay)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed+15); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}

//move back and turn
void motor_backward_left(uint8 l_speed, uint8 r_speed, uint32 delay)
{
    MotorDirLeft_Write(1);      // set LeftMotor backward mode
    MotorDirRight_Write(1);     // set RightMotor backward mode
    PWM_WriteCompare1(l_speed); 
    PWM_WriteCompare2(r_speed); 
    vTaskDelay(delay);
}

//wait for button press
void button_press()
{
    while(SW1_Read() == 1)
    {
        vTaskDelay(100);
    }
    while(SW1_Read() == 0)
    {
        vTaskDelay(100);
    }
}

//follow to line start
void line_start(struct sensors_ dig)
{
    reflectance_digital(&dig);
    while(dig.r2 + dig.r1 + dig.l1 + dig.l2 != 4)     //while all the sensors are not reading black
    {
        reflectance_digital(&dig);
        if(dig.r1 == 1)
        {
           motor_f(150,0);
        }
        else
        {
           motor_turn(0,150,1);
        }
    }
    motor_f(0, 0);  
}

//sensor calibration
long sensorCalibration(long *array,int count)
{
    long average_array[6] ={0, 0, 0, 0, 0, 0};
    int i = 0, j = 0, counter = 0;
    struct sensors_ ref;
    reflectance_read(&ref);
    while(i < count)
    {
        while(j < 20) 
        {
            reflectance_read(&ref);
            average_array[0] += ref.l3;
            average_array[1] += ref.l2;
            average_array[2] += ref.l1;
            average_array[3] += ref.r1;
            average_array[4] += ref.r2;
            average_array[5] += ref.r3;
            vTaskDelay(50);
            j++;
            counter++;
        }
        Beep(500, 100);
        button_press();
        j = 0;
        i++;
    }

    for(int k=0; k < 6; k++)
    {
        array[k] = (average_array[k]/counter) + 3000;
    }
    return(*array);
}


//SUMO CHALLENGE FUNCTIONS

//calculates hit angle
void hit_angle(double x, double y)
{
    TickType_t hit;
    float hit_x = x/16000;
    float hit_y = y/16000;
    double angle;
    hit = xTaskGetTickCount();
    float value = hit_x/hit_y;
    if (x < 0)
        {
            angle =((atan(value))*180)/M_PI;
            angle = 270 - angle;
            print_mqtt(ZUMO, "/hit %d angle %.2f", hit, angle);
        }
    else
        {
            angle =((atan(value))*180)/M_PI;
            angle = 90 - angle;
            print_mqtt(ZUMO, "/hit %d angle %.2f", hit, angle);
        }
}

// sumo function
void sumo(struct sensors_ dig)
{
    int hit = 0;
    LSM303D_Start();
    struct accData_ data;
    LSM303D_Read_Acc(&data);
    reflectance_digital(&dig);
    while(dig.l1 == 0 && dig.l2 == 0 && dig.l3 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0)       //while all the reflective sensors read white. 
    {
        LSM303D_Read_Acc(&data);
        reflectance_digital(&dig);
        if((data.accX >= 13500 || data.accX <= -13500) && hit == 0)               //checking the accelerometer for any hits in the front or back of the robot and prints hit angle to mqtt.
        {
            hit = 1;
            LSM303D_Read_Acc(&data);
            hit_angle(data.accX, data.accY);                        
            motor_f(100, 10);
        }
        else if((data.accY >= 13500 || data.accY <= -13500) && hit == 0)          //checking the accelerometer for any hits in the sides of the robot and prints hit angle to mqtt.
        {
            hit = 1;
            LSM303D_Read_Acc(&data);
            hit_angle(data.accX, data.accY);                        
            motor_f(100, 10);
        }
        if((data.accY <= 13500 || data.accY >= -13500) && (data.accX <= 13500 || data.accX >= -13500))
        {
            hit = 0;
        }
        if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1)      //if all sensors read black, spin and continue to move forward.
        {
            motor_backward_left(100, 100, 200);
            spin(200, 500, 1);
            reflectance_digital(&dig);
        }

        else if(dig.r3 == 1 || dig.l3 == 1)     ////if middle 2 sensors read black, spin and continue to move forward.
        {
            motor_backward_left(150, 150, 500);
            spin(200, 400, 0);
            reflectance_digital(&dig);
        } 
        else            //if all sensors are reading white still continue to move forward
        {
            motor_f(150, 10);
        }
    }

    motor_f(0, 0);
}

 //LINE FOLLOWING FUNCTIONs

void line_follow(struct sensors_ dig)
{
    int counter = 0;
    int isblack = 0;
    int miss = 0;
    reflectance_digital(&dig);
    motor_f(150, 150);
    while(counter < 2){
        reflectance_digital(&dig);
        
        if(dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3 == 6 && isblack == 0){
            isblack = 1;
            counter++;
        }
        if(dig.l2 + dig.l3 >= 1 && dig.l1!= 1 && miss == 0)
        {
            print_mqtt(ZUMO, "/miss %d", xTaskGetTickCount());
            miss = 1;
        }
        if(dig.r2 + dig.r3 >= 1 && dig.r1!= 1 && miss == 0)
        {
            print_mqtt(ZUMO, "/miss %d", xTaskGetTickCount());
            miss = 1;
        }
        if(dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3 < 6){
            isblack = 0;
        }
        
        if( dig.l1 + dig.r1 >= 1 && miss == 1)
        {
            print_mqtt(ZUMO, "/line %d", xTaskGetTickCount());
            miss = 0;
        }
        
        if (dig.l1 == 1 && dig.r1 == 1 && dig.l3 + dig.l2 + dig.r3 + dig.r2 == 0)  //moving forward
        {
            motor_f(225, 0);
        }
        else if(dig.l1 + dig.l2 == 2 && dig.l3 + dig.r2 + dig.r3 == 0)      //slight turn right
        {
            motor_turn(0,225,0);
        }
        else if(dig.r1 + dig.r2 == 2 && dig.l3 + dig.l2 + dig.r3 == 0)      //slight turn left
        {
            motor_turn(225,0,0);
        }
        else if(dig.l2 + dig.l3 == 2 && dig.l1 + dig.r1 + dig.r2 + dig.r3 == 0)     //greater turn right
        {
            motor_turn(0, 225, 0);          
        }
        else if(dig.r2 + dig.r3 == 2 && dig.l1 + dig.r1 + dig.l2 + dig.l3 == 0)     //greater turn left
        {
            motor_turn(225, 0, 0); 
        }
    }

}

// MAZE FUNCTIONS


#if 0
    
// sumo challenge
    
void zmain(void)
{
    struct sensors_ dig;    
    long threshold_dig[6] = {0, 0, 0, 0, 0, 0}; 

    BatteryLed_Write(1);
    IR_Start();
    reflectance_start();
    motor_start();
    TickType_t start;
    TickType_t end;
    
    vTaskDelay(100);
    reflectance_digital(&dig);
    button_press();
    sensorCalibration(threshold_dig, 2);        //calibrates the program to differentiate between black and white
    reflectance_set_threshold(threshold_dig[0], threshold_dig[1], threshold_dig[2], threshold_dig[3], threshold_dig[4], threshold_dig[5]);      //sets the threshold for defining between black or white.
    line_start(dig);
    print_mqtt(ZUMO, "/ready zumo");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt(ZUMO, "/start %d", start);
    motor_f(100, 400);
    BatteryLed_Write(0);
    sumo(dig); 
    motor_stop();
    end = xTaskGetTickCount();
    print_mqtt(ZUMO, "/stop %d", end);
    print_mqtt(ZUMO, "time %d", end-start);

    while(SW1_Read() == 1)
    {
        BatteryLed_Write(1);
        vTaskDelay(200); // sleep (in an infinite loop)
        BatteryLed_Write(0);
        vTaskDelay(200);
    }
 }   
#endif


//Line following CHALLENGE
#if 1
void zmain(void)
{
    struct sensors_ dig;    
    long threshold_dig[6] = {0, 0, 0, 0, 0, 0};
    TickType_t start;
    TickType_t end;

    BatteryLed_Write(1);
    IR_Start();
    reflectance_start();
    motor_start();

    
    vTaskDelay(100);
    reflectance_digital(&dig);
    button_press();
    sensorCalibration(threshold_dig, 2);        //calibrates the program to differentiate between black and white
    reflectance_set_threshold(threshold_dig[0], threshold_dig[1], threshold_dig[2], threshold_dig[3], threshold_dig[4], threshold_dig[5]);      //sets the threshold for defining between black or white.
    line_start(dig);
    print_mqtt(ZUMO, "/ready line");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt(ZUMO, "/start %d", start);
    line_follow(dig);
    motor_stop();
    end = xTaskGetTickCount();
    print_mqtt(ZUMO, "/end %d", end);
    print_mqtt(ZUMO, "/time %d", end-start);
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
    
#endif

//MAZE CHALLENGE
#if 0
    
    
    
#endif
    

/* [] END OF FILE */
