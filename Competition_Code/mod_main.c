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

#define ZUMO "Zumo102"

// GENERAL FUNCTIONS

void spin(uint8 speed, uint32 delay, bool dir){
	//LEFT
	if(dir == 1){
		MotorDirLeft_Write(1);                                                              // set LeftMotor backward mode
		MotorDirRight_Write(0);                                                             // set RightMotor forward mode
		PWM_WriteCompare1(speed); 
		PWM_WriteCompare2(speed);
		
	}
	
	//RIGHT
	else{
		MotorDirLeft_Write(0);                                                              // set LeftMotor forward mode
		MotorDirRight_Write(1);                                                             // set RightMotor backward mode
		PWM_WriteCompare1(speed+15); 
		PWM_WriteCompare2(speed);
		
	}
    vTaskDelay(delay);
}

//move forward with motor control
void motor_f(uint8 speed,uint32 delay){
    MotorDirLeft_Write(0);                                                                  // set LeftMotor forward mode
    MotorDirRight_Write(0);                                                                 // set RightMotor forward mode
    PWM_WriteCompare1(speed+15); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}

//move back and turn
void motor_backward_left(uint8 l_speed, uint8 r_speed, uint32 delay){
    MotorDirLeft_Write(1);                                                                  // set LeftMotor backward mode
    MotorDirRight_Write(1);                                                                 // set RightMotor backward mode
    PWM_WriteCompare1(l_speed); 
    PWM_WriteCompare2(r_speed); 
    vTaskDelay(delay);
}

//wait for button press
void button_press(){
    while(SW1_Read() == 1){
        vTaskDelay(100);
    }
    while(SW1_Read() == 0){
        vTaskDelay(100);
    }
}

//follow to line start
void line_start(struct sensors_ dig){
    reflectance_digital(&dig);
    while(dig.r2 + dig.r1 + dig.l1 + dig.l2 != 4){                                          //  while all the sensors are not reading black
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
long sensorCalibration(long *array,int count){                                          // This function takes an array as a pointer from the main function and will populate it with a weighted average of the refectance sensors
    long average_array[6] ={0, 0, 0, 0, 0, 0};                                          // based on a sample size of 20 x the number passed as argument. This will set to 2 counts, 1 black and 1 white. The pointed array is then returned
    int i = 0, j = 0, counter = 0;                                                      // to the main function where the stored values will be used as the thresholds between what is black and what is white.
    struct sensors_ ref;
    reflectance_read(&ref);
    while(i < count){
        while(j < 20){
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

    for(int k=0; k < 6; k++){
        array[k] = (average_array[k]/counter) + 3000;
    }
    return(*array);
}
//SUMO CHALLENGE FUNCTIONS

//calculates hit angle
void hit_angle(double x, double y){                                                        // This function will take the accelerometer readings for the x and y axis and then based on the signed 
    TickType_t hit;                                                                        // value of x will calculate the angle that the hit came from and print a message through mqtt.
    float hit_x = x/16000;
    float hit_y = y/16000;
    double angle;
    hit = xTaskGetTickCount();
    float value = hit_x/hit_y;
    if (x < 0){
        angle =((atan(value))*180)/M_PI;
        angle = 270 - angle;
        print_mqtt(ZUMO, "/hit %d angle %.2f", hit, angle);
    }
    else{
        angle =((atan(value))*180)/M_PI;
        angle = 90 - angle;
        print_mqtt(ZUMO, "/hit %d angle %.2f", hit, angle);
    }
}

// MAZE FUNCTIONS
typedef struct position{
        int x;
        int y;
        int dir;
}position;


void mazeReady(struct sensors_ *ref){                                                       //  MazeReady makes the robot wait for a button press and then drives to the line and sends the mqtt start msg.
    while(SW1_Read() == 1){
        vTaskDelay(100);
    }
  
    while(!(ref->l3 > 18000 && ref->r3 > 18000 && ref->l1 > 18000)){
        reflectance_read(ref);

        if(ref->r1 > 23000)
            motor_turn(200,0,1);
        
        else
            motor_turn(0,200,1);
        
    }
    print_mqtt(ZUMO "/ready","maze");
    motor_forward(0,0);
}

void mazeForward(void){                                                                     //  Quick function to move forward a bit when no obstacle is in front of the intersection.
    motor_forward(100,150);
    motor_forward(0,0);
}

void mazeLeft(struct sensors_ *ref, position *pos){                                         //  Turning left in the maze and updating the direction of the robot.
                                                                                            //  Turns with a small delay to move the sensor off the black line, then continues to turn until r1 sees the line.
    spin(100,500,1);                                                                        //  Uses the position structure pointer to modifiy the value in main instead of only modifying it in the function.
    
    reflectance_read(ref);
    while (ref->r1 < 16000){
        reflectance_read(ref);
        spin(100,1,1);
    }
    
    pos->dir--;
    
    motor_forward(0,0);
}

void mazeRight(struct sensors_ *ref, position *pos){                                        //  Same as mazeLeft but to the right direction, and checks with l1 not r1.
    
    spin(100,500,0);
    
    reflectance_read(ref);
    while (ref->l1 < 16000){
        reflectance_read(ref);
        spin(100,1,0);
    }
    pos->dir++;
    
    motor_forward(0,0);
}

void mazeStartPosition(struct sensors_ *ref, position *pos){                                //  My solution starts with the robot in the rightmost bottom corner.
                                                                                            //  This function just moves the robot to the corner while sending mqtt reports about the coordinates.
    int i = 0;
    bool onLine = 0;
    
    while (i < 4){
        reflectance_read(ref);
        if ((ref->l3 > 16000 || ref->r3 > 16000) && ref->l1 > 16000 && onLine){
            onLine = 0;                                                                     //  This boolean is a safety measure that only checks for intersections if the robot is following the line.
                                                                                            //  It is set to 0 when it reaches the line, and is set to 1 again once the robot starts following the line.
            if(pos->dir < 0)
                pos->x--;
            else if (pos->dir > 0)
                pos->x++;
            
            print_mqtt(ZUMO, "/position", "position %d %d", pos->x, pos->y);
            
            motor_forward(100,275);                                                         //  This moves the robot forward so the intersection is approximately under the middle of the motors.
            
            if(i == 0)                                                                      //  Turn right on the first intersection.
               mazeRight(ref, pos);
            
            else if (i == 3){
                mazeLeft(ref,pos);                                                          // Turn left on the last intersection.
            }
            
            else{
                mazeForward();                                                              //  Otherwise just go forward.
            }
            i++;
            
        }
        
        else{
            if(ref->r1 > 23000){                                                            //  Simple line following algorithm
                motor_turn(200,0,1);
                onLine = 1;                                                                 //  onLine set to 1 when the sensor sees black and starts following the line
            }
            else
                motor_turn(0,200,1);
            
        }
    }
    motor_forward(0,0);
}




//  MAZE
#if 0

void zmain(void)                                                                            //  Main part of the program. My solution starts with robot at the bottom right. It goes straight until it sees an obstacle.
{                                                                                           //  Upon reaching the obstacle it turns left moves one intersections and turns right to check again. This process repeats until it can go forward or sees an obstacle on the left of the robot.
    reflectance_start();                                                                    //  If there is an obstacle on the left, it turns 180 deg. and checks the same way as before but moves to the right direction.
    motor_start();                                                                          //  After moving to an intersection in the y direction it resets and start moving left again.
    IR_Start();                                                                             //  Coordinates are calculated by the direction of the robot.
    Ultra_Start();                                                                          //  The robot is currently programmed to finish the maze with only 3 obstacles. It shouldn't be too difficult to make it work for any amount if you do not have to back up. Since the only time it goes wrong is if it reaches the far right/left side of the maze without seeing an obstacle on its side.
    
    TickType_t start_time;
    TickType_t end_time;
    
    position pos;                                                                           //  Sets the position values to 0
    pos.y = pos.dir = pos.x = 0;
    
    struct sensors_ ref;
    int forceturnleft = 0;                                                                  //  Forceturn variables are used to manage the turning of the robot in the maze
    int forceturnright = 0;
    int blockedline = -1;                                                                   //  Blockedline is used to see if there is an obstacle on line y. If it is blocked robot starts moving to the right direction. for example there is a block at x= 0 y= 5 so blockedline would be 5
    
    bool onLine = 1;                                                                        //  Same as before
    
    mazeReady(&ref);
    
    IR_wait();
    start_time = xTaskGetTickCount();
    print_mqtt(ZUMO, "/start","%d",start_time);
    
    mazeForward();
    
    
    mazeStartPosition(&ref,&pos);                                                           //  Moves to bot right
    
    while(pos.y < 11){                                                                      //  The maze traversing algorithm is looping until we get to the last full line of the maze.
        reflectance_read(&ref);
        if ((ref.l3 > 18000 || ref.r3 > 18000) && ref.l1 > 18000 && onLine){                //  Condition to check intersections.
            
            onLine = 0;
            motor_forward(100,275);
            
            if(pos.dir == 0)                                                                //  Updating coordinates using the robot's direction.
                pos.y++;                                                                    //  0   Robot is straight
            if(pos.dir < 0)                                                                 //  1   Robot is facing right   (In respect to the starting position)
                pos.x--;                                                                    //  -1  Robot is facing left
            if(pos.dir > 0)
                pos.x++;
            print_mqtt(ZUMO, "/position", "position %d %d", pos.x, pos.y);
            
            if(Ultra_GetDistance() < 15 && pos.dir == 0){                                   //  Checks for obstacles every time when robot is on the intersection facing straight. If it would check in all directions 
                if(blockedline == pos.y){                                                   //  there would be a bug where it would turn left even when there is a way to go straight   (hard to explain in text, check link https://imgur.com/a/cmuyF5V)
                    forceturnleft = 0;                                                      //  Always turning left until the line is blocked from the left
                    forceturnright = 2;
                }
                
                else{
                    forceturnleft = 2;
                    forceturnright = 0;
                }  
            }
            
            if(forceturnright > 0){                                                         //  When moving rightx
                if (forceturnright == 2)                                                    //  First turn right, go to next intersection
                    mazeRight(&ref,&pos);
                    
                if (forceturnright == 1){                                                   //  Then turn left
                    mazeLeft(&ref,&pos);
                  
                    if(Ultra_GetDistance() < 15){                                           //  check for obstacles. If there are none go straight, otherwise set forceturnright to 2 and turn right.
                        forceturnright = 2;                                                 //  after setting it to 2 it gets decremented to 1 meaning it goes to the next intersection on the right an checks for obstacles again.
                        mazeRight(&ref,&pos);
                    }        
                }
                forceturnright--;
            }
            
            if (forceturnleft > 0){                                                         //  Same principle as forceturnright but after the first turn it checks for obstacle on the left.
                if (forceturnleft == 2){
                    mazeLeft(&ref,&pos);
                    
                    if(Ultra_GetDistance() < 15){                                           //  If there is an obstacle to the left of the robot, it does a 180 deg turn and continues doing the forceturnright.
                        blockedline = pos.y;
                        forceturnright = 1;                                                 //  We set it to 1 and not 2 becase we already turn right here to make it easier.
                        forceturnleft = 0;

                        mazeRight(&ref,&pos);
                        mazeRight(&ref,&pos);
                       
                    }
                }
                    
                if(forceturnleft == 1){                                                     //  Again the same as before but to the other direction.
                    mazeRight(&ref,&pos);
                    
                    if(Ultra_GetDistance() < 15){
                        forceturnleft = 2;
                        mazeLeft(&ref,&pos);
                    }
                }           
                forceturnleft--;
            }
          
            else
                mazeForward();                                                              //  If there are no obstacles just go straight
        }
        else{
            if(ref.r1 > 23000){                                                             //  Simple line follow between the intersection
                motor_turn(200,0,1);
                onLine = 1;
            }
            else
                motor_turn(0,200,1);
        }
        reflectance_read(&ref);
    }

    if(pos.x > 0)                                                                           //  After reaching the last full line the robot checks if it is to the right/left of the middle.
        mazeLeft(&ref,&pos);
    else
        mazeRight(&ref,&pos);
        
    while(pos.x != 0){                                                                      //  After checking and turning towards the middle line it starts moving forward until it stops at x = 0
        reflectance_read(&ref);
        if ((ref.l3 > 18000 || ref.r3 > 18000) && ref.l1 > 18000 && onLine){
            onLine = 0;
            if(pos.dir > 0)                                                                 //  Checking for direction of the robot so the same algorithm can be used from both directions
                pos.x++;
            else
                pos.x--;
            
            print_mqtt(ZUMO, "/position", "position %d %d", pos.x, pos.y);
            motor_forward(100,275);
        }
        else{
            if(ref.r1 > 23000){
                motor_turn(200,0,1);
                onLine = 1;
            }
            else
                motor_turn(0,200,1);
        }
    }
    
    if(pos.dir > 0)                                                                         //  Now again checks the direction of the robot to decide how to turn to face the end of the maze
        mazeLeft(&ref,&pos);
    else
        mazeRight(&ref,&pos);
    
    mazeForward();
    while(pos.y < 13){                                                                      //  At the end just go forward and count intersections until you go to the end of the maze
        reflectance_read(&ref);                                                             //  The movement is same as before so there is no need to explain it again.
        if ((ref.l3 > 18000 && ref.r3 > 18000) && ref.l1 > 18000 && onLine){
            onLine = 0;
            pos.y++;
            print_mqtt(ZUMO, "/position", "position %d %d", pos.x, pos.y);
            mazeForward();
        }
        else{
            if(ref.r1 > 23000){
                motor_turn(200,0,1);
                onLine = 1;
            }
            else
                motor_turn(0,200,1);
        }
        
    }
    
    end_time = xTaskGetTickCount();                                                         //  Get final tick count and send final mqtt messages.                                 
    print_mqtt(ZUMO, "/stop","%d",end_time);
    print_mqtt(ZUMO, "/time","%d",end_time-start_time);
    
    motor_forward(255, 300);
    motor_stop();
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif


// SUMO CHALLENGE MAIN
#if 0
void zmain(void)                                                                          // Our main function for SUMO begins with a calibration of the sensors by setting thresholds for digitally recognising between black and white.
{                                                                                         // It then moves onto waiting for the user button to be pressed which will then call function to move to the starting line and printing out "ready" 
    LSM303D_Start();                                                                      // and will then wait for an IR signal. Once signal is received the robot will move forward off the black line and begin moving straight until it hits 
    reflectance_start();                                                                  // the black line where it will reverse, turn and then continue straight again. If an impact is detected above the threshold a print message will be sent 
    motor_start();                                                                        // stating hit angle and time. Once the robot is no longer in the circle an end and total time within the circle will be printed.
    IR_Start();
    
    struct sensors_ dig;    
    struct accData_ data;
    long threshold_dig[6] = {0, 0, 0, 0, 0, 0}; 
    int hit = 0;
    TickType_t start;
    TickType_t end;
    
    LSM303D_Read_Acc(&data);
    reflectance_digital(&dig);
    button_press();
    sensorCalibration(threshold_dig, 2);                                                                                                        //calibrates the program to differentiate between black and white
    reflectance_set_threshold(threshold_dig[0], threshold_dig[1], threshold_dig[2], threshold_dig[3], threshold_dig[4], threshold_dig[5]);      //sets the threshold for defining between black or white.
    line_start(dig);
    print_mqtt(ZUMO, "/ready zumo");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt(ZUMO, "/start %d", start);
    motor_f(100, 400);
    
    LSM303D_Read_Acc(&data);
    reflectance_digital(&dig);
    
    while(dig.l1 == 0 && dig.l2 == 0 && dig.l3 == 0 && dig.r1 == 0 && dig.r2 == 0 && dig.r3 == 0){                  //while all the reflective sensors read white. 
        LSM303D_Read_Acc(&data);
        reflectance_digital(&dig);
        if((data.accX >= 13500 || data.accX <= -13500) && hit == 0){                                                //checking the accelerometer for any hits in the front or back of the robot and prints hit angle to mqtt only if hit flag  
            hit = 1;                                                                                                //is set to 0 so to remove multiple printouts.
            LSM303D_Read_Acc(&data);
            hit_angle(data.accX, data.accY);                        
            motor_f(100, 10);
        }
        else if((data.accY >= 13500 || data.accY <= -13500) && hit == 0){                                           //checking the accelerometer for any hits in the sides of the robot and prints hit angle to mqtt  only if hit flag 
            hit = 1;                                                                                                //is set to 0 so to remove multiple printouts.
            LSM303D_Read_Acc(&data);
            hit_angle(data.accX, data.accY);                        
            motor_f(100, 10);
        }
        if((data.accY <= 13500 || data.accY >= -13500) && (data.accX <= 13500 || data.accX >= -13500)){             //once the robot no longer detects a impact and returns to normal accelerometer range the hit flag will return to 0
            hit = 0;                                                                                                //allowing for new hits to register.
        }
        if(dig.l3 == 1 && dig.l2 == 1 && dig.l1 == 1 && dig.r1 == 1 && dig.r2 == 1 && dig.r3 == 1){                 //if all sensors read black, move back, spin and continue to move forward.
            motor_backward_left(100, 100, 200);
            spin(200, 500, 1);
            reflectance_digital(&dig);
        }

        else if(dig.r3 == 1 || dig.l3 == 1){                                                                        //if middle 2 sensors read black, spin and continue to move forward.
            motor_backward_left(150, 150, 500);
            spin(200, 400, 0);
            reflectance_digital(&dig);
        } 
        else{            //if all sensors are reading white still continue to move forward
            motor_f(150, 10);
        }
    }

    motor_f(0, 0);
    motor_stop();
    end = xTaskGetTickCount();
    print_mqtt(ZUMO, "/stop %d", end);
    print_mqtt(ZUMO, "time %d", end-start);

    while(SW1_Read() == 1){
    
        vTaskDelay(200);
    }
 }   

#endif

// LINE FOLLOWING
#if 1
void zmain(void)                                                                            // Our main function for LINE FOLLOWING begins with a calibration of the sensors by setting thresholds for digitally recognising between black and white.
{                                                                                           // It then moves onto waiting for the user button to be pressed which will then call function to move to the starting line and printing out "ready"
    IR_Start();                                                                             // and will then wait for an IR signal. Once signal is received the robot will move forward off the black line and begin moving straight. The robot
    reflectance_start();                                                                    // will then begin to follow a line using the reflective sensors deciding how it will move based on which sensors are reading black. The program will also
    motor_start();                                                                          // notify with a message and timestamp when any of the central sensors move off black and when it returns to following the line. To prevent message
                                                                                            // spam a flag is used to ensure that only one message of the robot left the line is sent until it returns back to reading black. At the end of the track
    struct sensors_ dig;                                                                    // the robot will stop on the second perpendicular line by using a counter to detect when all sensors see black at once.
    long threshold_dig[6] = {0, 0, 0, 0, 0, 0};
    TickType_t start;
    TickType_t end;
    int counter = 0;
    int isblack = 0;
    int miss = 0;

    vTaskDelay(100);
    reflectance_digital(&dig);
    button_press();
    sensorCalibration(threshold_dig, 2);                                                                                                        //calibrates the program to differentiate between black and white
    reflectance_set_threshold(threshold_dig[0], threshold_dig[1], threshold_dig[2], threshold_dig[3], threshold_dig[4], threshold_dig[5]);      //sets the threshold for defining between black or white.
    
    line_start(dig);
    print_mqtt(ZUMO, "/ready line");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt(ZUMO,"/start %d", start);
    motor_f(150, 150);
    while(counter < 2){                                                                                                                         //the stop criteria is all sensors reading black twice.
        reflectance_digital(&dig);
        
        if(dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3 == 6 && isblack == 0){                                                           // checking for all black reading on sensors to then increase the counter by 1. 
            isblack = 1;
            counter++;
        }
        else if(dig.l2 + dig.l3 >= 1 && dig.l1!= 1 && miss == 0){                                                                               //robot has moved off the line detected on the right hand side of the robot looking down above it.
            print_mqtt(ZUMO,"/miss %d", xTaskGetTickCount());
            miss = 1;
        }
        else if(dig.r2 + dig.r3 >= 1 && dig.r1!= 1 && miss == 0){                                                                               //robot has moved off the line detected on the left hand side of the robot looking down above it.
            print_mqtt(ZUMO,"/miss %d", xTaskGetTickCount());
            miss = 1;
        }
        else if(dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3 < 6){                                                                       // resets the flag for all sensors to detect black again for counter increments.
            isblack = 0;
        }
        else if( dig.l1 + dig.r1 >= 1 && miss == 1){                                                                                            // resets the "hit" flag for when the robot leaves the line allowing for new instances to be registered.
            print_mqtt(ZUMO,"/line %d", xTaskGetTickCount());
            miss = 0;
        }
        else if (dig.l1 == 1 && dig.r1 == 1 && dig.l3 + dig.l2 + dig.r3 + dig.r2 == 0){                                                         //moving forward
            motor_f(225, 0);
        }
        else if(dig.l1 + dig.l2 == 2 && dig.l3 + dig.r2 + dig.r3 == 0){                                                                         //slight turn right
            motor_turn(0,225,0);
        }
        else if(dig.r1 + dig.r2 == 2 && dig.l3 + dig.l2 + dig.r3 == 0){                                                                         //slight turn left
            motor_turn(225,0,0);
        }
        else if(dig.l2 + dig.l3 == 2 && dig.l1 + dig.r1 + dig.r2 + dig.r3 == 0){                                                                //greater turn right
            motor_turn(0, 225, 0);          
        }
        else if(dig.r2 + dig.r3 == 2 && dig.l1 + dig.r1 + dig.l2 + dig.l3 == 0){                                                                //greater turn left
            motor_turn(225, 0, 0); 
        }
    }
    motor_stop();
    end = xTaskGetTickCount();
    print_mqtt(ZUMO, "/end %d", end);
    print_mqtt(ZUMO, "/time %d", end-start);
    
    while(1){
    
        vTaskDelay(100);                                                                                                                         // sleep (in an infinite loop)
    }
 }   
#endif