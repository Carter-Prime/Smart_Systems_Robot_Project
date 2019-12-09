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

/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/


// CUSTOM
void spin(uint8 speed, uint32 delay, bool dir){
	//LEFT
	if(dir == 1){
		MotorDirLeft_Write(1);      // set LeftMotor backward mode
		MotorDirRight_Write(0);     // set RightMotor forward mode
		PWM_WriteCompare1(speed); 
		PWM_WriteCompare2(speed);
		
	}
	
	//RIGHT
	else{
		MotorDirLeft_Write(0);      // set LeftMotor forward mode
		MotorDirRight_Write(1);     // set RightMotor backward mode
		PWM_WriteCompare1(speed+15); 
		PWM_WriteCompare2(speed);
		
	}
    vTaskDelay(delay);
}

// MAZE FUNCTIONS
typedef struct position{
        int x;
        int y;
        int dir;
}position;


void mazeReady(struct sensors_ *ref){                                   //  MazeReady makes the robot wait for a button press and then drives to the line and sends the mqtt start msg.
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
    print_mqtt("Zumo102/ready","maze");
    motor_forward(0,0);
}

void mazeForward(void){                                                 //  Quick function to move forward a bit when no obstacle is in front of the intersection.
    motor_forward(100,150);
    motor_forward(0,0);
}

void mazeLeft(struct sensors_ *ref, position *pos){                     //  Turning left in the maze and updating the direction of the robot.
                                                                        //  Turns with a small delay to move the sensor off the black line, then continues to turn until r1 sees the line.
    spin(100,500,1);                                                    //  Uses the position structure pointer to modifiy the value in main instead of only modifying it in the function.
    
    reflectance_read(ref);
    while (ref->r1 < 16000){
        reflectance_read(ref);
        spin(100,1,1);
    }
    
    pos->dir--;
    
    motor_forward(0,0);
}

void mazeRight(struct sensors_ *ref, position *pos){                    //  Same as mazeLeft but to the right direction, and checks with l1 not r1.
    
    spin(100,500,0);
    
    reflectance_read(ref);
    while (ref->l1 < 16000){
        reflectance_read(ref);
        spin(100,1,0);
    }
    pos->dir++;
    
    motor_forward(0,0);
}

void mazeStartPosition(struct sensors_ *ref, position *pos){                            //  My solution starts with the robot in the rightmost bottom corner.
                                                                                        //  This function just moves the robot to the corner while sending mqtt reports about the coordinates.
    int i = 0;
    bool onLine = 0;
    
    while (i < 4){
        reflectance_read(ref);
        if ((ref->l3 > 16000 || ref->r3 > 16000) && ref->l1 > 16000 && onLine){
            onLine = 0;                                                                 //  This boolean is a safety measure that only checks for intersections if the robot is following the line.
                                                                                        //  It is set to 0 when it reaches the line, and is set to 1 again once the robot starts following the line.
            if(pos->dir < 0)
                pos->x--;
            else if (pos->dir > 0)
                pos->x++;
            
            print_mqtt("Zumo102/position", "position %d %d", pos->x, pos->y);
            
            motor_forward(100,275);                                                     //  This moves the robot forward so the intersection is approximately under the middle of the motors.
            
            if(i == 0)                                                                  //  Turn right on the first intersection.
               mazeRight(ref, pos);
            
            else if (i == 3){
                mazeLeft(ref,pos);                                                      // Turn left on the last intersection.
            }
            
            else{
                mazeForward();                                                          //  Otherwise just go forward.
            }
            i++;
            
        }
        
        else{
            if(ref->r1 > 23000){                                                        //  Simple line following algorithm
                motor_turn(200,0,1);
                onLine = 1;                                                             //  onLine set to 1 when the sensor sees black and starts following the line
            }
            else
                motor_turn(0,200,1);
        }
    }
    motor_forward(0,0);
}

//  MAZE
#if 1
void zmain(void)                                                                        //  Main part of the program. My solution starts with robot at the bottom right. It goes straight until it sees an obstacle.
{                                                                                       //  Upon reaching the obstacle it turns left moves one intersections and turns right to check again. This process repeats until it can go forward or sees an obstacle on the left of the robot.
    reflectance_start();                                                                //  If there is an obstacle on the left, it turns 180 deg. and checks the same way as before but moves to the right direction.
    motor_start();                                                                      //  After moving to an intersection in the y direction it resets and start moving left again.
    IR_Start();                                                                         //  Coordinates are calculated by the direction of the robot.
    Ultra_Start();                                                                      //  The robot is currently programmed for 3 obstacles. It shouldn't be difficult to modify it for any number. The only problem is going to ends of maze while not finding an obstacle.
    
    TickType_t start_time;
    TickType_t end_time;
    
    position pos;                                                                       //  Sets the position values to 0
    pos.y = pos.dir = pos.x = 0;
    
    struct sensors_ ref;
    int forceturnleft = 0;                                                              //  Forceturn variables are used to manage the turning of the robot in the maze
    int forceturnright = 0;
    int blockedline = -1;                                                               //  Blockedline is used to see if there is an obstacle on line y. If it is blocked robot starts moving to the right direction. for example there is a block at x= 0 y= 5 so blockedline would be 5
    
    bool onLine = 1;                                                                    //  Same as before
    
    mazeReady(&ref);
    
    IR_wait();
    start_time = xTaskGetTickCount();
    print_mqtt("Zumo102/start","%d",start_time);
    
    mazeForward();
    
    
    mazeStartPosition(&ref,&pos);                                                       //  Moves to bot right
    
    while(pos.y < 11){                                                                  //  The maze traversing algorithm is looping until we get to the last full line of the maze.
        reflectance_read(&ref);
        if ((ref.l3 > 18000 || ref.r3 > 18000) && ref.l1 > 18000 && onLine){            //  Condition to check intersections.
            
            onLine = 0;
            motor_forward(100,275);
            
            if(pos.dir == 0)                                                            //  Updating coordinates using the robot's direction.
                pos.y++;                                                                //  0   Robot is straight
            if(pos.dir < 0)                                                             //  1   Robot is facing right   (In respect to the starting position)
                pos.x--;                                                                //  -1  Robot is facing left
            if(pos.dir > 0)
                pos.x++;
            print_mqtt("Zumo102/position", "position %d %d", pos.x, pos.y);
            
            if(Ultra_GetDistance() < 15 && pos.dir == 0){                               //  Checks for obstacles every time when robot is on the intersection facing straight. If it would check in all directions 
                if(blockedline == pos.y){                                               //  there would be a bug where it would turn left even when there is a way to go straight   (hard to explain in text, check link https://imgur.com/a/cmuyF5V)
                    forceturnleft = 0;                                                  //  Always turning left until the line is blocked from the left
                    forceturnright = 2;
                }
                
                else{
                    forceturnleft = 2;
                    forceturnright = 0;
                }  
            }
            
            if(forceturnright > 0){                                                     //  When moving rightx
                if (forceturnright == 2)                                                //  First turn right, go to next intersection
                    mazeRight(&ref,&pos);
                    
                if (forceturnright == 1){                                               //  Then turn left
                    mazeLeft(&ref,&pos);
                  
                    if(Ultra_GetDistance() < 15){                                       //  check for obstacles. If there are none go straight, otherwise set forceturnright to 2 and turn right.
                        forceturnright = 2;                                             //  after setting it to 2 it gets decremented to 1 meaning it goes to the next intersection on the right an checks for obstacles again.
                        mazeRight(&ref,&pos);
                    }        
                }
                forceturnright--;
            }
            
            if (forceturnleft > 0){                                                     //  Same principle as forceturnright but after the first turn it checks for obstacle on the left.
                if (forceturnleft == 2){
                    mazeLeft(&ref,&pos);
                    
                    if(Ultra_GetDistance() < 15){                                       //  If there is an obstacle to the left of the robot, it does a 180 deg turn and continues doing the forceturnright.
                        blockedline = pos.y;
                        forceturnright = 1;                                             //  We set it to 1 and not 2 becase we already turn right here to make it easier.
                        forceturnleft = 0;

                        mazeRight(&ref,&pos);
                        mazeRight(&ref,&pos);
                       
                    }
                }
                    
                if(forceturnleft == 1){                                                 //  Again the same as before but to the other direction.
                    mazeRight(&ref,&pos);
                    
                    if(Ultra_GetDistance() < 15){
                        forceturnleft = 2;
                        mazeLeft(&ref,&pos);
                    }
                }           
                forceturnleft--;
            }
          
            else
                mazeForward();                                                          //  If there are no obstacles just go straight
        }
        else{
            if(ref.r1 > 23000){                                                         //  Simple line follow between the intersection
                motor_turn(200,0,1);
                onLine = 1;
            }
            else
                motor_turn(0,200,1);
        }
        reflectance_read(&ref);
    }

    if(pos.x > 0)                                                                       //  After reaching the last full line the robot checks if it is to the right/left of the middle.
        mazeLeft(&ref,&pos);
    else
        mazeRight(&ref,&pos);
        
    while(pos.x != 0){                                                                  //  After checking and turning towards the middle line it starts moving forward until it stops at x = 0
        reflectance_read(&ref);
        if ((ref.l3 > 18000 || ref.r3 > 18000) && ref.l1 > 18000 && onLine){
            onLine = 0;
            if(pos.dir > 0)                                                             //  Checking for direction of the robot so the same algorithm can be used from both directions
                pos.x++;
            else
                pos.x--;
            
            print_mqtt("Zumo102/position", "position %d %d", pos.x, pos.y);
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
    
    if(pos.dir > 0)                                                                     //  Now again checks the direction of the robot to decide how to turn to face the end of the maze
        mazeLeft(&ref,&pos);
    else
        mazeRight(&ref,&pos);
    
    mazeForward();
    while(pos.y < 13){                                                                  //  At the end just go forward and count intersections until you go to the end of the maze
        reflectance_read(&ref);                                                         //  The movement is same as before so there is no need to explain it again.
        if ((ref.l3 > 18000 && ref.r3 > 18000) && ref.l1 > 18000 && onLine){
            onLine = 0;
            pos.y++;
            print_mqtt("Zumo102/position", "position %d %d", pos.x, pos.y);
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
    
    end_time = xTaskGetTickCount();                                                     //  Get final tick count and send final mqtt messages.                                 
    print_mqtt("Zumo102/stop","%d",end_time);
    print_mqtt("Zumo102/time","%d",end_time-start_time);
    
    motor_forward(255, 300);
    motor_stop();
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

/* [] END OF FILE */
