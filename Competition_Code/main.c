/**
* @mainpage ZumoBot Project
* @brief    You can make your own ZumoBot with various sensors.
* @details  <br><br>
    <p>
    <B>General</B><br>
    You will use Pololu Zumo Shields for your robot project with CY8CKIT-059(PSoC 5LP) from Cypress semiconductor.This 
    library has basic methods of various sensors and communications so that you can make what you want with them. <br> 
    <br><br>
    </p>
    
    <p>
    <B>Sensors</B><br>
    &nbsp;Included: <br>
        &nbsp;&nbsp;&nbsp;&nbsp;LSM303D: Accelerometer & Magnetometer<br>
        &nbsp;&nbsp;&nbsp;&nbsp;L3GD20H: Gyroscope<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Reflectance sensor<br>
        &nbsp;&nbsp;&nbsp;&nbsp;Motors
    &nbsp;Wii nunchuck<br>
    &nbsp;TSOP-2236: IR Receiver<br>
    &nbsp;HC-SR04: Ultrasonic sensor<br>
    &nbsp;APDS-9301: Ambient light sensor<br>
    &nbsp;IR LED <br><br><br>
    </p>
    
    <p>
    <B>Communication</B><br>
    I2C, UART, Serial<br>
    </p>
*/

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





#if 1
//  MAZE
void zmain(void)                                                                        //  Main part of the program. My solution starts with robot at the bottom right. It goes straight until it sees an obstacle.
{                                                                                       //  Upon reaching the obstacle it turns left moves one intersections and turns right to check again. This process repeats until it can go forward or sees an obstacle on the left of the robot.
    reflectance_start();                                                                //  If there is an obstacle on the left, it turns 180 deg. and checks the same way as before but moves to the right direction.
    motor_start();                                                                      //  After moving to an intersection in the y direction it resets and start moving left again.
    IR_Start();                                                                         //  Coordinates are calculated by the direction of the robot.
    Ultra_Start();                                                                      //  The robot is currently programmed to finish the maze with only 3 obstacles. It shouldn't be too difficult to make it work for any amount if you do not have to back up. Since the only time it goes wrong is if it reaches the far right/left side of the maze without seeing an obstacle on its side.
    
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

#if 0
//LINE FOLLOW FINAL
    
void zmain(void)
{
    reflectance_start();
    motor_start();
    IR_Start();
    
    const long black = 20000;
    const long half = 15000;
    const long white = 10000;
    TickType_t start;
    TickType_t end;
    
    int counter = 0;
    bool isblack = 1;
    
    struct sensors_ ref;
    
    while(SW1_Read() == 1){
        vTaskDelay(10);
    } 
   
    reflectance_read(&ref);
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < black*6){
        reflectance_read(&ref);
        motor_f(100,1);
    }
    motor_f(0,0);
    print_mqtt("Zumo009/ready", "line");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo009/start", "%d", start);
    
    while(counter < 2){
        reflectance_read(&ref);
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 > black*6 && isblack == 0){
            isblack = 1;
            counter++;
        }
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < black*6){
            isblack = 0;
        }
        
        if(ref.r1 > black){
            motor_turn(254,0,0);
        }
        else{
            motor_turn(0,254,0);
        }
    }
    end = xTaskGetTickCount();
    print_mqtt("Zumo009/end", "%d", end);
    print_mqtt("Zumo009/time", "%d", end-start);
    
    motor_stop();
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// VALUE CHECK
    
void zmain(void)
{
    reflectance_start();
    motor_start();
    IR_Start();
    
    const long black = 20000;
    const long white = 10000;
    TickType_t start;
    TickType_t end;
    
    
    struct sensors_ ref;
    while(1){
    while(SW1_Read() == 1){
        vTaskDelay(10);
    }
    reflectance_read(&ref);
    print_mqtt("Zumo009/cali", "BLACK : %d %d %d %d %d %d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
    
    vTaskDelay(1000);
    Beep(300,100);
    
    while(SW1_Read() == 1){
        vTaskDelay(10);
    }
    reflectance_read(&ref);
    print_mqtt("Zumo009/cali", "WHITE : %d %d %d %d %d %d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
    
    vTaskDelay(1000);
    Beep(300,100);
    
    while(SW1_Read() == 1){
        vTaskDelay(10);
    }
    reflectance_read(&ref);
    print_mqtt("Zumo009/cali", "LINE : %d %d %d %d %d %d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
    print_mqtt("Zumo009/cali", " ");
    vTaskDelay(1000);
    }
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif



#if 0
// mqtt intersection
    
void zmain(void)
{
    reflectance_start();
    motor_start();
    IR_Start();
    
    TickType_t start;
    TickType_t end;
    int count = 0;
    struct sensors_ ref; 
    bool onblack = 0;
    
    motor_start();
    
    IR_wait();
    
    while(count < 2){
        reflectance_read(&ref);
        motor_f(100,1);
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 > 60000 && onblack == 0){
            onblack = 1;
            count++;
            if(count == 1){
                motor_f(0,0);
                IR_wait();   
                start = xTaskGetTickCount();
            }
        }
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 60000){
            onblack = 0;
        }
        
        
    }
    end = xTaskGetTickCount();
    float result = ((float)(end-start))/1000;
    print_mqtt("Zumo009/lap", "Time: %.2f", result);
    
    
    motor_stop();
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif


#if 0
// mqtt sonic
    
void zmain(void)
{
    Ultra_Start();
    motor_start();
    
    while(1){
        if(Ultra_GetDistance() < 5){
            motor_backward(100,1000);
            
            
            if(rand()%2 == 0){
                spin(100,700,1);
                print_mqtt("Zumo009/turn", "LEFT");
            }
            
            else{
                spin(100,700,0);
                print_mqtt("Zumo009/turn", "RIGHT");
            }
            
        }
        motor_f(0,0);
    
    }
    motor_stop();
    
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// RTC
    
void zmain(void)
{
    RTC_Start();
    RTC_TIME_DATE now;
    
    int hours = 0,minutes = 0;
    
    while((hours < 1 || hours > 24)||(minutes < 0 || minutes > 60)){
        printf("enter the hours and minutes please\n");
        scanf("%d", &hours);
        scanf("%d", &minutes);
    }
    
    now.Hour = hours;
    now.Min = minutes;
    now.Sec = 1;
    now.DayOfMonth = 13;
    now.Month = 6;
    now.Year = 2000;
    RTC_WriteTime(&now);
    
    
    while(1){
        if (SW1_Read() == 0){
            RTC_DisableInt();
            now = *RTC_ReadTime();
            RTC_EnableInt();    
            
            printf("Time: %2d:%02d\n", now.Hour,now.Min);
            print_mqtt("Zumo009/output", "Time: %2d:%02d", now.Hour,now.Min);
        }
        vTaskDelay(250);
    }
    
    
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// line follow v1
    
void zmain(void)
{
    reflectance_start();
    motor_start();
    IR_Start();
    
    struct sensors_ ref;
	
    reflectance_read(&ref);
    
    IR_wait();
    reflectance_read(&ref);
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    motor_f(0,0);
    IR_wait();
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 > 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
     motor_f(0,0);
    
    
    while(1){
        reflectance_read(&ref);
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 > 70000){
            motor_f(0,0);
            break;
        }
        
        if(ref.r2 > 9000){
            motor_turn(254,0,0);
        }
        else{
            motor_turn(0,254,0);
        }
    }
    motor_stop();
    
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif



#if 0
// INTERSECTIONS
    
void zmain(void)
{
    reflectance_start();
    motor_start();
    IR_Start();
    
    struct sensors_ ref;
	
    IR_wait();
    reflectance_read(&ref);
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    motor_f(0,0);
    IR_wait();
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 > 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
     motor_f(0,0);
    
    
    ////////////
    
    
    
    
     while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    motor_forward(100,300);
    
    reflectance_read(&ref);
    while (ref.l1 < 10000){
         reflectance_read(&ref);
        printf("%d\n", ref.r1);
         spin(100,1,1);
    }
    
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    motor_forward(100,300);
    
    spin(100,750,0);
      
    reflectance_read(&ref);
    while (ref.l2 < 9000){
        reflectance_read(&ref);
        spin(100,1,0);
    }
    
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    
    motor_forward(100,300);
    spin(100,750,0);  
    reflectance_read(&ref);
    while (ref.l2 < 9000){
        reflectance_read(&ref);
        spin(100,1,0);
    }
    
    while(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 70000){
        reflectance_read(&ref);
        motor_f(100,0);
    }
    motor_forward(100,300);
    
    
    
    

    
    
	
    
    
    motor_stop();
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif


#if 0
// SKIPPING LINES
void zmain(void)
{
    IR_Start();
    struct sensors_ ref;
    reflectance_start();
    motor_start();
    int check = 0;
    bool linechk = 0;
    
    /*
    while(SW1_Read()){
        vTaskDelay(10);
    }
    */
    IR_wait();
    
    while (check < 4){
        reflectance_read(&ref);
        printf("%5d %5d %5d %5d %5d %5d %5d \n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3, check );
        motor_f(100, 0);
        
        if (ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3  > 70000 && linechk == 0){
            check++;
            linechk = 1;
            if(check == 1 || check == 4){
                motor_forward(0, 1000);
                if(check == 1){
                   IR_wait();
                }
            }
        }
        
        if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 60000){
            linechk = 0;
        }
    
    }
    
    motor_stop();
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif


#if 0
// SKIPPING LINES
void zmain(void)
{
    IR_Start();
    struct sensors_ ref;
    reflectance_start();
    motor_start();
    int check = 0;
    bool linechk = 0;
    
    IR_wait();
    
    while (1){
    reflectance_digital(&ref);
    printf("%5d %5d %5d %5d %5d %5d %5d \n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3, check );
    motor_forward(100, 0);
    
    if (ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3  == 6 && linechk == 0){
        linechk = 1;
        if(check == 3 || check == 0){
            motor_forward(0, 1000);
        }
        check++;
     
    }
    
    if(ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3 < 6){
        linechk = 0;
    }
    
    }
    
    motor_stop();
    
    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// SQUARE
void zmain(void)
{
    vTaskDelay(3000);
	motor_start();
    motor_f(100, 3500);
	spin(100, 590, 0);
	motor_f(100, 3500);
	spin(100, 590, 0);
    motor_f(100, 3500);
	spin(100, 590, 0);
    vTaskDelay(500);
	motor_turn(120, 100, 3500);
    
    motor_stop();

    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// accelerometer
void zmain(void)
{
    struct accData_ data[2];
    uint8 spd =  170;
    
    vTaskDelay(3000);
	motor_start();
    
    if(!LSM303D_Start()){
        printf("FAIL");
        while (1){
             vTaskDelay(100);
        }
    }
    else{
        printf("all is ok\n");
    }
 
    while (1){
        if (data[0].accX < -3000){
        
            motor_backward(50, 2000);
            spin(100, 590, 0);
        
        }
        motor_forward(spd, 10);
        LSM303D_Read_Acc(&data[0]);
       
        LSM303D_Read_Acc(&data[1]);
       
        printf("%10d %10d\n",data[0].accX, data[0].accY);
 
        
        
       
        
    }
    
    
    motor_stop();

    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif






												// WEEK 1
#if 0
// SOS
void zmain(void)
{
	int i,j;
	while(1){
		if(SW1_Read() == 0){
			for(i = 0; i < 3; i++){
				for(j = 0; j < 3; j++){
					BatteryLed_Write(1);
					if(i == 1){
						vTaskDelay(1000);
					}
					vTaskDelay(500);
					
					BatteryLed_Write(0);
					vTaskDelay(500);
				}
			}
		}
	}


    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// age reaction time
void zmain(void)
{
	TickType_t start;
	int age = 0, total;
	
	printf("Enter your age\n");
	start = xTaskGetTickCount();
	scanf("%d", &age);
	total = ((int)(xTaskGetTickCount() - start)) / 1000;
	printf("%d\n", total);
	
	
	if(age <= 21 ){
		if(total < 3){
			printf("Super fast dude!\n");
		}
		else if(total > 5){
			printf("My granny is faster than you!\n");
		}
		else{
			printf("So mediocre.\n");
		}
	}
	else if(age > 50){
		if(total < 3){
			printf("Still going strong\n");
		}
		else if(total > 5){
			printf("Do they still allow you to drive?\n");
		}
		else{
			printf("You are doing ok for your age.\n");
		}
		
	}
	else{
		if(total < 3){
			printf("Be quick or be dead\n");
		}
		else if(total > 5){
			printf("Have you been smoking something illegal?\n");
		}
		else{
			printf("You’re so average.\n");
		}
		
	}


    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
// age reaction time
void zmain(void)
{
	
	


    while(1)
    {
        vTaskDelay(100); // sleep (in an infinite loop)
    }
 }   
#endif

#if 0
//battery level//
void zmain(void)
{
    ADC_Battery_Start();        

    int16 adcresult =0;
    float volts = 0.0;
	bool led = 1;

    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed

    while(true)
    {
        //char msg[80];
        ADC_Battery_StartConvert(); // start sampling
        if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) {   // wait for ADC converted value
            adcresult = ADC_Battery_GetResult16(); // get the ADC value (0 - 4095)
            // convert value to Volts
            // you need to implement the conversion
			
			volts = (((adcresult)/(pow(2,12)-1))*5)/(20000.0/30000.0);
			
			if(volts < 4){
				while(SW1_Read() == 1){
					BatteryLed_Write(led);
					vTaskDelay(250);
					led = !led;
				}

			}
			else{
				BatteryLed_Write(0);
				vTaskDelay(500);
			}
			
			
			
			
            
            // Print both ADC results and converted value
            printf("%d %f\r\n",adcresult, volts);
        }
        vTaskDelay(500);
    }
 }   
#endif

#if 0
// Name and age
void zmain(void)
{
    char name[32];
    int age;
    
		
    printf("\n\n");
    
    printf("Enter your name: ");
    //fflush(stdout);
    scanf("%s", name);
    printf("Enter your age: ");
    //fflush(stdout);
    scanf("%d", &age);
    
    printf("You are [%s], age = %d\n", name, age);

    while(true)
    {
        BatteryLed_Write(!SW1_Read());
        vTaskDelay(100);
    }
 }   
#endif

#if 0
// button
void zmain(void)
{
    while(true) {
        printf("Press button within 5 seconds!\n");
        int i = 50;
        while(i > 0) {
            if(SW1_Read() == 0) {
                break;
            }
            vTaskDelay(100);
            --i;
        }
        if(i > 0) {
            printf("Good work\n");
            while(SW1_Read() == 0) vTaskDelay(10); // wait until button is released
        }
        else {
            printf("You didn't press the button\n");
        }
    }
}
#endif

#if 0
// button
void zmain(void)
{
    printf("\nBoot\n");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 
    
    //uint8 button;
    //button = SW1_Read(); // read SW1 on pSoC board
    // SW1_Read() returns zero when button is pressed
    // SW1_Read() returns one when button is not pressed
    
    bool led = false;
    
    while(true)
    {
        // toggle led state when button is pressed
        if(SW1_Read() == 0) {
            led = !led;
            BatteryLed_Write(led);
            if(led) printf("Led is ON\n");
            else printf("Led is OFF\n");
            Beep(1000, 150);
            while(SW1_Read() == 0) vTaskDelay(10); // wait while button is being pressed
        }        
    }
 }   
#endif


#if 0
//ultrasonic sensor//
void zmain(void)
{
    Ultra_Start();                          // Ultra Sonic Start function
    
    while(true) {
        int d = Ultra_GetDistance();
        // Print the detected distance (centimeters)
        printf("distance = %d\r\n", d);
        vTaskDelay(200);
    }
}   
#endif

#if 0
//IR receiverm - how to wait for IR remote commands
void zmain(void)
{
    IR_Start();
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    bool led = false;
    // Toggle led when IR signal is received
    while(true)
    {
        IR_wait();  // wait for IR command
        led = !led;
        BatteryLed_Write(led);
        if(led) printf("Led is ON\n");
        else printf("Led is OFF\n");
    }    
 }   
#endif



#if 0
//IR receiver - read raw data
void zmain(void)
{
    IR_Start();
    
    uint32_t IR_val; 
    
    printf("\n\nIR test\n");
    
    IR_flush(); // clear IR receive buffer
    printf("Buffer cleared\n");
    
    // print received IR pulses and their lengths
    while(true)
    {
        if(IR_get(&IR_val, portMAX_DELAY)) {
            int l = IR_val & IR_SIGNAL_MASK; // get pulse length
            int b = 0;
            if((IR_val & IR_SIGNAL_HIGH) != 0) b = 1; // get pulse state (0/1)
            printf("%d %d\r\n",b, l);
        }
    }    
 }   
#endif


#if 0
//reflectance
void zmain(void)
{
    struct sensors_ ref;
    struct sensors_ dig;

    reflectance_start();
    reflectance_set_threshold(9000, 9000, 11000, 11000, 9000, 9000); // set center sensor threshold to 11000 and others to 9000
    

    while(true)
    {
        // read raw sensor values
        reflectance_read(&ref);
        // print out each period of reflectance sensors
        printf("%5d %5d %5d %5d %5d %5d\r\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
        
        // read digital values that are based on threshold. 0 = white, 1 = black
        // when blackness value is over threshold the sensors reads 1, otherwise 0
        reflectance_digital(&dig); 
        //print out 0 or 1 according to results of reflectance period
        printf("%5d %5d %5d %5d %5d %5d \r\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);        
        
        vTaskDelay(200);
    }
}   
#endif


#if 0
//motor
void zmain(void)
{
    motor_start();              // enable motor controller
    motor_forward(0,0);         // set speed to zero to stop motors

    vTaskDelay(3000);
    
    motor_forward(100,2000);     // moving forward
    motor_turn(200,50,2000);     // turn
    motor_turn(50,200,2000);     // turn
    motor_backward(100,2000);    // moving backward
     
    motor_forward(0,0);         // stop motors

    motor_stop();               // disable motor controller
    
    while(true)
    {
        vTaskDelay(100);
    }
}
#endif

#if 0
/* Example of how to use te Accelerometer!!!*/
void zmain(void)
{
    struct accData_ data;
    
    printf("Accelerometer test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Device Ok...\n");
    }
    
    while(true)
    {
        LSM303D_Read_Acc(&data);
        printf("%8d %8d %8d\n",data.accX, data.accY, data.accZ);
        vTaskDelay(50);
    }
 }   
#endif    

#if 0
// MQTT test
void zmain(void)
{
    int ctr = 0;

    printf("\nBoot\n");
    send_mqtt("Zumo01/debug", "Boot");

    //BatteryLed_Write(1); // Switch led on 
    BatteryLed_Write(0); // Switch led off 

    while(true)
    {
        printf("Ctr: %d, Button: %d\n", ctr, SW1_Read());
        print_mqtt("Zumo01/debug", "Ctr: %d, Button: %d", ctr, SW1_Read());

        vTaskDelay(1000);
        ctr++;
    }
 }   
#endif


#if 0
void zmain(void)
{    
    struct accData_ data;
    struct sensors_ ref;
    struct sensors_ dig;
    
    printf("MQTT and sensor test...\n");

    if(!LSM303D_Start()){
        printf("LSM303D failed to initialize!!! Program is Ending!!!\n");
        vTaskSuspend(NULL);
    }
    else {
        printf("Accelerometer Ok...\n");
    }
    
    int ctr = 0;
    reflectance_start();
    while(true)
    {
        LSM303D_Read_Acc(&data);
        // send data when we detect a hit and at 10 second intervals
        if(data.accX > 1500 || ++ctr > 1000) {
            printf("Acc: %8d %8d %8d\n",data.accX, data.accY, data.accZ);
            print_mqtt("Zumo01/acc", "%d,%d,%d", data.accX, data.accY, data.accZ);
            reflectance_read(&ref);
            printf("Ref: %8d %8d %8d %8d %8d %8d\n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);       
            print_mqtt("Zumo01/ref", "%d,%d,%d,%d,%d,%d", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3);
            reflectance_digital(&dig);
            printf("Dig: %8d %8d %8d %8d %8d %8d\n", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            print_mqtt("Zumo01/dig", "%d,%d,%d,%d,%d,%d", dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
            ctr = 0;
        }
        vTaskDelay(10);
    }
 }   

#endif

#if 0
void zmain(void)
{    
    RTC_Start(); // start real time clock
    
    RTC_TIME_DATE now;

    // set current time
    now.Hour = 12;
    now.Min = 34;
    now.Sec = 56;
    now.DayOfMonth = 25;
    now.Month = 9;
    now.Year = 2018;
    RTC_WriteTime(&now); // write the time to real time clock

    while(true)
    {
        if(SW1_Read() == 0) {
            // read the current time
            RTC_DisableInt(); /* Disable Interrupt of RTC Component */
            now = *RTC_ReadTime(); /* copy the current time to a local variable */
            RTC_EnableInt(); /* Enable Interrupt of RTC Component */

            // print the current time
            printf("%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
            
            // wait until button is released
            while(SW1_Read() == 0) vTaskDelay(50);
        }
        vTaskDelay(50);
    }
 }   
#endif

/* [] END OF FILE */
