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
/**
 * @file    main.c
 * @brief   
 * @details  ** Enable global interrupt since Zumo library uses interrupts. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/


// CUSTOM
void spin(uint8 speed, uint32 delay, bool dir)
{
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
		PWM_WriteCompare1(speed); 
		PWM_WriteCompare2(speed);
		
	}
    vTaskDelay(delay);
}


void motor_f(uint8 speed,uint32 delay)
{
    MotorDirLeft_Write(0);      // set LeftMotor forward mode
    MotorDirRight_Write(0);     // set RightMotor forward mode
    PWM_WriteCompare1(speed+15); 
    PWM_WriteCompare2(speed); 
    vTaskDelay(delay);
}

#if 1
// dumb turns
    
void zmain(void)
{
    
	
    motor_start();
     
    vTaskDelay(3000);

    //motor_f(100,3500);
    
    spin(100,1500,0);
    
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
    struct sensors_ dig;
	bool dir = 1;
	
	reflectance_start();
    motor_start();
     
    vTaskDelay(3000);
    reflectance_digital(&dig);
    for(i = 0; i < 4; i++){
		if(i > 0){
			dir = 0;			
		}
        while(dig.l3 + dig.l2 + dig.l1 + dig.r1 + dig.r2 + dig.r3 <= 60000){
            motor_forward(100,0);
            reflectance_read(&dig);
            printf("%5d %5d %5d %5d %5d %5d \n",dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
        }
		
		if(i == 4){
			motor_forward(0,0);
			break;			
		}
		
        motor_forward(100,200);
		
		while(dig.l1 <= 10000){
			spin(100,0,dir);
			reflectance_read(&dig);
			printf("%5d %5d %5d %5d %5d %5d \n",dig.l3, dig.l2, dig.l1, dig.r1, dig.r2, dig.r3);
		}
		motor_forward(0,0);
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
    
    struct sensors_ ref;
    reflectance_start();
    motor_start();
    int check = 0;
    bool linechk = 0;
    
    vTaskDelay(3000);
    
    while (1){
    reflectance_read(&ref);
    printf("%5d %5d %5d %5d %5d %5d %5d \n", ref.l3, ref.l2, ref.l1, ref.r1, ref.r2, ref.r3, check );
    motor_forward(100, 0);
    
    if (ref.l3 + ref.l2 + ref.l1 + ref.r1 + ref.r2 + ref.r3  > 70000 && linechk == 0){
        check++;
        linechk = 1;
        if(check == 1 || check == 4){
            motor_forward(0, 1000);
            if(check == 3){
                break;
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
    
    struct sensors_ ref;
    reflectance_start();
    motor_start();
    int check = 0;
    bool linechk = 0;
    
    while(SW1_Read()){
        vTaskDelay(10);
    }
    
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
