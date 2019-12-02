#include <time.h> //include so the time caculation can happen
time_t start, stop;

//prints when robot is ready to start at the line
print_mqtt(ZUMO,"/ready zumo");

//initialise this at the start of the code
RTC_Start(); 
RTC_TIME_DATE now;
//set current time here, works for the whole code - stop and start time
now.Hour = 9;
now.Min = 31;
now.Sec = 16;
RTC_WriteTime(&now); 

//for registering start time, need all 5 lines
RTC_DisableInt(); 
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt(ZUMO,"/start %2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
time(&start);

//for registering stop time, need all 5 lines
RTC_DisableInt(); 
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt(ZUMO,"/stop %2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);
time(&stop);


//for calculating the total time when the robot was fighting
double time_taken = double(end - start);
print_mqtt(ZUMO,"/time &2d:%02d.%02d", time_taken);

//on hit prints
print_mqtt(ZUMO,"/hit hit");
