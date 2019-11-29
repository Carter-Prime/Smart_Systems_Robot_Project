print_mqtt("/ready zumo");

RTC_Start(); //initialised in the code
RTC_TIME_DATE now;
//set current time here, works for the whole code - stop and start
now.Hour = 9;
now.Min = 31;
now.Sec = 16;
RTC_WriteTime(&now); 

int start = now.Hour, now.Min, now.Sec;
int stop = now.Hour, now.Min, now.Sec; 
int difference; 

RTC_DisableInt(); //for registering start time, need all 3 lines
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt("/start %2d:%02d.%02d\n", start, now.Hour, now.Min, now.Sec);
scanf(&start);

RTC_DisableInt(); //for registering stop time, need all 3 lines
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt("/stop %2d:%02d.%02d\n", stop, now.Hour, now.Min, now.Sec);
scanf(&stop);


difference = stop - start;
print_mqtt("/time stop - start = difference", start, stop, difference);

//on hit
print_mqtt("/hit hit");