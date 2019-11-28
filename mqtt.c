print_mqtt("Zumo009/ready","zumo");

RTC_Start(); //initialised in the code
RTC_TIME_DATE now;
//set current time here, works for the whole code - stop and start
now.Hour = 21;
now.Min = 56;
now.Sec = 16;
RTC_WriteTime(&now); 

RTC_DisableInt(); //for registering start time, need all 3 lines
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt("Zumo009/start","%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);

RTC_DisableInt(); //for registering stop time, need all 3 lines
 now = *RTC_ReadTime(); 
 RTC_EnableInt(); 

print_mqtt("Zumo009/stop","%2d:%02d.%02d\n", now.Hour, now.Min, now.Sec);


print_mqtt("Zumo009/time","zumo");