#include <stdio.h>
#include "ds1307.h"
#include "lcd.h"

#define SYSTICK_TIM_CLK   16000000UL

/* Enable this macro if you want to test RTC on LCD */
#define PRINT_LCD  


void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}


char *get_day_of_week(uint8_t day)
{
    char *days[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};  
    return days[day - 1];
}

//hh:mm:ss
char *time_to_string(RTC_time_t *rtc_time)
{
    static char time_str[9]; // HH:MM:SS\0
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", rtc_time->hours, rtc_time->minutes, rtc_time->seconds);
    return time_str;
}

char *date_to_string(RTC_date_t *rtc_date)
{
    static char date_str[11]; // DD/MM/YYYY\0
    snprintf(date_str, sizeof(date_str), "%02d/%02d/20%02d", rtc_date->date, rtc_date->month, rtc_date->year);
    return date_str;
}

int main(){
    RTC_time_t current_time;
    RTC_date_t current_date;

    if (ds1307_init()){
        printf("DS1307 initialization failed\n");
        while (1); // Infinite loop
    } else {
        printf("DS1307 initialized successfully\n");
    }


#ifndef PRINT_LCD
	printf("RTC test\n");
#else
	lcd_init();

	lcd_print_string("RTC Test...");

	mdelay(2000);

	lcd_display_clear();
	lcd_display_return_home();
#endif

    init_systick_timer(1); // 1 Hz

    current_date.day = FRIDAY;
    current_date.date = 15;
    current_date.month = 9;
    current_date.year = 23; // 2023

    current_time.seconds = 30;
    current_time.minutes = 45;
    current_time.hours = 14; // 2 PM
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);

    ds1307_get_current_time(&current_time);
    ds1307_get_current_date(&current_date);

    char *am_pm;
    if (current_time.time_format != TIME_FORMAT_24HRS){
        am_pm = (current_time.time_format) ? " PM" : " AM";
#ifndef PRINT_LCD     
        printf("Current Time = %s%s\n", time_to_string(&current_time), am_pm);   
#else    
        lcd_set_cursor(1, 1);   
        lcd_print_string(time_to_string(&current_time));    
        lcd_print_string(am_pm);
#endif
    }
    else
    {
#ifndef PRINT_LCD
        printf("Current Time = %s\n", time_to_string(&current_time)); 
#else  
				lcd_set_cursor(1, 1);   
        lcd_print_string(time_to_string(&current_time));
#endif
    }
#ifndef PRINT_LCD
    printf("Current Date = %s <%s>", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
    lcd_set_cursor(2, 1);   
    lcd_print_string(date_to_string(&current_date));
#endif

    while(1);

    return 0;
}

void SysTick_Handler(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    ds1307_get_current_time(&current_time);
    ds1307_get_current_date(&current_date);

    
    char *am_pm;
    if (current_time.time_format != TIME_FORMAT_24HRS){
        am_pm = (current_time.time_format) ? " PM" : " AM";
#ifndef PRINT_LCD
        printf("Current Time = %s%s\n", time_to_string(&current_time), am_pm);   
#else
        lcd_set_cursor(1, 1);
        lcd_print_string(time_to_string(&current_time));    
        lcd_print_string(am_pm);
#endif
    }
    else
    {
#ifndef PRINT_LCD
        printf("Current Time = %s\n", time_to_string(&current_time));   
#else
			  lcd_set_cursor(1, 1);   
        lcd_print_string(time_to_string(&current_time));    
#endif
    }
#ifndef PRINT_LCD
    printf("Current Date = %s <%s>", date_to_string(&current_date), get_day_of_week(current_date.day));
#else
    lcd_set_cursor(2, 1);   
    lcd_print_string(date_to_string(&current_date));
#endif
}