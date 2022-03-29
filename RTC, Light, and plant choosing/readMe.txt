IOC:
- under timers: RTC
	activate clock source
	active calendar
	RTC OUT: RTC output on the Tamper
- parameter settings:
	see pictures

MAIN.C:
- in USER CODE INCLUDES:
	#include "time.h"
- in USER CODE BEGIN PV:
	RTC_TimeTypeDef gTime;
	
	uint8_t plant_choose = 0; //this is the plant choosing variable, ASCII 65-70, this is what is received from the pi

	struct plant //this is the struct i use to classify the plants
	{
		float upper_pH;
		float lower_pH;
		float current_pH;
		float location; //0 = outside, 1 = inside
	};

- in USER CODE PFP (private function prototypes)
	void set_Plant(uint8_t a, struct plant *PLANT_STRUCT);

- in USER CODE BEGIN 0:
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds
	
- in int main(void)
	- under USER CODE BEGIN 1:
		struct plant PLANT_STRUCT; //this creates that plant struct	
	- under USER CODE BEGIN 2:
		set_Plant(plant_choose, &PLANT_STRUCT) //calls the function and sets the plant values based on what plant choose variable is seen

- in while(1)
	//code below starts the clock and gets the current hours, minutes, seconds
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //this starts the RTC at what it is initialized at, 00:00:00
	hours = gTime.Hours; //these variables are assigned the values of what the Real Time Clock is at
	minutes = gTime.Minutes; //these won't be accurate to the current time of day, but are only used for timing 
	seconds = gTime.Seconds;

	//code below is the logic for turning the lights on and off at 0 and 18 hours, based off of the "hour" variable
	if (PLANT_STRUCT.location == 1) //if the plant is set to be grown indoors
	{
		switch (hours) //this works! just make sure you know what plant and location you're chooosing
		{
			case 0: //time to turn on lights at 0 hours
				light_bar(500);
			break;
			case 18: //turn lights off at 18 hours
				light_bar(0);
			break;
			default:
				break;
		  }
	  }
	  else
	  {
	  }	

- in USER CODE BEGIN 4 (at the very bottom where the functions are defined):

void set_Plant(uint8_t a, struct plant *PLANT_STRUCT) 
{
	switch(a)
	{
	case 65: //tomatoes outdoors
		PLANT_STRUCT->upper_pH = 7.5; //-> sets the value
		PLANT_STRUCT->lower_pH = 5.5;
		PLANT_STRUCT->location = 0;
	break;
	case 66: //sunflowers outdoors
		PLANT_STRUCT->upper_pH = 7.5;
		PLANT_STRUCT->lower_pH = 6.0;
		PLANT_STRUCT->location = 0;
	break;
	case 67: //potato outdoors
		PLANT_STRUCT->upper_pH = 6.5;
		PLANT_STRUCT->lower_pH = 4.8;
		PLANT_STRUCT->location = 0;
	break;
	case 68: //tomatoes inside
		PLANT_STRUCT->upper_pH = 7.5; //-> sets the value
		PLANT_STRUCT->lower_pH = 5.5;
		PLANT_STRUCT->location = 1;
	break;
	case 69: //sunflowers inside
		PLANT_STRUCT->upper_pH = 7.5;
		PLANT_STRUCT->lower_pH = 6.0;
		PLANT_STRUCT->location = 1;
	break;
	case 70: //potatoes indoors
		PLANT_STRUCT->upper_pH = 6.5;
		PLANT_STRUCT->lower_pH = 4.8;
		PLANT_STRUCT->location = 1;
	break;
	default:
		break;
	}
}