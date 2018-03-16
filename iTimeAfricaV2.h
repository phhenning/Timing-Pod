/**
*header file for ITA v2 PCB
*/

#define VERSION "V3.3"
//GLOBAL DEFINITIONS         

#define DS3231_I2C_ADDRESS 	0x68

//PCB pin definition for V32
#define LCD_RST				A1
#define SQ_WAVE				A2
#define CONFIG 				A3
#define SDA					SDA
#define SCL					SCL
#define V_MEAS				A7
#define STATUS_LED			1
#define BUZZER 				2
#define LED_TOP				3
#define LED_BOT				A0
#define LCD_CLK				4
#define LCD_DIN				5
#define LCD_COM 			6
#define LCD_CE 				7
#define UART_TX				8
#define UART_RX				9
#define RFID_CS				10 
#define RFID_MOSI			MOSI 
#define RFID_MISO			MISO
#define RFID_SCK			SCK 


// EEPROM stuffs
// Arduino have different amounts of EEPROM
// 1024 bytes on the ATmega328P

#define ADDRESS_POD_TYPE 0

#define BATT_GOOD       3.8
#define BATT_LOW        3.5
#define BATT_OK_FLASH   2000
#define BATT_LOW_FLASH  100
// BELOW IS STATION/POD IDENTIFICATION!
#define LIST_SIZE 16
#define START_POD  0
#define FINISH_POD 1

struct podType {
	int block;
	char podID[16]; 
	int  role;
} ;


struct podType podTypeList [LIST_SIZE] = {
	{ 8 , "Stage 1 Start" , START_POD },
	{ 9 , "Stage 1 Finish", FINISH_POD }, 
	{ 12, "Stage 2 Start" , START_POD },
	{ 13, "Stage 2 Finish", FINISH_POD },
	{ 16, "Stage 3 Start" , START_POD },
	{ 17, "Stage 3 Finish", FINISH_POD },
	{ 20, "Stage 4 Start" , START_POD },
	{ 21, "Stage 4 Finish", FINISH_POD },
	{ 24, "Stage 5 Start" , START_POD },
	{ 25, "Stage 5 Finish", FINISH_POD },
	{ 28, "Stage 6 Start" , START_POD },
	{ 29, "Stage 6 Finish", FINISH_POD },
	{ 32, "Stage 7 Start" , START_POD },
	{ 33, "Stage 7 Finish", FINISH_POD },
	{ 36, "Stage 8 Start" , START_POD },
	{ 37, "Stage 8 Finish", FINISH_POD },
    };
// variouse values
#define CONFIG_TIME_MS 15000  
// int main()
// {
//     printf("Hello World\n");
//     printf( "%i\n",STRUCTSIZE);
//     for ( int i = 0 ; i < STRUCTSIZE ; i++ ) {
//          printf("%s\n", podTypeList[i].podID);
//     }

//     return 0;
// }
