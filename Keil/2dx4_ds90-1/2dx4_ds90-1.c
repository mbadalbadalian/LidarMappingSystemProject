//Matthew Badal-Badalian
//400187878
//badalbam

//Import libraries and functions from other documents
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include <math.h>
#define I2C_MCS_ACK             0x00000008  //Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  //Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  //Acknowledge Address
#define I2C_MCS_STOP            0x00000004  //Generate STOP
#define I2C_MCS_START           0x00000002  //Generate START
#define I2C_MCS_ERROR           0x00000002  //Error
#define I2C_MCS_RUN             0x00000001  //I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  //I2C Busy
#define I2C_MCR_MFE             0x00000010  //I2C Master Function Enable
#define MAXRETRIES              5           //number of receive attempts before giving up

//Initialize variables
uint16_t	ToFAddress=0b1010010;
int status=0;

//Sets device in interrupt mode
#define isInterrupt 1 

//Initialize function names 
void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//Data from VL53L1X are stored for inspection
uint16_t debugArray[100];

//Port 0 is where the input of the button goes and ports 2-5 are used for outputs
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;  //Activate the clock for Port M		
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	//Allow time for clock to stabilize
	GPIO_PORTM_DIR_R = 0b00111100; 
  GPIO_PORTM_DEN_R = 0b00111101; //Enable digital I/O on PM0,PM2-PM5	
	return;
}

//Port 0 is an output (Onboard LED)
void PortF_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;	//Activate the clock for Port F	
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	//Allow time for clock to stabilize
	GPIO_PORTF_DIR_R = 0b01;        								
  GPIO_PORTF_DEN_R = 0b01;	//Enable digital I/O on PF0																							
	return;
}

//Used for the external LED whe PL3 is an output
void PortL_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;	//Activate the clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	//Allow time for clock to stabilize
	GPIO_PORTL_DIR_R = 0b00001000;        								    								
  GPIO_PORTL_DEN_R = 0b00001000;	//Enable digital I/O on PL3	   								    									
	return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
	//Use PortG0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port G
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
	GPIO_PORTG_DIR_R &= 0b00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0b01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0b01;                                        // enable digital I/O on PG0                                                                                                 
  GPIO_PORTG_AMSEL_R &= ~0b01;                                     // disable analog functionality on PN0
  return;
}

//Initialize I2C ports on the microcontroller
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           //activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          //activate port B
  while((SYSCTL_PRGPIO_R&0b0010) == 0){}; //ready?
	GPIO_PORTB_AFSEL_R |= 0b1100;           // 3) enable alt funct on PB2,3       0b00001100
	GPIO_PORTB_ODR_R |= 0b1000;             // 4) enable open drain on PB3 only
	GPIO_PORTB_DEN_R |= 0b1100;             // 5) enable digital I/O on PB2,3
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
	I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 😎 configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}


//XSHUT This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
	GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
	GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
	FlashAllLEDs();
	SysTick_Wait10ms(10);
	GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)    
}

//Main function where the motor rotates and the sensor collects the data measurements
void dataCollection(int xValue){
	uint8_t byteData, sensorState=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint16_t ToFAddress= 0b1010010;
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(ToFAddress, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
  status = VL53L1_RdByte(ToFAddress, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
	status = VL53L1_RdWord(ToFAddress, 0x010F, &wordData);
	status = VL53L1X_GetSensorId(ToFAddress, &wordData);
	
	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(ToFAddress, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	FlashAllLEDs();
	status = VL53L1X_ClearInterrupt(ToFAddress); /* clear interrupt has to be called to enable next interrupt*/
	status = VL53L1X_SensorInit(ToFAddress);
	status = VL53L1X_StartRanging(ToFAddress);   /* This function has to be called to enable the ranging */
	
	double theta = 0;	
  double fac = 0.01227185;
	
	//Loops 512 times for the number of steps the motor takes to complete a full revolution
	for(int iteration=0; iteration<512; iteration++){ 
		
		//Turns off the external LED and turns on the internal LED 
		GPIO_PORTL_DATA_R = 0b00000000;
		GPIO_PORTF_DATA_R = 0b00000001; 
			
		//This controls the movement of the motor
		uint32_t t= 5;
		SysTick_Wait10ms(t); 
		GPIO_PORTM_DATA_R = 0b00100100;
		SysTick_Wait10ms(t);
		GPIO_PORTM_DATA_R = 0b00110000;
		SysTick_Wait10ms(t);
		GPIO_PORTM_DATA_R = 0b00011000;
		SysTick_Wait10ms(t);
		GPIO_PORTM_DATA_R = 0b00001100;
		
		//Performs data measurements
		while (dataReady == 0){
			status = VL53L1X_CheckForDataReady(ToFAddress, &dataReady);
			FlashLED3(1);
			VL53L1_WaitMs(ToFAddress, 1);
		}
		dataReady = 0;
		//Gets the RangeStatus and Distance value
		status = VL53L1X_GetRangeStatus(ToFAddress, &RangeStatus);
		status = VL53L1X_GetDistance(ToFAddress, &Distance);
		FlashLED4(1);
		
		debugArray[iteration] = Distance;
	
		theta = iteration*fac;
		
		//If RangeStatus is 0, the x y z coordinates are calculated and displayed 
		if (RangeStatus == 0){
			status = VL53L1X_ClearInterrupt(ToFAddress); /* clear interrupt has to be called to enable next interrupt*/
			sprintf(printf_buffer,"%d %f	%f\r\n", xValue, Distance*sin(theta), Distance*cos(theta)); //displace cartesian coordinates 
			UART_printf(printf_buffer);
		}
		//Turns off onboard LED
		GPIO_PORTF_DATA_R = 0b00000000; 
		SysTick_Wait10ms(t);
	}
	VL53L1X_StopRanging(ToFAddress);				
}
	
int main(void) {
	//Initialize ports and background processes
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortL_Init();
	PortM_Init();
	PortF_Init();
	
	//Displays these initial messages on RealTerm
	UART_printf("Matthew Badal-Badalian\r\n");
	sprintf(printf_buffer,"Ready to take room measurements\n");
	UART_printf(printf_buffer);
	
	//Initializes the x coordinate value
	int xValue=0;
	
	//Ensures that the user can use the device an unlimited amount of times
	while(1){
		if((GPIO_PORTM_DATA_R & 0b00000001) == 0b00000000){ //When the button is pressed, the statements below are executed
			dataCollection(xValue);  
			xValue -= 200; //Reduces the x distance by 200mm
		}
		
		//Nothing happens while the button is not pressed
		else{ 
			GPIO_PORTL_DATA_R=0b00001000;
			GPIO_PORTF_DATA_R = 0b000000000;
	  }
	}	
}











