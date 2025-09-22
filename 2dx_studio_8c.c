/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.


            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
						
						Last Update: March 03, 2022
						Updated by Hafez Mousavi
						__ the dev address can now be written in its original format. 
								Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
								
						Modified March 16, 2023 
						by T. Doyle
							- minor modifications made to make compatible with new Keil IDE

*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
//Global Variable for Different Functions


//I2C init Function
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}



//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ) 
}



volatile uint8_t blinking_rate = 64; 
volatile uint8_t motorAngle = 0;
volatile uint8_t set_flag_data = 0; 
volatile int stepMode = 64;
volatile int motorRunning = 0;
volatile int motorDirection = 0;
volatile uint32_t stepCount = 0;
volatile int returnToHomeCounter = 0;
volatile int returnToHomeFlag = 0;
volatile int data_flag = 0;
volatile uint8_t data_receive_flag = 1;

//Port N Init Function For Onboard LEDs PN1 as LED1 and PN0 as LED2
void PortN_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;				// activate clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};	// allow time for clock to stabilize
	GPIO_PORTN_DIR_R |= 0x03;        								// make PN0 and PN1 out (PN0-1 built-in LED1 and LED0)
  GPIO_PORTN_AFSEL_R &= ~0x03;     								// disable alt funct on PN0 and PN1
  GPIO_PORTN_DEN_R |= 0x03;        								// enable digital I/O on PN0 and PN1
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x03;     								// disable analog functionality on PN0 and PN1		
	
	GPIO_PORTN_DATA_R ^= 0b00000011; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTN_DATA_R ^= 0b00000011;	
	return;
}

//Port F Init Function For Onboard LEDs PF4 as LED3 and PF0 as LED4
void PortF_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				// activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0x11;        								// make PF0 and PF4 out (built-in LED2 and LED 3)
  GPIO_PORTF_AFSEL_R &= ~0x11;     								// disable alt funct on PF0 and PF4
  GPIO_PORTF_DEN_R |= 0x11;        								// enable digital I/O on PF0 and PF4
																									
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTF_AMSEL_R &= ~0x011;     								// disable analog functionality on PF0 and PF4	
	
	GPIO_PORTF_DATA_R ^= 0b00010001; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTF_DATA_R ^= 0b00010001;	
	return;
}




//Project Deliverable 1 External Button Pins Init  PM0-1
void PortM_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R &= ~0x03;        								// make PM0 and PM1 as inputs 
  GPIO_PORTM_AFSEL_R &= ~0x03;     								// disable alt funct on PM0-1
  GPIO_PORTM_DEN_R |= 0x03;        								// enable digital I/O on PM0-1

  GPIO_PORTM_AMSEL_R &= ~0x03;     								// disable analog functionality on PM0		
	
	GPIO_PORTM_DATA_R ^= 0x03; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTM_DATA_R ^= 0x03;	
	return;
}



//Onboard Button Init Function PJ0 for Start/Stop PJ1 for Data Receiveng Start/Stop
void PortJ_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;				// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
	GPIO_PORTJ_DIR_R &= ~0x03;        								// make PJ out 0001 1111
  GPIO_PORTJ_AFSEL_R &= ~0x03;     								// disable alt funct on PM0 
  GPIO_PORTJ_DEN_R |= 0x03;        								// enable digital I/O on PM0
	GPIO_PORTJ_PUR_R |= 0x03;

  GPIO_PORTJ_AMSEL_R &= ~0x03;     								// disable analog functionality on PM0		
	
	GPIO_PORTJ_DATA_R ^= 0x03; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTJ_DATA_R ^= 0x03;	
	return;
}

void PortL_Init(void){
	//Use PortN onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0x1F;        								// make PM0 out 0001 1111
  GPIO_PORTL_AFSEL_R &= ~0x1F;     								// disable alt funct on PM0 
  GPIO_PORTL_DEN_R |= 0x1F;        								// enable digital I/O on PM0

  GPIO_PORTL_AMSEL_R &= ~0x1F;     								// disable analog functionality on PM0		
	
	GPIO_PORTL_DATA_R ^= 0x1F; 								//hello world!
	SysTick_Wait10ms(10);														//.1s delay
	GPIO_PORTL_DATA_R ^= 0x1F;	
	return;
}

		

void DutyCycle_Percent(int duty){
	float percent;
	int time =0;
	
	
		percent = (duty*1000)/255;
		int intPercent = (int)percent;
		GPIO_PORTL_DATA_R ^= 0b00010000;//Toggle_Bit();  // Turn ON
		SysTick_Wait10ms(intPercent);  // ON time
						
		GPIO_PORTL_DATA_R ^= 0b00010000;//Toggle_Bit();  // Turn OFF
		SysTick_Wait10ms((1000 - intPercent));  // OFF time
	

}

void rotateMotor(int direction){
	static int step = 0;
	
	if(direction== 1){
	//blue pl0, pink pl1, yellow pl2, orange pl3, red 5V
	//step 1 blue 1, pink 1, pm0,pm1
	//step 2 pink 1 yellow 1 pm1,pm2
	//step 3 yellow 1, orange 1 pm2,pm3
	//step 4 blue 1 orange 1 pm0,pm3
		//rotateClockWiseInner();
		switch (step) {
            case 0:
                GPIO_PORTL_DATA_R = 0b00000011; // Step 1
                break;
            case 1:
                GPIO_PORTL_DATA_R = 0b00000110; // Step 2
                break;
            case 2:
                GPIO_PORTL_DATA_R = 0b00001100; // Step 3
                break;
            case 3:
                GPIO_PORTL_DATA_R = 0b00001001; // Step 4
                break;
	}}
	if(direction==0){
		//rotateCounterClockWiseInner();
		switch (step) {
            case 0:
                GPIO_PORTL_DATA_R = 0b00001001; // Step 1
                break;
            case 1:
                GPIO_PORTL_DATA_R = 0b00001100; // Step 2
                break;
            case 2:
                GPIO_PORTL_DATA_R = 0b00000110; // Step 3
                break;
            case 3:
                GPIO_PORTL_DATA_R = 0b00000011; // Step 4
                break;
        }
	}
	step = (step + 1) % 4; // Move to the next step
	
	stepCount++;

	
	if(stepCount%stepMode==0){
			data_flag =1;
		/*if(stepMode==64){
			GPIO_PORTF_DATA_R |= 0x10;
		}
		else{
			GPIO_PORTF_DATA_R &= ~0x10;
		}*/
	} else data_flag = 0;
	
	if(returnToHomeCounter%2048==0){
		returnToHomeCounter =0;
	}
	returnToHomeCounter++;
	if(returnToHomeFlag){
		returnToHomeCounter--;
	}
	
  SysTick_Wait10ms(100); // Small delay between steps
}


void returnToHome(int direction){
	while (returnToHomeCounter > 0) {  // Use > 0 to avoid potential underflow
        rotateMotor(!direction);       // Rotate in the opposite direction
        returnToHomeCounter--;        // Decrement the counter
        SysTick_Wait10ms(10);         // Small delay to allow motor movement
    }
	
	GPIO_PORTN_DATA_R &= ~0b00000010; // Turn off LED0 (PN1)
	GPIO_PORTN_DATA_R &= ~0b00000001;
	GPIO_PORTF_DATA_R &= ~0b00010001;

}

/*int getRunFlag(void){
		uint32_t input_state0 = GPIO_PORTJ_DATA_R & 0x01; // Read PJ0 (Button 0)
    return (input_state0 == 0) ? 1 : 0; // Active low: pressed = 0, not pressed = 1
		SysTick_Wait10ms(20);
}*/

/*int getDirectionFlag(void) {
    uint32_t input_state1 = GPIO_PORTJ_DATA_R & 0x02; // Read PJ1 (Button 1)
    return (input_state1 == 0) ? 1 : 0; // Active low: pressed = 0, not pressed = 1
}*/
int getStepModeFlag(void) {
    uint32_t input_state1 = GPIO_PORTM_DATA_R & 0x02; // Read PM1 (Button 2) //left side
    return (input_state1 == 0) ? 0 : 1; // Active high: pressed = 1, not pressed = 0
}

int getHome(void) {
    uint32_t input_state1 = GPIO_PORTM_DATA_R & 0x01; // Read PM0 (Button 3) //right side
    return (input_state1 == 0) ? 0 : 1; // Active high: pressed = 1, not pressed = 0
}

int running(void){
		return (GPIO_PORTJ_DATA_R & 0x01) ? 0: 1; //pj0
}

int receiving(void){
		return (GPIO_PORTJ_DATA_R & 0x02) ? 0: 1; //pj1
}
void toggleStepMode(void) {
    if (stepMode == 64) {
        stepMode = 256;
    } 
		else {
        stepMode = 64;
    }
}

void toggleDirection(void) {
    if (motorDirection == 1) {
        motorDirection = 0;
    } 
		else {
        motorDirection = 1;
    }
}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortN_Init();																		// Initialize Port N GPIO
	PortM_Init();																		// Initialize Port M GPIO
	PortF_Init();
	PortJ_Init();
	PortL_Init();
	
	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
	UART_printf(printf_buffer);
	
	status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
	sprintf(printf_buffer,"(Model_ID)=0x%x, \r\n",byteData);
	UART_printf(printf_buffer);
	status = VL53L1_RdByte(dev, 0x0110, &byteData); //for model type (0xcc)
	sprintf(printf_buffer,"(Module_Type)=0x%x, \r\n",byteData);
	UART_printf(printf_buffer);
	status = VL53L1_RdWord(dev, 0x010F, &wordData); //for model type (0xcc)
	sprintf(printf_buffer,"(Module_Type)=0x%x, \r\n",wordData);
	UART_printf(printf_buffer);
	
	
/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);
	
	while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
		}
		FlashAllLEDs();
		UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
		
		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		/* 2 Initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev);
		Status_Check("SensorInit", status);
	
		
	while(1)
	{
		DutyCycle_Percent(128);
		//stepMode=64;
		/*if (getRunFlag()) {
            motorRunning = !motorRunning; // Toggle motor state
            SysTick_Wait10ms(200); // Debounce delay
        }
		
		if (getDirectionFlag()) {
            motorDirection = !motorDirection; // Toggle motor direction
            SysTick_Wait10ms(200); // Debounce delay
        }*/
		
		
		
		if(running()==1){
			SysTick_Wait10ms(100);
			if(running()==1){
				motorRunning = !motorRunning;
				GPIO_PORTF_DATA_R = (motorRunning ? 0x01 : 0x00); // updating PF0, which is D4
				while(running() == 1){}; // we are waiting for button release here. 
			}
		}
		
		if(receiving() == 1) //
		{
			SysTick_Wait10ms(100); // debounce delay
			if(receiving() == 1)			// double checking that the direction is still same
			{ 	    
				data_receive_flag = !data_receive_flag; // we are toggling the data coming in or not
				// GPIO_PORTN_DATA_R = (data_acquisition ? 0x01 : 0x00); // When motordirection is true (1), it means we are receiving data, otherwise no 
				while(receiving() == 1){} // waiting for the button to be released
			}
			
		}
		
		if (getStepModeFlag()) {
            toggleStepMode();
            SysTick_Wait10ms(200);   // Debounce delay
        }
		
		if (getHome()) {
            returnToHomeFlag = 1;        // Set return-to-home flag
            SysTick_Wait10ms(200);   // Debounce delay
        }

        // Control motor and LED based on motorRunning state
	
        if (motorRunning) {
					if(returnToHomeFlag){
						returnToHome(motorDirection);
						returnToHomeFlag = 0;
						motorRunning=0;
					}
					else{
							//GPIO_PORTN_DATA_R |= 0b00000010; // Turn on LED0 (PN1)
						if(motorDirection==1){
							GPIO_PORTN_DATA_R |= 0b00000001;
						}
						else{
							GPIO_PORTN_DATA_R &= ~0b00000001;
						}
						if(stepCount%2048==0){
							toggleDirection();
							
						}
							rotateMotor(motorDirection); // Rotate motor clockwise
							// 1 Wait for device ToF booted
							
							
							

							
							/* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
							//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
							//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
							//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
							status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
						
							// Get the Distance Measures 50 times
							//for(int i = 0; i < 50; i++) {
								
									//5 wait until the ToF sensor's data is ready
								//while (dataReady == 0){
									//status = VL53L1X_CheckForDataReady(dev, &dataReady);
										//	FlashLED3(1);
										//	VL53L1_WaitMs(dev, 5);
								//}
								// dataReady = 0;
								
								//7 read the data values from ToF sensor
								
								// print the resulted readings to UART
								
							//}
							if(motorDirection==1){
								if (data_flag==1 && data_receive_flag==1)
									{
										status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
										status = VL53L1X_GetDistance(dev, &Distance);					//7 The Measured Distance value, implimented this
										status = VL53L1X_GetSignalRate(dev, &SignalRate);
										status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
										status = VL53L1X_GetSpadNb(dev, &SpadNum); 
										//status = VL53L1_RdByte(dev, 0x010F, &byteData); //for model ID (0xEA)
										//status = VL53L1_RdByte(dev, 0x0110, &byteData); //for module type (0xCC)
										//status = VL53L1_RdWord(dev, 0x010F, &wordData); //for both model ID and type
										
										FlashLED1(1);
										FlashLED3(1);

										status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
										
										sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate,SpadNum);
										UART_printf(printf_buffer);
										SysTick_Wait10ms(50);
									}
								}
							
								VL53L1X_StopRanging(dev);
					}
        } 
				else {
            GPIO_PORTN_DATA_R &= ~0b00000010; // Turn off LED0 (PN1)
						GPIO_PORTN_DATA_R &= ~0b00000001;
						GPIO_PORTF_DATA_R &= ~0b00010001;
        }

        // Small delay to allow button state to be checked frequently
        SysTick_Wait10ms(10);
				
    }
		
	}


