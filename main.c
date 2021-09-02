/* 	
	Course		: UVIC MECHATRONICS 458
	Milestone	: 5 
	Title		: Final Project Demo

	Name 1:	Clayton, Moxley		Student ID:V00876017
	Name 2:	Braidon, Joe		Student ID:V00822287
*/

/**************************************************************************************
***************************** HEADER FILE INCLUDES ************************************
***************************************************************************************/

#include <stdlib.h>			//include standard libraries
#include <avr/io.h>			//include I/O functionality
#include <avr/interrupt.h>	//include interrupt functionality

/**************************************************************************************
****************************** PREPROCESSOR MACROS ************************************
***************************************************************************************/

//Motor Constants
#define BRAKE 0				
#define CW 1					
#define CCW 2				
#define STEP_90 50			//steps corresponding to 90 degree stepper motor rotation
#define STEP_180 100		//steps corresponding to 90 degree stepper motor rotation
#define M_DELAY 20			//20ms delay for stepper motor and contact bounce (mTimer)

//Object Type Identifiers
#define WHITE 1
#define BLACK 2
#define STEEL 3 
#define ALUMINUM 4

//ADC Object Reflective Index Limits
#define MIN_AL 0			
#define MAX_AL 350
#define MIN_ST 351
#define MAX_ST 699
#define MIN_WT 700
#define MAX_WT 940
#define MIN_BK 941
#define MAX_BK 1023

/**************************************************************************************
************************ DECLARATION OF FIFO LINK STRUCTURE ***************************
***************************************************************************************/

typedef struct link{		//definition of link structure
	
	unsigned int obj_type;	//1 = White, 2 = Black, 3 = Steel, 4 = Aluminum
	struct link* next;

} link;

link *head;
link *tail;
link *newLink;
link *rtnLink;

/**************************************************************************************
*************************** DECLARATION OF FUNCTIONS **********************************
***************************************************************************************/

//Functions for Setup of I/O, Interrupts, ADC, PWM, and Timers

void setup_IO();		
void setup_INT();		
void setup_ADC();		
void pwm();				
void mTimer(int t);

//FIFO Linked List Functions

void setup(link **h, link **t); 
void initLink(link **newLink);
void enqueue(link **h, link **t, link **nL);
void dequeue(link **h, link **t, link **deQueuedLink);
int size(link **h, link **t);

//Object Processing, Sorting and Display Functions

void process_obj();
void sort_obj();
void display();

//Stepper Motor Functions

void stepper_INIT();
void stepper_RUN(int dir, int step);


/**************************************************************************************
************************ DECLARATION OF GLOBAL VARIABLES ******************************
***************************************************************************************/

volatile char STATE;	//variable for state machine

/******************************** MOTOR VARIABLES *************************************/

volatile int mStep = 0;		//variable for tracking stepper motor signal position

int mS_signal[] = {0x36, 0x2E, 0x2D, 0x35};	//Array containing output signals for the stepper motor	
											//(0b110110, 0b101110,0b101101,0b110101)
											
int mDC_signal[] = {0x00, 0x02};			//Array containing output signals for the DC motor (brake H, CW)										

int accel_90 [] =	{20,17,15,14,13,11,11,10,9,9,	//Array containing 90 degree acceleration delays for mTimer
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					9, 9,10,11,11,13,14,15,17,20};


int accel_180 [] =	{20,19,17,16,15,15,14,13,13,12,	//Array containing 1800 degree acceleration delays for mTimer
					11,11,11,10,10,9, 9, 9, 9, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
					8, 9, 9, 9, 9,10,10,11,11,11,
					12,13,13,14,15,15,16,17,19,20};


/******************************** ADC VARIABLES ***************************************/

volatile unsigned int ADC_value;	//used for comparing ADC results
volatile unsigned int ADC_result;	//used for storing current ADC result
volatile unsigned int ADC_final;	//used for storing lowest ADC result

/************************* SORTING AND PROCESSING VARIABLES ***************************/

volatile unsigned int object;		//current object type (processing/sorting)
volatile unsigned int bin;			//current bin position (sorting)

//Variables for keeping count of bin totals
volatile unsigned int bin_WT;
volatile unsigned int bin_BK;
volatile unsigned int bin_AL;
volatile unsigned int bin_ST;

volatile int flag_HE;		//flag for HE sensor
volatile int flag_pause;	//flag for indicating system is paused

/**************************************************************************************
******************************** MAIN ROUTINE *****************************************
***************************************************************************************/				 

int main(){
	
	STATE = 0;
	
	cli();  // disable all interrupts 

	setup_IO();		//setup IO registers
	setup_INT();	//setup external interrupts
	setup_ADC();	//setup ADC registers 
	pwm();			//generate 50% duty cycle PWM waveform @488 Hz
	
	setup(&head,&tail);	//initialize head and tail to NULL
	newLink = NULL;
	rtnLink = NULL;
		
	sei();	//enable all interrupts	
	
	stepper_INIT();	//initialize stepper motor position to black
	
	PORTC = 0x00;	//clear output to LEDs
	STATE = 3;		//go to start stage (DC motor forward)
	 
	goto POLLING_STAGE;

	POLLING_STAGE:
	
		switch(STATE){
			case (0) :
				goto POLLING_STAGE;	
				break;
			case (1) :
				goto REFLECTIVE_STAGE;
				break;
			case (2) :
				goto SORTING_STAGE;
				break;
			case (3) :
				goto START_STAGE;
				break;
			case (4) :
				goto PAUSE_STAGE;
				break;
			case (5) :
				goto END;
			default :
				goto POLLING_STAGE;
		}
	
	REFLECTIVE_STAGE:
		process_obj();	//process object based on ADC result
		STATE = 0;		
		goto POLLING_STAGE;

	SORTING_STAGE:
		PORTB = mDC_signal[BRAKE];	//stop DC motor
		sort_obj();		//sort object into specified bin
		free(rtnLink);	//remove sorted object from FIFO queue
		STATE = 3;		//drive DC motor forward
		goto POLLING_STAGE;
		
	START_STAGE:
		PORTB = mDC_signal[CW];	//start DC motor in clockwise direction
		STATE = 0;		
		goto POLLING_STAGE;
		
	PAUSE_STAGE:
		PORTB = mDC_signal[BRAKE];	//stop DC motor
		display();		//display objects sorted in bins and on belt
		STATE = 3;		//start DC motor
		goto POLLING_STAGE;
	
	END:
	return(0);
}//end main


/**************************************************************************************
*************************** INTERRUPT SERVICE ROUTINES ********************************
***************************************************************************************/

/* DESC: Pause Button: Interrupt Service Routine 
*/
ISR(INT0_vect){ 
	if(flag_pause == 0){	//check if system is currently paused
		
		mTimer(M_DELAY);	//delay for contact bounce
		flag_pause = 1;		//set pause flag to indicate system is paused
		STATE = 4;			//go to pause stage
	}
	else{					//if system is currently paused
		mTimer(M_DELAY);	//delay for contact bounce
		flag_pause = 0;		//clear pause flag to indicate system not paused
	}	
}

/* DESC: Reflective Sensor OR: Interrupt Service Routine 
*/
ISR(INT1_vect){
	ADC_result = 0xFFFF;	//set ADC to highest value, ADC will search for the minimum 
	ADCSRA |= _BV(ADSC);	//start ADC conversion
	initLink(&newLink);		//initialize a new link
	enqueue(&head, &tail, &newLink); //add new link to FIFO queue
}

/* DESC: End of Travel Sensor EX: Interrupt Service Routine 
*/
ISR(INT2_vect){
	STATE = 2;	//begin sorting object
}

/* DESC: Hall Effect Sensor HE: Interrupt Service Routine 
*/
ISR(INT3_vect){ 
	flag_HE = 1;	//set flag after bin position is initialized to black
}

/* DESC: ADC: Interrupt Service Routine (triggered when ADC conversion is complete) 
*/
ISR(ADC_vect){
	if (ADC < ADC_result){	//if new ADC value is less than current
		ADC_result = ADC;	//assign new ADC value to ADC result 
	}
	if ((PIND & 0x02) == 0x02) {	//if object in front of OR sensor
		ADCSRA |= _BV(ADSC);	//start ADC conversion again
	}
	else{
		ADC_final = ADC_result;	//if object is past OR sensor, store final result
		STATE = 1;				//go to reflective stage (object classification)
	}
}

/* DESC: Will flash 10101010 on LEDS until system is reset 
*/
ISR(BADISR_vect){
	while(1) {	
		PORTC = 0xAA;	//display 10101010 to LED bank
		mTimer(500);	//wait 0.5 s
		PORTC = 0x00;	//display 00000000 to LED bank
		mTimer(500);	//wait 0.5 s
	}
}


/**************************************************************************************
******** FUNCTION DEFINITIONS FOR I/O, INTERRUPTS, ADC, TIMER, AND PWM SETUP **********
***************************************************************************************/

/* DESC: Setup of I/O Ports 
*/
void setup_IO() {
	DDRA = 0xFF;	//Set PORTA to output for stepper motor
	DDRB = 0xFF;	//Set PORTB to output for motor (PINB7 is PWM)
	DDRC = 0xFF;	//Set PORTC to output for LEDs
	DDRD = 0xF0;	//Set PINS D0:D3 to input for external interrupts INT0:INT3
	DDRF = 0xFD;	//Set PINF1 to input for ADC (PINF1 is ADC1)
	return;
}

/* DESC: Setup of External Interrupt Registers 
*/
void setup_INT() {
	EIMSK |= _BV(INT0);		//INT0 enable for pause button interrupt
	EICRA |= _BV(ISC01); 	//Falling edge interrupt enable triggers on pause button press
	
	EIMSK |= _BV(INT1);					//INT1 enable for optical sensor interrupt OR
	EICRA |= _BV(ISC11) | _BV(ISC10);	//rising edge interrupt enable triggers when object reaches sensor
	
	EIMSK |= _BV(INT2);		//INT2 enable for end of travel sensor EX
	EICRA |= _BV(ISC21);	//falling edge interrupt enable triggers when object reaches sensor
	
	EIMSK |= _BV(INT3);		//INT3 enable for hall effect sensor HE
	EICRA |= _BV(ISC31);	//falling edge interrupt enable triggers when sorting bin has calibrated position 
	
	return;
}

/* DESC: Setup of ADC Registers 
*/
void setup_ADC() {
	ADCSRA |= _BV(ADEN);				//enable ADC
	ADCSRA |= _BV(ADIE);				//enable interrupt of ADC
	ADMUX  |= _BV(REFS0) | _BV(MUX0);	//Sets voltage reference for ADC and selects channel ADC1
	return;
}

/* DESC: Generates a 50% duty cycle PWM waveform at 488 Hz using 8-bit timer 
*/
void pwm(){
	
	TCCR0B |= _BV(CS01);	//Sets Timer/Counter0 Control Register B to logic 1.
							//This sets the clock 0 prescaler value to 1/8
	
	TCCR0A |= _BV(WGM00)|_BV(WGM01);	//Sets bit 0 and 1 (WGM00 & WGM01) of Timer/Counter0 Control Register A to logic 1.
										//WGM02 bit of Timer/Counter0 Control Register B is initialized to logic 0.
										//This sets Waveform Generation mode to fast PWM (TOP = 0xFF, TOV on MAX)
	
	TCCR0A |= _BV(COM0A1);				//Sets bit 8 (COM0A1) of Timer/Counter0 Control Register A to logic 1.
										//COM0A0 bit is initialized to logic 0.
										//This sets the compare match mode to clear OC0A on compare match and set OC0A at TOP
	
	
	OCR0A = 0x50;	//Set the Output Compare Register 0A value to 127D
					//corresponding to a 50% duty cycle
	
	return;
}

/* DESC: Provides a delay in ms based on the value of passed integer variable count
   INPUT: The integer variable count, corresponding to desired delay time in ms 
*/
void mTimer(int count){	
	int i = 0;				//Integer variable used to count number of loop cycles
	
	TCCR1B |=_BV(CS10);		/*Set Clock Select bit CS10 in Timer/Counter1 
							Control Register B to logic 1 (no prescaling)*/
	TCCR1B |= _BV(WGM12);	//Sets bit 3 (WGM12) of Timer/Counter1 Control Register B to logic 1.
							//Set Waveform Generation mode to Clear Time on Compare Math mode (CTC Mode) only
	
	OCR1A = 0x03e8;			//Set Output Compare Register (TOP value) to 1000, corresponding to 1ms
	
	TCNT1 = 0x0000;			//Set Timer/Counter 1 initial value to 0;
	
	TIFR1 |= _BV(OCF1A);	//Set bit 2 (OCF1A) of Timer/Counter Interrupt Flag Register to logic 1
							//Clears the output compare match flag and begins timer
	
	while(i < count){		//While loop runs until desired delay in ms is reached
		
		if((TIFR1 & 0x02) == 0x02) {	//Check if output compare match flag has been set (has reached TOP value)
			
			TIFR1 |= _BV(OCF1A);	//Set bit 2 (OCF1A) of Timer/Counter Interrupt Flag Register to logic 1
									//in order to clear the output compare match flag
			
			i++;				//Increment loop counter
		}
	}
	return;
}

/**************************************************************************************
********************* MOTOR AND SORTING FUNCTION DEFINITIONS***************************
***************************************************************************************/

/* DESC: Initialization of Stepper Motor Position to Black Bin  
*/
void stepper_INIT(){	
	while(((PIND & 0x08) == 0x08) && (flag_HE == 0)){ //while HE sensor has not been triggered
		
		mStep++;					//increase motor step signal position
		
		if(mStep > 3){mStep = 0;}	//if current motor step is outside array bounds, set to first position in signal array
		
		PORTA = mS_signal[mStep];	//output motor signal at current motor step to PORTA
		
		mTimer(M_DELAY);			//delay 20ms before next motor step						
	}
	bin = BLACK;					//set current bin to black
	return;	
}


/* DESC: Rotates the stepper motor a number of steps in a specified direction
		 based on the value of the passed integer variables dir and step.
   INPUT: The integer variables dir and step, corresponding to the motor direction and number of steps 
*/
void stepper_RUN(int dir, int step) { 
	
	int i = 0;	//counter for number of steps taken
	
	if (dir == CW) {	//check if direction is clockwise
		
		while(i < step){	//while steps taken less than specified number of steps
			mStep++;		//increase motor step signal position
			
			if(mStep > 3){mStep = 0;}	//If current motor step is outside array bounds, set to first signal position
			
			PORTA = mS_signal[mStep];	//output motor signal at current motor step to PORTA
			
			if (step == STEP_90){		//if 90 degree rotation needed
				mTimer(accel_90[i]);	//mTimer delay using 90 degree acceleration values
			}
			else{
				mTimer(accel_180[i]);	//mTimer delay using 180 degree acceleration values
			}
			i++;	//increase step count
		}
	}
	else if (dir == CCW) {		//check if direction is counter clockwise
		while(i < step){		//while steps taken less than specified number of steps
			mStep--;			//decrease motor step signal position
		
			if(mStep < 0){mStep = 3;}	//If current motor step is outside array bounds (negative) set to last position in signal array
		
			PORTA = mS_signal[mStep];		//Output motor signal at current motor step to PORT C
		
			if (step == STEP_90){		//if 90 degree rotation needed
				mTimer(accel_90[i]);	//mTimer delay using 90 degree acceleration values
			}else{
				mTimer(accel_180[i]);	//mTimer delay using 180 degree acceleration values
			}
		i++;	//increase step count
		}
	}
	return;	
}

/* DESC: Compares ADC value to material reflectivity index limits to classify object type 
*/
void process_obj(){
	if(ADC_final <= MAX_AL){			//check if final ADC result is within aluminum limits
		tail->obj_type = ALUMINUM;		//set FIFO link object type to aluminum
	}
	if((ADC_final >= MIN_ST)&&(ADC_final <= MAX_ST)){	//check if final ADC result is between steel limits
		tail->obj_type = STEEL;							//set FIFO link object type to steel
	}
	if((ADC_final >= MIN_WT)&&(ADC_final <= MAX_WT)){	//check if final ADC result is between white limits
		tail->obj_type = WHITE;							//set FIFO link object type to white
	}
	if((ADC_final >= MIN_BK)&&(ADC_final <= MAX_BK)){	//check if final ADC result is between black limits
		tail->obj_type = BLACK;							//set FIFO link object type to black
	}
	return;
}

/* DESC: Obtains object type from FIFO link and compares to current bin position. Stepper motor
		 is rotated to reach corresponding bin position and bin count is incremented 
*/
void sort_obj(){
	
	dequeue(&head, &tail, &rtnLink);	//dequeue first element from FIFO queue
	object = rtnLink->obj_type;			//get current object type from FIFO link
	
	if (object == ALUMINUM) {			//check if object type is aluminum
		if (bin == STEEL) {				
			stepper_RUN(CCW,STEP_180);
		}
		if (bin == WHITE) {
			stepper_RUN(CCW,STEP_90);
		}
		if (bin == BLACK) {
			stepper_RUN(CW,STEP_90);
		}
		bin = ALUMINUM;					//set current bin to aluminum
		bin_AL++;						//increment aluminum bin count
	}
	if (object == STEEL) {				//check if object type is steel
		if (bin == ALUMINUM) {
			stepper_RUN(CCW,STEP_180);
		}
		if (bin == WHITE) {
			stepper_RUN(CW,STEP_90);
		}
		if (bin == BLACK) {
			stepper_RUN(CCW,STEP_90);
		}
		bin = STEEL;					//set current bin to steel
		bin_ST++;						//increment steel bin count
	}
	if (object == WHITE) {				//check if object type is white
		if (bin == BLACK) {
			stepper_RUN(CCW,STEP_180);	
		}
		if (bin == ALUMINUM) {
			stepper_RUN(CW,STEP_90);
		}
		if (bin == STEEL) {
			stepper_RUN(CCW,STEP_90);
		}
		bin = WHITE;					//set current bin to white
		bin_WT++;						//increment white bin count
	}
	if (object == BLACK) {				//check if object type is black
		if (bin == WHITE) {
			stepper_RUN(CCW,STEP_180);
		}
		if (bin == ALUMINUM) {
			stepper_RUN(CCW,STEP_90);
		}
		if (bin == STEEL) {
			stepper_RUN(CW,STEP_90);
		}
		bin = BLACK;					//set current bin to black
		bin_BK++;						//increment black bin count
	}

	return;
}

/* DESC: When system is paused, the number of sorted objects of each type and the
		 remaining number of items on the belt is displayed to the LED bank with a
		 1s delay time
*/
void display(){
	unsigned int lDelay = 1000;		//used to provide 1s delay between counts
		
	while(flag_pause == 1){		//while system is paused, output bin and belt counts
			
		PORTC = 0x10 + bin_BK;	//black bin count
		mTimer(lDelay);
		PORTC = 0x20 + bin_ST;	//steel bin count			
		mTimer(lDelay);
		PORTC = 0x40 + bin_WT;	//white bin count
		mTimer(lDelay);
		PORTC = 0x80 + bin_AL;	//aluminum bin count
		mTimer(lDelay);			
		PORTC = 0xF0 + size(&head,&tail);	//number of items remianing on belt 
		mTimer(lDelay);
		}
	return;
}


/**************************************************************************************
******************** FIFO LINKED LIST FUNCTION DEFINITIONS ****************************
***************************************************************************************/

/*	DESC: Initializes the head and tail to 'NULL'
	INPUT: the head and tail pointers by reference 
*/
void setup(link **h,link **t) {
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}

/*	DESC: Initializes a new link 
*/
void initLink(link **newLink) {
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}

/*	DESC: Adds the new link to the end of the FIFO queue.
	INPUT: The head, tail, and newLink pointers by reference 
*/
void enqueue(link **h, link **t, link **nL){
	if (*t != NULL) {		//if not an empty queue
		(*t)->next = *nL;
		*t = *nL;			
	}
	else {					//if an empty queue
		*h = *nL;			
		*t = *nL;
	}
	return;
}

/*	DESC: Removes link from the head of the FIFO queue and assigns to dequeued link.
	INPUT: The head, tail, and deQueuedLink pointers by reference 
*/
void dequeue(link **h, link **t, link **deQueuedLink){
	*deQueuedLink = *h;		
	if (*h != NULL) {		//check if an empty queue
		*h = (*h)->next;
	}
	
	if(*h == NULL) {
	*t = NULL;
	}
	return;
}

/*	DESC: Obtains the size of the FIFO queue (number of links)
	INPUT: The head and tail pointers by reference
	RETURNS: An integer value of the number of links in the queue	 
*/
int size(link **h, link **t){
	link *temp;			//will store current link while moving through queue
	int numElements;
	numElements = 0;
	temp = *h;			//point to the first item in the list
	
	while(temp != NULL){
		numElements++;
		temp = temp->next;
	}			
	return(numElements);
}