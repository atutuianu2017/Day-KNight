/*
 Embedded Systems
 Final Project
 Alina Tutuianu & Paunteh'A Jamkhu
 */

#include <msp430.h> 
#define c 261   //frequency for specific notes
#define e 329
#define g 391
#define ALARM BIT3 //piezo buzzer
#define LIGHT BIT4  //led
#define LOCK BIT5   //solenoid door lock


int i = 0, j=0; //counters
int weight = 0, actual_weight = 10; //variables to store weight sensor reading
int ir_sensor1 = 0, actual_ir_sensor1=0; //variables to store ir1 sensor reading
int ir_sensor2 = 0, actual_ir_sensor2=0;    //variables to store ir2 sensor reading
int stop_write = 0, person_door = 0;    //flags
int ir1=0, ir2=0;           //flags
int nr_people=0;        //number of people that passed the ir beam toward the house
int ADCReading[3];      //array for ADC reading
unsigned char b[4];        //array for transmitting data via UART

//Function prototypes
void configureAdc(void);
void getAnalogValues();
void delay_ms(unsigned int ms);
void delay_us(unsigned int us);
void beep(unsigned int note, unsigned int duration);
void configureSerialCom();
void serialWriteCharacter(unsigned char d);
void serialWriteString(const char *str);
void serialWriteInt(unsigned int temp);
//unsigned char serialRead(); //not needed for this project, reading is done in interrupt

int main(void)
 {
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    P1OUT = 0;
    P2OUT = 0;
    P1DIR = 0;
    P1REN = 0;
    P2REN = 0;
    P2DIR = 0;
    P2DIR |=  (ALARM | LIGHT | LOCK); //Setting P2.3, 2.4, 2.5 as outputs


    if(CALBC1_1MHZ == 0xFF) //if calibration constant erased
    { while (1);}           //do not load, trap CPU
    DCOCTL = 0;             //select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;  //set range
    DCOCTL = CALDCO_1MHZ;   //set DCO step and modulation

    configureSerialCom();   //configure serial communication
    __delay_cycles(250);
    configureAdc();         //configure ADC conversation
     __delay_cycles(250);
    getAnalogValues();      //get first set of analog values
    __delay_cycles(250);
    _enable_interrupts();   //enable interrupts

    //reading actual values for future comparison
    actual_weight = weight;
    actual_ir_sensor1 = ir_sensor1;
    actual_ir_sensor2 = ir_sensor2;
    __delay_cycles(250);

    //forever loop
    for (;;)
    {
        getAnalogValues(); //read analog values

        //check P1.6 to see if contact switch is open
        if ((P1IN & 0x40) == 0x00) //contact switch open - window open
        {
            serialWriteString("Alert: window open\n");
            serialWriteString("Alarm ON\n");
            serialWriteString("\n");
            while ((P1IN & 0x40) == 0x00)   //turn alarm on
            {
                beep(c, 500);
                beep(e, 500);
                beep(g, 500);
                delay_ms(250);
            }
        }

        //Convert ir sensor input to digital 1 or 0
        if(ir_sensor1 <= actual_ir_sensor1*0.7) {ir1=1;}
        else if(ir_sensor1 >= actual_ir_sensor1*0.9) {ir1=0;}

        if(ir_sensor2 <= actual_ir_sensor2*0.7) {ir2=1;}
        else if(ir_sensor2 >= actual_ir_sensor2*0.9) {ir2=0;}

        //Check ir sensor values
        //Someone entered the property, turn alarm on
        if(ir1==1 && ir2==0)
            {
            nr_people++;
            beep(c, 500);
            beep(e, 500);
            beep(g, 500);
            delay_ms(250);
            serialWriteString("Someone entered your property\n");
            serialWriteString("Number of people still on your property:\n");
            serialWriteInt(nr_people);
            serialWriteString("\n");
            delay_ms(1000);
            }
        //someone left the property
        if(ir2==1 && ir1==0)//exit
        {
            nr_people--;
            serialWriteString("Someone left your property\n");
            serialWriteString("Number of people still on your property:\n");
            serialWriteInt(nr_people);
            serialWriteString("\n");
            delay_ms(1000);
        }

       //Convert weight sensor input to digital 1 or 0
       if (weight >= actual_weight * 1.80) {person_door = 1;}
       else if(weight <=actual_weight * 1.40) { person_door = 0;}

       //if weight sensed at the door, transmit an alert
       if (person_door == 1 && stop_write == 0)
        {
            serialWriteString("Alert: person at the door\n");
            serialWriteString("\n");
            stop_write = 1;//set flag in order to not repeat sending the string
            delay_ms(100);
        }
        if (person_door == 0)
        {
            stop_write = 0;//reset flag
        }
    }
    return 0;
}
//Function to configure ADC reading
void configureAdc(void)
{
    ADC10CTL1 = INCH_5 | CONSEQ_1;             // A5 + A4 + A3, single sequence
    ADC10CTL0 = ADC10SHT_2 | MSC | ADC10ON;
    while (ADC10CTL1 & BUSY);
    ADC10DTC1 = 0x03;                      // 3 conversions
    ADC10AE0 |= (BIT3 | BIT4 | BIT5);        // ADC10 option select
}

//Function to get analog values
void getAnalogValues()
{
    // set all analog values to zero
    i = 0;
    weight = 0;
    ir_sensor1 = 0;
    ir_sensor2 = 0;
    for (i = 1; i <= 5; i++) // read all three analog values 5 times each and average
    {
        ADC10CTL0 &= ~ENC;
        while (ADC10CTL1 & BUSY);                         //Wait while ADC is busy
        ADC10SA = (unsigned) &ADCReading; //RAM Address of ADC Data, must be reset every conversion
        ADC10CTL0 |= (ENC | ADC10SC);                     //Start ADC Conversion
        while (ADC10CTL1 & BUSY);                         //Wait while ADC is busy
        // sum  all 5 readings for the three variables
        weight += ADCReading[2];
        ir_sensor1 += ADCReading[1];
        ir_sensor2 += ADCReading[0];
    }
    // Average the 5 reading for the three variables
    weight = weight / 5;
    ir_sensor1 = ir_sensor1 / 5;
    ir_sensor2 = ir_sensor2 / 5;
}

//Function to delay miliseconds
void delay_ms(unsigned int ms)
{
    for(i=0; i<=ms; i++)
        __delay_cycles(500);
}

//Function to delay microseconds
void delay_us(unsigned int us)
{
    unsigned int i;
    for (i=0; i<=us/2; i++)
     __delay_cycles(1);
}

//Function to play a certain frequency on the piezzo buzzer
void beep(unsigned int note, unsigned int duration)
{
    int i;
    long delay = (long)(10000/note);
    long time = (long)(duration*100)/(delay*2);
    for(i=0; i<time;i++)
    {
        P2OUT |= ALARM;
        delay_us(delay);
        P2OUT &= ~ALARM;
        delay_us(delay);
    }
    delay_ms(20);
}

//Function to configure serial communication
void configureSerialCom()
{
    P1SEL= BIT1 + BIT2; //P1.1 = RXD P1.2=TXD
    P1SEL2= BIT1 +BIT2; // P1.1=RXD & P1.2=TXD
    UCA0CTL1|= UCSSEL_2; // SMCLK
    UCA0BR0=104; // BAUDRATE AT 1 MHz 9600
    UCA0BR1=0;//1MHz 9600
    UCA0MCTL= UCBRS0; // MODULATION UCBRSx=1
    UCA0CTL1&=~UCSWRST; // ** INITIALIZE USCI STATE MACHINE
    IE2|= UCA0RXIE; // ENABLE VSCI_A0 RX INTERRUPT
}

//Function for transmiting a single character
void serialWriteCharacter(unsigned char d)
{
    while(!(IFG2 & UCA0TXIFG));  // USCI_A0 TX buffer ready ?
    UCA0TXBUF=d; // TX
}

//Function for transmiting a string of characters
void serialWriteString(const char *str)
{
    while(*str)
        serialWriteCharacter(*str++);
}

//Function to transmit an integer
void serialWriteInt(unsigned int temp)
{
    for(i=0;i<4;i++)
    {
        b[i]=temp%10;
        temp=temp/10;
    }

    for( j=3;j>=0;j--)
    {
        serialWriteCharacter(b[j] + 48);
    }

    serialWriteCharacter(' ');
    serialWriteCharacter('\n');
}

//Function for serial reading. Not needed for this project since we are using the interrupt vector
/*unsigned char serialRead()
{
    //while(!(IFG2 & UCA0RXIFG));   //USCI_A0 RX buffer ready ?
    return UCA0RXBUF;

}*/

//interrupt for ADC reading
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    __bic_SR_register_on_exit(CPUOFF);
}

//interrupt for receving data
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if (UCA0RXBUF == '0') //if character 0 received
    {
        P2OUT |= LOCK;  //open the lock for 8 seconds, then close
        delay_ms(8000);
        P2OUT &= ~LOCK;
    }
    else if (UCA0RXBUF == '1') //if character 1 received
    {
        beep(c, 500);   //turn alarm on
        beep(e, 500);
        beep(g, 500);
        delay_ms(250);
    }
    else if (UCA0RXBUF == '2')  //if character 2 received
    {
        P2OUT ^= LIGHT;    //toggle light
    }
}
