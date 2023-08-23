#include "msp.h"
#include "CortexM.h"
#include "TimerA0.h"

/****************************************
 * CLOCK Init
 ***************************************/
uint32_t ClockFrequency = 3000000; // cycles/second

int32_t Prewait = 0;                   // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0;                   // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0;                  // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0;             // loops before the crystal stabilizes (expect small)


void Clock_Init48MHz(void){
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x0C;
  PJ->SEL1 &= ~0x0C;                    // configure built-in 48 MHz crystal for HFXT operation
  CS->KEY = 0x695A;                     // unlock CS module for register access
  CS->CTL2 = (CS->CTL2&~0x00700000) |   // clear HFXTFREQ bit field
           0x00600000 |                 // configure for 48 MHz external crystal
           0x00010000 |                 // HFXT oscillator drive selection for crystals >4 MHz
           0x01000000;                  // enable HFXT
  CS->CTL2 &= ~0x02000000;              // disable high-frequency crystal bypass
  // wait for the HFXT clock to stabilize
  while(CS->IFG&0x00000002){
    CS->CLRIFG = 0x00000002;              // clear the HFXT oscillator interrupt flag
    Crystalstable = Crystalstable + 1;
    if(Crystalstable > 100000){
      return;                           // time out error
    }
  }
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
  FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL&~0x0000F000)|FLCTL_BANK0_RDCTL_WAIT_2;
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
  FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL&~0x0000F000)|FLCTL_BANK1_RDCTL_WAIT_2;
  CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
           0x00100000 |                 // configure for HSMCLK divider /2
           0x00000200 |                 // configure for ACLK sourced from REFOCLK
           0x00000050 |                 // configure for SMCLK and HSMCLK sourced from HFXTCLK
           0x00000005;                  // configure for MCLK sourced from HFXTCLK
  CS->KEY = 0;                          // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
}

// delay function
// which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
  //Code Composer Studio Code
void delay(unsigned long ulCount){
  __asm (  "pdloop:  subs    r0, #1\n"
      "    bne    pdloop\n");
}


// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n){
  while(n){
    delay(ClockFrequency/9162);   // 1 msec, tuned at 48 MHz
    n--;
  }
}


/****************************************
 *  SysTick Timer Init
 ***************************************/
void SysTick_Init2(void){
  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;           // enable SysTick with no interrupts
}
void SysTick_Wait(uint32_t delay){
  SysTick->LOAD = (delay - 1);// count down to zero
  SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
  Motor_Stop();
  while(( SysTick->CTRL&0x00010000) == 0){};

}
// Time delay using busy wait.
// assumes 48 MHz bus clock

void SysTick_Wait10ms(uint32_t delay){
    uint32_t i;
    for(i=0; i<delay; i++){
        SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
    }
}

void SysTick_Wait1ms(uint32_t delay){
    uint32_t i;
    for(i=0; i<delay; i++){
        SysTick_Wait(65536);  // wait 1ms (assumes 48 MHz clock)
    }
}
void SysTick_Wait1us(uint32_t delay){
    uint32_t i;
    for(i=0; i<delay; i++){
        SysTick_Wait(64);  // wait 1us (assumes 48 MHz clock)
    }
}

/***************************
 * TimerA0
 ***************************/
uint8_t justTurned; //variable remembering the previous turned direction
uint16_t ctr;
void FlashLED (void){
    Motor_Stop();
    if(ctr == 210){
        if(justTurned == 1){
            P6->OUT ^= BIT0; //Flash LEFT LED
        }
        else{
            P6->OUT ^= BIT2; //Flash RIGHT LED
        }
        ctr = 0;
    }else if(ctr < 210){
        ctr++;
    }
    else{
        ctr = 0;    // in case of ctr > 210
        return;
    }
}
void Guard (void){
    if(ctr == 700){
        Motor_Stop();
    }else if(ctr < 700){
        ctr++;
    }
    else{
        Motor_Stop();
        ctr = 0;    // in case of ctr > 700
        return;
    }
}


/*************************************
 *  Launchpad init
 ************************************/
void LaunchPad_Init(void){
  P6->SEL0 &= ~0x07;    // configure P6.0 (left light), 6.1 (break light), and 6.2 (right light)
  P6->SEL1 &= ~0x07;    // as GPIO
  P6->DIR |= 0x07;      // make P6.0, P6.1 and P6.2 out
  P6->DS |= 0x07;       // increase drive strength
  //P6->REN |= 0x07;      //Setup resistor (unused; physical resistors were used instead)
  P6->OUT &= ~0x07;     // all LEDs off

  P1->SEL0 &= ~BIT0;   // configure P1.0 (RED LED) as GPIO
  P1->SEL1 &= ~BIT0;
  P1->DIR |= BIT0;     // output P1.0
  P1->DS |= BIT0;      // increase drive strength
  P1->REN |= BIT0;     // internal resistor
  P1->OUT &= ~BIT0;    // LED off
}

/***************************
 * Motor init & motor related functions
 ***************************/

void Motor_Init(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P1.6 and P1.7 and make them outputs

    P1->SEL0 &= ~0xC0; // Set P1.6, and P1.7 as GPIO
    P1->SEL1 &= ~0xC0; // DIR
    P1->DIR |= 0xC0;   // Set P1.6 and P1.7 as output
    P1->REN |= 0xC0;   // Set resistor

    P2->SEL0 &= ~0xC0; // Set P2.6 and P2.7 as GPIO
    P2->SEL1 &= ~0xC0; // PWM
    P2->DIR |= 0xC0;   // Set P2.6 and P2.7 as output
    P2->REN |= 0xC0;  // Set resistor

    P3->SEL0 &= ~0xC0; // Set P3.6 and P3.7 as GPIO
    P3->SEL1 &= ~0xC0; // SLP
    P3->DIR |= 0xC0;   // Set P3.6 and P3.7 as output
    P3->OUT &= ~0xC0;  // Put drivers to sleep
    P3->REN |= 0xC0;  // Set resistor
}

void Motor_Stop(void){
// Stops both motors, puts driver to sleep
// Returns right away

    P1->OUT &= ~0xC0;
    P2->OUT &= ~0xC0;   // off
    P3->OUT &= ~0xC0;   // low current sleep mode
}
void Motor_Forward(uint16_t duty, uint32_t time){
// Drives both motors forward for time duration (units=1ms) at duty.
    uint32_t duration = time;  // record initial time
    uint16_t period = 12;  // 15ms period
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT &= ~0xC0; // PH(0 forward)
        P3->OUT |= 0xC0;  // wake up
        P2->OUT |= 0xC0;    // Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0xC0;
        P2->OUT &= ~0xC0;   // off
        P3->OUT &= ~0xC0;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));  // Prevent reverse overflow by verifying that the
                                              // current time is less than the initial time
    Motor_Stop();
    return;

}
void Motor_Backward(uint16_t duty, uint32_t time){
// Drives both motors forward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 12;  //period of 12ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT |= 0xC0; // PH(1 backward)
        P3->OUT |= 0xC0;  // wake up
        P2->OUT |= 0xC0;    // Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0xC0;
        P2->OUT &= ~0xC0;   // off
        P3->OUT &= ~0xC0;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));   // Prevent reverse overflow by verifying that the
                                               // current time is less than the initial time
    Motor_Stop();
    return;
}
void Motor_LeftForward(uint16_t duty, uint32_t time){
// Drives just the left motor forward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 12;  //period of 12ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT &= ~0x80; //PH(0 forward)
        P3->OUT |= 0x80;  //wake left, SLP right
        P2->OUT |= 0x80;    //Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0x80;
        P2->OUT &= ~0x80;   // off
        P3->OUT &= ~0x80;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));    // Prevent reverse overflow by verifying that the
                                                // current time is less than the initial time
    Motor_Stop();
    return;

}
void Motor_RightForward(uint16_t duty, uint32_t time){
// Drives just the right motor forward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 12;  //period of 12ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT &= ~0x40; //PH(0 forward)
        P3->OUT |= 0x40;  //wake right, SLP left
        P2->OUT |= 0x40;    //Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0x40;
        P2->OUT &= ~0x40;   // off
        P3->OUT &= ~0x40;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while(time > 0 && time < duration);    // Prevent reverse overflow by verifying that the
                                            // current time is less than the initial time
    Motor_Stop();
    return;
}
void Motor_LeftBackward(uint16_t duty, uint32_t time){
// Drives just the left motor backward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 12;  //period of 12ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT |= 0x80; //PH(1 backward)
        P3->OUT |= 0x80;  //wake left, SLP right
        P2->OUT |= 0x80;    //Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0x80;
        P2->OUT &= ~0x80;   // off
        P3->OUT &= ~0x80;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));
    Motor_Stop();
    return;

}

void Motor_RightBackward(uint16_t duty, uint32_t time){
// Drives just the right motor backward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 12;  //period of 12ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT |= 0x40; //PH(1 backward)
        P3->OUT |= 0x40;  //wake right, SLP left
        P2->OUT |= 0x40;    //Motor Active
        Clock_Delay1ms(duty);
        P1->OUT &= ~0x40;
        P2->OUT &= ~0x40;   // off
        P3->OUT &= ~0x40;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));    // Prevent reverse overflow by verifying that the
                                                // current time is less than the initial time
    Motor_Stop();
    return;
}

void Motor_LFRB(uint16_t duty, uint32_t time){
// Drives just the Left motor forward, right motor backward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 15;  //period of 15ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT |= 0x40; //PH(Left dir = 0 forward, right dir = 1 backward) 0100 0000
        P3->OUT |= 0xC0;  //wake both motors
        P2->OUT |= 0xC0;    //Motor Active
        Clock_Delay1ms(duty);

        P2->OUT &= ~0xC0;   // off
        P3->OUT &= ~0xC0;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));    // Prevent reverse overflow by verifying that the
                                                // current time is less than the initial time
    Motor_Stop();
    return;
}

void Motor_LBRF(uint16_t duty, uint32_t time){
// Drives just the Left motor backward, right motor forward for time duration (units=1ms) at duty.
    uint32_t duration = time;
    uint16_t period = 15;  //period of 15ms
    uint16_t duty1 = period - duty;
    do{
        P6->OUT &= ~BIT1; // Turn off break LED
        P1->OUT |= 0x80; //PH(Left dir = 1 backward, right dir = 0 forward) 1000 0000
        P3->OUT |= 0xC0;  //wake both motors
        P2->OUT |= 0xC0;    //Motor Active
        Clock_Delay1ms(duty);

        P2->OUT &= ~0xC0;   // off
        P3->OUT &= ~0xC0;   // low current sleep mode
        Clock_Delay1ms(duty1);
        time -= period;

    }while((time > 0) && (time < duration));    // Prevent reverse overflow by verifying that the
                                                // current time is less than the initial time
    Motor_Stop();
    return;
}

void TurnAround(void){
    Motor_Stop();
    P6->OUT |= BIT1;
    SysTick_Wait10ms(10);// 100ms
    Motor_Backward(1,100);// Go backwards, stay clearance from the obstacle
    //DisableInterrupts();
    Motor_LFRB(3,720);// Turn around
    //EnableInterrupts();
}

void Turn_Right(void){
    P6->OUT |= BIT2;
    DisableInterrupts();
    TimerA0_Init(&FlashLED);
    EnableInterrupts();
    uint8_t i;
    for (i = 0; i < 4; i++){//both was 5
        Motor_RightForward(1,170);
        Motor_LeftForward(1,170); // Straighten out
    }
    Motor_LFRB(3,400); //340
    Motor_LeftBackward(1,130);
    if(P6->OUT && BIT2){
        WaitForInterrupt();
    }
    TimerA0_Stop();
    //P6->OUT &= ~BIT2;
}

void Turn_Left(void){
    P6->OUT |= BIT0;
    DisableInterrupts();
    TimerA0_Init(&FlashLED);
    EnableInterrupts();
    uint8_t i;
    for (i = 0; i < 4; i++){
        Motor_LeftForward(1,170); // Straighten out
        Motor_RightForward(1,170);
    }
    Motor_LBRF(3,400);
    Motor_RightBackward(1,130);
    if(P6->OUT && BIT0){
        WaitForInterrupt();
    }
    TimerA0_Stop();
    //P6->OUT &= ~BIT0;
}


/***************************
 * Bumper sensors
 ***************************/
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt(void){
    P4->SEL0 &= ~0xFF;  // Configure P4.0, P4.2, P4.3, P4.5, P4.6, and P4.7 as GPIO
    P4->SEL1 &= ~0xFF;
    P4->DIR &= ~0xED;   // Input
    P4->REN |= 0xFF;    // Internal resistor
    P4->OUT |= 0xED;    // *pull-up
    P4->IES |= 0xED;    // Falling edge event
    P4->IFG &= ~0xED;   // Clear flags
    P4->IE |= 0xED;     // Arm interrupt
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00800000;  // Priority 4 Port 4
    NVIC->ISER[1] = 0x00000040;  // enable
}



// Read current state of 6 switches
uint8_t Bump_Read(void){
    // write this as part of Lab 7
    uint8_t value = P4->IN;
    return value|0x12;
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge

uint8_t CollisionData;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   SysTick_Wait10ms(10);
   TurnAround();
   CollisionData = 0xFF;


}
void BumpInt_Init(void(*task)(uint8_t)){
    SysTick_Wait10ms(1);
    uint8_t newData = Bump_Read();
    if(newData != CollisionData){
        (*task)(newData);
    }

}
void PORT4_IRQHandler(void){
    P4->IFG &= ~0xED;  // ack
    BumpInt_Init(&HandleCollision);

}

/***************************
 * Line sensors init & related functions
 ***************************/
void LineInt(void){
    P5->SEL0 &= ~BIT3;  //Set Pin3 as GPIO
    P5->SEL1 &= ~BIT3;
    P5->DS |= BIT3;
    P5->REN |= BIT3;
    P5->DIR |= BIT3;    //Set as Output
    P5->OUT &= ~BIT3; //Set initially low

    P7->SEL0 &= ~0xFF;  //Set all pins of P7 as GPIO
    P7->SEL1 &= ~0xFF;
    P7->REN |= 0xFF;
    P7->DIR &= ~0xFF; //Set as Input


}

uint8_t SensorData;
uint8_t Line_Read(void){

    /*Charging Capacitor*/
    P5->OUT |= BIT3;    //turn on IR Led
    P7->REN &= ~0xFF;   //Turn off resistor when charging
    P7->DIR |= 0xFF;   //P7 set as outputs, set them as high
    P7->OUT |=0xFF;
    SysTick_Wait1us(10);   //wait for 10us
    P7->DIR &= ~0xFF;    //change the direction to input
    SysTick_Wait1ms(1);    //wait for 1ms

    /*Reading Data*/
    SensorData = P7->IN;
    P7->REN |= 0xFF;    //Turn on resistor when detecting
    P5->OUT &= ~BIT3;   //Turn off IR LED
    return SensorData;
}





/***************************
 * main.c
 ***************************/
uint8_t AllReady; //flag to determine if the device lines up initially
uint8_t toTurnFlag; //flag to double-check if the device should turn in case of accidental overrun or faulty detection
int main(void){
    DisableInterrupts();
    /* Initializing device */
    LaunchPad_Init();
    Clock_Init48MHz();   // 48 MHz clock
    SysTick_Init2();


    CollisionData = 0xFF;
    justTurned = 0;// default value
    AllReady = 0;
    toTurnFlag = 0;
    ctr = 0;

    LineInt();
    Motor_Init();
    BumpInt();

    while(AllReady == 0){
        P1->OUT &= ~BIT0;        // LED signals device lined up
        if(Line_Read() == 0x18){
            P1->OUT |= BIT0;
            AllReady = 1;
        }
    }

    while(AllReady){
        Motor_Stop();
        //SysTick_Wait10ms(1);
        Motor_Stop();
        //DisableInterrupts();
        //TimerA0_Init(&Guard);// In case of some unforeseen condition making the robot turn around
        //EnableInterrupts();
        P1->OUT |= BIT0;
        if(Line_Read() == 0x18 || Line_Read() == 0x08 || Line_Read() == 0x1C || Line_Read() == 0x10 || Line_Read() == 0x19){
            //Motor_LeftForward(1,110);// 0001 1000 OR 0000 1000 OR 0001 1100 OR 0001 0000 OR 0001 1001 Straight
            //Motor_RightForward(1,110);
            P6->OUT &= ~0x05;
            Motor_Forward(1,85);
        }
        /*Left course correction*/
        else if (Line_Read() == 0x06 || Line_Read() == 0x0C ){
            P6->OUT &= ~0x05;
            Motor_LeftForward(1,160);//0000 0110 OR 0000 1100 OR Tilt Right//40,40,50
        }
        else if (Line_Read() == 0x04 || Line_Read() == 0x02 || Line_Read() == 0x63){
            P6->OUT &= ~0x05;
            Motor_LeftForward(1,160);//0000 0100 OR 0000 0010 OR 0110 0011 Tilt Right
        }
        else if(Line_Read() == 0x03 || Line_Read() == 0x01 || Line_Read() == 0x0E || Line_Read() == 0x07){
            P6->OUT &= ~0x05;
            Motor_LeftForward(1,170);//0000 0011 OR 0000 0001 OR 0000 1110 Tilt Right
        }
        /*Right course correction*/
        else if(Line_Read() == 0x60 || Line_Read() == 0x30 || Line_Read() == 0x70 || Line_Read() == 0x38 || Line_Read() == 0x31){
            P6->OUT &= ~0x05;
            Motor_RightForward(1,160);//0110 0000 OR 0011 0000 OR 0111 0000  OR 0011 1000 OR 0011 0001 Tilt left
        }
        else if (Line_Read() == 0x20 || Line_Read() == 0x40){
            P6->OUT &= ~0x05;
            Motor_RightForward(1,160);//0010 0000 OR 0100 0000 Tilt left
        }
        else if (Line_Read() == 0xC0 || Line_Read() == 0x80){
            P6->OUT &= ~0x05;
            Motor_RightForward(1,170);//1100 0000 OR 1000 0000 Tilt left
        }
        /* Turn left situation*/
        else if(Line_Read() == 0xF8 || Line_Read() == 0xFC  || Line_Read() == 0xF0 ){
            if(toTurnFlag == 0){
                toTurnFlag = 1;
                Motor_Stop();
                Motor_LeftBackward(2,130);
                Motor_RightBackward(2,130);
            }
            else{
                justTurned = 1;
                Motor_Stop();
                Turn_Left();    //1111 1000 OR 1111 1100 OR 1111 0000 Turn Left
                toTurnFlag = 0;
            }


        }
        /*Turn right situation*/
        else if (Line_Read() == 0x1F || Line_Read() == 0x0F){
            if(toTurnFlag == 0){
                toTurnFlag = 1;
                Motor_Stop();
                Motor_RightBackward(2,130);
                Motor_LeftBackward(2,130);
            }
            else{
                justTurned = 2;
                Motor_Stop();
                Turn_Right();   //0001 1111 OR 0000 1111 Turn Right
                toTurnFlag = 0;
            }

        }
        /*T intersection*/
        else if(Line_Read() == 0xFF){
            if (justTurned == 1){      // keep track of directions robot has turned
                Motor_Stop();
                Turn_Left();    //1111 1111 deleted 0011 1111
                justTurned = 0;
            }
            else{
                Motor_Stop();
                Turn_Right();
                justTurned = 0;
            }
        }

        /*Treasure Found*/
        else if(Line_Read() == 0x99 || Line_Read() == 0xCC || Line_Read() == 0x91 || Line_Read() == 0x33 || Line_Read() == 0x98 || Line_Read() == 0x66){
                //1001 1001 OR 1100 1100 OR 1001 0001 OR 0011 0011 OR 1001 1000 OR 0110 0110 OR 0001 1001Stop
            Motor_Stop();
            P6->OUT &= ~0x07;
            while(1){
                P6->OUT ^= 0x07;
                SysTick_Wait10ms(50);
            }
        }
        /*Detect no line*/
        else if(Line_Read() == 0){
            P6->OUT &= ~0x05;
            if(toTurnFlag == 0){
                toTurnFlag = 1;
                Motor_Stop();

                Motor_Backward(1,100);
            }
            else{
                TurnAround();
                toTurnFlag = 0;
            }
        }
        /*Hazard State*/
        else if (Line_Read() == 0x3F || Line_Read() == 0x7C || Line_Read() == 0xFE || Line_Read() == 0x11){
            P6->OUT &= ~0x05;
            Motor_Stop();
            Motor_RightBackward(2,110);  //0011 1111 OR 0111 1100 OR 1111 1110 OR 0001 0001
            Motor_LeftBackward(2,110);
            P1->OUT &= ~BIT0;
        }
        /* Default Action*/
        else{
            P6->OUT &= ~0x05;
            Motor_RightForward(2,120);//was 60
            Motor_LeftForward(2,120);
            P1->OUT &= ~BIT0;

        }
    }
}
