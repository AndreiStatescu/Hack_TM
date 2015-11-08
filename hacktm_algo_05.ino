#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |=  _BV(bit))//turn led ON
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))//turn led OFF

//===============DECLARATIONS FRONT SENSOR  ===================================================================
volatile unsigned long cnt1 = 0;
unsigned long oldcnt1 = 0;
unsigned long t1 = 0;
unsigned long last1;
//==============================================================================================================


//===============DECLARATIONS LEFT SENSOR  =====================================================================
volatile unsigned long cnt2 = 0;
unsigned long oldcnt2 = 0;
unsigned long t2 = 0;
unsigned long last2;
//==============================================================================================================

void irq1()
{
  cnt1++;
}

void irq2()
{
  cnt2++;
}


const int LDR0=0;              //analog pin to which LDR0 is connected, here we set it to 0 so it means A0
const int LDR1=1;              //analog pin to which LDR1 is connected, here we set it to 1 so it means A1
const int LDR2=2;              //analog pin to which LDR2 is connected, here we set it to 2 so it means A2
const int LDR3=3;              //analog pin to which LDR3 is connected, here we set it to 3 so it means A3
const int UNCONNECTED_PIN=5;   //analog pin used for the randomizing function

const int delay_viraj = 220;//constanta pe care o folosesc sa si delay la viraje

const int START_S=0;           //sensor start value
const int COMMANDS_SIZE=128;   //the number of command we keep history of
const int TIME_STEP=10;        //the size of a time step in millis

const int THRESHOLD_LOW_0=600;     //low threshold of sensor value
const int THRESHOLD_HIGH_0=850;    //high threshold of sensorvalue
const int THRESHOLD_LOW_1=600;     //low threshold of sensor value
const int THRESHOLD_HIGH_1=850;    //high threshold of sensorvalue
const int THRESHOLD_LOW_4=20;     //low threshold of sensor value
const int THRESHOLD_HIGH_4=40;    //high threshold of sensorvalue

//0 for front
//1 for right
//2 for left
//3 for back
int s0, s1, s2, s3, s4, s5;
int lasts0, lasts1, lasts2, lasts3;
boolean f0, f1, f2, f3, f4, f5, f_front, f_right, f_left;
unsigned long hz1, hz2;

int command;
int commandIndex = -1;
int commands[COMMANDS_SIZE];

boolean backtrack = false;


String explanation;

void addCommand(int command){
  commandIndex++;
  commands[commandIndex % COMMANDS_SIZE]=command;
}

int getPrevCommand(int deltaIndex){
  //test whether the desired command's index is negative
  if(commandIndex - deltaIndex < 0)
    return -1;

  //test whether the desired command's index is still stored
  if(deltaIndex >= COMMANDS_SIZE)
    return -1;

  return commands[(commandIndex-deltaIndex)%COMMANDS_SIZE];
}

void setFlagBySVal(int index, boolean& flag, int s){
  if (index == 0) {
    if(s > THRESHOLD_HIGH_0 && flag == false){
      flag = true;
    }
    if(s < THRESHOLD_LOW_0 && flag == true){
      flag = false;
    }
  } else if (index == 1 || index == 2){
    if(s > THRESHOLD_HIGH_1 && flag == false){
      flag = true;
    }
    if(s < THRESHOLD_LOW_1 && flag == true){
      flag = false;
    }
  } else if (index == 3){
    if(s > THRESHOLD_HIGH_1 && flag == false){
      flag = true;
    }
    if(s < THRESHOLD_LOW_1 && flag == true){
      flag = false;
    }
  } else if (index == 4){
    if(s > THRESHOLD_HIGH_4 && flag == false){
      flag = true;
    }
    if(s < THRESHOLD_LOW_4 && flag == true){
      flag = false;
    }
  } else {
    flag = false;
  }
}

int getSensorValue(int sensor_index){
  switch(sensor_index){
    case 0:return analogRead(LDR0);
    case 1:return analogRead(LDR1);
    case 2:return analogRead(LDR2);
    case 3:return analogRead(LDR3);
    case 4:return hz1;
    case 5:return hz2;
  }
  return -1;
}

void printSensorValuesAndFlags(){
  Serial.println("LDR0:"+String(s0)+",flag:"+String(f0));
  Serial.println("LDR1:"+String(s1)+",flag:"+String(f1));
  Serial.println("LDR2:"+String(s2)+",flag:"+String(f2));
  Serial.println("LDR3:"+String(s3)+",flag:"+String(f3));
  Serial.println("???4:"+String(s4)+",flag:"+String(f4));
}

void printPrevCommand(int index){
  String dir="no dir";
  switch(getPrevCommand(index)){
    case 0:dir="up";break;
    case 1:dir="right";break;
    case 2:dir="left";break;
    case 3:dir="down";break;
  }
  Serial.println("Direction: "+dir);
}

void printCommand(){
  Serial.println(command);
}

void printExplanation() {
  Serial.println("Explanation: "+explanation);
}

void setup(){
  Serial.begin(115200);                 // Note higher baud rate
  Serial.println("Starting...");
  

//===============OUTPUT PINS========================================================================================================================================================  
  DDRB = (1 << PORTB0) |(1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3) | (1 << PORTB4)  |(1 << PORTB5)  ;//PORTB 0,1,2,3 for the engines PORTB4 pt samnalizare la stanga, PORTB5 pt semnalizare la dreapta
//==================================================================================================================================================================================

//============ RIGHT UP SENSOR  SETUP =============================================================================
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), irq1, RISING);
//==============================================================================================================

//============ LEFT  UP SENSOR  SETUP ==============================================================================
  pinMode(4, INPUT);
  attachInterrupt(digitalPinToInterrupt(4), irq2, RISING);
//==============================================================================================================


//============================= these are for the PWM SETUP ========================================================================================================================

  DDRD = (1 << PORTD5) | (1 << PORTD6) | (1 << PORTD7);//PWMA = PORTD5     PWMB = PORTD6   STDBY = PORTD7
  
  TCCR0A = (1 << COM0A1) | (1 << WGM00)|(1 << WGM01)|(1 << COM0B1);//set CTC(clear time and commpare) bit, for reset the count at a moment and start again from the beginning 
  
  TCCR0B = (1 <<WGM02);//|(1 << CS02) | (1 << CS00);//FAST PWM
  
  TCCR0B = (1 << CS02) | (1 << CS00);//start the TIMERS, start at 1024 prescalar

  // Bit 7 – ICNC1 - Input Capture Noise Canceler – Acest bit setat va activa opțiunea de noise cancel  a modului de captură.
  // Norma bitul ICNC1 trebuia setat pentru TCCR1B
  // Dacă modul de captură foloseste opțiunea de noise cancel, detectarea evenimentului va dura de 4 ori mai mult decât in mod normal.
  // Acestă obțiune este folosită pentru o mai bună filtrare a semnalului de intrare, algoritmul de filtrare constând în generarea evenimentului
  // de captură doar dacă ultimele 4 monitorizări au valori egale.
  
  //================================== AICI SETEZ VITEZA MOTOARELOR ======================================================
  //OCR0A = 120;//comparison for A - 8 bit TIMER (50% on and 50% off)    If I put OCR1A = 100, the engines will be 100 %
  //OCR0B = 120;
 
  
  //TIMSK  = the register that control the interrupts, here we can set--- whe this happens, I want this to  happen, this interrupt to occur
  //BIT1 - OCIE0A this is the bit that says..I want an interrupt to setup, such that when my comparison value for A matches my timmer value , I want to setup
   
  //TOIEO is te interrupt used to increase permanently the speed of ENGINES 
  //TIMSK0 = (1 << TOIE0);//TOIE is that kind of interrupts that is used when it's overflow the comparison
  //TIMSK0 = (1 << OCIE0A);//COMPARE
  
  //TCNT0 hold the TICS from TIMMER

  last1 = millis();
  last2 = millis();
  
  s0=s1=s2=s3=s4=s5=START_S;
  lasts0=lasts1=lasts2=lasts3=START_S;
  f0=f1=f2=f3=f4=f5=false;

  randomSeed(analogRead(UNCONNECTED_PIN));
//===================================================================================================================================================================================  
}

void loop(){
  /*
  Serial.println("Cnt1: " + String(cnt1 - oldcnt1));
  Serial.println("Cnt2: " + String(cnt2 - oldcnt2));
  */
  
  start_RIGHT_SENSOR();
  start_LEFT_SENSOR();
  
  s0=getSensorValue(0);
  s1=getSensorValue(1);
  s2=getSensorValue(2);
  s3=getSensorValue(3);
  s4=getSensorValue(4);
  s5=getSensorValue(5);
  
  setFlagBySVal(0,f0,s0);
  setFlagBySVal(1,f1,s1);
  setFlagBySVal(2,f2,s2);
  setFlagBySVal(3,f3,s3);
  setFlagBySVal(4,f4,s4);
  setFlagBySVal(5,f5,s5);

  //_delay_ms(600);
  printSensorValuesAndFlags();

  if (s4 == -1) {
    f4 = false;  
  }

  f_front = f0 | f3;
  f_right = f1 | f4;
  f_left = f2;
  
  command=0;
  
  //no frontal obstacle and no back move was just made
  //in this case go front
  if(f_front == false){
    command = 0;
  }
  else
  {
    if(f_right == false && f_left == false){
      command = random(1, 3);
    }
    
    //no right obstacle
    else if(f_right == false){
      command = 1;
    }
    
    //no left obstacle
    else if(f_left == false){
      command = 2;
    }
    
    // both left and right are obstacles so go back
    else {
      command = random(1, 3);
    }
  }
  
  //printExplanation();
  printCommand();
  
  switch (command) {
    case 0: goFRONT(); break;
    case 1: goRIGHT(); break;
    case 2: goLEFT(); break;
    //case 3: goRIGHT(); backtrack = false; break;
  }


  
  lasts0 = s0;
  lasts1 = s1;
  lasts2 = s2;
  lasts3 = s3;
  
  //_delay_ms(1000);
    
}



void goFRONT()
{
   //merge inainte
   OCR0A = 180;//OCR0A este PWM-ul pentru rotile din partea dreapta
   OCR0B = 180;
   
   sbi(PORTB,PORTB0);//Rotile din stanga
   cbi(PORTB,PORTB1);//Rotile din stanga
                  
   sbi(PORTD,PORTD7); //STDBY va fi tot timpul 1
   
   sbi(PORTB,PORTB2);//SET PWMA ON    pt rotile din dreapta
   cbi(PORTB,PORTB3);//SET PWMA ON    pt rotile din dreapta
   //_delay_ms(150);
       
}

void goBACK()
{
   //merge in spate
   OCR0A = 180;//OCR0A este PWM-ul pentru rotile din partea dreapta
   OCR0B = 180;
   
   cbi(PORTB,PORTB0);//Rotile din stanga
   sbi(PORTB,PORTB1);//Rotile din stanga
                
   sbi(PORTD,PORTD7); //STDBY va fi tot timpul 1
   
   cbi(PORTB,PORTB2);//SET PWMA ON    pt rotile din dreapta
   sbi(PORTB,PORTB3);//SET PWMA ON    pt rotile din dreapta     
}

void goLEFT()
{
  //=========SEMNALIZARE INAINTEA VIRARII LA STANGA==================================
                 stopCAR();
   for(int i = 0 ; i < 5 ; i++)
   {
   PORTB ^= (1 << PORTB5);
   _delay_ms(delay_viraj);// delay 500 ms cand a iesit din for
   }
   //=====================================================================================
  
   //VIREAZA STANGA
   OCR0A = 235;
   OCR0B = 235;
   sbi(PORTB,PORTB0);//Rotile din stanga
   cbi(PORTB,PORTB1);//Rotile din stanga
                  
   sbi(PORTD,PORTD7); //STDBY va fi tot timpul 1
   //sbi(PORTB,PORTB5);
  
   
   cbi(PORTB,PORTB2);//SET PWMA ON    pt rotile din dreapta
   sbi(PORTB,PORTB3);//SET PWMA ON    pt rotile din dreapta

   //=========SEMNALIZARE PE PARCURSUL VIRARII LA STANGA==================================
   for(int i = 0 ; i < 5 ; i++)
   {
   PORTB ^= (1 << PORTB5);
   _delay_ms(delay_viraj);// delay 500 ms cand a iesit din for
   }
   //=====================================================================================
   PORTB &= ~(1 << PORTB5);
}
void goRIGHT()
{
  stopCAR();
  //=========SEMNALIZARE PE PARCURSUL VIRARII LA STANGA==================================
   for(int i = 0 ; i < 5 ; i++)
   {
   PORTB ^= (1 << PORTB4);
   _delay_ms(delay_viraj);// delay 500 ms cand a iesit din for
   }
   //=====================================================================================
   //VIREAZA DREAPTA
   OCR0A = 235;
   OCR0B = 235;
   
   cbi(PORTB,PORTB0);//Rotile din stanga
   sbi(PORTB,PORTB1);//Rotile din stanga
                
   sbi(PORTD,PORTD7); //STDBY va fi tot timpul 1
   
   sbi(PORTB,PORTB2);//SET PWMA ON    pt rotile din dreapta
   cbi(PORTB,PORTB3);//SET PWMA ON    pt rotile din dreapta
   

   //=========SEMNALIZARE PE PARCURSUL VIRARII LA STANGA==================================
   for(int i = 0 ; i < 5 ; i++)
   {
   PORTB ^= (1 << PORTB4);
   _delay_ms(delay_viraj);// delay 500 ms cand a iesit din for
   }
   //=====================================================================================
   PORTB &= ~(1 << PORTB4);
}

void stopCAR()
{
cbi(PORTB,PORTB0);
cbi(PORTB,PORTB1);
cbi(PORTB,PORTB2);  
cbi(PORTB,PORTB3);
}

void start_RIGHT_SENSOR()
{
//================ FRONT SENSOR  START ========================================================================
 //if (millis() - last1 >= 1000)
 // {
 
    t1 = cnt1;
    hz1 = (t1 - oldcnt1) / (millis() - last1);
    last1 = millis();
    
    Serial.print("FREQ RIGHT: "); 
    Serial.print(hz1);
    Serial.print("\t = "); 
    Serial.print((hz1+50)/100);  // +50 == rounding last digit
    Serial.println(" mW/m2");
    
    oldcnt1 = t1;
    
  //}
//=============================================================================================================
}

void start_LEFT_SENSOR()
{
  
//================ LEFT SENSOR START ==========================================================================
  //if (millis() - last2 >= 1000)
  //{
    
    t2 = cnt2;
    hz2 = (t2 - oldcnt2) / (millis() - last2);
    last2 = millis();
    
    Serial.print("FREQ LEFT: "); 
    Serial.print(hz2);
    Serial.print("\t = "); 
    Serial.print((hz2+50)/100);  // +50 == rounding last digit
    Serial.println(" mW/m2");
    
    oldcnt2 = t2;
  //}
  
}
//=============================================================================================================




