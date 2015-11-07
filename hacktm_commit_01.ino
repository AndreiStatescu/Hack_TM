
//===============DECLARATIONS FRONT SENSOR  ===================================================================
int LDR1 = PORTC0;     //analog pin to which LDR is connected, here we set it to 0 so it means A0
int LDR1Value = 0;      //that’s a variable to store LDR values
//==============================================================================================================


//===============DECLARATIONS LEFT SENSOR  =====================================================================
int LDR2 = PORTC1;     //analog pin to which LDR is connected, here we set it to 0 so it means A0
int LDR2Value = 0;      //that’s a variable to store LDR values
//==============================================================================================================

//===============DECLARATIONS RIGHT SENSOR  =====================================================================
int LDR3 = PORTC2;     //analog pin to which LDR is connected, here we set it to 0 so it means A0
int LDR3Value = 0;      //that’s a variable to store LDR values
//==============================================================================================================

//===============DECLARATIONS BACK SENSOR  =====================================================================
int LDR4 = PORTC3;     //analog pin to which LDR is connected, here we set it to 0 so it means A0
int LDR4Value = 0;      //that’s a variable to store LDR values
//==============================================================================================================

void setup() {
  Serial.begin(115200);                 // Note higher baud rate
  Serial.println("Starting...");
}  
  

void loop() {


start_FRONT_SENSOR();
Serial.println("LDR1:  " + String(LDR1Value) );

start_LEFT_SENSOR();
Serial.println("LDR2:  " + String(LDR2Value) );

start_RIGHT_SENSOR();
Serial.println("LDR3:  " + String(LDR3Value) );

start_BACK_SENSOR();
Serial.println("LDR4:  " + String(LDR4Value) );

_delay_ms(1000);
}

void start_FRONT_SENSOR()
{
//================ FRONT SENSOR  START ========================================================================
 LDR1Value = analogRead(LDR1);      //reads the ldr’s value through LDR 
  
//=============================================================================================================
}

void start_LEFT_SENSOR()
{
  
//================ LEFT SENSOR START ==========================================================================
 LDR2Value = analogRead(LDR2);      //reads the ldr’s value through LDR 
  
//=============================================================================================================
}

void start_RIGHT_SENSOR()
{
//================ RIGHT SENSOR  START ========================================================================
 LDR3Value = analogRead(LDR3);      //reads the ldr’s value through LDR 
  
}
//=============================================================================================================


void start_BACK_SENSOR()
{
//================ BACK SENSOR  START ========================================================================
 LDR4Value = analogRead(LDR4);      //reads the ldr’s value through LDR 
  
}
//=============================================================================================================
