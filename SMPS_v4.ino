/* 
Author: James Fotherby
  A program to control the output voltage of a non-synchronous boost converter used as a battery charger. A current sensor measures output current and
  resistor dividors measure input and output voltages. The Duty cycle to the MOSFET is continuously adjusted to maintain the current output voltage and
  current.

  To prevent control loop oscillations, the duty cycle is adjusted relatively slowly. This is fine for most applications but not all since the output voltage
  dips for a few douzen milliseconds on attachement of a load. The duty cycle is immediately cut back to zero if the output voltage exceeds a set threshold. This 
  prevents damagingly high voltages from developing when a load suddenly is removed, which is a major issue with non-syncronous boost converters. 

  A test is done every 15 seconds or so to see whether disabling pwm causes the output voltage to drop to 19v. This would indicate nothing is connected to the output
  and that it is RC decaying through the voltage dividor for the Arduino's ADC.
   
*/ 

// Pin descriptions
#define PWM_FET               9                                                 //OC1A - 16bit PWM output

#define LED                   A1
#define VinSense              A3
#define IoutSense             A4
#define VoutSense             A5

// Constants
#define VOLTAGE_SENSE_CONST   0.0537109
#define CURRENT_SENSE_CONST   0.02639
                
// Variables
int ICR_Reg = 299;                                                              // 53KHz PWM frequency                                                           
float VoltageSetPoint = 32.8;                                                   // 4.125v per cell. Battery longevity increased by not fully charging it.
float CurrentSetPoint = 1.4;                                                    // This value means under 4A input for the whole charge (inductor's saturation limit)
float Vout;
float Vin;
float Iout;                                                             

byte LED_Duty;

unsigned long CurrentMicros[5] = {0,0,0,0,0};
unsigned long PreviousMicros[5] = {0,0,0,0,0};
unsigned long Interval[5] = {100, 5000, 60000, 500000, 15000000};

float Charge_Levels[10] = {29.6, 30.4, 30.8, 31.2, 31.6, 32.0, 32.4, 32.8, 33.2, 33.8};

// Methods
void Check_Battery_Status(void);
void Iterate_Control_Loop(void);
void Display_Debug_Data(void);
void Over_Volt_Check(void);
void Toggle_LED(void);

//##############################################################################
// SETUP
//##############################################################################
void setup()
{                                      
  Serial.begin(115200);                                                         // Starts Serial driver

  ICR1 = ICR_Reg;
  OCR1A = 0;

  pinMode(LED, OUTPUT);  
  pinMode(PWM_FET, OUTPUT);  
  pinMode(VinSense, INPUT);
  pinMode(IoutSense, INPUT);
  pinMode(VoutSense, INPUT);
  
  TCCR1A = 0b11000010;
  TCCR1B = 0b00011001;                                                          // Fast PWM, No Prescaling of the clock
   
  Serial.println("Starting up...");
  delay(500); 
}

//##############################################################################
// LOOP  
//##############################################################################
void loop() 
{   
// This loop calls various functions with a regular period (as stored in the Interval[] array)
  
  for(int i = 0;i < 5;i++)
  {
    CurrentMicros[i] = micros();
    if (CurrentMicros[i] - PreviousMicros[i] >= Interval[i]) 
    {
      PreviousMicros[i] = CurrentMicros[i]; 

      switch (i) {
      case 0:
        Over_Volt_Check();                                                    // 1) Perform check to see if output voltage has risen too high
        break;
        
      case 1:
        Iterate_Control_Loop();                                               // 2) Adjust PWM duty cycle until desired output voltage or current is reached
        break;
        
      case 2:
        Toggle_LED();                                                         // 3) Flash LED to indicate current state
        break;
        
      case 3:
        Display_Debug_Data();                                                 // 4) Display Debug Info
        break;
        
      case 4:
        Check_Battery_Status();                                               // 5) Check Whether Battery Connected etc
        break;                
      }
    }
  }
} 

//##############################################################################
// SUBROUTINES
//##############################################################################
void Over_Volt_Check()
{
  Vout = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                           // Convert Analogue read to real voltage value
  
  if(Vout > VoltageSetPoint + 2.0)                                              // If voltage goes 2 volts higher than the set point. Shut off immediately
  {
    OCR1A = 0;
    Serial.println("OVER VOLTAGE, RESET PWM");
    delay(10);      
  }  
}

//==============================================================================
void Iterate_Control_Loop()
{
  Vout = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                           // Convert Analogue read to real voltage value
  Iout = (analogRead(IoutSense)  - 512) * CURRENT_SENSE_CONST; 
  
  if((Vout < VoltageSetPoint) && (Iout < CurrentSetPoint))
  {                                                  
    OCR1A = min(OCR1A, (ICR_Reg - 50));                                         // Limit maximum duty cycle (~83%) 
    OCR1A++;
  } 
  else 
  {
    OCR1A = max(OCR1A, 1);                                                      // Limit minimum duty cycle to 0% obviously
    OCR1A--;
  }
}

//==============================================================================
void Toggle_LED()
{
  static byte ledState;
  static byte LED_Duty_Counter = 0;

  LED_Duty_Counter++;
  if(LED_Duty_Counter > 9)
    LED_Duty_Counter = 0;
  
  if(LED_Duty_Counter >= LED_Duty)
    ledState = LOW; 
  else 
    ledState = HIGH; 
       
  digitalWrite(LED, ledState);                                                  // Set the LED with the ledState of the variable: 
}

//==============================================================================
void Display_Debug_Data()
{
  Serial.print("Vout: ");
  Serial.print(Vout);
  
  Serial.print("/");
  Serial.println(VoltageSetPoint);   
  
  Serial.print("Iout: ");
  Serial.print(Iout);    
 
  Serial.print("/");
  Serial.println(CurrentSetPoint); 
  
  Serial.print("OCR1A: ");
  Serial.print(OCR1A);    
  Serial.print("/");
  Serial.println(ICR_Reg);
  
  Serial.print("Vin: ");
  Serial.println(Vin);  
  Serial.println();  

  Vin = analogRead(VinSense) * VOLTAGE_SENSE_CONST;
  if(Vin < 17.0)
  {
    OCR1A = 0;
    Serial.println("Input voltage below 18 volts! PAUSED");
    
    while((analogRead(VinSense) * VOLTAGE_SENSE_CONST) < 18.0) 
    {
      LED_Duty = 5;
      Toggle_LED();
      delay(10);      
    }
  }
}

//==============================================================================
void Check_Battery_Status()
{
  OCR1A = 0;                                                                    // Switch off output
  delay(100);                                                                   // And wait...

  float V_test = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;

  // Checks if battery is fully charged. If so, the target voltage will be reached and current flow will be zero. Also checks for disconnect...
  float TempStore = VoltageSetPoint;
  byte counter = 0;
  while(V_test >= TempStore && Iout <= 0.1)
  { 
    TempStore = VoltageSetPoint - 0.25;
    
    LED_Duty = 10;
    Toggle_LED();
    delay(10);  

    counter++;
    counter = min(counter, 100);
    if(counter == 99)    {      
      Serial.println("Battery Fully Charged... PAUSED");
    }
    
    V_test = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
  }

  // The decay was fast enough to suspect it was due to self discharge. Otherwise the battery would maintain the voltage.
  static byte Suspect_Disconnected_Battery = 0;
  if(counter > 0 && counter < 15)  {
    Suspect_Disconnected_Battery++;
  }  else  {
    Suspect_Disconnected_Battery -= 2;
    Suspect_Disconnected_Battery = max(Suspect_Disconnected_Battery, 0);    
  }

  // If we've had 4 Suspect_Disconnected_Battery +ve tests do a final check then indicate such condition and wait until a battery is connected 
  if(Suspect_Disconnected_Battery >= 4)
  {
    delay(7500);                                                                // Should allow enough time for voltage to decay to ~19volts
    Suspect_Disconnected_Battery = 0;
    V_test = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
    
    while(V_test < 22.0)
    {
      LED_Duty = 0;
      Toggle_LED(); 
      delay(2500); 
      
      LED_Duty = 10;
      Toggle_LED(); 
      delay(50);

      Serial.println("Battery Disconnected... PAUSED");
      V_test = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
    }
  }


  // Flash the LED at a duty that represents the batteries charge status.
  LED_Duty = 1;                                                           
  for(int i = 0; i < 9; i++)
  {
    if(V_test > Charge_Levels[i])
      LED_Duty++;
  }  
}



















