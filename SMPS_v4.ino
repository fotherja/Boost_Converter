/* 
Author: James Fotherby
  A program to control the output voltage of a non-synchronous boost converter used as a battery charger. A current sensor measures output current and
  resistor dividors measure input and output voltages. The Duty cycle to the MOSFET is continuously adjusted to maintain the set output voltage and
  current.

  To prevent control loop oscillations, the duty cycle is adjusted relatively slowly. This is fine for most applications but not all since the output voltage
  dips for a few douzen milliseconds on attachement of a load. The duty cycle is immediately cut back to zero if the output voltage exceeds a set threshold. This 
  prevents damagingly high voltages from developing when a load is suddenly removed, which is a major issue with non-syncronous boost converters. 

  A test is done every 15 seconds to see whether disabling pwm causes the output voltage to drop towards 19v. This would indicate nothing is connected to the output
  and that it is RC decaying through the voltage dividers for the Arduino's ADC. If such a scenario is detected the circuit idles and stops boosting. Reconnection 
  of a battery starts things up again.
   
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
float VoltageSetPoint = 50.0;       //###### VERY IMPORTANT #######             // Battery longevity increased by not fully charging it.
float CurrentSetPoint = 0.8;        //###### VERY IMPORTANT #######             // This value means under 4A input for the whole charge (inductor's saturation limit) It slightly underestimates...
float Vout;
float Vin;
float Iout;                                                             

byte LED_Duty;

unsigned long CurrentMicros[5] = {0,0,0,0,0};
unsigned long PreviousMicros[5] = {0,0,0,0,0};
unsigned long Interval[5] = {100, 5000, 60000, 500000, 15000000};

//float Charge_Levels[10] = {29.6, 30.4, 30.8, 31.2, 31.6, 32.0, 32.4, 32.8, 33.2, 33.8};         // 8S
float Charge_Levels[10] = {44.4, 45.6, 46.2, 46.8, 47.4, 48.0, 48.6, 49.2, 49.8, 50.7};           // 12S

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
  Vout = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                    // Convert Analogue read to real voltage value
  
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
  Vout = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                    // Convert Analogue read to real voltage value
  Iout = ((float)analogRead(IoutSense)  - 512.0) * CURRENT_SENSE_CONST; 
  
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

  Vin = (float)analogRead(VinSense) * VOLTAGE_SENSE_CONST;
  if(Vin < 17.0)
  {
    OCR1A = 0;
    Serial.println("Input voltage below 18 volts! PAUSED");
    
    while(((float)analogRead(VinSense) * VOLTAGE_SENSE_CONST) < 18.0) 
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
  Serial.println("Battery Check");
  
  OCR1A = 0;                                                                    // Switch off output and immediately measure output voltage
  delay(1);

  float Measured_Battery_Voltage = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
  Serial.print("Measured Battery Voltage: ");
  Serial.println(Measured_Battery_Voltage);

  static byte Suspect_Disconnected_Battery = 0;
  float Voltage_to_Reach = VoltageSetPoint - 0.25;  
  byte counter = 0;

  // Checks if battery is fully charged. If so, the target voltage will be reached and current flow will be zero.
  while(Measured_Battery_Voltage >= Voltage_to_Reach && Iout <= 0.1)            // If the standalone Battery Voltage >= The Set point and less than 0.1A is flowing stay in this loop 
  { 
    Voltage_to_Reach = VoltageSetPoint - 0.5;                                   // Add a bit of Hysterisis in. This makes it easier to stay in this loop by reducing the set point slightly once here
    
    LED_Duty = 10;                                                              // Simply ensures LED is always on
    Toggle_LED();
    delay(10);  

    counter++;                                                                  // Increment counter every 10ms whilst in this loop. 
    counter = min(counter, 100);
    if(counter == 99)    {                                                      // If we're staying in this loop it means a battery must be connected. And it must be fully charged     
      Serial.println("Battery Fully Charged... PAUSED");
      Suspect_Disconnected_Battery = 0;
    }
    
    Measured_Battery_Voltage = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
  }

  Serial.print("counter: ");
  Serial.println(counter);  
  
  if(counter == 0)                                                              // We haven't charged the battery yet! (Or has the Measured_Battery_Voltage been pulled down very quickly?)  
  {  
  }  
  else if(counter > 1 && counter < 30)                                          // The decay was fast enough to suspect it was due to self discharge. Otherwise the battery would have maintained the voltage.
  {
    Suspect_Disconnected_Battery++;    
  }  
  else                                                                          // The decay was quite slow. Unlikely to be because of a disconnected output
  {
    Suspect_Disconnected_Battery = 0;   
  }

  Serial.print("SDB count: ");
  Serial.println(Suspect_Disconnected_Battery);   

  // If we've had 4 Suspect_Disconnected_Battery +ve tests do a final check then indicate such condition and wait until a battery is connected 
  if(Suspect_Disconnected_Battery >= 4)
  {
    Serial.println("Suspect Disconnected Battery!");
    delay(15000);                                                               // Should allow enough time for voltage to decay to ~19volts
    
    Suspect_Disconnected_Battery = 0;
    Measured_Battery_Voltage = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
    
    while(Measured_Battery_Voltage < 22.0)
    {
      LED_Duty = 0;
      Toggle_LED(); 
      delay(2500); 
      
      LED_Duty = 10;
      Toggle_LED(); 
      delay(50);
      
      Measured_Battery_Voltage = (float)analogRead(VoutSense) * VOLTAGE_SENSE_CONST;
      Serial.print("Battery Disconnected... PAUSED. Voltage on output: ");
      Serial.println(Measured_Battery_Voltage);
    }
  }

  // Flash the LED at a duty that represents the batteries charge status.
  LED_Duty = 1;                                                           
  for(int i = 0; i < 9; i++)
  {
    if(Measured_Battery_Voltage > Charge_Levels[i])
      LED_Duty++;
  }    
}



















