/* 
Author: James Fotherby
*/ 

//pin descriptions
const int PWM_FET = 9;                                                          //OC1A - 16bit PWM output

const int LED = A1;
const int VinSense  = A3;
const int IoutSense = A4;
const int VoutSense = A5;

const float VOLTAGE_SENSE_CONST = 0.0537109;
const float CURRENT_SENSE_CONST = 0.02639;
                
// Variables
int ICR_Reg = 299;                                                              // 53KHz PWM frequency                                                           
float VoltageSetPoint = 32.8;                                                   // 4.125v per cell. Battery longevity increased by not fully charging it.
float CurrentSetPoint = 1.4;                                                    // This value means under 4A in for the whole charge (the inductor's saturation limit)
float Vout;
float Vin;
float Iout;

int ledState = LOW;                                                             // LedState used to set the LED

unsigned long previousMillis = 0;                                               // Will store last time LED was updated
unsigned long previousMicrosA = 0;                                              // Will store last time check was performed
unsigned long previousMicrosB = 0;                                              // Will store last time PWM was updated
const long intervalLED = 400;                                                   // Interval at which to blink LED (milliseconds)
const long intervalPWM = 5000;                                                  // Interval in which to sample voltage and update PWM values (microseconds)
const long intervalChk = 100;                                                   // Interval to check for over voltage. Short enough to catch runnaways from sudden disconnections

//##############################################################################
// SETUP
//##############################################################################
void setup()
{                                      
  Serial.begin(115200);                                                         // Starts Serial driver
  delay(100);
  Serial.println();  

  ICR1 = ICR_Reg;
  OCR1A = 0;
  pinMode(PWM_FET, OUTPUT);  
  
  TCCR1A = 0b11000010;
  TCCR1B = 0b00011001;                                                          // Fast PWM, No Prescaling of the clock
  
  if((analogRead(VinSense) * VOLTAGE_SENSE_CONST) < 18)
   {
     Serial.println("Paused until input voltage above 18 volts");
     while((analogRead(VinSense) * VOLTAGE_SENSE_CONST) < 18) 
     {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);       
     }
   }
   
  Serial.println("Starting up...");
  delay(2000); 
}

//##############################################################################
// LOOP  
//##############################################################################
void loop() 
{   
  
// Perform check to see if output voltage has risen too high -------------------
  unsigned long currentMicrosA = micros();
  if (currentMicrosA - previousMicrosA >= intervalChk) 
  {    
    previousMicrosA = currentMicrosA;     
  
    Vout = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                         // Convert Analogue read to real voltage value
    
    if(Vout > VoltageSetPoint + 2.0)                                            // If voltage go 2volts higher than the set point. Shut off immediately
    {
      OCR1A = 0;
      Serial.println("OVER VOLTAGE, RESET PWM");
      delay(10);      
    }
  }    
  
// Adjust PWM duty cycle until desired output voltage and current is reached ----  
  unsigned long currentMicrosB = micros();
  if (currentMicrosB - previousMicrosB >= intervalPWM) 
  {    
    previousMicrosB = currentMicrosB;     
  
    Vout = analogRead(VoutSense) * VOLTAGE_SENSE_CONST;                         // Convert Analogue read to real voltage value
    Iout = (analogRead(IoutSense)  - 512) * CURRENT_SENSE_CONST; 
    
    if((Vout < VoltageSetPoint) && (Iout < CurrentSetPoint))
    {
      OCR1A = min(OCR1A, (ICR_Reg));                                            // Limit maximum 
      OCR1A = min(OCR1A, (ICR_Reg - 50));
      OCR1A++;
    } else {
      OCR1A = max(OCR1A, 1);                                                    // Limit minimum duty cycle to 0% obviously
      OCR1A--;
    }
  }  
  
// Debuging ----------------------------------------------------------------------
  unsigned long currentMillis = millis();    
  if (currentMillis - previousMillis >= intervalLED) 
  {    
    previousMillis = currentMillis; 
    
    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }    
    digitalWrite(LED, ledState);                                                // Set the LED with the ledState of the variable:
    
    Iout = (analogRead(IoutSense)  - 512) * CURRENT_SENSE_CONST;    
    Vin = analogRead(VinSense) * VOLTAGE_SENSE_CONST;

    Serial.print("Output voltage: ");
    Serial.println(Vout);
    
    Serial.print("Taget output voltage: ");
    Serial.println(VoltageSetPoint);   
    
    Serial.print("Output current: ");
    Serial.println(Iout);    
   
    Serial.print("Taget output current: ");
    Serial.println(CurrentSetPoint); 
    
    Serial.print("OCR1A: ");
    Serial.print(OCR1A);
    
    Serial.print(" / ");
    Serial.println(ICR_Reg);
    
    Serial.print("VinSense: ");
    Serial.println(Vin);  
    Serial.println(); 
    
    
    if(Vin < 17.0)
    {
      Serial.println("Input voltage below 18 volts! PAUSED");
      OCR1A = 0;
      while((analogRead(VinSense) * VOLTAGE_SENSE_CONST) < 18) 
      {
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
        delay(100);        
      }
    }
  }  

} 


