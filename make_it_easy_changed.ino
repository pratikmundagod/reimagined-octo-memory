// Sensorless brushless DC (BLDC) motor control with Arduino UNO (Arduino DIY ESC).
// This is a free software without any warranty.
 
#define SPEED_UP          A0
#define SPEED_DOWN        A1
#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      100
#define PWM_START_DUTY    180
int SPEED=180;
 
byte bldc_step = 0, motor_speed;
unsigned int i;
void setup() {
  DDRD  |= 0x38;           // Configure pins 3, 4 and 5 as outputs
  PORTD  = 0x00;
  DDRB  |= 0x0E;           // Configure pins 9, 10 and 11 as outputs
  ////////////////////////////////////NEW//////////////////////////////////////////////////////
  digitalWrite(5,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(11,HIGH);
  digitalWrite(10,LOW);
  delay(1500);
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
  delay(1500);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
  PORTB  = 0x31;
  // Timer1 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR1A = 0;
  TCCR1B = 0x01;
  // Timer2 module setting: set clock source to clkI/O / 1 (no prescaling)
  TCCR2A = 0;
  TCCR2B = 0x01;
  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
//  pinMode(SPEED_UP,   INPUT_PULLUP);//  changebyme
//  pinMode(SPEED_DOWN, INPUT_PULLUP);
  Serial.begin(9600);
}
// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce
  for(i = 0; i < 10; i++) {
    if(bldc_step & 1){
      if(!(ACSR & 0x20)) i -= 1;
    }
    else {
      if((ACSR & 0x20))  i -= 1;
    }
  }
  bldc_move();
  bldc_step++;
  bldc_step %= 6;//6
}
void bldc_move(){        // BLDC motor commutation function
  switch(bldc_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}
 
void loop() {
  SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  i = 5500;//5000
  // Motor start
  while(i > 100) {//i>100
    delayMicroseconds(i);
    bldc_move();
    bldc_step++;
    bldc_step %= 6;
    i = i - 1;//i = i - 20
  }
  motor_speed = PWM_START_DUTY;
  int PWM = PWM_START_DUTY;
  Serial.println(PWM);
  ACSR |= 0x08;                    // Enable analog comparator interrupt
  while(1) {
    if(Serial.available()>0){
     int Spd=Serial.read();
     if(Spd==97){
      SPEED=160;
      while(PWM>SPEED){
        PWM--;
       Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }
      }
     else if(Spd==98){
      SPEED=180;
      if(SPEED>PWM){
        while(SPEED>PWM){
        PWM++;
        Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
       if(SPEED<PWM){
        while(SPEED<PWM){
         PWM--;
       Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
      }
      else if(Spd==99){
      SPEED=200;
       if(SPEED>PWM){
        while(SPEED>PWM){
        PWM++;
        Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
       if(SPEED<PWM){
        while(SPEED<PWM){
         PWM--;
       Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
      }
      else if(Spd==100){
      SPEED=230;
       if(SPEED>PWM){
        while(SPEED>PWM){
        PWM++;
        Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
       if(SPEED<PWM){
        while(SPEED<PWM){
         PWM--;
       Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
      }
      else if(Spd==101){
      SPEED=255;
       if(SPEED>PWM){
        while(SPEED>PWM){
        PWM++;
        Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
       if(SPEED<PWM){
        while(SPEED<PWM){
         PWM--;
       Serial.println(PWM);
        SET_PWM_DUTY(PWM);
        delay(100);
        }}
      }
      }
//    while((PWM=!SPEED) && PWM < SPEED){
//      PWM++;
//      Serial.println(PWM);
//      SET_PWM_DUTY(PWM);
//      delay(100);
//    }
//    while((PWM=!SPEED) && PWM > SPEED){
//      PWM--;
//      Serial.println(PWM);
//      SET_PWM_DUTY(PWM);
//      delay(100);
//    }
     //Serial.println(motor_speed);
  }
}
 
void BEMF_A_RISING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR |= 0x03;            // Set interrupt on rising edge
}
void BEMF_A_FALLING(){
  ADCSRB = (0 << ACME);    // Select AIN1 as comparator negative input
  ACSR &= ~0x01;           // Set interrupt on falling edge
}
void BEMF_B_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR |= 0x03;
}
void BEMF_B_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 as comparator negative input
  ACSR &= ~0x01;
}
void BEMF_C_RISING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR |= 0x03;
}
void BEMF_C_FALLING(){
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 as comparator negative input
  ACSR &= ~0x01;
}
 
void AH_BL(){
  PORTB  =  0x04;
  PORTD &= ~0x18;
  PORTD |=  0x20;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void AH_CL(){
  PORTB  =  0x02;
  PORTD &= ~0x18;
  PORTD |=  0x20;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void BH_CL(){
  PORTB  =  0x02;
  PORTD &= ~0x28;
  PORTD |=  0x10;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void BH_AL(){
  PORTB  =  0x08;
  PORTD &= ~0x28;
  PORTD |=  0x10;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void CH_AL(){
  PORTB  =  0x08;
  PORTD &= ~0x30;
  PORTD |=  0x08;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
}
void CH_BL(){
  PORTB  =  0x04;
  PORTD &= ~0x30;
  PORTD |=  0x08;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
}
 
void SET_PWM_DUTY(byte duty){
  if(duty < PWM_MIN_DUTY)
    duty  = PWM_MIN_DUTY;
  if(duty > PWM_MAX_DUTY)
    duty  = PWM_MAX_DUTY;
  OCR1A  = duty;                   // Set pin 9  PWM duty cycle
  OCR1B  = duty;                   // Set pin 10 PWM duty cycle
  OCR2A  = duty;                   // Set pin 11 PWM duty cycle
}
