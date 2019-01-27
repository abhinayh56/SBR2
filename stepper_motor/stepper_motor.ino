// This program drives the stepper motor if W_L, and W_R are given as input as rpm

float n_L, n_0_L, n0_memry_L, W_L = 100;
float n_R, n_0_R, n0_memry_R, W_R = 100;

unsigned long t;

void setup(){
  Serial.begin(57600);
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  
  //Make sure that the TCCR2B register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 = 20us/(1s/(16.000.000MHz/8))-1
  TCCR2A |= (1 << WGM21);                                                   //Set timer 2 to CTC (clear timer on compare) mode
  sei();

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  t = micros();
}

void loop(){
  n0_memry_L = (0.50*3750)/W_L;
  n0_memry_R = (0.50*3750)/W_R;
  
  while(micros()-t<4000);
  t = micros();
}

ISR(TIMER2_COMPA_vect){
  // Left motor
  if(n_L==0 || n_L>=n_0_L){                                 //If the number of loops is larger then the n_memory variable
    n_L = 0;                                                //Reset the n variable
    n_0_L = n0_memry_L;
    if(n_0_L<0){
      PORTD &= 0b11011111;                                  //Set output 5 low to reverse the direction of the stepper controller
      n_0_L = -n_0_L;
    }
    else PORTD |= 0b00100000;                               //Set output 5 high for a forward direction of the stepper motor
    PORTD |= 0b00010000;                                    //Set output 4 high to create a pulse for the stepper controller
  }
  else if(n_L==1)PORTD &= 0b11101111;                       //Set output 4 low because the pulse only has to last for 20us 
  n_L ++;

  // Right Motor
  if(n_R==0 || n_R>=n_0_R){                                 //If the number of loops is larger then the n_memory variable
    n_R = 0;                                                //Reset the n variable
    n_0_R = n0_memry_R;
    if(n_0_R<0){
      PORTD |= 0b00001000;                                  //Set output 3 low to reverse the direction of the stepper controller
      n_0_R = -n_0_R;
    }
    else PORTD &= 0b11110111;
    PORTD |= 0b00000100;                                    //Set output 2 high to create a pulse for the stepper controller
  }
  else if(n_R==1)PORTD &= 0b11111011;                       //Set output 2 low because the pulse only has to last for 20us 
  n_R ++;
}
