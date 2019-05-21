#define encoder0PinA  2
#define encoder0PinB  3
#define PWM 5
//#define direction_input A0
#define sp_in A0 
#define dir1 6
#define dir2 7
//#define max_rpm 290
int sp = 0;
volatile signed int encoder0Pos = 0;
int v = 0;
int s = 0;
int e = 0;
int cnt = 0;
int number_of_cycles = 12;
int Dir= 1;
int s_dir = -1;
int factor;
float err, err_prev, total_err, changeErr;
float kp, ki, kd;
int desired;
int pidTerm, pidTerm_scaled;

void PIDcalculation(){
  desired = ((400 * analogRead(sp_in))/1023) - 200;
  err = desired - v;
  
  changeErr = err - err_prev; // derivative term
  total_err += err; //accumalate errors to find integral term
  pidTerm = (kp * err) + (ki * total_err) + (kd * changeErr);//total gain
  pidTerm = constrain(pidTerm, -255, 255);//constraining to appropriate value
  pidTerm_scaled = abs(pidTerm);//make sure it's a positive value

  err_prev = err;
}
void setup() {
  //////////speed 
  
  //factor =(sp*255)/(max_rpm);
  //pinMode(PWM, OUTPUT);
  total_err = 0;
  err_prev = 0;
  kp = 0.08;
  ki = 0.4;
  kd = 0.002;
  //analogWrite(PWM, factor);
  pinMode(dir1,OUTPUT);
  //digitalWrite(direction_output1, HIGH);
  pinMode(dir2,OUTPUT);
  //digitalWrite(direction_output2, LOW);
  /////////////////////////////
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor

  attachInterrupt(0, doEncoder, RISING);  // encoder pin on interrupt 0 - pin 2
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

void loop() {
  // do some stuff here - the joy of interrupts is that they take care of themselves
  PIDcalculation();
   if (desired > 0 && v != desired) {
    digitalWrite(dir1, LOW);// Forward motion
    digitalWrite(dir2, HIGH);
  } else if (desired < 0 && v != desired) {
    digitalWrite(dir1, HIGH);//Reverse motion
    digitalWrite(dir2, LOW);
  }

  analogWrite(PWM, pidTerm_scaled);
 
}

void doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  cnt = cnt + 1;
  cnt = cnt % number_of_cycles;

  if (cnt == 0)
    s = millis();
 if (cnt == number_of_cycles - 1)
 {
  e = millis();
  int l = 60000;
  v = l / (e - s);    
  Serial.println(v, DEC); 
 }
   if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    v = v;
  } else {
    v = -v;
  }
  Serial.println (encoder0Pos, DEC);
}

/* See this expanded function to get a better understanding of the
   meanings of the four possible (pinA, pinB) value pairs:
*/
void doEncoder_Expanded() {
  if (digitalRead(encoder0PinA) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(encoder0PinB) == LOW) {  // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else if(digitalRead(encoder0PinA) == LOW)                                        // found a high-to-low on channel A
  {
    if (digitalRead(encoder0PinB) == LOW) {   // check channel B to see which way
      // encoder is turning
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }

  }
  Serial.println (encoder0Pos, DEC);          // debug - remember to comment out
  // before final program run
  // you don't want serial slowing down your program if not needed
}
