#include <PinChangeInt.h>
#include <Servo.h>
#include <PID_v1.h>
#include <MsTimer2.h>
 
//RECEIVER OUTPUT TO UNO INPUT
#define ch1 8 
#define ch2 9 
#define ch3 10  
#define ch4 11 
#define ch5 12  
#define ch6 13 // we could choose any pin 

//UNO OUTPUT TO APM INPUT 
Servo apm1; 
Servo apm2;
Servo apm3;
Servo apm4;
Servo apm5;

float t_int = 0.2; //digital filter parameters (common to receiver PWM filtering and sensor filtering
float tau_f = 0.2; // " " " "
float h_ms = 50; // init read sensor data (cm)
float h_us = 50; 
float h_fms_prev = 0; //init previous sensor data
float h_fus_prev = 0; 
volatile int pls2 = 0; //flagged by timer for digital filter of both sensors 
volatile int pls3 = 0; //flagged by timer for PWM signal filtering from receiver
float h_alp = 0.5; //init alpha value of sigmoid
double duration=0;
int alt = 0;

const int trigPin = A1; //PIN ALLOTMENT for sensors and LEDs
const int echoPin = A2;
const int maxb_analPin = A0;
const int ledAHpin=A3;
const int ledFMpin=A4;
const int ledGround=A5;


double distance=50; //initialise distance variable (output of sensor fusion)

volatile float pul_ch[] = {0,0,0,0,0,0}; //stores PWM values read from receiver 6 channels
volatile float pul_chef[] = {0,0,0,0,0,0}; //stores exponsmooth filtered PWM values read from receiver 6 channels
volatile float pul_ch_low[] ={0,0,0,0,0,0}; // low time 
volatile float pul_ch_high[] ={0,0,0,0,0,0}; // high time
volatile int pls_ch[] = {0,0,0,0,0,0}; //pls sense boolean
volatile float pul_chprev[] = {0,0,0,0,0,0}; 

//INITIALISE KALMAN FILTER PARAMETERS

const float ru = 0.25; //US variance
float pu = 10; //initialise US estimate variance 
float ku = 0.5; //intialise US kalman gain

const float rm = 0.15; //MS variance
float pm = 10; //initialise MS estimate variance 
float km = 0.5; //intialise MS kalman gain

const float rc = 0.15; //Channel variance
float pc = 10; //initialise Channel estimate variance 
float kc = 0.5; //intialise Channel kalman gain

float h_kfms=50; //initialise MS kalman filtered output
float h_kfus=50; //initialise US kalman filtered output  



//**************************************************************//
//                     PID CONTROL PARAMETERS                   //
//**************************************************************//

float kp_pitch=6; // order of mag = 
float ki_pitch=0.000;  //order of mag = 
float kd_pitch=2;  //order of mag =
float dist_error=0;
float prev_dist_error=0;
double dist_ref=75; //reference distance of object

//int n=0;int d=0; //were used to check sample time (loop time)

//unsigned long lastTime;
//float error_sum,last_error; //were previously used for manually written PID loop
double U_pitch=0; //control input
double U_pitch_ref=1500; //  is the PWM value for zero pitch and zero pitch rate (drift eliminated)
PID quad(&distance, &U_pitch, &dist_ref,kp_pitch,ki_pitch,kd_pitch,DIRECT); //PID control library has been used here to init 'quad' PID object

//****************************************************************//
//                   MAIN CODE STARTS                            //
//***************************************************************//



void setup() {
   Serial.begin(250000);  
  noInterrupts();
  pinMode(2,OUTPUT); //just needed an extra high pin to power sensors and stuff
  digitalWrite(2,HIGH); //turn off US sensor please
  quad.SetMode(MANUAL); //manual mode first, always
  APM_setup();
  sensors_setup();
  RC_setup();
  led_setup();
  MsTimer2::set(200, flag); // 200ms period
  MsTimer2::start();
  interrupts();//keep all setup code within noInterrupts and interrupts
}


void loop() {
  
  for (int v=0;v<6;v++) //this is to store the previous PWM signal value read from RC
  {
    pul_chprev[v]=pul_chef[v];
  }
 read_RC(); //read PWM value from receiver
 
 if(pul_ch[5]>1750 && pul_ch[4]>1750) //ALT HOLD mode + detectionON conditional
  {
    ledAH(1);
    if (pls2 == 1)
    {
    distance=(double)get_distance();
    //Serial.print(distance);
    };
    
    if(abs(distance-dist_ref)<35)
    { 
      ledFM(1);
      Follow_me(); //FOLLOW ME mode conditional
    }
    else
    {
      Raw_control(); 
    };
  }
  else
  {
    Raw_control(); //raw control
  }
 
}

   
//******************************************************//
//                  END OF MAIN CODE                    //
//******************************************************//


float get_distance() //performs individual sensor data digital filtering, sensor fusion using sigmoid and returns 'distance'
{
    pls2 = 0;   
  
    h_ms = analogRead(maxb_analPin); //Maxbotix read
    if(h_ms>150)
    {
      h_ms=150.0;
    }; 
    h_kfms=kalmanm(h_kfms,h_ms); // performs KF
    alt=1; //alternator variable used when switch case method was used
      
    digitalWrite(2,LOW);
   /*
    //US trig and calc
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH, 40000);
    // Calculating the distance
    h_us = duration * 0.034 / 2;
    h_kfus=kalmanu(h_kfus,h_us); // performs KF  
    alt=0;
    */

//Serial.print("MS");
//Serial.print(h_kfms);

//Serial.print("  US");
//Serial.println(h_kfus);

        

    //SENSOR FUSION
    float h_dist = 1 * h_kfms + (0) * h_kfus; //calculate  fused distance
    h_alp = 1 / (1 + exp(0.6 * (30 - h_dist))); //update alpha value of sigmoid k=0.5, dref=30 was the initial setting

        
    return h_dist;       
}


void Raw_control()
{
 ledAH(0);
 ledFM(0);

  channel_filter();
  apm1.writeMicroseconds(pul_chef[0]); 
  apm2.writeMicroseconds(pul_chef[1]); 
  apm3.writeMicroseconds(pul_chef[2]);
  apm4.writeMicroseconds(pul_chef[3]); 
  apm5.writeMicroseconds(pul_chef[4]);
  
   
}



void Follow_me()
{
//Raw_control();//just for safety as of now
//Uncomment the below code to test control algorithm

 pidcalc();

 channel_filter();
 apm1.writeMicroseconds(pul_chef[0]); 
 apm2.writeMicroseconds(pul_chef[1]+U_pitch); //channel value used instead of U_pitch_ref because on-line drift elimination changes upitch ref
 apm3.writeMicroseconds(pul_chef[2]);
 apm4.writeMicroseconds(pul_chef[3]); 
 apm5.writeMicroseconds(pul_chef[4]);
} 




void channel_filter()
{
 
   for(int r=0;r<6;r++){
    pul_chef[r]=   0.002*pul_ch[r] + 0.998*pul_chprev[r]; //give very low weightage (0.005 to 0.01) to new reading (verified!)
  }

}

  

void pidcalc() //function that calculates control input value
{
   /*Basic Code for PID manually written
   //How long since we last calculated
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   //Compute all the working error variables
   double error = (double)(dist_ref - distance);
   error_sum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   //Compute PID Output
   U_pitch =float( kp_pitch * error + ki_pitch * errSum + kd_pitch * dErr);
  
  // Remember some variables for next time
   last_error = error;
   lastTime = now; */
   
   quad.SetMode(AUTOMATIC); //PID library command to set automatic mode
   quad.Compute();  //computes control input
   quad.SetOutputLimits(-60,60); //max output of U_pitch
   quad.SetControllerDirection(DIRECT); //because pitch channel is reversed
   
   
}


//****************************************************************//
//                       ALL KALMAN FILTERS                         //
//****************************************************************//


float kalmanm(float x, float meas)
{
  km=pm/(pm+rm)+ 0.4; //modified kalman gain, 0.07-0.25 was best for US, 0.135-0.4 best for MS
  if(km>1){km=1;};
  if(km<0){km=0;};
  x=x*(1-km)+(km)*meas;
  pm=(1-km)*pm;
  return x;  
}


float kalmanu(float y, float measd)
{
  ku=pu/(pu+ru)+ 0.2; //modified kalman gain, 0.07 was best for US, 0.135 best for MS
   if(ku>1){ku=1;};
  if(ku<0){ku=0;};
  y=y*(1-ku)+(ku)*measd;
  pu=(1-ku)*pu;
  return y;  
}


float kalmanc(float z, float measrd)
{
  kc=pc/(pc+rc)+ 0.2; //modified kalman gain, 0.07 was best for US, 0.135 best for MS
   if(kc>1){kc=1;};
  if(kc<0){kc=0;};
  z=z*(1-kc)+(kc)*measrd;
  pc=(1-kc)*pc;
  return z;  
}


//*****************************************************************//
//                       ALL SETUP CODE                            //
//*****************************************************************//

void RC_setup()
{
  pinMode(ch1, INPUT_PULLUP);
  pinMode(ch2, INPUT_PULLUP);
  pinMode(ch3, INPUT_PULLUP);
  pinMode(ch4, INPUT_PULLUP);
  pinMode(ch5, INPUT_PULLUP);
  pinMode(ch6, INPUT_PULLUP);
  attachPinChangeInterrupt(ch1, CH1_RISING, RISING);
  attachPinChangeInterrupt(ch2, CH2_RISING, RISING);
  attachPinChangeInterrupt(ch3, CH3_RISING, RISING);
  attachPinChangeInterrupt(ch4, CH4_RISING, RISING);
  attachPinChangeInterrupt(ch5, CH5_RISING, RISING);
  attachPinChangeInterrupt(ch6, CH6_RISING, RISING);
}

void APM_setup()
{
  apm1.attach(3);
  apm2.attach(4);
  apm3.attach(5);
  apm4.attach(6);
  apm5.attach(7);  
}

void sensors_setup() 
{
   pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(maxb_analPin, INPUT);
}

void read_RC()  //read PWM values from receiver
{
   if((pul_ch_low[0]-pul_ch_high[0]) > 0 && pls_ch[0] == 1){
    pls_ch[0] = 0;
    pul_ch[0] = (pul_ch_low[0]-pul_ch_high[0]);
    }
     if((pul_ch_low[1]-pul_ch_high[1]) > 0 && pls_ch[1] == 1){
    pls_ch[1] = 0;
    pul_ch[1] = (pul_ch_low[1]-pul_ch_high[1]);
    }
    if((pul_ch_low[2]-pul_ch_high[2]) > 0 && pls_ch[2] == 1){
    pls_ch[2] = 0;
    pul_ch[2] = (pul_ch_low[2]-pul_ch_high[2]);
    }
    if((pul_ch_low[3]-pul_ch_high[3]) > 0 && pls_ch[3] == 1){
    pls_ch[3] = 0;
    pul_ch[3] = (pul_ch_low[3]-pul_ch_high[3]);
    }
    if((pul_ch_low[4]-pul_ch_high[4]) > 0 && pls_ch[4] == 1){
    pls_ch[4] = 0;
    pul_ch[4] = (pul_ch_low[4]-pul_ch_high[4]);
    }
    if((pul_ch_low[5]-pul_ch_high[5]) > 0 && pls_ch[5] == 1){
    pls_ch[5] = 0;
    pul_ch[5] = (pul_ch_low[5]-pul_ch_high[5]);
    }

}


//*****************************************************************//
//                       COOL LED CODE                             //
//*****************************************************************//
/* No light indicates stabilise.
 *  One light ON indicates ALTHOLD
 *  Both lights ON indicates FOLLOW_ME
 */

void led_setup()
{
  pinMode(ledAHpin,OUTPUT);
  pinMode(ledFMpin,OUTPUT);
  pinMode(ledGround,OUTPUT);
  digitalWrite(ledGround, LOW);
}

void ledAH(bool m)
{
  if (m)
  {
    digitalWrite(ledAHpin,HIGH);
  }
  else
  {
    digitalWrite(ledAHpin,LOW);
  }
}

void ledFM(bool n)
{
  if (n)
  {
    digitalWrite(ledFMpin,HIGH);
  }
  else
  {
    digitalWrite(ledFMpin,LOW);
  }
}



//************************************************//
//                TIMER CODE ONLY                 //
//************************************************//



void flag()
{ 
  // Make a flag high when 20 ms are up
  pls2 = 1;
  pls3=1;

  
}


//***************************************************************//
//                       CHANNEL RISING CODE                     //
//**************************************************************//


void CH1_RISING()
{
  pul_ch_high[0] = micros();
  detachPinChangeInterrupt(ch1);
  attachPinChangeInterrupt(ch1, CH1_FALLING, FALLING);
}
 
void CH1_FALLING() {
  pls_ch[0] = 1;
  pul_ch_low[0] = micros();
  detachPinChangeInterrupt(ch1);
  attachPinChangeInterrupt(ch1, CH1_RISING, RISING);  
  }

void CH2_RISING()
{
  pul_ch_high[1] = micros();
  detachPinChangeInterrupt(ch2);
  attachPinChangeInterrupt(ch2, CH2_FALLING, FALLING);
}
 
void CH2_FALLING() {
  pls_ch[1] = 1;
  pul_ch_low[1] = micros();
  detachPinChangeInterrupt(ch2);
  attachPinChangeInterrupt(ch2, CH2_RISING, RISING);  
  }

 void CH3_RISING()
{
  pul_ch_high[2] = micros();
  detachPinChangeInterrupt(ch3);
  attachPinChangeInterrupt(ch3, CH3_FALLING, FALLING);
}
 
void CH3_FALLING() {
  pls_ch[2] = 1;
  pul_ch_low[2] = micros();
  detachPinChangeInterrupt(ch3);
  attachPinChangeInterrupt(ch3, CH3_RISING, RISING);  
  }

 void CH4_RISING()
{
  pul_ch_high[3] = micros();
  detachPinChangeInterrupt(ch4);
  attachPinChangeInterrupt(ch4, CH4_FALLING, FALLING);
}
 
void CH4_FALLING() {
  pls_ch[3] = 1;
  pul_ch_low[3] = micros();
  detachPinChangeInterrupt(ch4);
  attachPinChangeInterrupt(ch4, CH4_RISING, RISING);  
  }
    
void CH5_RISING()
{
  pul_ch_high[4] = micros();
  detachPinChangeInterrupt(ch5);
  attachPinChangeInterrupt(ch5, CH5_FALLING, FALLING);
}
 
void CH5_FALLING() {
  pls_ch[4] = 1;
  pul_ch_low[4] = micros();
  detachPinChangeInterrupt(ch5);
  attachPinChangeInterrupt(ch5, CH5_RISING, RISING);  
  }

void CH6_RISING()
{
  pul_ch_high[5] = micros();
  detachPinChangeInterrupt(ch6);
  attachPinChangeInterrupt(ch6, CH6_FALLING, FALLING);
}
 
void CH6_FALLING() {
  pls_ch[5] = 1;
  pul_ch_low[5] = micros();
  detachPinChangeInterrupt(ch6);
  attachPinChangeInterrupt(ch6, CH6_RISING, RISING);  
  }
