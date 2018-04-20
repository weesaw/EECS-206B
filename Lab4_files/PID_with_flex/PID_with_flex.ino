
int incomingByte = 0;
float input = 0.0;
float reference, error;
float Kp = 0.3, Ki = 1, Kd = 20;
float a, b, c;

float prev_error = 0.0, prev_prev_error = 0.0;

const int FLEX_PIN = A0; // Pin connected to voltage divider output
const float VCC = 5.078; // Measured voltage of Ardunio 5V line
const float R_DIV = 46872.0; // Measured resistance of 3.3k resistor
const float STRAIGHT_RESISTANCE = 13500; // resistance when straight
const float BEND_RESISTANCE = 26840.495; // resistance at 90 deg


int count = 0;
int delta_t = 10;
float t, delayTime;

void setup(){

  pinMode(3, OUTPUT);
  // set mosfet gate pin to an output
  
  pinMode(FLEX_PIN, INPUT);
  
  reference = STRAIGHT_RESISTANCE;

  a = Kp + Ki*delta_t/2 + Kd/delta_t;
  b = -Kp + Ki*delta_t/2 - 2*Kd/delta_t;
  c = Kd/delta_t;
  
  Serial.begin(9600);
  // opens serial port, sets data rate to 9600 bps
}


void loop(){

  t = millis();
  int flexADC = analogRead(FLEX_PIN);
  
  float flexV = flexADC * VCC / 1023.0;
  
  float flexR = R_DIV * (VCC / flexV - 1.0);
          
//Read the wanted angle
  if (Serial.available() > 0)
  
  {
    //read the incoming entered byte:                                                                                                                
    incomingByte = Serial.parseInt();
    Serial.print("I received: ");
    Serial.println(incomingByte);

  }
   
   incomingByte = min(incomingByte, 90);
   reference = map(incomingByte, 0.0, 90.0, STRAIGHT_RESISTANCE, BEND_RESISTANCE);
   
   error = reference - flexR;
   input += a*error + b*prev_error + c*prev_prev_error;
   prev_error = error;
   prev_prev_error = prev_error; 

   input = min(input, 100);
   input = max(0, input);


   if (count >= 100)
   {
     Serial.print("The error is: ");
     Serial.println(error);
     Serial.print("The input is: ");
     Serial.println(input);
     count = 0;
   }
    
  analogWrite(3, input); //write the user entered value(0-254)to pwm outdput

  count++;

  delayTime = t + delta_t - millis();
  if(delayTime>0)
  {
    //Serial.println("Yahoo!");
    delay(delayTime);
  }//You may change this to be whatever frequency you want
 }
 
