// MarioEMG - BioAmp EXG Pill
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 BigBrainRobos
// Copyright (c) 2021 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <Servo.h>
#define SampleRate 500 //Sampling frequency for bio-amp EXG Pill
#define BaudRate 9600  //Set desired Baudrate
#define inputPin A0
#define BufferSize 128
#define positions 5   //Change this to increase number of positions to be stored
#define BUTTON_PIN 6  // Pin on which button is connected

Servo s1 ,s2, s3;

int spos[3]={90,70,50};                              //initial position of the three servos
int circularBuffer[BufferSize];
int index, sum,k=0,envelop=0;
int pos[positions][3];                               //Array for storing positions
bool flag=false, fwd=false, set=true;

void setup() {
	// Serial connection begin
 Serial.begin(BaudRate);
 pinMode(BUTTON_PIN,INPUT_PULLUP);
 s1.attach(9);
 s2.attach(10);
 s3.attach(11);
 
 Serial.println("MARIO EMG");
 Serial.println("Set The Positions of joints and press Enter....");
 
 ///////////////////////////////////////////////////////////// STORE POSITIONS USING POTENTIOMETERS//////////////////////////////////////////////////////////
 
 for(int i=0;i<positions;i++){
  if(i<positions){
    set=true;
  }
  Serial.print("Enter the Position of Joint ");
  Serial.print(i+1);
  Serial.println("...");
  while(set){
    spos[0]=map(analogRead(A2),0,1023,0,180);
    spos[1]=map(analogRead(A3),0,1023,0,180);
    spos[2]=map(analogRead(A4),0,1023,0,180);
    s1.write(spos[0]);
    s2.write(spos[1]);
    s3.write(spos[2]);
    if(digitalRead(6)==LOW){
      pos[i][0]=spos[0];
      pos[i][1]=spos[1];
      pos[i][2]=spos[2];
      Serial.print("Servo Positions of Position ");
      Serial.print(i+1);
      Serial.println(".....");
      Serial.println(pos[i][0]);
      Serial.println(pos[i][1]);
      Serial.println(pos[i][2]);
      delay(800);
      set=false;
    }
  }
}
  Serial.println("Now flex Your arm To move through set positions");
}

void loop() {
  while(1){
    static unsigned long past = 0;
    unsigned long present = micros();
    unsigned long interval = present - past;
    past = present;
    ////////////////////// Run timer////////////////////////
    static long timer = 0;
    timer -= interval;

  // Sample
  if(timer < 0){
    timer += 1000000 / SampleRate;
    float sensorValue = analogRead(inputPin);
    float EMGSignal = EMGFilter(sensorValue);
    envelop= getEnvelop(abs(EMGSignal));
    Serial.println(envelop);
  }
  if(envelop<3){
    fwd=true;
  }
  if((envelop>8)&&fwd){
     break;
  }
 }
  s1.write(pos[k][0]); 
  s2.write(pos[k][1]);
  s3.write(pos[k][2]);
  k++;
  if(k==positions){
    k=0;
  }
  fwd=false;
  delay(1000);
}

////////////////////////// Envelop detection algorithm///////////////////////////////////

int getEnvelop(int absEMG){
	sum -= circularBuffer[index];
	sum += absEMG;
	circularBuffer[index] = absEMG;
	index = (index + 1) % BufferSize;
	return (sum/BufferSize) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py


float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
