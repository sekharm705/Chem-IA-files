
//State values:
//0: Idle, before fire
//1: Firing
//2: After fire, return avg thrust

//BASIC ALGORITHM:
//This program runs in a series of states: 0,1,2.
//
//State 0: Active when the motor is still on the pad, and has not fired. The last n values are updated (I used n = 6 for my experiment, but triggerDepth sets this).
//Every reading, the program checks to see if all n values are on an upward trend. If the values are purely increasing, then that signifies that the motor has been
//ignited and the program moves to state 1.
//
//State 1: This is active when the motor is firing. On every reading, the value is read, added to a running sum, and a running count of the total readings is incremented.
//This makes it easy to compute an average value. Additionally, the last n readings (I used n = 3 for my experiment, but it is set by endingDepth) are recorded. If all three
//of these values are negative and in proximity to each other (off by 0.1 grams), then the rocket motor has stopped firing and the program moves to State 2.
//
//State 2: In this stage, some final adjustments are made to account for the error in the reported value since some propellant burns off, creating a difference in weight.
//The final average thrust is then printed.





#include <Q2HX711.h> //HX711 weight cell sensor. Open source library
const byte hx711_data_pin = A4;
const byte hx711_clock_pin = A2;
const int triggerDepth = 6;
const int endingDepth = 3;
const double scale = 52.675;

Q2HX711 hx711(hx711_data_pin, hx711_clock_pin);
double err;
float endingNums[endingDepth];
int state;
float prevValues[triggerDepth];
int runningSum, runningCount = 0;

void setup() {
  Serial.begin(250000);
  err = (hx711.read() / 100.0);
  state = 0;
  for (int i = 0; i < triggerDepth; i++) {
    prevValues[i] = triggerDepth - i;
  }
}

void loop() {

  double temp2 = ((hx711.read() / 100.0) - err) / scale;
  String temp = String(temp2);
  float currentValue = (temp.substring(0, temp.length() - 1)).toFloat();

  if (state == 0) {
    
    for (int i = 0; i < triggerDepth - 1; i++) {
      prevValues[i] = prevValues[i + 1];
    }
    prevValues[triggerDepth - 1] = currentValue;

    //    String out = "";
    //    for (int i = 0; i < triggerDepth; i++) {
    //      out += String(prevValues[i]);
    //      out += "\t";
    //    }
    //    Serial.println(out);
    
    bool check = true;
    for (int i = 0; i < triggerDepth - 1; i++) {
      if (!(prevValues[i] < prevValues[i + 1])) {
        check = false;
      }
    }
    
    if (check) {
      state = 1;
      for (int i = 0; i < triggerDepth; i++) {
        runningSum += prevValues[i];
      }
      runningCount += triggerDepth;
      Serial.println("Moved to state 1");

      for (int i = 0; i < endingDepth; i++) {
        endingNums[i] = 10;
      }
    }
    
  } else if (state == 1) {
    
    runningSum += currentValue;
    runningCount += 1;
    
    for (int i = 0; i < endingDepth - 1; i++) {
      endingNums[i] = endingNums[i + 1];
    }
    
    endingNums[endingDepth - 1] = currentValue;

    bool check = true;
    
    for (int i = 0; i < endingDepth - 1; i++) {
      if (!((sqrt(pow((endingNums[i] - endingNums[i + 1]), 2)) < 0.2) && endingNums[i] < 0 )) {
        check = false;
      }
    }
    
    if (check) {
      state = 2;
      for (int i = 0; i < endingDepth; i++) {
        runningSum -= endingNums[i];
      }
      runningCount -= endingDepth;
    }

  } else if (state == 2) {
    
    float lostWeight = -endingNums[endingDepth - 1];
    
    //This assumes a linear decrease of mass. The actual extra force of the fuel, due to the weight drop because of the lost fuel, at any given time
    //is therefore (recordedForce + time * (lostWeight/(totalTime)). Therefore, the total impulse of the rocket motor is the integral of this expression
    //with respect to time, which is ((time * averageForce) + 0.5 * (lostWeight/totalTime) * time^2). Our bounds are (0, totalTime), which simplifies the
    //expression to (totalTime * averageForce + 0.5 * totalTime * lostWeight).
    
    float actualImpulse = runningSum + (lostWeight * runningCount);
    float finalAverageThrust = actualImpulse / runningCount;
    Serial.println("The avg thrust is " + String(finalAverageThrust / 98) + " Newtons.");
    state = 3; //just to make the program stop

  }


  delay(5); //for stability, time control. 5 milliseconds.
}
