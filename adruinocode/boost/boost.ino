
#include <PID_v1.h>
int pwmPin = 6; 
int analogPin = 0;

unsigned long previousMillis = 0; // previous time
const unsigned long interval = 0.1; // interval in milliseconds

class PIController {
public:
    PIController(double kp, double ki, double setpoint)
        : kp_(kp), ki_(ki), setpoint_(setpoint), integral_(0.0), last_error_(0.0)
    {
    }

    double calculate(double input, double dt) {
        double error = setpoint_ - input;
        integral_ += error * dt;
        double derivative = (error - last_error_) / dt;
        last_error_ = error;
        return kp_ * error + ki_ * integral_;
    }

private:
    double kp_;
    double ki_;
    double setpoint_;
    double integral_;
    double last_error_;
};



void setup() {

 /*  pinMode(pwmPin,OUTPUT); */
  analogWrite(pwmPin, 10);

  
   
  TCCR0B = (TCCR0B & 0b11111000) | 0x01; // 62KHz @see http://playground.arduino.cc/Main/TimerPWMCheatsheet
  Serial.begin(9600);

}


void loop() {

   // create a PI controller with kp=1.0, ki=0.5, and setpoint=12.0

PIController controller(11.5, 0.01, 15);



int analog_input = analogRead(analogPin);


 unsigned long currentMillis = millis(); // current time
  
 // check if the interval has elapsed
  if (currentMillis - previousMillis >= interval) {
    // do something here
    float input = analog_input * (5 / 1023.0)*10;
    double output = controller.calculate(input, interval);
    previousMillis = currentMillis; // save the current time

     if(output<=0){

      output=0;


    };

     if(output>=255){

      output=255;


    };

    if(output>0&&output<255){

       output = controller.calculate(input, interval);

    }; 

    analogWrite(pwmPin, output);


  } 




   



  
}