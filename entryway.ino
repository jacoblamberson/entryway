/* 
	FPS_Enroll.ino - Library example for controlling the GT-511C3 Finger Print Scanner (FPS)
	Created by Josh Hawley, July 23rd 2013
	Licensed for non-commercial use, must include this license message
	basically, Feel free to hack away at it, but just give me credit for my work =)
	TLDR; Wil Wheaton's Law

	This sketch will attempt to identify a previously enrolled fingerprint.
*/

#include "FPS_GT511C3.h"
#include "SoftwareSerial.h"

// Hardware setup - FPS connected to:
//	  digital pin 4(arduino rx, fps tx)
//	  digital pin 5(arduino tx - 560ohm resistor fps tx - 1000ohm resistor - ground)
//		this brings the 5v tx line down to about 3.2v so we dont fry our fps

const int OPEN_LENGTH = 3000;

const int FPS_TX = 1;
const int FPS_RX = 0;

const int MOTION_INPUT = 3;
const int MOTOR_OUTPUT = 2;
const int STATUS_LED = 4;

bool elapsed(unsigned int delta, unsigned long prev, unsigned long current = millis()){
  // Takes care of ulong overflow
  // FIXME - I'd prefer a compile-time expression instead of the hard-coded ulong max
  return current >= prev + delta || (current < prev && 4294967295L - prev + current >= delta);
}

// Supports actuation and timed activation
class AsyncOut {
  public:
  
    AsyncOut(int pin_){
      pin = pin_;
      state = false;
      //lastTime = 0; //unnecessary
      timing = false;
    }

    void init(){
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }

    void actuate(bool state_){
      state = state_;
      if(state){
        digitalWrite(pin, HIGH);
      }else{
        digitalWrite(pin, LOW);
      }
    }

    void activate(unsigned int length_){
      lastTime = millis();
      actuate(true);
      deltaTime = length_;
      timing = true;
    }

    void process(unsigned long currentTime_ = millis()){
      if(timing && elapsed(deltaTime, lastTime, currentTime_)){
        timing = false;
        actuate(false);
      }
    }

  private:
    
    int pin;
    bool state;
    bool timing;
    unsigned long lastTime;
    unsigned int deltaTime;
};

class In {
  public:
    In(int pin_){
      pin = pin_;
    }

    void init(){
      pinMode(pin, INPUT);
    }

    bool getState(){
      return digitalRead(pin) == HIGH; // FIXME - Cull pin polling for profit
    }

    void process(){
      return;
    }
  
  private:
    int pin;
};

class Entryway {
  public:
    Entryway(int txPin_, int rxPin_, int motorPin_, int motionPin_, int ledPin_) :
      motor(motorPin_),
      led(ledPin_),
      motion(motionPin_),
      fps(txPin_, rxPin_)
    {
      return;
    }

    void init(){
      motor.init();
      led.init();
      motion.init();
      fps.Open();
      fps.SetLED(false);
    }
    
    bool verifyFinger(){
      if (fps.IsPressFinger())
      {
        fps.CaptureFinger(false);
        int id = fps.Identify1_N();
        if (id <200)
        {
          digitalWrite(13, HIGH);
          delay(500);
          digitalWrite(13, LOW);
          return true;
        }
      }
      return false;
    }

    void process(){
      motion.process();
      led.process();
      motor.process();

      if(motion.getState()){
        led.activate(500);
        fps.SetLED(true);
        //motor.activate(OPEN_LENGTH);
        if(verifyFinger()){
          motor.activate(OPEN_LENGTH);
        }
        fps.SetLED(false);
      }
    }

  private:
    FPS_GT511C3 fps;
    AsyncOut motor;
    AsyncOut led;
    In motion;
};

Entryway entryway(FPS_TX, FPS_RX, MOTOR_OUTPUT, MOTION_INPUT, STATUS_LED);

void setup()
{
	//Serial.begin(9600);
	delay(100);
  entryway.init();
}

void loop()
{
	entryway.process();
	delay(100);
}






