#ifndef timer_h
#define timer_h
#define timer_h_Version 12 //0.0-25.5 = 0-255
//Changed check(int --> unsigned int)
//Added checkBig(unsigned long) for big numbers :D
//added active time to .check
/*
V1.1 made reset() public
V1.2 fixed overflow bug of .check and .checkBig, used to compare (timer + WantedDelay <= millis()) and (timer + WantedDelay + activeTime <= millis()), which is buggy after ~50 days
*/

class timer {
private:
   unsigned long timer;
public:
   void reset() {
     timer = millis();
   }


   //Check if the timer is up, only gets reset, when this function is called.
   //Active time is so the .check stays active for this duration
  bool check(unsigned int WantedDelay, unsigned int activeTime = 0) { //0-65.000
    if(millis() < timer)
      timer::reset();

    if(millis() - timer >= WantedDelay) {
      if((millis() - (timer + activeTime)) >= WantedDelay) 
        timer::reset(); //Only reset, when the active time is run out
      return 1;
    }
    else return 0;
  }

  bool checkBig(unsigned long WantedDelay) { //0-4.200.000.000
    if(millis() < timer)
      timer::reset();
    
    if ((millis() - timer) >= WantedDelay) {
      timer::reset();
      return 1;
    }
    else return 0;
  }
};
#endif