#ifndef _ESC_H_
#define _ESC_H_

/** ESC class used to controll standard Electronic Speed Controllers for brushless motors of RC models.
 *  Simple usage example:
 *  @code
 *
#include "mbed.h"
#include "esc.h" //include the ESC class


int main()
{
    ESC esc1(p26); //declare the ESC as connected to pin p26.
    float throttle_var = 0.5 //that means 50%.
        
    while(1)
    {
        //... update throttle_var ...
    
        esc1 = throttle_var; //memorize the throttle value (it doesn't send it to the ESC).
        
        //... do whatever you want - e.g. call esc1.setThrottle(throttle_var) again ...
        
        esc1(); //actually sets the throttle to the ESC.
        
        wait_ms(20);  //20ms is the default period of the ESC pwm; the ESC may not run faster.
    }
}
 * @endcode
 * 
 * Another example:
 * 
 * @code
 *
#include "mbed.h"
#include "esc.h"
 
ESC esc1(PTD4);
ESC esc2(PTA12);
ESC esc3(PTA4);
ESC esc4(PTA5);
 
Serial pc(USBTX, USBRX);    // tx, rx
 
int main() {
    
    char c;
    int var = 0;
 
    while(1) {
        c = pc.getc();
        
        if (c == 'u') {
            if (var < 100) {
                var++;
            }
            if (esc1.setThrottle(var) && esc2.setThrottle(var) && esc3.setThrottle(var) && esc4.setThrottle(var)) {
                printf("%i\r\n", var);
            }
        }
        else if (c == 'd') {
            if (var > 0) {
                var--;
            }
            if (esc1.setThrottle(var) && esc2.setThrottle(var) && esc3.setThrottle(var) && esc4.setThrottle(var)) {
                printf("%i\r\n", var);
            }
        }
        else if (c == 'r') {
            var = 0;
            if (esc1.setThrottle(var) && esc2.setThrottle(var) && esc3.setThrottle(var) && esc4.setThrottle(var)) {
                printf("%i\r\n", var);                
            }
        }
        
        esc1.pulse();
        esc2.pulse();
        esc3.pulse();
        esc4.pulse();
        wait_ms(20);  // 20ms is the default period of the ESC pwm
    }
}
 * @endcode
 */
 
class ESC
{    
  private:
  
    PwmOut esc;
    int period;
    int throttle;
    
  public:
  
    /** Initializes the PwmOut for minimum throttle (1000us).
     *  @param pwmPinOut is the pin connected to the ESC.
     *  @param period is the PWM period in ms (default is 20ms).
     */
    ESC (const PinName pwmPinOut, const int period=20);
    
    /** Sets the throttle value without output (see pulse()).
     *  @param t in in the range [0.0;1.0]
     *  @return true if throttle value is in range; false otherwise.
     */
    bool setThrottle (const float t);
    ///Alias of setThrottle(float)
    bool operator= (const float t);
    
    /** Get the last setted throttle value
     *  @return throttle in range [0.0-1.0].
     */
    float getThrottle () const;
    ///Alias of getThrottle()
    operator float () const;
    
    /** Output the throttle value to the ESC.
     */
    void pulse ();
    ///Alias of pulse()
    void operator() ();
 };

#endif