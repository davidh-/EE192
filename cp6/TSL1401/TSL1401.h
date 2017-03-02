#ifndef TSL1401_H
#define TSL1401_H

#include "mbed.h"
#include "USBSerial.h"
extern USBSerial serial;
class   TSL1401
{
    public:
        TSL1401(PinName s, PinName c, PinName a );
        ~TSL1401();
        int *Capture( int LineStart, int LineStop);     /*caputure image */

    
        int   ImageData[128];    /* カメラの値 */
        int   Max,Min;           /*カメラ読み取り最大値、最小値 */
    private:
        PinName SI;
        PinName CLK;
        AnalogIn *A0;
 };
#endif