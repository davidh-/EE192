#include "TSL1401.h"

/* Macro */
#define TAOS_SI_HIGH    TAOS_SI = 1
#define TAOS_SI_LOW     TAOS_SI = 0
#define TAOS_CLK_HIGH   TAOS_CLK = 1
#define TAOS_CLK_LOW    TAOS_CLK = 0

/* Constructor */
TSL1401::TSL1401(PinName s, PinName c, PinName a )
{

    SI = s;
    CLK = c;
    A0 = new AnalogIn( a );
}
/* Destructor */
TSL1401::~TSL1401()
{
    if( A0 != NULL)
    {
        delete  A0;
    }
}
/* Image Caputure */
int *TSL1401::Capture( int LineStart, int LineStop)
{
    int i;
    DigitalOut TAOS_SI(SI);
    DigitalOut TAOS_CLK(CLK);
    
    Max = 0,Min = 70000;
    TAOS_SI_HIGH;
    TAOS_CLK_HIGH;
    TAOS_SI_LOW;
    ImageData[0] = 0;
    TAOS_CLK_LOW;
    for(i = 1; i < LineStart; i++) {        
        TAOS_CLK_HIGH;      
        TAOS_CLK_LOW;
    }
    for(i = LineStart; i < LineStop; i++) {                  
        TAOS_CLK_HIGH;
        ImageData[i] = A0->read_u16();  // inputs data from camera (one pixel each time through loop) 
        TAOS_CLK_LOW;
            
        if(Max < ImageData[i]){
            Max = ImageData[i];
        }           
        if(Min > ImageData[i]){
            Min = ImageData[i];
        }
            
    }
    return ImageData;
}


    
