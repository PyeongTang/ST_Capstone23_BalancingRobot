#include        "mbed.h"

#define         POS_MAP_MIN   -360
#define         POS_MAP_MAX   360
#define         M_PI          3.14159265358979323846

class PointDetector {
    public : 
        PointDetector           (PinName x_encA, PinName x_encB, PinName y_encA, PinName y_encB);
        
        void printCount         (Serial* pc_, bool plot = false);

        int getPositionX        ();
        int getPositionY        ();

        int getDistance         ();
        int getTheta            ();

    private :
        void xencoderRead       ();
        void yencoderRead       ();

        int calculateDistance   (int _x, int _y);
        int calculateAngle      (int _x, int _y);
        int mapCount            (int countValue);
        template <class T> T map(T x, T in_min, T in_max, T out_min, T out_max);

        InterruptIn     xencA;
        DigitalOut      xencB;

        InterruptIn     yencA;
        DigitalOut      yencB;

        int xencoderCount   = 0;
        int yencoderCount   = 0;

        int x_pos           =   mapCount(xencoderCount);
        int y_pos           =   mapCount(yencoderCount);

        int distance        =   calculateDistance(x_pos, y_pos);
        int theta           =   calculateAngle(x_pos, y_pos);
};