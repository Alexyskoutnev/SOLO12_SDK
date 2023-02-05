#include "MotorDriver.h"


class MotorDriver;
class Motor {
    Motor();
    void SetCurrentReference(double);
    void SetVelocityReference(double);
    void SetPositionReference(double);
    void SetPositionOffset(double);
    void SetKp(double);
    void SetKd(double);
    void SetMaxAmps(double);
    void SetDriver(MotorDriver *driver);
    void Print();
    void Enable();
    void Disable();

    bool isReady();
    bool isEnabled();
    bool IndexDectected();
    bool GetIndexToggleBit();
    
    double getCurrent();
    double getVelocity();
    double getPosition();
    double getOffset();

    MotorDriver *driver;

    private:
        double position;
        double velocity;
        double current;
        double offsetPosition;
         
        bool isEnabledFlag;
        bool isReadyFlag;
        bool indexToggleBitFlag;
        bool indexDectectedFlag;

        double position_cmd;
        double velocity_cmd;
        double current_cmd;
        double current_max; 
        double kp;
        double kd;

};