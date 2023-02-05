#ifndef MOTORDRIVER_HEADER
#define MOTORDRIVER_HEADER

class Motor; //Defines Class Motor
class MotorDriver{
    public:
        MotorDriver();
        ~MotorDriver();
        void EnableMotorDriver();
        void Enable();
        void Disable();
        void SetTimeout(int);
        void SetMotors(Motor*, Motor*);
        int GetDriverStatus();
        bool isEnable = false;
        int timeout;
        int error_code;
        Motor* motor1;
        Motor* motor2;
};

#endif