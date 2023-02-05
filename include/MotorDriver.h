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
        int enable = false;
        int timeout;
    private:
        Motor* motor1;
        Motor* motor2;
};

#endif