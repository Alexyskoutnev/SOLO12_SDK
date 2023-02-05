

class Motor; //Define Class Motor
class MotorDriver{
    public:
        MotorDriver();
        ~MotorDriver();
        void EnableMotorDriver();
        void MotorDriver::Enable();
        void MotorDriver::Disable();
        void MotorDriver::SetTimeout(int);
        // void DisableMotorDriver();
        void MotorDriver::SetMotors(Motor*, Motor*);
        int GetDriverStatus();


        int enable = false;
        int timeout;
        


    private:
        Motor* motor1;
        Motor* motor2;
};