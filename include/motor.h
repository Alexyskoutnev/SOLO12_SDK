#include "motor_driver.h"


class MotorDriver;
class Motor {
    public:
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

        double position;
        double velocity;
        double current;
        double offsetPosition;
         
        bool isEnabledFlag;
        bool isReadyFlag;
        bool indexToggleBitFlag;
        bool indexDectectedFlag;
        bool position_index_offset_compensation_flag;
        bool enable_position_rollover_error_flag;

        double position_cmd;
        double velocity_cmd;
        double current_cmd;
        double current_max; 
        double kp;
        double kd;

};