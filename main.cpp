#include "ctre/phoenix6/TalonFX.hpp"
#include "RobotBase.hpp"
#include "Joystick.hpp"
#include "wiringPi.h"
#include <iostream>
#include <termios.h>
#include "wiringSerial.h"
#include <unistd.h>


using namespace ctre::phoenix6;
using namespace std;

/**
 * This is the main robot. Put all actuators, sensors,
 * game controllers, etc. in this class.
 */
class Robot : public RobotBase {
private:
    /* This can be a CANivore name, CANivore serial number,
     * SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CANBUS_NAME = "*";

    /* devices */
    hardware::TalonFX leftLeader{0, CANBUS_NAME};
    hardware::TalonFX rightLeader{1, CANBUS_NAME};

    /* control requests */
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};

    /* joystick */
    Joystick joy{0};

    bool isEnabled = false;
    bool firstPress = true;

    double deadzone = 0.2;
    
    int serialPort = 0;

public:
    /* main robot interface */
    void RobotInit() override;
    void RobotPeriodic() override;

    bool IsEnabled() override;
    void EnabledInit() override;
    void EnabledPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;
};

/**
 * Runs once at code initialization.
 */
void Robot::RobotInit()
{
    configs::TalonFXConfiguration left_fx_cfg{};
    configs::TalonFXConfiguration right_fx_cfg{};

    /* the left motor is CCW+ */
    left_fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    left_fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    left_fx_cfg.CurrentLimits.SupplyCurrentLimit = 20;
    left_fx_cfg.CurrentLimits.SupplyTimeThreshold = 0.1;
    left_fx_cfg.CurrentLimits.SupplyCurrentThreshold = 10;
    left_fx_cfg.CurrentLimits.StatorCurrentLimit = 20;
    left_fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    left_fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    leftLeader.GetConfigurator().Apply(left_fx_cfg);
    

    /* the right motor is CW+ */
    right_fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    right_fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    right_fx_cfg.CurrentLimits.SupplyCurrentLimit = 20;
    right_fx_cfg.CurrentLimits.SupplyTimeThreshold = 0.1;
    right_fx_cfg.CurrentLimits.SupplyCurrentThreshold = 10;
    right_fx_cfg.CurrentLimits.StatorCurrentLimit = 20;
    right_fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    right_fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    rightLeader.GetConfigurator().Apply(right_fx_cfg);

    //initializes the serial port to read the thing
    
}

/**
 * Runs periodically during program execution.
 */
void Robot::RobotPeriodic()
{
    /* periodically check that the joystick is still good */
    joy.Periodic();

    //leftLeader.Get();

    if(joy.GetButton(4) && firstPress){
        isEnabled = !isEnabled;
        firstPress = false;
    }
    if(!joy.GetButton(4) && !firstPress){
        firstPress = true;
    }
}

/**
 * Returns whether the robot should be enabled.
 */
bool Robot::IsEnabled()
{
    return isEnabled;
}

/**
 * Runs when transitioning from disabled to enabled.
 */
void Robot::EnabledInit() {
    struct termios options;
    int fd = serialOpen("/dev/ttyACM1", 9600);
    printf("%u", fd);
    tcgetattr(fd, &options);
    options.c_lflag |= ICANON;
    options.c_iflag |= IGNCR;
    options.c_cc [VMIN] = 0; // setting both of these to 0 is bad but it should make it work like i want it to
    options.c_cc [VTIME] = 0;
    options.c_cflag &= ~PARENB;     
    options.c_cflag &= ~CSTOPB;     
    options.c_cflag &= ~CRTSCTS;
    tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);
    serialPort = fd;
}

/**
 * Runs periodically while enabled.
 */
void Robot::EnabledPeriodic()
{
    /* arcade drive */
    double speed = -joy.GetAxis(1, deadzone); // SDL_CONTROLLER_AXIS_LEFTY

    leftOut.Output = speed;
    rightOut.Output = speed;

    leftLeader.SetControl(leftOut);
    rightLeader.SetControl(rightOut);

    //cout << leftLeader.GetRotorVelocity();
    serialFlush(serialPort);

    if (joy.GetButton(5)) {
        
        int m;
        unsigned char buffer[5];
        m = read(serialPort, buffer, sizeof (buffer) - 1);

        if (m != 0) {
            buffer[m] = 0;
            printf("bytes read: %u serial data: %s", m, buffer);
            printf("\n");
        }
    }
    //  if (serialDataAvail(serialPort)) {
    //        cout << serialGetchar(serialPort);
    //}
}

/**
 * Runs when transitioning from enabled to disabled,
 * including after robot startup.
 */
void Robot::DisabledInit() {
    serialClose(serialPort);
}

/**
 * Runs periodically while disabled.
 */
void Robot::DisabledPeriodic()
{
    leftLeader.SetControl(controls::NeutralOut{});
    rightLeader.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main()
{
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}
