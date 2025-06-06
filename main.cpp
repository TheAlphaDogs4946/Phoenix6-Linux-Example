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
    static constexpr char const *CANBUS_NAME = "test0";

    /* devices */
    hardware::TalonFX leftMotor{0, CANBUS_NAME};
    hardware::TalonFX rightMotor{1, CANBUS_NAME};

    /* control requests */
    controls::DutyCycleOut dutyOut{0};
    controls::TorqueCurrentFOC torqueReq{units::current::ampere_t{0}};
    controls::VelocityTorqueCurrentFOC velocityReq{units::angular_velocity::revolutions_per_minute_t{0}};

    double deadzone;
    double pedalVal;

    int serialPort = 0;

    int tick = 0;

    int supplyCurrentLimit = 65;
    int statorCurrentLimit = 100;
    //May 7, test 1, 55 amps, regen ended at dumpster
    //May 7, test 2, 55 amps, 10 P, regen ended at dumpster

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
    configs::TalonFXConfiguration leftConfig{};
    configs::TalonFXConfiguration rightConfig{};

    /* the left motor is CCW+ */
    leftConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    leftConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    leftConfig.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{supplyCurrentLimit};
    leftConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{statorCurrentLimit};
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.TorqueCurrent.PeakForwardTorqueCurrent = units::current::ampere_t{85};
    leftConfig.TorqueCurrent.PeakReverseTorqueCurrent = units::current::ampere_t{85};
    leftConfig.Slot0.kP = 10;
    leftConfig.Slot0.kI = 0;
    leftConfig.Slot0.kD = 0;
    leftConfig.Slot0.kV = 0;
    leftConfig.Slot0.kS = 0;

    leftMotor.GetConfigurator().Apply(leftConfig);

    /* the right motor is CW+ */
    rightConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    rightConfig.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{supplyCurrentLimit};
    rightConfig.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{statorCurrentLimit};
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.TorqueCurrent.PeakForwardTorqueCurrent = units::current::ampere_t{85};
    rightConfig.TorqueCurrent.PeakReverseTorqueCurrent = units::current::ampere_t{85};
    rightConfig.Slot0.kP = 10;
    rightConfig.Slot0.kI = 0;
    rightConfig.Slot0.kD = 0;
    rightConfig.Slot0.kV = 0;
    rightConfig.Slot0.kS = 0;
    
    rightMotor.GetConfigurator().Apply(rightConfig);

    
}

/**
 * Runs periodically during program execution.
 */
void Robot::RobotPeriodic()
{
    
}

/**
 * Returns whether the robot should be enabled.
 */
bool Robot::IsEnabled()
{
    return true;
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
    //Ticks control how often the serial port flushes to prevent single-byte reads
    tick ++;
    if(tick>=10){
        serialFlush(serialPort);
        tick = 0;
    }
    int n;
        
    int m;
    unsigned char buffer[5];
    m = read(serialPort, buffer, sizeof (buffer) - 1);
    double final;
    if (m == 4) {
        buffer[m] = 0;
        printf("bytes read: %u serial data: %s", m, buffer);
        printf("\n");

        int hundredsDig = buffer[0] - '0';
        int hundredsDigRes = 100*hundredsDig;
        int tensDig = buffer[1] - '0';
        int tensDigRes = 10*tensDig;
        int onesDig = buffer[2] - '0';
        int onesDigRes = onesDig;

        final = hundredsDigRes + tensDigRes + onesDigRes;
    }

    
    double motorSpeed = (leftMotor.GetVelocity().GetValueAsDouble() + rightMotor.GetVelocity().GetValueAsDouble())/2;

    if(final > 160 && final < 900){
        pedalVal = (final-180)/690; // SDL_CONTROLLER_AXIS_LEFTY
    if(pedalVal>1){
        pedalVal = 1;
    } else if(pedalVal<0){
        pedalVal = 0;
    }
    }

    cout << "Speed: ";
    cout << pedalVal;
    printf("\n");

    cout << "Deadzone: ";
    cout << deadzone;
    printf("\n");

    cout << "Pedal: ";
    cout << final;
    printf("\n");

    cout << "Left Toruqe Current: ";
    cout << leftMotor.GetTorqueCurrent();
    printf("\n");

    cout << "Left Stator Current: ";
    cout << leftMotor.GetStatorCurrent();
    printf("\n");

    cout << "Left Motor Supply Current: ";
    cout << leftMotor.GetSupplyCurrent();
    printf("\n");

    cout << "Left Motor Stall Current: ";
    cout << leftMotor.GetMotorStallCurrent();
    printf("\n");

    cout << "Left Velocity: ";
    cout << leftMotor.GetVelocity();
    printf("\n");

    cout << "Left Temperature: ";
    cout << leftMotor.GetDeviceTemp();
    printf("\n");

    cout << "Right Toruqe Current: ";
    cout << rightMotor.GetTorqueCurrent();
    printf("\n");

    cout << "Right Stator Current: ";
    cout << rightMotor.GetStatorCurrent();
    printf("\n");

    cout << "Right Motor Supply Current: ";
    cout << rightMotor.GetSupplyCurrent();
    printf("\n");

    cout << "Right Motor Stall Current: ";
    cout << rightMotor.GetMotorStallCurrent();
    printf("\n");

    cout << "Right Velocity: ";
    cout << rightMotor.GetVelocity();
    printf("\n");

    cout << "Right Temperature: ";
    cout << rightMotor.GetDeviceTemp();
    printf("\n");


    if(pedalVal > 0.4){
        double upperMappedPedal = (pedalVal-0.4)/0.6;
        torqueReq.Output = units::current::ampere_t{upperMappedPedal*supplyCurrentLimit};
        leftMotor.SetControl(torqueReq);
        rightMotor.SetControl(torqueReq);
    }else if(pedalVal <=0.4 && pedalVal >= 0.3) {
        leftMotor.SetControl(controls::CoastOut{});
        rightMotor.SetControl(controls::CoastOut{});
    } else if(motorSpeed < 5 && pedalVal < 0.1){
        leftMotor.SetControl(controls::CoastOut{});
        rightMotor.SetControl(controls::CoastOut{});
        
    }else {
        if(motorSpeed >0){
            velocityReq.Velocity = units::angular_velocity::revolutions_per_minute_t{pedalVal*6000};
        } else {
            velocityReq.Velocity = units::angular_velocity::revolutions_per_minute_t{0};
        }
        leftMotor.SetControl(velocityReq);
        rightMotor.SetControl(velocityReq);
    }

    printf("\n");
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
    leftMotor.SetControl(controls::NeutralOut{});
    rightMotor.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main()
{
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}
