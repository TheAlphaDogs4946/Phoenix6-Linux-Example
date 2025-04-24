#include "ctre/phoenix6/TalonFX.hpp"
#include "RobotBase.hpp"
#include "Joystick.hpp"
#include "wiringPi.h"
#include <wiringPiI2C.h>
#include <iostream>
#include <termios.h>
#include "wiringSerial.h"
#include <unistd.h>

#define ADS1115_ADDRESS 0x48

#define CONVERSION_REG 0X00
#define CONFIG_REG 0x01

#define CONFIG_OS_SINGLE 0x8000
#define CONFIG_MUX_SINGLE_0 0x4000
#define CONFIG_GAIN_ONE 0x0200
#define CONFIG_MODE_SINGLE 0x0100
#define CONFIG_DR_1600SPS 0x0080
#define CONFIG_COMP_QUE_DISABLE 0x0003

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
    //controls::MusicTone music {456, 1};

    /* joystick */
    //Joystick joy{0};

    bool isEnabled = false;
    bool firstPress = true;

    double deadzone;

    int serialPort = 0;
    int thingy = 0;
    int fd = 0;

    int tick = 0;

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
    left_fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    left_fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    left_fx_cfg.CurrentLimits.SupplyCurrentLimit = 55;
    left_fx_cfg.CurrentLimits.SupplyTimeThreshold = 0.1;
    left_fx_cfg.CurrentLimits.SupplyCurrentThreshold = 46;
    left_fx_cfg.CurrentLimits.StatorCurrentLimit = 55;
    left_fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    left_fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    leftLeader.GetConfigurator().Apply(left_fx_cfg);

    /* the right motor is CW+ */
    right_fx_cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
    right_fx_cfg.MotorOutput.NeutralMode = signals::NeutralModeValue::Coast;
    right_fx_cfg.CurrentLimits.SupplyCurrentLimit = 55;
    right_fx_cfg.CurrentLimits.SupplyTimeThreshold = 0.1;
    right_fx_cfg.CurrentLimits.SupplyCurrentThreshold = 46;
    right_fx_cfg.CurrentLimits.StatorCurrentLimit = 55;
    right_fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    right_fx_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    
    rightLeader.GetConfigurator().Apply(right_fx_cfg);
}

/**
 * Runs periodically during program execution.
 */
void Robot::RobotPeriodic()
{
    /* periodically check that the joystick is still good */
    // joy.Periodic();

    // if(joy.GetButton(4) && firstPress){
    //     isEnabled = !isEnabled;
    //     firstPress = false;
    // }
    // if(!joy.GetButton(4) && !firstPress){
    //     firstPress = true;
    // }
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
    // struct termios options;
    // int fd = serialOpen("/dev/ttyACM0", 9600);
    // printf("%u", fd);
    // tcgetattr(fd, &options);
    // options.c_lflag |= ICANON;
    // options.c_iflag |= IGNCR;
    // options.c_cc [VMIN] = 0; // setting both of these to 0 is bad but it should make it work like i want it to
    // options.c_cc [VTIME] = 0;
    // options.c_cflag &= ~PARENB;     
    // options.c_cflag &= ~CSTOPB;     
    // options.c_cflag &= ~CRTSCTS;
    // tcsetattr(fd, TCSANOW | TCSAFLUSH, &options);
    // serialPort = fd;
    fd = wiringPiI2CSetup(ADS1115_ADDRESS); //change to 0x180 if no work
    if(fd == -1) {
        std::cerr << "Failed to initialize I2C communication\n";
    } else {
        std::cout << "I2C communication successfully set up\n";
    }

    
}

/**
 * Runs periodically while enabled.
 */
void Robot::EnabledPeriodic()
{int config = CONFIG_OS_SINGLE | CONFIG_MUX_SINGLE_0 | CONFIG_GAIN_ONE | CONFIG_MODE_SINGLE | CONFIG_DR_1600SPS | CONFIG_COMP_QUE_DISABLE;
    wiringPiI2CWriteReg16(fd, CONFIG_REG, (config >> 8) | (config << 8));

    delay(30);

    thingy = fd;

    int16_t value = wiringPiI2CReadReg16(thingy, CONVERSION_REG);
    value = (value >> 8) | (value <<8);
        std::cout << "OVERFLOW ADC Value: \t" << value << "\n";

    // if(value > 0) {
    //     double newValue = ((double)value-6900)/14000.0;
    //     std::cout << "OVERFLOW ADC Value: \t" << value << ", \t" << newValue <<"\n";
    // } else {
    //     double newValue = (double)value/-90.0;
    //     std::cout << "NORMAL ADC Value: \t" << value << ", \t" << newValue <<"\n";

    // }


    // float voltage = (value * 4.096) / 32768.0;

    // std::cout << "Voltage: " << voltage << " V\n";
    //Ticks control how often the serial port flushes to prevent single-byte reads
    // tick ++;
    // if(tick>=10){
    //     //serialFlush(serialPort);
    //     tick = 0;
    // }
        
    // int m;
    // unsigned char buffer[5];
    // m = read(serialPort, buffer, sizeof (buffer) - 1);
    // double final;
    // if (m == 4) {
    //     buffer[m] = 0;
    //     printf("bytes read: %u serial data: %s", m, buffer);
    //     printf("\n");

    //     int hundredsDig = buffer[0] - '0';
    //     int hundredsDigRes = 100*hundredsDig;
    //     int tensDig = buffer[1] - '0';
    //     int tensDigRes = 10*tensDig;
    //     int onesDig = buffer[2] - '0';
    //     int onesDigRes = onesDig;

    //     final = hundredsDigRes + tensDigRes + onesDigRes;
    //}

    
    // double speed = (final-177)/690; // SDL_CONTROLLER_AXIS_LEFTY
    // if(speed>1){
    //     speed = 1;
    // } else if(speed<0){
    //     speed = 0;
    // }

    // deadzone  = speed*0.8+0.1;
    
    // if(speed<deadzone){
    //     speed = 0;
    // }
    // cout << "Speed: ";
    // cout << speed;
    // printf("\n");

    // cout << "Deadzone: ";
    // cout << deadzone;
    // printf("\n");

    // leftOut.Output = speed;
    // rightOut.Output = speed;
    // leftLeader.SetControl(leftOut);
    // rightLeader.SetControl(rightOut);

    leftOut.Output = 0.02;
    
    leftLeader.SetControl(leftOut);
    //leftLeader.SetControl(controls::MusicTone);

    //leftLeader.GetGenericSignal("whatever goes here");

    // //cout << leftLeader.Get;
    // printf("\n");
} 

/**
 * Runs when transitioning from enabled to disabled,
 * including after robot startup.
 */
void Robot::DisabledInit() {
    // serialClose(serialPort);
}

/**
 * Runs periodically while disabled.
 */
void Robot::DisabledPeriodic()
{
    // leftLeader.SetControl(controls::NeutralOut{});
    // rightLeader.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main()
{
    /* create and run robot */
    Robot robot{};
    // robot.SetLoopTime(20_ms); // optionally change loop time for periodic calls
    return robot.Run();
}
