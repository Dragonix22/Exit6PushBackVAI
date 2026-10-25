using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern motor leftFrontMotor;
extern motor leftBackBottomMotor;
extern motor rightFrontMotor;
extern motor leftBackTopMotor;
extern motor rightBackTopMotor;
extern motor rightBackBottomMotor;

void vexcodeInit( void );