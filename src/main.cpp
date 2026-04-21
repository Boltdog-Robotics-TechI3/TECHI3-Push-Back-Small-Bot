#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.reset();
	chassis.setInputScale(Chassis::InputScale::SINSQUARED);

	leverInitialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	// Setup
	chassis.startTracking();
	chassis.setPose(14, 48, 0); // starting point

	// Grab 2 mid blocks
	setLeverState(LeverState::INTAKING);
	hoodPiston.retract(); // open hood
	chassis.moveToPose({.targetPose={45, 7, 0}, .timeout=5000, .maxMoveSpeed=80, .maxTurnSpeed=55}); // pick up 2 blocks under long goal
	
	// Drive to mid goal
	chassis.moveToPose({.targetPose={17, 15, 0}, .timeout=2000, .maxMoveSpeed=80}); // go to mid score
	chassis.turnToAngle({.targetAngle=140, .timeout=2000}); // line up with mid goal
	hoodPiston.extend(); // open hood
	pros::delay(200);
	chassis.moveToPose({.targetPose={10, 10, 0}, .timeout=1000, .maxMoveSpeed=60}); // go to mid score
	leverSpeed = 70;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);
	hoodPiston.retract();

	// Travel to loader.
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose={47, 48, 0}, .timeout=3000, .maxMoveSpeed=80}); // line up with match load
	chassis.turnToAngle({.targetAngle=185, .timeout=2000, .maxTurnSpeed=50}); // face match load
	matchLoadPiston.extend(); // lower match load
	liftPiston.extend(); // raise lift
	hoodPiston.retract(); // close hood
	chassis.moveToPose({.targetPose={47, 65, 0}, .timeout=1000, .maxMoveSpeed=50}); // ram to match load

	// Wiggle
	setLeverState(LeverState::INTAKING);
	pros::delay(500);
	chassis.tank(-50, -50);
	pros::delay(300);
	chassis.tank(50, 50);
	pros::delay(150);
	chassis.tank(-50, -50);
	pros::delay(500);

	// Long Goal Score 1
	chassis.moveToPose({.targetPose={47, 24, 0}, .timeout=1500, .maxMoveSpeed=100, .maxTurnSpeed=80}); // ram to match load
	hoodPiston.extend(); // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);

	setLeverState(LeverState::INTAKING);
	// chassis.setPose(48.5, 15, M_PI_2); // reset position

	// Go back to Loader
	chassis.moveToPose({.targetPose={47, 65, 0}, .timeout=2000, .maxMoveSpeed=60}); // ram to match load
	liftPiston.extend(); // raise lift
	hoodPiston.retract(); // close hood
	pros::delay(500);
	chassis.tank(-50, -50);
	pros::delay(300);
	chassis.tank(50, 50);
	pros::delay(150);
	chassis.tank(-50, -50);
	pros::delay(500);	

	// Spit out enemy color
	liftPiston.retract();
	hoodPiston.extend(); // open hood
	setIntakeSpeed(0);
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);
	setLeverState(LeverState::INTAKING);

	// Go intake match loads
	hoodPiston.retract(); // close hood
	liftPiston.extend(); // raise lift
	pros::delay(500);
	chassis.tank(-50, -50);
	pros::delay(300);
	chassis.tank(50, 50);
	pros::delay(150);
	chassis.tank(-50, -50);
	pros::delay(300);
	chassis.tank(50, 50);
	pros::delay(150);
	chassis.tank(-50, -50);
	pros::delay(500);

	// Long Goal Score 2
	chassis.moveToPose({.targetPose={47, 24, 0}, .timeout=2000, .maxMoveSpeed=80, .maxTurnSpeed=80}); // ram to match load
	hoodPiston.extend(); // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);

	// Wing it 
	chassis.moveToPose({.targetPose={0, 36, 0}, .timeout=1500, .maxMoveSpeed=100, .maxTurnSpeed=100});
	chassis.turnToAngle({.targetAngle=180,.timeout=5000});
	hoodPiston.retract();
	chassis.moveToPose({.targetPose={33, 9, 0}, .timeout=1500, .maxMoveSpeed=80});
	controller.set_text(0, 0, chassis.getPose().to_string());
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	int leftY, rightX;
	std::cout << "Hello" << std::endl;

	chassis.setPose(0, 0, 0);
	chassis.startTracking();

	while (true) {
		leftY = controller.get_analog(ANALOG_LEFT_Y);
		rightX = controller.get_analog(ANALOG_RIGHT_X) * (1.0/2.0);
		
		chassis.arcade(leftY, rightX);

		leverPeriodic();

		// controller.set_text(0, 0, std::to_string(leverMotor.get_position()));
		controller.set_text(0, 0, chassis.getPose().to_string());

		 if(controller.get_digital_new_press(DIGITAL_RIGHT)){
		 	autonomous();
		 }

		//  if(controller.get_digital_new_press(DIGITAL_A)){
		//  	autonomous();
		//  }

		pros::delay(20);
	}
}
