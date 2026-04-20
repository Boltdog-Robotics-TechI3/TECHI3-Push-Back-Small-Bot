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
	chassis.startTracking();
	chassis.setPose(14, 48, 0); // starting point
	setLeverState(LeverState::INTAKING);
	hoodPiston.retract(); // open hood
	chassis.moveToPose({.targetPose={45, 7, 0}, .timeout= 5000, .maxMoveSpeed=80}); // pick up 2 blocks under long goal
	//setLeverState(LeverState::IDLE);
	chassis.moveToPose({.targetPose={11, 11, 0}, .timeout=10000, .maxMoveSpeed=80}); // go to mid score
	chassis.turnToAngle({.targetAngle=130,.timeout=2000}); // line up with mid goal
	leverSpeed = 70;
	setLeverState(LeverState::SCORING);
	pros::delay(800);
	hoodPiston.retract();
	//setLeverState(LeverState::OUTTAKING);
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose={49, 48, 0}, .timeout=2000, .maxMoveAccel=80}); // line up with match load
	chassis.turnToAngle({.targetAngle=180,.timeout=1000}); // face match load
	matchLoadPiston.extend(); // lower match load
	liftPiston.extend(); // raise lift
	hoodPiston.retract(); // close hood
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=400, .maxMoveAccel=80}); // ram to match load
	chassis.moveToPose({.targetPose={48.5, 55, 0}, .timeout=400, .maxMoveAccel=80}); // back up
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=400, .maxMoveAccel=80}); // ram to match load
	chassis.moveToPose({.targetPose={48.5, 15, 0}, .timeout=2000, .maxMoveAccel=80}); // ram to match load
	hoodPiston.extend(); // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);
	// chassis.setPose(48.5, 15, M_PI_2); // reset position
	// chassis.startTracking();
	//chassis.moveToPose({49, 48, 0}, 5000, 80); // line up with match load
	liftPiston.extend(); // raise lift
	hoodPiston.retract(); // close hood
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=400, .maxMoveAccel=80}); // ram to match load
	chassis.moveToPose({.targetPose={48.5, 55, 0}, .timeout=400, .maxMoveAccel=80}); // back up
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=400, .maxMoveAccel=80}); // ram to match load
	liftPiston.retract();
	hoodPiston.extend(); // open hood
	setIntakeSpeed(0);
	setLeverState((LeverState)7);
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);
	hoodPiston.retract(); // close hood
	liftPiston.extend(); // raise lift
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=800, .maxMoveAccel=80}); // ram to match load
	chassis.moveToPose({.targetPose={48.5, 55, 0}, .timeout=800, .maxMoveAccel=80}); // back up
	chassis.moveToPose({.targetPose={48.5, 60, 0}, .timeout=800, .maxMoveAccel=80}); // ram to match load
	chassis.moveToPose({.targetPose={48.5, 15, 0}, .timeout=2000, .maxMoveAccel=80}); // ram to match load
	hoodPiston.extend(); // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState()==LeverState::SCORING);
	chassis.moveToPose({.targetPose={34, 38, 0}, .timeout=1500, .maxMoveSpeed=80});
	chassis.turnToAngle({.targetAngle=180,.timeout=5000});
	hoodPiston.retract();
	chassis.moveToPose({.targetPose={34, 9, 0}, .timeout=1500, .maxMoveAccel=80});
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

	// chassis.setPose(14, 48, 0);
	// chassis.startTracking();

	while (true) {
		leftY = controller.get_analog(ANALOG_LEFT_Y);
		rightX = controller.get_analog(ANALOG_RIGHT_X) * (1.0/2.0);
		
		chassis.arcade(leftY, rightX);

		leverPeriodic();

		// controller.set_text(0, 0, std::to_string(leverMotor.get_position()));
//		controller.set_text(0, 0, chassis.getPose().to_string());

		 if(controller.get_digital_new_press(DIGITAL_RIGHT)){
		 	autonomous();
		 }

		//  if(controller.get_digital_new_press(DIGITAL_A)){
		//  	autonomous();
		//  }

		pros::delay(20);
	}
}
