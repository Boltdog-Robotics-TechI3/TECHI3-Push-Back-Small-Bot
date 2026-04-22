#include "main.h"
#include <math.h>

void goalRush(){
	chassis.setPose(14,48,M_PI_2);
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose = {47, 48, 0}, .timeout = 3000, .maxMoveSpeed = 80}); // line up with match load
	matchLoadPiston.extend();															  // lower match load
	liftPiston.extend();																  // raise lift
	hoodPiston.retract();																  // close hood
	chassis.turnToAngle({.targetAngle = 180, .timeout = 2000, .maxTurnSpeed = 50});		  // face match load
	chassis.moveToPose({.targetPose = {47.5, 65, 0}, .timeout = 1000, .maxMoveSpeed = 50}); // ram to match load

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
	chassis.moveToPose({.targetPose = {47.5, 24, 0}, .timeout = 1500, .maxMoveSpeed = 100, .maxTurnSpeed = 80}); // ram to match load
	hoodPiston.extend();																					   // open hood
	leverSpeed = 100;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);
	

	// Wing it
	chassis.moveToPose({.targetPose = {47, 43, 0}, .timeout = 5000, .maxMoveSpeed = 100, .maxTurnSpeed = 100});
	hoodPiston.retract();
	turn.setI(.2);
	chassis.turnToAngle({.targetAngle = 135, .timeout = 10000});
	matchLoadPiston.retract();
	chassis.moveToPose({.targetPose = {33, 9, 0}, .timeout = 1500, .maxMoveSpeed = 80, .turnStartTime = 250});
	controller.set_text(0, 0, chassis.getPose().to_string());
}

void AWP(){
	// Drive to mid goal
	chassis.moveToPose({.targetPose = {17, 15, 0}, .timeout = 2000, .maxMoveSpeed = 80}); // go to mid score
	chassis.turnToAngle({.targetAngle = 140, .timeout = 2000});							  // line up with mid goal
	hoodPiston.extend();																  // open hood
	pros::delay(200);
	chassis.moveToPose({.targetPose = {10, 10, 0}, .timeout = 1000, .maxMoveSpeed = 60}); // go to mid score
	leverSpeed = 70;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);
	hoodPiston.retract();

	// Travel to loader.
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose = {47, 48, 0}, .timeout = 3000, .maxMoveSpeed = 80}); // line up with match load
	chassis.turnToAngle({.targetAngle = 185, .timeout = 2000, .maxTurnSpeed = 50});		  // face match load
	matchLoadPiston.extend();															  // lower match load
	liftPiston.extend();																  // raise lift
	hoodPiston.retract();																  // close hood
	chassis.moveToPose({.targetPose = {47, 65, 0}, .timeout = 1000, .maxMoveSpeed = 50}); // ram to match load

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
	chassis.moveToPose({.targetPose = {47, 24, 0}, .timeout = 1500, .maxMoveSpeed = 100, .maxTurnSpeed = 80}); // ram to match load
	hoodPiston.extend();																					   // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING)
		;

	setLeverState(LeverState::INTAKING);
	// chassis.setPose(48.5, 15, M_PI_2); // reset position

	// Go back to Loader
	chassis.moveToPose({.targetPose = {47, 65, 0}, .timeout = 2000, .maxMoveSpeed = 60}); // ram to match load
	liftPiston.extend();																  // raise lift
	hoodPiston.retract();																  // close hood
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
	while (getLeverState() == LeverState::SCORING)
		;
	setLeverState(LeverState::INTAKING);

	// Go intake match loads
	hoodPiston.retract(); // close hood
	liftPiston.extend();  // raise lift
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
	chassis.moveToPose({.targetPose = {47, 24, 0}, .timeout = 2000, .maxMoveSpeed = 80, .maxTurnSpeed = 80}); // ram to match load
	hoodPiston.extend();																					  // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING)
		;

	// Wing it
	chassis.moveToPose({.targetPose = {0, 36, 0}, .timeout = 1500, .maxMoveSpeed = 100, .maxTurnSpeed = 100});
	chassis.turnToAngle({.targetAngle = 180, .timeout = 5000});
	hoodPiston.retract();
	chassis.moveToPose({.targetPose = {33, 9, 0}, .timeout = 1500, .maxMoveSpeed = 80});
	controller.set_text(0, 0, chassis.getPose().to_string());
}