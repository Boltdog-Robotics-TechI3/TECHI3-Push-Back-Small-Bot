#include "main.h"
#include <math.h>

void goalRush(){
	turn.setGains(100, .15, .1);
	chassis.setPose(14,48,M_PI_2);
	chassis.startTracking();
	setLeverState(LeverState::INTAKING);
	chassis.moveToPose({.targetPose = {47.8, 48, 0}, .timeout = 3000, .maxMoveSpeed = 115}); // line up with match load
	matchLoadPiston.extend();															  // lower match load
	liftPiston.extend();																  // raise lift
	hoodPiston.retract();																  // close hood
	chassis.turnToAngle({.targetAngle = 180, .timeout = 2000, .maxTurnSpeed = 100});		  // face match load
	chassis.moveToPose({.targetPose = {48.5, 68.5, 0}, .timeout = 1000, .maxMoveSpeed = 50}); // ram to match load
	chassis.tank(25, 25);
	pros::delay(350);
	chassis.tank(0,0);
	pros::delay(600);
	chassis.tank(75,75);
	pros::delay(300);
	chassis.tank(0,0);
	
	matchLoadPiston.retract();															  // raise match load

	// Wiggle
	// setLeverState(LeverState::INTAKING);
	// for(int i = 0; i < 3; i++){
	// 	chassis.tank(50, -50);
	// 	pros::delay(200);
	// 	chassis.tank(-50, 50);
	// 	pros::delay(200);
	// }
	
	// chassis.turnToAngle({.targetAngle = (int) Pose::radToDeg(odometry.getRotationRadians() + chassis.getPose().angleTo({48.5, 24, 0})), .timeout = 2000, .maxTurnSpeed = 100});		  // face match load

	// Long Goal Score 1
	setLeverState(LeverState::IDLE);
	chassis.moveToPose({.targetPose = {48, 24, 0}, .timeout = 1500, .maxMoveSpeed = 115, .maxTurnSpeed = 80}); // ram to match load
	// for(int i = 0; i < 3; i++){
	// 	chassis.tank(-25, 100);
	// 	pros::delay(100);
	// 	chassis.tank(100, -25);
	// 	pros::delay(100);
	// }
	setLeverState(LeverState::INTAKING);
	pros::delay(200);
	chassis.tank(0, 0);


	hoodPiston.extend();																					   // open hood
	leverSpeed = 100;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);
	

	// Wing it
	chassis.moveToPose({.targetPose = {48.5, 42, 0}, .timeout = 3000, .maxMoveSpeed = 115, .maxTurnSpeed = 100, .turnStartTime = 1000});
	hoodPiston.retract();
	turn.setI(.2);
	chassis.turnToAngle({.targetAngle = 142, .timeout = 3000, .maxTurnSpeed = 100, .smallErrorRange=1, .largeErrorRange=2});
	matchLoadPiston.retract();
	chassis.moveToPose({.targetPose = {37, 5, 0}, .timeout = 1500, .maxMoveSpeed = 100, .turnStartTime = 200});
	controller.set_text(0, 0, chassis.getPose().to_string());
}

void AWP(){
	chassis.setPose(14,48,0);
	// Grab 2 mid blocks
	setLeverState(LeverState::INTAKING);
	hoodPiston.retract(); // open hood
	chassis.moveToPose({.targetPose={45, 7, 0}, .timeout=5000, .maxMoveSpeed=80, .maxTurnSpeed=55}); // pick up 2 blocks under long goal
	
	// Drive to mid goal
	chassis.moveToPose({.targetPose = {17, 15, 0}, .timeout = 2000, .maxMoveSpeed = 80}); // go to mid score
	chassis.turnToAngle({.targetAngle = 140, .timeout = 2000});							  // line up with mid goal
	hoodPiston.extend();																  // open hood
	pros::delay(200);
	chassis.moveToPose({.targetPose = {10, 10, 0}, .timeout = 1000, .maxMoveSpeed = 60}); // go to mid score
	leverSpeed = 70;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);
	liftPiston.extend();
	hoodPiston.retract();

	// Travel to loader.
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
	chassis.moveToPose({.targetPose = {47.5, 24, 0}, .timeout = 1500, .maxMoveSpeed = 80, .maxTurnSpeed = 80}); // ram to match load
	hoodPiston.extend();																					   // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);

	setLeverState(LeverState::INTAKING);
	// chassis.setPose(48.5, 15, M_PI_2); // reset position

	// Go back to Loader
	chassis.moveToPose({.targetPose = {47.5, 65, 0}, .timeout = 2000, .maxMoveSpeed = 60}); // ram to match load
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
	while (getLeverState() == LeverState::SCORING);
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
	chassis.moveToPose({.targetPose = {47.5, 24, 0}, .timeout = 2000, .maxMoveSpeed = 80, .maxTurnSpeed = 80}); // ram to match load
	hoodPiston.extend();																					  // open hood
	leverSpeed = 127;
	setLeverState(LeverState::SCORING);
	while (getLeverState() == LeverState::SCORING);

	
	// Wing it
	chassis.moveToPose({.targetPose = {47.5, 50, 0}, .timeout = 5000, .maxMoveSpeed = 100, .maxTurnSpeed = 100});
	hoodPiston.retract();
	turn.setI(.2);
	chassis.turnToAngle({.targetAngle = 135, .timeout = 3000});
	matchLoadPiston.retract();
	chassis.moveToPose({.targetPose = {36, 9, 0}, .timeout = 1500, .maxMoveSpeed = 80, .turnStartTime = 250});
	controller.set_text(0, 0, chassis.getPose().to_string());
}