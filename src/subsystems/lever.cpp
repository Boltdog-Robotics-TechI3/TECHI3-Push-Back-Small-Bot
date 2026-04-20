#include "main.h"

LeverState leverState = LeverState::IDLE;

std::atomic<bool> isLeverSettled = false;
std::atomic<bool> isInterrupted = false;

pros::Task leverTask = pros::Task([]() {
    while(1) {
        switch (leverState) {
            case IDLE:
				isInterrupted = false;
                setIntakeSpeed(30);
				if (leverMotor.get_position() > 0) {
					setLeverSpeed(-30);
				} else {
					setLeverSpeed(0);
				}
                break;
            case INTAKING:
                setIntakeSpeed(127);
                break;
            case OUTTAKING:
                setIntakeSpeed(-127);
                break;
            case SCORING:
                setIntakeSpeed(127);
                setLeverSpeed(leverSpeed);
                pros::delay(200);
                while (leverMotor.get_actual_velocity() > 10);
                pros::delay(500);
                setLeverSpeed(-80);
                setIntakeSpeed(-30);
                pros::delay(200);
                while (leverMotor.get_actual_velocity() > 10);
                setLeverState(LeverState::IDLE);
                break;
        }

        pros::delay(20);
    }
});

void leverInitialize() {
    intakeMotor.set_brake_mode(MOTOR_BRAKE_COAST);
    leverMotor.set_brake_mode(MOTOR_BRAKE_COAST);
}

void setLeverState(LeverState state) {
    leverState = state;
}

LeverState getLeverState() {
    return leverState;
}

void setIntakeSpeed(int speed) {
    intakeMotor.move(speed);
}

void setLeverSpeed(int speed) {
    leverMotor.move(speed);
}

void leverPeriodic() {
    if (controller.get_digital(DIGITAL_L1) && leverState != LeverState::SCORING) {
        hoodPiston.retract();
        setLeverState(LeverState::INTAKING);
    }
    else if (controller.get_digital(DIGITAL_A) && leverState != LeverState::SCORING) {
        hoodPiston.retract();
        setLeverState(LeverState::OUTTAKING);
    }    
    else if (controller.get_digital_new_press(DIGITAL_R2)) {
        if (leverState == LeverState::SCORING) {
            isInterrupted = true;
        }
        else {
            leverSpeed = 127;
            hoodPiston.extend();
            setLeverState(LeverState::SCORING);
        }
    } 
    else if (controller.get_digital_new_press(DIGITAL_R1)) {
		if (leverState == LeverState::SCORING) {
			isInterrupted = true;
		}
        else if (hoodPiston.is_extended()) {
            leverSpeed = 100;
            setLeverState(LeverState::SCORING);
        }
        else {
            hoodPiston.extend();
        }
    } 
    else if (controller.get_digital_new_press(DIGITAL_X)){
		if (leverState == LeverState::SCORING) {
			isInterrupted = true;
		}
        else if (hoodPiston.is_extended()) {
            leverSpeed = 60;
            setLeverState(LeverState::SCORING);
        }
        else {
            hoodPiston.extend();
        }
    }
    else if (leverState != LeverState::SCORING) {
        setLeverState(LeverState::IDLE);
    }

    if (controller.get_digital_new_press(DIGITAL_UP)) {
        liftPiston.extend();
        hoodPiston.extend();
    }
    else if (controller.get_digital_new_press(DIGITAL_DOWN)) {
        liftPiston.retract();
        hoodPiston.retract();
    }

    if (controller.get_digital_new_press(DIGITAL_L2)) {
        hoodPiston.retract();
    } 
    else if (controller.get_digital_new_release(DIGITAL_L2))  {
        hoodPiston.extend();
    }
        
    if (controller.get_digital_new_press(DIGITAL_LEFT)) {
        matchLoadPiston.toggle();
    }


}

void setLeverPosition(float targetPosition, float maxVel, float maxAccel) {
    isLeverSettled = false;

    Timer timeoutTimer(2000, +[]() { isLeverSettled = true; });
    Timer largeErrorTimer(500, +[]() { isLeverSettled = true; });
    
    int output;
    int startTime = pros::millis();

    leverPID.reset();

    leverPID.setOutputLimits(-maxVel, maxVel);
    leverPID.setLargeErrorRange(50);
    leverPID.setSlewRate(maxAccel);
    
    timeoutTimer.start();

    while (!isLeverSettled && !isInterrupted) {  
        int output = leverPID.calculate(leverMotor.get_position(), targetPosition);
    	setLeverSpeed(output);

        if (leverMotor.get_actual_velocity() <= 5 && 1500 < (pros::millis() - startTime)){
            isLeverSettled = true;
        }

        if (leverPID.isInLargeErrorRange()) {
            largeErrorTimer.start();
        }
        else {
            largeErrorTimer.stop();
        }

        pros::delay(20);
    }

    largeErrorTimer.stop();
    timeoutTimer.stop();
    setLeverSpeed(0);
}
