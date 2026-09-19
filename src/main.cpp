// test
#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
pros::MotorGroup left_mg({-9, -10}, pros::v5::MotorGears::blue);
pros::MotorGroup right_mg({1, 2}, pros::v5::MotorGears::blue);

void autonomous() {
	const int backward_power = 80;
	const int forward_power = 127;
	const int64_t quarter_second = 250;
	const int64_t one_second = 1000;

	int64_t start = pros::millis();
	while (pros::millis() - start < quarter_second) {
		left_mg.move(-backward_power);
		right_mg.move(-backward_power);
		pros::delay(20);
	}

	start = pros::millis();
	while (pros::millis() - start < one_second) {
		left_mg.move(forward_power);
		right_mg.move(forward_power);
		pros::delay(20);
	}

	start = pros::millis();
	while (pros::millis() - start < quarter_second) {
		left_mg.move(-backward_power);
		right_mg.move(-backward_power);
		pros::delay(20);
	}

	start = pros::millis();
	while (pros::millis() - start < one_second) {
		left_mg.move(forward_power);
		right_mg.move(forward_power);
		pros::delay(20);
	}

	left_mg.move(0);
	right_mg.move(0);
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor intake(11, pros::v5::MotorGears::green);
	pros::MotorGroup lift({3, -8}, pros::v5::MotorGears::green);
	pros::Motor arm(7, pros::v5::MotorGears::green);
	pros::Motor holder(6, pros::v5::MotorGears::green);
	pros::Rotation lift_rotation(5);
	constexpr double lift_min = 0;
	constexpr double lift_max = 9.89 * 360 * 100;
	constexpr double arm_start = 0.0;
	constexpr double arm_90 = 90.0;
	constexpr double arm_release_clear = 120.0;
	constexpr double arm_test_position = 1800.0;
	constexpr int64_t lift_macro_delay_ms = 150;
	constexpr int64_t holder_release_time_ms = 250;
	constexpr int64_t arm_macro_timeout_ms = 1500;
	const int holder_default_speed = 50;
	const int holder_macro_speed = 127;
	const int holder_release_speed = -127;
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	holder.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift.tare_position();
	lift_rotation.reset_position();
	arm.tare_position();
	double lift_target = lift_min;
	enum class ArmMacroState {
		IDLE,
		LIFT_UP,
		ARM_OUT,
		ARM_RETURN
	};
	enum class ReleaseMacroState {
		INACTIVE,
		RELEASE_HOLDER,
		RAISE_CLEAR
	};
	ArmMacroState arm_macro_state = ArmMacroState::IDLE;
	ReleaseMacroState release_macro_state = ReleaseMacroState::INACTIVE;
	bool r1_pressed_last = false;
	bool up_pressed_last = false;
	bool down_pressed_last = false;
	bool arm_extended = false;
	bool arm_test_at_90 = false;
	int64_t macro_state_start_ms = 0;
	int64_t release_start_ms = 0;


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = -master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		dir = static_cast<int>(std::clamp(dir * 2.0, -127.0, 127.0));
		turn = static_cast<int>(std::clamp(turn * 2.0, -127.0, 127.0));
		left_mg.move(-(dir - turn));
		right_mg.move(-(dir + turn));

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
			intake.move(-127);
		} else {
			intake.move(0);
		}

		bool r1_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		if (r1_pressed && !r1_pressed_last) {
			if (arm_macro_state == ArmMacroState::IDLE && release_macro_state == ReleaseMacroState::INACTIVE) {
				if (!arm_extended) {
					arm_macro_state = ArmMacroState::LIFT_UP;
					macro_state_start_ms = pros::millis();
					arm_extended = true;
				} else {
					arm_macro_state = ArmMacroState::ARM_RETURN;
					macro_state_start_ms = pros::millis();
					arm_extended = false;
				}
			}
		}
		r1_pressed_last = r1_pressed;

		bool up_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		if (up_pressed && !up_pressed_last && arm_macro_state == ArmMacroState::IDLE &&
		    release_macro_state == ReleaseMacroState::INACTIVE && arm_extended && arm.get_position() >= arm_90 - 3.0) {
			release_macro_state = ReleaseMacroState::RELEASE_HOLDER;
			release_start_ms = pros::millis();
		}
		up_pressed_last = up_pressed;

		bool down_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
		if (down_pressed && !down_pressed_last && arm_macro_state == ArmMacroState::IDLE &&
		    release_macro_state == ReleaseMacroState::INACTIVE) {
			arm_test_at_90 = !arm_test_at_90;
		}
		down_pressed_last = down_pressed;

		bool holder_macro_active = arm_macro_state == ArmMacroState::LIFT_UP ||
			arm_macro_state == ArmMacroState::ARM_OUT ||
			(arm_macro_state == ArmMacroState::IDLE && arm_extended);

		double lift_position = lift_rotation.get_position();
		double lift_motor_position = lift.get_position();
		if (release_macro_state == ReleaseMacroState::RELEASE_HOLDER) {
			holder.move(holder_release_speed);
			if (pros::millis() - release_start_ms >= holder_release_time_ms) {
				release_macro_state = ReleaseMacroState::RAISE_CLEAR;
				release_start_ms = pros::millis();
			}
		} else if (release_macro_state == ReleaseMacroState::RAISE_CLEAR) {
			arm.move_absolute(arm_release_clear, 200);
			holder.move(holder_default_speed);
			if (arm.get_position() >= arm_release_clear - 3.0 ||
			    pros::millis() - release_start_ms >= arm_macro_timeout_ms) {
				arm.move_absolute(arm_release_clear, 200);
				release_macro_state = ReleaseMacroState::INACTIVE;
			}
		} else if (arm_macro_state == ArmMacroState::LIFT_UP) {
			lift.move(127);
			holder.move(holder_macro_speed);
			if (pros::millis() - macro_state_start_ms >= lift_macro_delay_ms) {
				arm_macro_state = ArmMacroState::ARM_OUT;
				macro_state_start_ms = pros::millis();
			}
		} else if (arm_macro_state == ArmMacroState::ARM_OUT) {
			arm.move_absolute(arm_90, 200);
			if (arm.get_position() >= arm_90 - 3.0 ||
			    pros::millis() - macro_state_start_ms >= arm_macro_timeout_ms) {
				arm.move_absolute(arm_90, 200);
				arm_macro_state = ArmMacroState::IDLE;
				holder.move(holder_default_speed);
			} else {
				holder.move(holder_macro_speed);
			}
		} else if (arm_macro_state == ArmMacroState::ARM_RETURN) {
			arm.move_absolute(arm_start, 200);
			holder.move(holder_default_speed);
			if (arm.get_position() <= arm_start + 3.0 ||
			    pros::millis() - macro_state_start_ms >= arm_macro_timeout_ms) {
				arm.move_absolute(arm_start, 200);
				arm_macro_state = ArmMacroState::IDLE;
			}
		} else {
			arm.move_absolute(arm_test_at_90 ? arm_test_position : arm_start, 200);
			if (holder_macro_active) {
				holder.move(holder_macro_speed);
			} else {
				holder.move(holder_default_speed);
			}
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
				if (lift_position < lift_max) {
					lift.move(127);
					lift_target = lift_motor_position;
				} else {
					lift_target = lift_motor_position;
					lift.move_absolute(lift_target, 100);
				}
			} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				if (lift_position > lift_min) {
					lift.move(-127);
					lift_target = lift_motor_position;
				} else {
					lift_target = lift_motor_position;
					lift.move_absolute(lift_target, 100);
				}
			} else {
				lift.move_absolute(lift_target, 100);
			}
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}