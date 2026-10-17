#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup left_mg({-9, -10}, pros::v5::MotorGears::blue);
pros::MotorGroup right_mg({1, 2}, pros::v5::MotorGears::blue);
pros::Imu imu(19);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(11, pros::v5::MotorGears::blue);
pros::MotorGroup lift({3, -8}, pros::v5::MotorGears::green);
pros::Motor arm(7, pros::v5::MotorGears::green);
pros::Motor holder(6, pros::v5::MotorGears::green);
pros::Rotation lift_rotation(5);
// reversed so the angle counts up as the arm raises, the same direction as the arm motor
pros::Rotation arm_rotation(-12);

// the arm sensor reads 100 degrees where the arm motor read 2700, so the motor turns 27 degrees per arm degree
constexpr double arm_motor_deg_per_deg = 27.0;

// arm angle from the rotation sensor, in degrees (get_position is in centidegrees)
double arm_angle() {
	return arm_rotation.get_position() / 100.0;
}

// last target sent by arm_move_to, as an arm angle and as the matching motor position
double arm_target_angle = 0;
double arm_motor_target = 0;
int64_t arm_move_last_ms = 0;

// move the arm to an absolute sensor angle; the motor's own PID does the moving, but the target is
// measured from the sensor so slop or slip between the motor and the arm doesn't shift it
// the motor target is only measured at the start of a move (new angle, or no call in the last 100 ms);
// re-measuring it every loop sends a slightly different target each time, which restarts the motor's
// speed ramp and leaves the arm crawling
void arm_move_to(double angle_deg, int velocity) {
	if (angle_deg != arm_target_angle || pros::millis() - arm_move_last_ms > 100) {
		arm_target_angle = angle_deg;
		arm_motor_target = arm.get_position() + (angle_deg - arm_angle()) * arm_motor_deg_per_deg;
	}
	arm_move_last_ms = pros::millis();
	arm.move_absolute(arm_motor_target, velocity);
}

// true once the arm is within 0.75 degrees of the arm_move_to target, or the motor has reached the
// position it was sent to (slop can leave the sensor a little short even then)
bool arm_settled() {
	return std::abs(arm_angle() - arm_target_angle) <= 0.75 || std::abs(arm.get_position() - arm_motor_target) <= 20;
}

// wait until the arm reaches angle_deg or timeout_ms has passed since start_ms
void arm_wait(double angle_deg, int velocity, int64_t start_ms, int64_t timeout_ms = 3000) {
	while (pros::millis() - start_ms < timeout_ms) {
		arm_move_to(angle_deg, velocity);
		if (arm_settled()) {
			break;
		}
		pros::delay(20);
	}
}

lemlib::Drivetrain drivetrain(&left_mg, &right_mg,
                               11.75, // track width, in inches
                               lemlib::Omniwheel::NEW_325, // 3.25" omni wheels
                               450, // wheel rpm after external gearing
                               2); // horizontalDrift, 2 since not using traction wheels

// no tracking wheels; odometry falls back to drive motor encoders + IMU
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// PID constants below are copied from the LemLib-stable project's tuned drivetrain (same motors, wheels and rpm);
// this robot's weight and track width differ, so fine-tune from here
lemlib::ControllerSettings lateral_controller(8.2, // proportional gain (kP)
                                               0, // integral gain (kI)
                                               35, // derivative gain (kD)
                                               3, // integral anti windup range
                                               1, // small error range, in inches
                                               100, // small error range timeout, in ms
                                               3, // large error range, in inches
                                               500, // large error range timeout, in ms
                                               20); // maximum acceleration (slew)

lemlib::ControllerSettings angular_controller(2.12, // proportional gain (kP)
                                               0.082, // integral gain (kI)
                                               12, // derivative gain (kD)
                                               3, // integral anti windup range
                                               1, // small error range, in degrees
                                               100, // small error range timeout, in ms
                                               3, // large error range, in degrees
                                               500, // large error range timeout, in ms
                                               0); // maximum acceleration (slew)

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                       10, // minimum output where drivetrain will move out of 127
                                       1.019); // expo curve gain

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                    10, // minimum output where drivetrain will move out of 127
                                    1.019); // expo curve gain

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	// zero once at program start (arm and lift down) so auton and driver control share the same reference
	lift.tare_position();
	lift_rotation.reset_position();
	arm_rotation.reset_position();
	chassis.calibrate();

	// show the odometry pose on the brain screen, used to measure/tune auton targets
	pros::Task pose_display([]() {
		while (true) {
			lemlib::Pose pose = chassis.getPose();
			// LemLib tracks in inches; shown in cm to match the auton distances
			pros::lcd::print(1, "X: %.1f cm", pose.x * 2.54);
			pros::lcd::print(2, "Y: %.1f cm", pose.y * 2.54);
			pros::lcd::print(3, "Theta: %.1f deg", pose.theta);
			pros::lcd::print(4, "Arm: %.1f deg", arm_angle());
			pros::delay(50);
		}
	});
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
// previous timed version of the auton, kept for reference; not called anywhere
void timed_autonomous() {

	holder.move(127);

	// arc right
	int64_t start = pros::millis();
	while (pros::millis() - start < 100) {
		right_mg.move(-127);
		left_mg.move(-127);
		pros::delay(20);
	}

	start = pros::millis();
	while (pros::millis() - start < 500) {
		right_mg.move(25);
		left_mg.move(127);
		pros::delay(20);
	}

	// arc left backward
	start = pros::millis();
	while (pros::millis() - start < 250) {
		right_mg.move(-25);
		left_mg.move(-127);
		pros::delay(20);
	}

	// arc left
	start = pros::millis();
	while (pros::millis() - start < 400) {
		right_mg.move(127);
		left_mg.move(25);
		pros::delay(20);
	}

	// drive coasts, so let it settle before reading heading
	left_mg.move(0);
	right_mg.move(0);
	pros::delay(100); 

	// turn left  degrees
	chassis.turnToHeading(chassis.getPose().theta - 75, 1500);
	chassis.waitUntilDone();

	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	double arm_target = arm_angle() + 55.6;
	arm_move_to(arm_target, 200);
	int64_t arm_start_ms = pros::millis();

	// reverse
	const float reverse_in = 65 / 2.54;
	lemlib::Pose pose = chassis.getPose();
	float heading_rad = lemlib::degToRad(pose.theta);
	chassis.moveToPoint(pose.x - reverse_in * std::sin(heading_rad),
	                    pose.y - reverse_in * std::cos(heading_rad), 2000, {.forwards = false});
	chassis.waitUntilDone();

	// wait
	arm_wait(arm_target, 200, arm_start_ms);

	pros::delay(300);
	holder.move(-127);
	pros::delay(300);
	holder.move(0);

	arm_target = 88.9;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	// arc right
	start = pros::millis();
	while (pros::millis() - start < 300) {
		right_mg.move(15);
		left_mg.move(127);
		pros::delay(20);
	}

	arm_wait(arm_target, 200, arm_start_ms);

	start = pros::millis();
	while (pros::millis() - start < 50) {
		right_mg.move(-127);
		left_mg.move(127);
		pros::delay(20);
	}

	right_mg.move(0);
	left_mg.move(0);

	lemlib::Pose drive_start_pose = chassis.getPose();
	//const float target_distance_in = 70.00f / 2.54f;
	const float target_distance_in = 55.00f / 2.54f;
	start = pros::millis();
	while (pros::millis() - start < 8000) {
		lemlib::Pose current_pose = chassis.getPose();
		float delta_x = current_pose.x - drive_start_pose.x;
		float delta_y = current_pose.y - drive_start_pose.y;
		if (std::sqrt(delta_x * delta_x + delta_y * delta_y) >= target_distance_in) {
			break;
		}
		left_mg.move(-127);
		//right_mg.move(-127);
		right_mg.move(-70);
		pros::delay(20);
	}
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	left_mg.move(0);
	right_mg.move(0);
    
	holder.move(127);

	arm_target = 55.6;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	arm_wait(arm_target, 200, arm_start_ms);

	// raise arm while turning
	arm_target = 88.9;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	// turn left 135 degrees at the end of auton
	chassis.turnToHeading(chassis.getPose().theta - 160, 1500);
	chassis.waitUntilDone();

	// drive backward
	start = pros::millis();
	while (pros::millis() - start < 1500) {
		left_mg.move(-127);
		right_mg.move(-127);
		pros::delay(20);
	}
	left_mg.move(0);
	right_mg.move(0);

	arm_wait(arm_target, 200, arm_start_ms);

	arm_start_ms = pros::millis();
	while (pros::millis() - arm_start_ms < 1000) {
		arm.move(-127);
		pros::delay(20);
	}
	arm.move(0);

	holder.move(-127);
	pros::delay(300);
	holder.move(0);

}

// pose offset from the robot's current position, in the frame of heading_deg; distances in cm
// early-exited motions leave the robot a few degrees off, so callers pass the heading it was aiming for
// LemLib heading is clockwise from +Y, so forward is (sin, cos) and right is (cos, -sin)
lemlib::Pose relativePose(float heading_deg, float forward_cm, float right_cm, float turn_deg) {
	// LemLib works in inches
	float forward_in = forward_cm / 2.54f;
	float right_in = right_cm / 2.54f;
	lemlib::Pose pose = chassis.getPose();
	float heading_rad = lemlib::degToRad(heading_deg);
	return lemlib::Pose(pose.x + forward_in * std::sin(heading_rad) + right_in * std::cos(heading_rad),
	                    pose.y + forward_in * std::cos(heading_rad) - right_in * std::sin(heading_rad),
	                    heading_deg + turn_deg);
}

// drive straight along heading_deg; distances in cm, negative distance drives backward
// min_speed + early_exit_cm let the robot carry speed into the next motion instead of stopping
void driveRelative(float heading_deg, float distance_cm, int timeout, float max_speed = 127, float min_speed = 0, float early_exit_cm = 0) {
	lemlib::Pose target = relativePose(heading_deg, distance_cm, 0, 0);
	chassis.moveToPoint(target.x, target.y, timeout,
	                    {.forwards = distance_cm >= 0, .maxSpeed = max_speed, .minSpeed = min_speed,
	                     .earlyExitRange = early_exit_cm / 2.54f});
}

// values marked TUNE were estimated from the old timed version; measure them with the pose display
void autonomous() {
	chassis.setPose(0, 0, 0);
	// the arm is fully down at the start of every match, so re-zero it here; the zero from initialize()
	// is off if the program was started with the arm raised, which shifts every absolute arm target up
	arm_rotation.reset_position();
	// hold so each motion stops where LemLib ends it instead of coasting into the next one
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	// the opening moves exit early at speed and flow into each other like the old timed arcs,
	// so chain them off the target heading rather than the heading they exited at
	float target_heading = 0;

	// short back up (was 100 ms full reverse)
	driveRelative(target_heading, -5, 500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	// arc right forward, pivoting on the right side (was 500 ms at left 127 / right 25)
	target_heading += 60; // TUNE angle
	chassis.swingToHeading(target_heading, lemlib::DriveSide::RIGHT, 1000,
	                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	
	target_heading -= 50; // TUNE angle
	chassis.swingToHeading(target_heading, lemlib::DriveSide::RIGHT, 1000,
	                       {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	
	// turn left in place with both sides (replaces the old backward-left and forward-left arcs)
	target_heading -= 40; // TUNE angle
	chassis.swingToHeading(target_heading, lemlib::DriveSide::LEFT, 1000,
	                      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// backward, forward, backward; each exits early at speed like the opening back up
	/*driveRelative(target_heading, 15, 500, 127, 127, 2.5); // TUNE distance
	chassis.waitUntilDone();

	driveRelative(target_heading, -15, 500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	driveRelative(target_heading, 15, 500, 127, 127, 2.5); // TUNE distance
	chassis.waitUntilDone();
    */
	// turn left 80 degrees (the arcs used to contribute 30 of this); this one stops precisely
	// since the 65 cm reverse depends on it
	target_heading -= 65;
	chassis.turnToHeading(target_heading, 1500);
	chassis.waitUntilDone();

	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	double arm_target = arm_angle() + 55.6;
	arm_move_to(arm_target, 200);
	int64_t arm_start_ms = pros::millis();

	// reverse 65 cm
	driveRelative(target_heading, -65, 2000);
	chassis.waitUntilDone();

	// wait
	arm_wait(arm_target, 200, arm_start_ms);

	pros::delay(300);
	holder.move(-127);
	pros::delay(300);
	holder.move(0);

	arm_target = 105;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	// forward, turn, backward; replaces a swing right 80 (right side locked) followed by a backward
	// curve 46 cm back, 27 cm right, 60 left, which together end 19.9 cm back and 28.3 cm left
	// of here, facing 20 right. turning 30 right makes that forward 29.1 cm then back 56.6 cm
	// both targets come from this pose up front, so early exits along the way don't shift the end point
	// extra_back_cm extends the back up past that point along the same 30 degree line
	const float extra_back_cm = 10; // TUNE distance
	const float back_line_rad = lemlib::degToRad(30);
	lemlib::Pose forward_target = relativePose(target_heading, 29.1, 0, 0);
	lemlib::Pose backward_target = relativePose(target_heading,
	                                            -19.9 - extra_back_cm * std::cos(back_line_rad),
	                                            -28.3 - extra_back_cm * std::sin(back_line_rad), 30);

	chassis.moveToPoint(forward_target.x, forward_target.y, 1000,
	                    {.forwards = true, .minSpeed = 127, .earlyExitRange = 2.5f / 2.54f});
	chassis.waitUntilDone();

	// point the back of the robot at the end point (30 right)
	chassis.turnToPoint(backward_target.x, backward_target.y, 1000,
	                    {.forwards = false, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	arm_wait(arm_target, 200, arm_start_ms);

	chassis.moveToPoint(backward_target.x, backward_target.y, 2000,
	                    {.forwards = false, .minSpeed = 127, .earlyExitRange = 2.54f / 2.54f});
	chassis.waitUntilDone();

	// the robot now faces 30 right instead of the old 20, but keep the old heading so the
	// 160 turn below still lands on the same absolute heading (it turns 170 instead)
	target_heading = backward_target.theta;

	holder.move(127);

	arm_target = 55.6;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(150);
	// raise arm while turning
	arm_target = 88.9;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	holder.move(0);

	// turn left 160 degrees
	target_heading -= 150;
	chassis.turnToHeading(target_heading, 1500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// drive backward (was 1.5 s full reverse); the timeout still ends it if the robot is pushing against something
	driveRelative(target_heading, -51, 1500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	arm_wait(arm_target, 200, arm_start_ms);

	// lower the arm (was 1 s at full reverse, roughly 44 degrees of travel from 89)
	arm_target = 44.4; // TUNE angle
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	arm_wait(arm_target, 200, arm_start_ms);

	holder.move(-127);
	pros::delay(300);
	holder.move(0);

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
	// LemLib motions run in their own task, so one still running when auton is cut off would keep
	// driving the motors and fight the joysticks until it times out
	chassis.cancelAllMotions();
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	constexpr double lift_min = 0;
	constexpr double lift_max = 9.89 * 360 * 100;
	constexpr double arm_start = 0.0;
	constexpr double arm_min = 0.0;
	constexpr double arm_max = 111.1;
	constexpr double arm_out_position = 80;
	constexpr double arm_release_clear = 100.0;
	constexpr double arm_release_min = 55.6;
	constexpr int64_t lift_macro_delay_ms = 150;
	constexpr int64_t holder_release_time_ms = 250;
	constexpr int64_t arm_macro_timeout_ms = 3000;
	const int holder_default_speed = 50;
	const int holder_macro_speed = 127;
	const int holder_release_speed = -127;
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	holder.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	double lift_target = lift.get_position();
	enum class ArmMacroState {
		IDLE,
		LIFT_UP,
		ARM_OUT,
		ARM_RETURN,
		LIFT_DOWN
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
	bool arm_extended = false;
	int64_t macro_state_start_ms = 0;
	int64_t release_start_ms = 0;


	while (true) {

		// Arcade control scheme, driven by the LemLib chassis
		int dir = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		// desaturateBias favors turning authority over throttle when motors would otherwise saturate,
		// so turning stays responsive even at high forward/backward speed
		chassis.arcade(dir, turn, false, 0.5);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			intake.move(127);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
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
					arm_macro_state = ArmMacroState::LIFT_DOWN;
					macro_state_start_ms = pros::millis();
					arm_extended = false;
				}
			}
		}
		r1_pressed_last = r1_pressed;

		bool up_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
		if (up_pressed && !up_pressed_last && arm_macro_state == ArmMacroState::IDLE &&
		    release_macro_state == ReleaseMacroState::INACTIVE && arm_angle() > arm_release_min) {
			release_macro_state = ReleaseMacroState::RELEASE_HOLDER;
			release_start_ms = pros::millis();
		}
		up_pressed_last = up_pressed;

		bool holder_macro_active = arm_macro_state == ArmMacroState::LIFT_UP ||
			arm_macro_state == ArmMacroState::ARM_OUT ||
			(arm_macro_state == ArmMacroState::IDLE && arm_extended);

		double lift_position = lift_rotation.get_position();
		double lift_motor_position = lift.get_position();
		if (release_macro_state == ReleaseMacroState::RELEASE_HOLDER) {
			holder.move(holder_release_speed);
			lift.move_absolute(lift_target, 100);
			if (pros::millis() - release_start_ms >= holder_release_time_ms) {
				// arm already past the clear height, so skip raising it (would pull it back down)
				if (arm_angle() > arm_release_clear) {
					release_macro_state = ReleaseMacroState::INACTIVE;
				} else {
					release_macro_state = ReleaseMacroState::RAISE_CLEAR;
					release_start_ms = pros::millis();
				}
			}
		} else if (release_macro_state == ReleaseMacroState::RAISE_CLEAR) {
			arm_move_to(arm_release_clear, 200);
			holder.move(holder_default_speed);
			lift.move_absolute(lift_target, 100);
			if (arm_settled() ||
			    pros::millis() - release_start_ms >= arm_macro_timeout_ms) {
				arm_move_to(arm_release_clear, 200);
				release_macro_state = ReleaseMacroState::INACTIVE;
			}
		} else if (arm_macro_state == ArmMacroState::LIFT_UP) {
			lift.move(127);
			holder.move(holder_macro_speed);
			if (pros::millis() - macro_state_start_ms >= lift_macro_delay_ms) {
				lift_target = lift_motor_position;
				arm_macro_state = ArmMacroState::ARM_OUT;
				macro_state_start_ms = pros::millis();
			}
		} else if (arm_macro_state == ArmMacroState::ARM_OUT) {
			arm_move_to(arm_out_position, 200);
			lift.move_absolute(lift_target, 100);
			if (arm_settled() ||
			    pros::millis() - macro_state_start_ms >= arm_macro_timeout_ms) {
				arm_move_to(arm_out_position, 200);
				arm_macro_state = ArmMacroState::IDLE;
				holder.move(holder_default_speed);
			} else {
				holder.move(holder_macro_speed);
			}
		} else if (arm_macro_state == ArmMacroState::ARM_RETURN) {
			arm_move_to(arm_start, 200);
			holder.move(holder_default_speed);
			lift.move_absolute(lift_target, 100);
			if (arm_settled() ||
			    pros::millis() - macro_state_start_ms >= arm_macro_timeout_ms) {
				arm_move_to(arm_start, 200);
				arm_macro_state = ArmMacroState::IDLE;
			}
		} else if (arm_macro_state == ArmMacroState::LIFT_DOWN) {
			holder.move(holder_default_speed);
			if (pros::millis() - macro_state_start_ms >= lift_macro_delay_ms || lift_position <= lift_min) {
				lift_target = lift_motor_position;
				lift.move_absolute(lift_target, 100);
				arm_macro_state = ArmMacroState::ARM_RETURN;
				macro_state_start_ms = pros::millis();
			} else {
				lift.move(-127);
			}
		} else {
			double arm_position = arm_angle();
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && arm_position < arm_max) {
				arm.move(127);
			} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && arm_position > arm_min) {
				arm.move(-127);
			} else {
				arm.move(0);
			}
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