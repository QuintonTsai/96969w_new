#include "main.h"
#include "lemlib/api.hpp"
#include <atomic>

pros::MotorGroup left_mg({-9, -10}, pros::v5::MotorGears::blue);
pros::MotorGroup right_mg({1, 2}, pros::v5::MotorGears::blue);
pros::Imu imu(19);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(13, pros::v5::MotorGears::blue);
pros::MotorGroup lift({3, -8}, pros::v5::MotorGears::green);
pros::Motor arm(7, pros::v5::MotorGears::green);
pros::Motor holder(6, pros::v5::MotorGears::green);
pros::Rotation lift_rotation(5);
// reversed so the angle counts up as the arm raises, the same direction as the arm motor
pros::Rotation arm_rotation(-12);

double heading() {
	return imu.get_rotation();
}

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
enum class AutonSelection { RIGHT, LEFT };
std::atomic<AutonSelection> selected_auton{AutonSelection::RIGHT};
std::atomic<bool> auton_selector_active{false};

void render_auton_selector() {
	AutonSelection selection = selected_auton.load();
	pros::lcd::print(0, "Auton: %s", selection == AutonSelection::RIGHT ? "RIGHT" : "LEFT");
	pros::lcd::print(1, "Left btn: Left");
	pros::lcd::print(2, "Right btn: Right");
	pros::lcd::print(3, selection == AutonSelection::LEFT ? "Left route not set" : "Right route ready");
}

void select_left_auton() {
	selected_auton.store(AutonSelection::LEFT);
	render_auton_selector();
}

void select_right_auton() {
	selected_auton.store(AutonSelection::RIGHT);
	render_auton_selector();
}

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

// lift_rotation counts in centidegrees
constexpr double lift_rotations = 360 * 100;

// drive the lift at full power, up or down, until lift_rotation reaches target_cdeg, then hold it there;
// gives up after timeout_ms in case it jams or the sensor stops reporting
void lift_move_to(double target_cdeg, int64_t timeout_ms = 1500) {
	bool up = lift_rotation.get_position() < target_cdeg;
	int64_t start_ms = pros::millis();
	while ((up ? lift_rotation.get_position() < target_cdeg : lift_rotation.get_position() > target_cdeg) &&
	       pros::millis() - start_ms < timeout_ms) {
		lift.move(up ? 127 : -127);
		pros::delay(10);
	}
	lift.move_absolute(lift.get_position(), 100);
}

// same as lift_move_to, in its own task so it can happen during a drive motion
void lift_move_async(double target_cdeg, int64_t timeout_ms = 1500) {
	pros::Task([target_cdeg, timeout_ms]() { lift_move_to(target_cdeg, timeout_ms); });
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
			if (!auton_selector_active.load()) {
				lemlib::Pose pose = chassis.getPose();
				// LemLib tracks in inches; shown in cm to match the auton distances
				pros::lcd::print(1, "X: %.1f cm", pose.x * 2.54);
				pros::lcd::print(2, "Y: %.1f cm", pose.y * 2.54);
				pros::lcd::print(3, "Theta: %.1f deg", pose.theta);
				pros::lcd::print(4, "Arm: %.1f deg", arm_angle());
				pros::lcd::print(5, "Lift: %.0f", lift_rotation.get_position());
			}
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
void competition_initialize() {
	auton_selector_active.store(true);
	pros::lcd::clear();
	pros::lcd::register_btn0_cb(select_left_auton);
	pros::lcd::register_btn2_cb(select_right_auton);
	render_auton_selector();
}

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
void right_autonomous() {
	chassis.setPose(0, 0, 0);
	// the arm is fully down at the start of every match, so re-zero it here; the zero from initialize()
	// is off if the program was started with the arm raised, which shifts every absolute arm target up
	arm_rotation.reset_position();
	// hold so each motion stops where LemLib ends it instead of coasting into the next one
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	double arm_target = arm_angle();


	// the opening moves exit early at speed and flow into each other like the old timed arcs,
	// so chain them off the target heading rather than the heading they exited at
	float target_heading = 0;

	// short back up (was 100 ms full reverse)
	driveRelative(target_heading, -5, 500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	// arc right forward, pivoting on the right side (was 500 ms at left 127 / right 25)
	target_heading += 40; // TUNE angle
	chassis.swingToHeading(target_heading, lemlib::DriveSide::RIGHT, 1000,
	                        {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	
	target_heading -= 40; // TUNE angle
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
	target_heading -= 62.5; // TUNE angle
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	arm_target += 55.6;
	arm_move_to(arm_target, 200);
	int64_t arm_start_ms = pros::millis();

	// reverse 65 cm
	driveRelative(target_heading, -65, 2000);
	chassis.waitUntilDone();

	// wait
	arm_wait(arm_target, 200, arm_start_ms);

	//FIRST PIN
	pros::delay(300);
	holder.move(-127);
	pros::delay(200);
	holder.move(0);

	arm_target = 105;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);


	pros::delay(150);

	// back up, turn right, back up; ends where the old forward/turn/back did, 28.6 cm back and
	// 33.3 cm left of here. turning 60 right makes that 9.3 cm back, then 38.5 cm back
	// (the turn has to be over about 50 for the first back up to stay backward)
	driveRelative(target_heading, 25, 1000, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();
	pros::delay(200);

	target_heading += 50; // TUNE angle
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	pros::delay(200);

	driveRelative(target_heading, -67.5, 2000, 127, 127, 2.54); // TUNE distance
	chassis.waitUntilDone();

	// the old path ended facing 30 right of here, so drop back to that heading; the left turn
	// below then lands on the same absolute heading as before (it turns 30 more than it says)

	holder.move(127);

	arm_target = 52.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();

	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(250);
	// raise arm while turning
	arm_target = 70;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	holder.move(0);

	// turn left; 150 from the old heading, 180 from where the robot actually faces, so force the
	// direction (shortest path is a coin flip at 180)
	target_heading -= 170;
	// lift goes up to 3 rotations on its sensor while the robot turns
	lift_move_async(3 * lift_rotations);
	chassis.turnToHeading(target_heading, 1500,
	                      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// drive backward (was 1.5 s full reverse); the timeout still ends it if the robot is pushing against something
	driveRelative(target_heading, -51, 1500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();
	
	//SECOND PIN
	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(300);
	holder.move(-127);
	pros::delay(200);
	holder.move(0);

	// raise the arm to 100
	arm_target = 110;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);

	// forward 30 cm, then lower the lift 3 rotations
	lift_move_to(lift_rotation.get_position() - 3 * lift_rotations);
	driveRelative(target_heading, 30, 1000, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	target_heading += 50;
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// backward 50 cm
	driveRelative(target_heading, -85, 1500, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();

	holder.move(127);

	// lower the arm to 55
	arm_target = 52.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(200);

	// turn left 120 while the arm raises to 70
	holder.move(0);
	arm_target = 72.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	target_heading -= 165;
	chassis.turnToHeading(target_heading, 1500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// backward 30 cm while the lift raises 6 rotations
	lift_move_async(lift_rotation.get_position() + 6 * lift_rotations, 3000);
	pros::delay(500);
	driveRelative(target_heading, -45, 1000, 127, 80, 2.5); // TUNE distance
	chassis.waitUntilDone();
	pros::delay(300);
	holder.move(-127);


}

void left_autonomous() {
	// reset pose and hold the drivetrain and arm during autonomous
	chassis.setPose(0, 0, 0);
	arm_rotation.reset_position();
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	double arm_target = arm_angle();
	float target_heading = 0;

	driveRelative(target_heading, -5, 500, 127, 80, 2.5);
	chassis.waitUntilDone();

	// mirrored opening swings, then turn toward the first pin
	target_heading -= 40;
	chassis.swingToHeading(target_heading, lemlib::DriveSide::LEFT, 1000,
	                        {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	target_heading += 40;
	chassis.swingToHeading(target_heading, lemlib::DriveSide::LEFT, 1000,
	                       {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	target_heading += 40;
	chassis.swingToHeading(target_heading, lemlib::DriveSide::RIGHT, 1000,
	                       {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	target_heading += 62.5;
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	arm_target += 55.6;
	arm_move_to(arm_target, 200);
	int64_t arm_start_ms = pros::millis();
	// reverse to the first pin while raising the arm
	driveRelative(target_heading, -65, 2000);
	chassis.waitUntilDone();
	arm_wait(arm_target, 200, arm_start_ms);

	pros::delay(300);
	holder.move(-127);
	pros::delay(200);
	holder.move(0);

	arm_target = 105;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(150);

	// clear the first pin, then turn and cross to the next scoring area
	driveRelative(target_heading, 25, 1000, 127, 80, 2.5);
	chassis.waitUntilDone();
	pros::delay(200);
	target_heading -= 50;
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	pros::delay(200);
	driveRelative(target_heading, -67.5, 2000, 127, 127, 2.54);
	chassis.waitUntilDone();

	holder.move(127);
	// lower the arm and prepare to turn across the field
	arm_target = 52.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(250);
	arm_target = 70;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	holder.move(0);

	target_heading += 170;
	// raise the lift while turning and backing toward the second pin
	lift_move_async(3 * lift_rotations);
	chassis.turnToHeading(target_heading, 1500,
	                      {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	driveRelative(target_heading, -51, 1500, 127, 80, 2.5);
	chassis.waitUntilDone();

	arm_wait(arm_target, 200, arm_start_ms);
	// release the second pin
	pros::delay(300);
	holder.move(-127);
	pros::delay(200);
	holder.move(0);

	arm_target = 110;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);

	// lower the lift, then reposition for the final approach
	lift_move_to(lift_rotation.get_position() - 3 * lift_rotations);
	driveRelative(target_heading, 30, 1000, 127, 80, 2.5);
	chassis.waitUntilDone();
	target_heading -= 50;
	chassis.turnToHeading(target_heading, 1000, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();
	driveRelative(target_heading, -85, 1500, 127, 80, 2.5);
	chassis.waitUntilDone();

	holder.move(127);
	arm_target = 52.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	arm_wait(arm_target, 200, arm_start_ms);
	pros::delay(200);
	holder.move(0);
	arm_target = 72.5;
	arm_move_to(arm_target, 200);
	arm_start_ms = pros::millis();
	// turn toward the final target while raising the lift
	target_heading += 165;
	chassis.turnToHeading(target_heading, 1500, {.minSpeed = 127, .earlyExitRange = 10});
	chassis.waitUntilDone();

	// raise the lift while driving to the final scoring position
	lift_move_async(lift_rotation.get_position() + 6 * lift_rotations, 3000);
	pros::delay(500);
	driveRelative(target_heading, -45, 1000, 127, 80, 2.5);
	chassis.waitUntilDone();
	pros::delay(300);
	holder.move(-127);
}

void autonomous() {
	auton_selector_active.store(false);
	if (selected_auton.load() == AutonSelection::RIGHT) {
		right_autonomous();
	} else {
		left_autonomous();
	}
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
	auton_selector_active.store(false);
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
	constexpr double arm_out_position = 70;
	constexpr double arm_release_clear = 100.0;
	// the UP release only raises the arm to arm_release_clear afterwards when the arm is above this
	constexpr double arm_raise_min = 50.0;
	// height the R1 macro raises the lift to so the arm can swing out, on lift_rotation (centidegrees)
	constexpr double lift_macro_height = 1.75 * 360 * 100; // TUNE, read it off the brain screen
	// fallback in case the lift jams or the sensor stops reporting
	constexpr int64_t lift_macro_timeout_ms = 1000;
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
	// lift height when the R1 macro started, so the return lowers it back to where the driver had it
	double lift_macro_start = 0;
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
					lift_macro_start = lift_rotation.get_position();
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
		// also allowed while the arm is still swinging out: ARM_OUT can take up to its 3 s timeout to
		// settle, and a press during that window used to be dropped. the arm motor keeps driving to
		// arm_out_position on its own, so ending ARM_OUT here doesn't stop the arm
		if (up_pressed && !up_pressed_last &&
		    (arm_macro_state == ArmMacroState::IDLE || arm_macro_state == ArmMacroState::ARM_OUT) &&
		    release_macro_state == ReleaseMacroState::INACTIVE) {
			arm_macro_state = ArmMacroState::IDLE;
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
				// only raise to clear when the arm is out; with it low the release just spits the piece out.
				// also skip when it's already past the clear height (raising would pull it back down)
				if (arm_angle() <= arm_raise_min || arm_angle() > arm_release_clear) {
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
			holder.move(holder_macro_speed);
			// already at or above the height (driver raised it) skips straight to the arm
			if (lift_position >= lift_macro_height ||
			    pros::millis() - macro_state_start_ms >= lift_macro_timeout_ms) {
				lift_target = lift_motor_position;
				lift.move_absolute(lift_target, 100);
				arm_macro_state = ArmMacroState::ARM_OUT;
				macro_state_start_ms = pros::millis();
			} else {
				lift.move(127);
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
			if (lift_position <= lift_macro_start || lift_position <= lift_min ||
			    pros::millis() - macro_state_start_ms >= lift_macro_timeout_ms) {
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