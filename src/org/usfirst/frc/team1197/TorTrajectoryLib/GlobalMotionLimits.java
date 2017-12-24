package org.usfirst.frc.team1197.TorTrajectoryLib;

public class GlobalMotionLimits {

	public static final double DANGER_FACTOR = 0.75;

	public static final double MAX_WHEEL_SPEED = 5.056; // See formulas in TorCAN/TorDrive
	public static final double MAX_VEL = DANGER_FACTOR * MAX_WHEEL_SPEED; // 3.8ish
	public static final double MAX_ACC = 6.0;
	public static final double MAX_JERK = 20.0;

	public static final double HALF_TRACKWIDTH = 0.2858; // Measured on the robot
	public static final double MAX_OMG = (DANGER_FACTOR * MAX_WHEEL_SPEED) / HALF_TRACKWIDTH; // 13.3ish												// rad/s
	public static final double MAX_ALF = 9.0;
	public static final double MAX_JETA = 40.0;

	public static final double MIN_TURN_RADIUS = 0.4;
	public static final double MAX_THROTTLE = DANGER_FACTOR * (MIN_TURN_RADIUS / (MIN_TURN_RADIUS + HALF_TRACKWIDTH));
	public static final double MAX_SPLINE_VEL = MAX_THROTTLE * MAX_WHEEL_SPEED;
	public static final double MAX_SPLINE_OMG = MAX_SPLINE_VEL / MIN_TURN_RADIUS;

}
