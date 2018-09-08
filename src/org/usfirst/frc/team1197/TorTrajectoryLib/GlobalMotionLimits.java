package org.usfirst.frc.team1197.TorTrajectoryLib;

public class GlobalMotionLimits {

	public static final double DANGER_FACTOR = 1;

	public static final double MAX_WHEEL_SPEED = 5.056; // See formulas in TorCAN/TorDrive STILL NEED TO DO
	public static final double MAX_VEL = DANGER_FACTOR * MAX_WHEEL_SPEED; // 3.8ish
	public static final double MAX_ACC = 6.0; // TO DO
	public static final double MAX_JERK = 20.0; // TO DO

	public static final double HALF_TRACKWIDTH = 0.2; //TO DO
	public static final double MAX_OMG = (DANGER_FACTOR * MAX_WHEEL_SPEED) / HALF_TRACKWIDTH; // 13.3ish												// rad/s
	public static final double MAX_ALF = 9.0; // TO DO
	public static final double MAX_JETA = 40.0; // TO DO

	public static final double MIN_TURN_RADIUS = 0.4; // TO DO
	public static final double MAX_THROTTLE = DANGER_FACTOR * (MIN_TURN_RADIUS / (MIN_TURN_RADIUS + HALF_TRACKWIDTH));
	public static final double MAX_SPLINE_VEL = MAX_THROTTLE * MAX_WHEEL_SPEED;
	public static final double MAX_SPLINE_OMG = MAX_SPLINE_VEL / MIN_TURN_RADIUS;

}
