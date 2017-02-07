package org.usfirst.frc.team1197.TorTrajectoryLib;

public class MotionLimit {
	
	public double vel;
	public double acc;
	public double jerk;
	
	public MotionLimit(double v, double a , double j){
		vel = v;
		acc = a;
		jerk = j;
	}
	
	public void set(double v, double a , double j){
		vel = v;
		acc = a;
		jerk = j;
	}
}
