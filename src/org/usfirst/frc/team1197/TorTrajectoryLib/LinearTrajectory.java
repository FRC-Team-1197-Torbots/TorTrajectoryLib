package org.usfirst.frc.team1197.TorTrajectoryLib;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal, 0.0);
		max_vel = GlobalMotionLimits.MAX_VEL;
		max_acc = GlobalMotionLimits.MAX_ACC;
		max_jerk = GlobalMotionLimits.MAX_JERK;
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
	}
	
	public double lookUpHeading(long t){
		return 0.0;
	}
	
	public double lookUpOmega(long t){
		return 0.0;
	}
	
	public double lookUpAlpha(long t){
		return 0.0;
	}

}