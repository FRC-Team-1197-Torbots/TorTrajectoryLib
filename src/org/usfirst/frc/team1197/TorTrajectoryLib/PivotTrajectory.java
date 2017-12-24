package org.usfirst.frc.team1197.TorTrajectoryLib;

public class PivotTrajectory extends TorTrajectory {
	
	public PivotTrajectory(double goal){
		super(0.0, goal * (Math.PI/180.0));
		max_omg = GlobalMotionLimits.MAX_OMG;
		max_alf = GlobalMotionLimits.MAX_ALF;
		max_jeta = GlobalMotionLimits.MAX_JETA;
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_head, max_omg, max_alf, max_jeta, rotation);
	}
	
	public double lookUpPosition(long t){
		return 0.0;
	}
	
	public double lookUpVelocity(long t){
		return 0.0;
	}
	
	public double lookUpAcceleration(long t){
		return 0.0;
	}

}