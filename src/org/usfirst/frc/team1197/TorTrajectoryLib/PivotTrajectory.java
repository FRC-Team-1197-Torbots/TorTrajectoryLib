package org.usfirst.frc.team1197.TorTrajectoryLib;

public class PivotTrajectory extends TorTrajectory {
	
	public PivotTrajectory(double goal){
		super(0.0, goal * (Math.PI/180.0));
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_head, max_omg, max_alf, max_jeta, rotation);
	}

}