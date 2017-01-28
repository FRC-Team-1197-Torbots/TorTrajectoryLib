package org.usfirst.frc.team1197.TorTrajectoryLib;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal, 0.0);
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
	}

}