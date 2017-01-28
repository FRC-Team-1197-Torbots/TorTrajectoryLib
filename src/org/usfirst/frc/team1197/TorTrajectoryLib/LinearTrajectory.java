package org.usfirst.frc.team1197.TorTrajectoryLib;

public class LinearTrajectory extends TorTrajectory{
	
	public LinearTrajectory(double goal) {
		super(goal, 0.0);
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
	}
	
	public double lookUpPosition(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return goal_pos;
		}
		return translation.get(i).pos;
	}
	public double lookUpVelocity(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return translation.get(i).vel;
	}
	public double lookUpAcceleration(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return translation.get(i).acc;
	}
	
	
	public double lookUpOmega(long t){
		return 0;
	}
	public double lookUpAlpha(long t){
		return 0;
	}
	public double lookUpHeading(long t){
		return 0;
	}
}