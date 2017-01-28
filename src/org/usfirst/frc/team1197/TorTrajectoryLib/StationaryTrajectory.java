package org.usfirst.frc.team1197.TorTrajectoryLib;

public class StationaryTrajectory extends TorTrajectory{
	
	public StationaryTrajectory() {
		super(0.0, 0.0);
	}
	
	public boolean lookUpIsLast(long t){
		return true;
	}
	
	public double lookUpPosition(long t){
		return goal_pos;
	}
	public double lookUpVelocity(long t){
		return 0.0;
	}
	public double lookUpAcceleration(long t){
		return 0.0;
	}
	
	public double lookUpHeading(long t){
		return 0;
	}
	public double lookUpOmega(long t){
		return 0;
	}
	public double lookUpAlpha(long t){
		return 0;
	}
	
	//TODO figure out what to do instead of this:
//	public void execute(){
//		TorMotionProfile.INSTANCE.loadTrajectory(this);
//	}

}
