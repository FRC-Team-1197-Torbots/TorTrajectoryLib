package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.usfirst.frc.team1197.TorTrajectoryLib.MotionState1D;
import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class SplineTrajectory extends TorTrajectory {
	protected PathSegment path;
	protected final static double absoluteMaxVel = 5.056; // See formulas in TorCAN/TorDrive
	protected final static double absoluteMinTurnRadius = 0.5;
	protected final static double halfTrackWidth = 0.2858;
	protected final static double dangerFactor = 0.6;
	protected final static double maxThrottle = dangerFactor * (absoluteMinTurnRadius 
													/ (absoluteMinTurnRadius + halfTrackWidth));
	
	public SplineTrajectory(PathSegment p, boolean backward) {
		path = p.clone();
		goal_pos = path.length();
		goal_head = path.headingAt(goal_pos);
		max_vel = maxThrottle * absoluteMaxVel;
		max_omg = max_vel/absoluteMinTurnRadius;
		
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
		
		walk();
		if(backward){
			flipSign(translation);
		}
	}
	
	protected SplineTrajectory() {
		// do nothing
	}

	protected void walk() {
		double s = 0.0;
		double v = 0.0;
		double head = 0.0;
		double omg = 0.0;
		double last_omg;
		double alf = 0.0;
		for (long t : time) {
			last_omg = omg;
			s = lookUpPosition(t);
			v = lookUpVelocity(t);
			head = path.headingAt(s);
			omg = v * path.curvatureAt(s);
			alf = (omg - last_omg) / dt;
			rotation.add(new MotionState1D(head, omg, alf));
		}
	}
	
}
