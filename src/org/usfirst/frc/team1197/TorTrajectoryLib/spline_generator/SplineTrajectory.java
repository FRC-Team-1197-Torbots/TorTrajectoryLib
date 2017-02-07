package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.usfirst.frc.team1197.TorTrajectoryLib.MotionState1D;
import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class SplineTrajectory extends TorTrajectory {
	private PathSegment path;
	public final static double absoluteMaxVel = 5.056; // See formulas in TorCAN/TorDrive
	public final static double absoluteMinTurnRadius = 0.5;
	public final static double maxThrottle = 0.6 * (absoluteMinTurnRadius/(absoluteMinTurnRadius+0.2858)); // (38%???)
	public final static double max_vel = maxThrottle * absoluteMaxVel;
	public final static double max_omg = max_vel/absoluteMinTurnRadius;
	

	public SplineTrajectory(PathSegment p) {
		path = p.clone();
		goal_pos = path.length();
		goal_head = path.headingAt(goal_pos);
		time.clear();
		translation.clear();
		rotation.clear();
		build(goal_pos, max_vel, max_acc, max_jerk, translation);
		walk();
	}

	private void walk() {
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
	
	@Override
	public double adjustedMaxAlpha(){
		return 0.0;
	}
}
