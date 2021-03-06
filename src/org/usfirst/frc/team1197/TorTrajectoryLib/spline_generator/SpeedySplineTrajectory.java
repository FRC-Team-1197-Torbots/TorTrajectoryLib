package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.usfirst.frc.team1197.TorTrajectoryLib.GlobalMotionLimits;

public class SpeedySplineTrajectory extends SplineTrajectory {
	UnivariateIntegrator integrator;
	UnivariateFunction integrand;

	public SpeedySplineTrajectory(PathSegment p, boolean backward) {
		super();
		path = p.clone();
		goal_pos = path.length();
		goal_head = path.headingAt(goal_pos);
		
		max_vel = GlobalMotionLimits.DANGER_FACTOR * GlobalMotionLimits.MAX_VEL; // TODO: Doing DF^2 is a hack, this deserves actual math.
		max_acc = GlobalMotionLimits.MAX_ACC;
		max_jerk = GlobalMotionLimits.MAX_JERK;
		
		// fastest speed we can go around a turn:
		double max_cornering_vel = GlobalMotionLimits.MAX_THROTTLE * GlobalMotionLimits.MAX_WHEEL_SPEED;
		max_omg = max_cornering_vel / GlobalMotionLimits.MIN_TURN_RADIUS;
		max_alf = GlobalMotionLimits.MAX_ALF;
		max_jeta = GlobalMotionLimits.MAX_JETA;
		
		time.clear();
		translation.clear();
		rotation.clear();
		
		buildSpeedySpline();
		walk();
		goal_pos = translation.get(translation.size()-1).pos;
		goal_head = rotation.get(rotation.size()-1).pos;;
		if(backward){
			flipSign(translation);
		}
	}

	private void buildSpeedySpline() {
		build(outsideWheelPathLength(), max_vel, max_acc, max_jerk, translation);
		double s = 0.0;
		double v = 0.0;
		double a = 0.0;
		double last_s;
		double last_v;
		double r_center;
		double r_outside;
		for (int i = 0; i < time.size(); i++) {
			last_s = s;
			last_v = v;
			if (path.curvatureAt(s) == 0.0) {
				v = translation.get(i).vel;
			} else {
				r_center = Math.abs(1.0 / path.curvatureAt(s));
				r_outside = Math.abs(1.0 / path.curvatureAt(s)) + GlobalMotionLimits.HALF_TRACKWIDTH;
				v = translation.get(i).vel * (r_center / r_outside);
			}
			s = last_s + 0.5 * (last_v + v) * dt;
			a = (v - last_v) / dt;
			translation.get(i).set(s, v, a);
		}
	}
	
	private double outsideWheelPathLength() {
		final double relativeAccuracy = 1.0e-6;
		final double absoluteAccuracy = 1.0e-8;
		final int minIterations = 8;
		final int maxIterations = 64;
		integrator = new SimpsonIntegrator(relativeAccuracy, absoluteAccuracy, minIterations, maxIterations);
		integrand = new UnivariateFunction() {
			private double r_center;
			private double r_outside;
			public double value(double s) {
				if(path.curvatureAt(s) == 0.0){
					return 1.0;
				} else {
					r_center = Math.abs(1.0 / path.curvatureAt(s));
					r_outside = Math.abs(1.0 / path.curvatureAt(s)) + GlobalMotionLimits.HALF_TRACKWIDTH;
					return r_outside / r_center;
				}
			}
		};
		return integrator.integrate(10000, integrand, 0, path.length());
	}
	
}