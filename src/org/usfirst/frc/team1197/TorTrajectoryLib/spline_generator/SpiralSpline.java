package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.usfirst.frc.team1197.TorTrajectoryLib.GlobalMotionLimits;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SpiralSpline extends TorSpline {
//	private static final double absoluteMaxAlpha = 9.0;
//	protected static final double absoluteMaxVel = 5.056; // See formulas in TorCAN/TorDrive
//	protected static final double absoluteMinTurnRadius = 0.5;
//	protected static final double halfTrackWidth = 0.2858;
//	protected static final double maxThrottle = 0.6 * (absoluteMinTurnRadius/(absoluteMinTurnRadius+halfTrackWidth)); // (38%???)
//	protected static final double max_vel = maxThrottle * absoluteMaxVel;
//	protected static final double absoluteMaxOmega = max_vel/absoluteMinTurnRadius;
	
	// Motion limits for all Spiral Splines:
	protected static final double maxWheelSpeed = GlobalMotionLimits.MAX_WHEEL_SPEED;
	protected static final double absoluteMinTurnRadius = GlobalMotionLimits.MIN_TURN_RADIUS;
	protected static final double absoluteMaxVel = GlobalMotionLimits.MAX_SPLINE_VEL;
	protected static final double absoluteMaxOmega = GlobalMotionLimits.MAX_SPLINE_OMG;
	protected static final double absoluteMaxAlpha = GlobalMotionLimits.MAX_ALF;
	protected static final double absoluteMaxJeta = GlobalMotionLimits.MAX_JETA;
	
	protected double max_omg;
	protected double max_alf;
	protected double kMax; // Maximum curvature of a particular spline
	
	double[] s;
	double[] A;
	double[] B;
	double[] C;
	
	public SpiralSpline() {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
	}
	
	public SpiralSpline(double angle) {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
		build(angle, absoluteMinTurnRadius);
	}
	
	public SpiralSpline(double angle, double min_radius) {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
		build(angle, min_radius);
	}
	
	public void build(double angle, double min_radius){
		System.out.println("Desired Minimum Radius: " + min_radius);
		this.clear();
		setTotalAngle(angle);
		setConstants(angle, min_radius, 7);
		for (int i = 0; i < 7; i++){
			if (Math.abs(s[i+1] - s[i]) >= absoluteAccuracy) {
				this.add(new CornuSpiral(A[i], B[i], C[i], 0, s[i+1] - s[i]));
			}
		}
	}
	
	public double pivot_x(double s){
		return rawPivotCoordinatesAt(s).getEntry(0);
	}
	
	public double pivot_y(double s){
		return rawPivotCoordinatesAt(s).getEntry(1);
	}
	
	public double pivot_x(){
		return pivot_x(0.5*(s[3]+s[4]));
	}
	
	public double pivot_y(){
		return pivot_y(0.5*(s[3]+s[4]));
	}
	
	public RealVector rawPivotCoordinatesAt(double s){
		double radius = Math.abs(1.0 / curvatureAt(s));
		double angle = rawHeadingAt(s) + Math.signum(totalAngle())*(Math.PI/2.0);
		RealVector pos = new ArrayRealVector(new double[] {Math.cos(angle), Math.sin(angle)});
		pos = pos.mapMultiply(radius);
		pos = pos.add(rawPositionAt(s));
		return pos;
	}
	
	protected void setConstants(double angle, double min_radius, int num_segments) {
		System.out.println("Desired Minimum Radius: " + min_radius);
		setMotionLimits(angle, min_radius);
		setTotalAngle(angle);
		setLength(setArclengthNodes(angle, num_segments));
		setSpiralCoefficients(angle, num_segments);
	}
	
	protected void setMotionLimits(double angle, double min_radius) {
		double absAngle = Math.abs(angle);
		max_omg = Math.min(absoluteMaxOmega, absoluteMaxVel/min_radius);
		max_alf = Math.min(absoluteMaxAlpha,
				   		   Math.min(Math.sqrt(max_omg * absoluteMaxJeta),
				   				    Math.pow((0.5 * absAngle * absoluteMaxJeta * absoluteMaxJeta), (1.0/3.0))));
		max_omg = Math.min(max_omg,
						   (-max_alf * max_alf + Math.sqrt(max_alf * max_alf * max_alf * max_alf
							+ 4 * absoluteMaxJeta * absoluteMaxJeta * max_alf * absAngle)) 
						    / (2 * absoluteMaxJeta) );
		kMax = max_omg / absoluteMaxVel;
		System.out.println("kMax: " + kMax + "\nMinimum Radius: " + (1/kMax));
	}
	
	protected double setArclengthNodes(double angle, int num_segments){
		s[0] = 0.0;
		if (num_segments < 1)
			return s[0];
		
		s[1] = max_alf * absoluteMaxVel / absoluteMaxJeta;
		if (num_segments == 1)
			return s[1];
		
		s[2] = kMax * absoluteMaxVel * absoluteMaxVel / max_alf;
		if (num_segments == 2)
			return s[2];
		
		s[3] = s[1] + s[2];
		if (num_segments == 3)
			return s[3];
		
		s[4] = Math.abs(angle) / kMax;
		if (num_segments == 4)
			return s[4];
		
		s[5] = s[1] + s[4];
		if (num_segments == 5)
			return s[5];
		
		s[6] = s[2] + s[4];
		if (num_segments == 6)
			return s[6];
		
		s[7] = s[3] + s[4];
		return s[7];
	}
	
	protected void setSpiralCoefficients(double angle, int num_segments){
		if (num_segments < 1)
			return;
		
		A[0] = absoluteMaxJeta / (2 * absoluteMaxVel * absoluteMaxVel * absoluteMaxVel);
		B[0] = 0.0;
		C[0] = 0.0;
		if (num_segments == 1) {
			checkAndNegateCoefficients(angle, 1);
			return;
		}
		
		A[1] = 0.0;
		B[1] = max_alf / (absoluteMaxVel * absoluteMaxVel);
		C[1] = A[0] * s[1] * s[1];
		if (num_segments == 2) {
			checkAndNegateCoefficients(angle, 2);
			return;
		}
		
		A[2] = -A[0];
		B[2] = -2 * A[2] * s[1];
		C[2] = kMax + A[2] * s[1] * s[1];
		if (num_segments == 3) {
			checkAndNegateCoefficients(angle, 3);
			return;
		}
		
		A[3] = 0.0;
		B[3] = 0.0;
		C[3] = kMax;
		if (num_segments == 4) {
			checkAndNegateCoefficients(angle, 4);
			return;
		}
		
		A[4] = A[2];
		B[4] = 0;
		C[4] = kMax;
		if (num_segments == 5) {
			checkAndNegateCoefficients(angle, 5);
			return;
		}
		
		A[5] = 0.0;
		B[5] = -B[1];
		C[5] = C[2];
		if (num_segments == 6) {
			checkAndNegateCoefficients(angle, 6);
			return;
		}
		
		A[6] = A[0];
		B[6] = -2 * A[6] * s[1];
		C[6] = C[1];
		checkAndNegateCoefficients(angle, 7);
		return;
	}
	
	protected void checkAndNegateCoefficients(double angle, int num_segments){
		if (angle < 0.0){
			for (int i = 0; i < num_segments; i++){
				A[i] = -A[i];
				B[i] = -B[i];
				C[i] = -C[i];
			}
		}
	}

}
