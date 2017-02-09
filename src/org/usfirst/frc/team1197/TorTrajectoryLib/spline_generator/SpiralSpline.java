package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SpiralSpline extends TorSpline {
	private double kMax;
	private static final double absoluteMaxAlpha = 9.0;
	private double max_alf;
	private static final double max_jeta = 40.0;
	protected final double ds = 0.0001;
	
	private static final double absoluteMaxVel = 5.056; // See formulas in TorCAN/TorDrive
	private static final double absoluteMinTurnRadius = 0.5;
	private static final double maxThrottle = 0.6 * (absoluteMinTurnRadius/(absoluteMinTurnRadius+0.2858)); // (38%???)
	private static final double max_vel = maxThrottle * absoluteMaxVel;
	private static final double absoluteMaxOmega = max_vel/absoluteMinTurnRadius;
	private double max_omg;
	
	double[] s;
	double[] A;
	double[] B;
	double[] C;

	public SpiralSpline(double angle, double min_radius) {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
		build(angle, min_radius);
	}
	
	public SpiralSpline() {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
	}
	
	public void build(double angle, double min_radius){
		this.clear();
		setTotalAngle(angle);
		setConstants(angle, min_radius, 7);
		for (int i = 0; i < 7; i++){
			if (Math.abs(s[i+1] - s[i]) > ds) {
				this.add(new CornuSpiral(A[i], B[i], C[i], s[i], s[i+1]));
			}
		}
	}
	
	public double pivot_x(double s){
		return rawPivotCoordinatesAt(s).getEntry(0);
	}
	
	public double pivot_y(double s){
		return rawPivotCoordinatesAt(s).getEntry(1);
	}
	
	public RealVector rawPivotCoordinatesAt(double s){
		double radius = 1.0 / curvatureAt(s);
		double angle = rawHeadingAt(s) + Math.signum(totalAngle())*(Math.PI/2.0);
		RealVector pos = new ArrayRealVector(new double[] {Math.cos(angle), Math.sin(angle)});
		pos.mapMultiply(radius);
		pos.add(rawPositionAt(s));
		return pos;
	}
	
	protected void setConstants(double angle, double min_radius, int num_segments) {
		setMotionLimits(angle, min_radius);
		setTotalAngle(angle);
		setLength(setArclengthNodes(angle, num_segments));
		setSpiralCoefficients(angle, num_segments);
	}
	
	protected void setMotionLimits(double angle, double min_radius) {
		double absAngle = Math.abs(angle);
		max_omg = Math.min(absoluteMaxOmega, max_vel/min_radius);
		max_alf = Math.min(absoluteMaxAlpha,
				   		   Math.min(Math.sqrt(max_omg * max_jeta),
				   				    Math.pow((0.5 * absAngle * max_jeta * max_jeta), (1.0/3.0))));
		max_omg = Math.min(max_omg,
						   (-max_alf * max_alf + Math.sqrt(max_alf * max_alf * max_alf * max_alf
							+ 4 * max_jeta * max_jeta * max_alf * absAngle)) 
						    / (2 * max_jeta) );
		kMax = max_omg / max_vel;
	}
	
	protected double setArclengthNodes(double angle, int num_segments){
		s[0] = 0.0;
		if (num_segments < 1)
			return s[0];
		
		s[1] = max_alf * max_vel / max_jeta;
		if (num_segments == 1)
			return s[1];
		
		s[2] = kMax * max_vel * max_vel / max_alf;
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
		
		A[0] = max_jeta / (2 * max_vel * max_vel * max_vel);
		B[0] = 0.0;
		C[0] = 0.0;
		if (num_segments == 1) {
			checkAndNegateCoefficients(angle, 1);
			return;
		}
		
		A[1] = 0.0;
		B[1] = max_alf / (max_vel * max_vel);
		C[1] = -(B[1] * B[1]) / (4 * A[0]);
		if (num_segments == 2) {
			checkAndNegateCoefficients(angle, 2);
			return;
		}
		
		A[2] = -A[0];
		B[2] = -2 * A[2] * s[3];
		C[2] = kMax + A[2] * s[3] * s[3];
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
		B[4] = -2 * A[4] * s[4];
		C[4] = kMax + A[4] * s[4] * s[4];
		if (num_segments == 5) {
			checkAndNegateCoefficients(angle, 5);
			return;
		}
		
		A[5] = 0.0;
		B[5] = -B[1];
		C[5] = C[1] - B[5] * (s[5] + s[2]);
		if (num_segments == 6) {
			checkAndNegateCoefficients(angle, 6);
			return;
		}
		
		A[6] = A[0];
		B[6] = -2 * A[6] * s[7];
		C[6] = A[6] * s[7] * s[7];
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
