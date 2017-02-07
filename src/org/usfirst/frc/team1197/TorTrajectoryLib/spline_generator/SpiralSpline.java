package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class SpiralSpline extends TorSpline {
	private double totalAngle;
	private double kMax;
	private double max_vel, max_omg, max_alf, max_jeta;
	double[] s;
	double[] A;
	double[] B;
	double[] C;

	public SpiralSpline(double angle, double max_curvature) {
		super(0.0, 0.0, 0.0);
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
		totalAngle = angle;
		build(max_curvature);
	}
	
	public void build(double max_curvature){
		setConstants(max_curvature);
		path.clear();
		for (int i = 0; i < 8; i++){
			path.add(new CornuSpiral(A[i], B[i], C[i], s[i], s[i+1]));
		}
	}
	
	private void setConstants(double max_curvature){
		setMotionLimits(max_curvature);
		setArclengthNodes();
		setSpiralCoefficients();
	}
	
	private void setMotionLimits(double max_curvature){
		double absAngle = Math.abs(totalAngle);
		kMax = Math.min(max_curvature, 1.0/SplineTrajectory.absoluteMinTurnRadius);
		max_vel = SplineTrajectory.max_vel;
		max_omg = Math.min(SplineTrajectory.max_omg, kMax*max_vel);
		max_jeta = SplineTrajectory.max_jeta;
		max_alf = Math.min(SplineTrajectory.max_alf,
				   		   Math.min(Math.sqrt(max_omg * max_jeta),
				   				    Math.pow((0.5 * absAngle * max_jeta * max_jeta), (1.0/3.0))));
		max_omg = Math.min(max_omg,
						   (-max_alf * max_alf + Math.sqrt(max_alf * max_alf * max_alf * max_alf
							+ 4 * max_jeta * max_jeta * max_alf * absAngle)) 
						    / (2 * max_jeta) );
		kMax = max_omg / max_vel;
	}
	
	private void setArclengthNodes(){
		s[0] = 0.0;
		s[1] = max_alf * max_vel / max_jeta;
		s[2] = kMax * max_vel * max_vel / max_alf;
		s[3] = s[1] + s[2];
		s[4] = Math.abs(totalAngle) / kMax;
		s[5] = s[1] + s[4];
		s[6] = s[2] + s[4];
		s[7] = s[3] + s[4];
	}
	
	private void setSpiralCoefficients(){
		A[0] = max_jeta / (2 * max_vel * max_vel * max_vel);
		B[0] = 0.0;
		C[0] = 0.0;
		
		A[1] = 0.0;
		B[1] = max_alf / (max_vel * max_vel);
		C[1] = -(B[1] * B[1]) / (4 * A[0]);
		
		A[2] = -A[0];
		B[2] = -2 * A[2] * s[2];
		C[2] = kMax + A[2] * s[3] * s[3];
		
		A[3] = 0.0;
		B[3] = 0.0;
		C[3] = kMax;
		
		A[4] = A[2];
		B[4] = -2 * A[4] * s[4];
		C[4] = kMax + A[4] * s[4] * s[4];
		
		A[5] = 0.0;
		B[5] = -B[1];
		C[5] = C[1] - B[5] * (s[5] + s[2]);
		
		A[6] = A[0];
		B[6] = -2 * A[6] * s[7];
		C[6] = A[6] * s[7] * s[7];
	}

}
