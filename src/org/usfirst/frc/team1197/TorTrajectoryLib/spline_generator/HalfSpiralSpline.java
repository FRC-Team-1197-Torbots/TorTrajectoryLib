package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class HalfSpiralSpline extends SpiralSpline{
	
	public HalfSpiralSpline(double angle, double min_radius) {
		super();
		s = new double[8];
		A = new double[7];
		B = new double[7];
		C = new double[7];
		totalAngle = angle;
		buildRisingLegOnly(min_radius);
	}

	public void buildRisingLegOnly(double min_radius){
		setConstants(1.0/min_radius);
		this.clear();
		for (int i = 0; i < 4; i++){
			if (Math.abs(s[i+1] - s[i]) > ds) {
				this.add(new CornuSpiral(A[i], B[i], C[i], s[i], s[i+1]));
			}
		}
	}
}
