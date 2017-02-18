package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class HalfSpiralSpline extends SpiralSpline{
	
	public HalfSpiralSpline() {
		super();
	}
	
	public HalfSpiralSpline(double angle) {
		super();
		buildFirstHalfOnly(angle, absoluteMinTurnRadius);
	}
	
	public HalfSpiralSpline(double angle, double min_radius) {
		super();
		buildFirstHalfOnly(angle, min_radius);
	}
	
	public void buildFirstHalfOnly(double angle, double min_radius) {
		this.clear();
		setTotalAngle(angle);
		setConstants(angle, min_radius, 4);
		for (int i = 0; i < 4; i++){
			if (Math.abs(s[i+1] - s[i]) > ds) {
				this.add(new CornuSpiral(A[i], B[i], C[i], s[i], s[i+1]));
			}
		}
	}
	
}
