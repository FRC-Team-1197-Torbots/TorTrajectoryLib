package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class HalfSpiralSpline extends SpiralSpline{
	
	public HalfSpiralSpline(double angle, double min_radius) {
		super();
		buildRisingLegOnly(angle, min_radius);
	}

	public HalfSpiralSpline() {
		super();
	}
	
	public void buildRisingLegOnly(double angle, double min_radius) {
		this.clear();
		setTotalAngle(angle);
		setConstants(angle, min_radius, 3);
		for (int i = 0; i < 3; i++){
			if (Math.abs(s[i+1] - s[i]) > ds) {
				this.add(new CornuSpiral(A[i], B[i], C[i], s[i], s[i+1]));
			}
		}
	}
	
	public double pivot_x(){
		return pivot_x(s[3]);
	}
	
	public double pivot_y(){
		return pivot_y(s[3]);
	}
	
}
