package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class SpiralSpline extends TorSpline {

	public SpiralSpline(LineSegment line1, LineSegment line2) {
		super(0.0, 0.0, 0.0);
	}
	public SpiralSpline(LineSegment line1, ArcSegment arc2) {
		super(0.0, 0.0, 0.0);
	}
	public SpiralSpline(ArcSegment arc1, LineSegment line2) {
		super(0.0, 0.0, 0.0);
	}
	public SpiralSpline(ArcSegment arc1, ArcSegment arc2){
		super(0.0, 0.0, 0.0);
	}

}
