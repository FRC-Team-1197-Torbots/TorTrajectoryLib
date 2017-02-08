package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

public class SmoothSpline extends TorSpline {
	
	TorSpline inputSpline;
	HalfSpiralSpline optimizingSpline;
	double computedRadius;
	double computedXLength;
	
	public SmoothSpline(TorSpline s) {
		super(0.0, 0.0, 0.0);
		inputSpline = s.clone();
		optimizingSpline  = new HalfSpiralSpline();
		PathSegment prev_segment = new ArcSegment(1.0, 1.0);
		PathSegment segment, next_segment;
		for (int i = 0; i < inputSpline.path.size(); i++){
			segment = inputSpline.path.get(i);
			if (i + 1 < inputSpline.path.size()){
				next_segment = inputSpline.path.get(i+1);
			} else {
				next_segment = new ArcSegment(1.0, 1.0);
			}
			if (segment.type() != SegmentType.ARC){
				this.add(segment);
			} else {
				replace(prev_segment, segment, next_segment);
			}
		}
	}
	
	private void replace(PathSegment line1, PathSegment arc, PathSegment line2){
		double angle = arc.totalAngle();
		computeMinRadius(angle, 1.0/arc.curvatureAt(0.0));
		this.add(line1.cloneTrimmedBy(computedXLength));
		this.add(new SpiralSpline(angle, computedRadius));
		this.add(line2.cloneTrimmedBy(computedXLength));
		
	}
	
	private void computeMinRadius(double angle, double radius){
		
	}

}
