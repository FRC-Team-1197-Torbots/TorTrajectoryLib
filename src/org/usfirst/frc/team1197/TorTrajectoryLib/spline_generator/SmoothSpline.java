package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;


public class SmoothSpline extends TorSpline {
	
	private TorSpline inputSpline;
	private HalfSpiralSpline optimizingSpline = new HalfSpiralSpline();
	private double computedPivotX;
	private double computedPivotY;
	
	public SmoothSpline(TorSpline s) {
		super(s.externalTranslation().getEntry(0), s.externalTranslation().getEntry(1), s.externalRotation());
		inputSpline = s.clone();
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
				replace(i, segment, next_segment);
				i++; //skip the line segment added by replace()
			}
		}
	}
	
	private void replace(int index, PathSegment arc, PathSegment line){
		//TODO: add a check to make sure we have line->arc->line
		double angle = arc.totalAngle();
		double radius = secantMethod(Math.abs(angle), Math.abs(1.0/arc.curvatureAt(0.0)));
		path.get(index-1).addToLength(-computedPivotX);
		this.addToLength(-computedPivotX);
		this.add(new SpiralSpline(angle, radius));
		this.add(line.cloneTrimmedBy(computedPivotX));
	}
	
	private double secantMethod(double angle, double radius){
		double accuracy = 1.0e-8;
		int max_iterations = 100;
		double x = radius, x_prev = radius * 1.001;
		double f = rootFunction(angle, x, radius);
		double f_prev = rootFunction(angle, x_prev, radius);
		double q;
		for(int i = 0; i < max_iterations; i++){
			f = rootFunction(angle, x, radius);
//			System.out.println("x = " + x + ";\tf = " + f);
			if (Math.abs(f) <= accuracy)
				break;
			q = slopeFunction(x, f, x_prev, f_prev);
			x_prev = x;
			f_prev = f;
			x = x - f/q;
		}
		return x;

	}
	
	private double rootFunction(double angle, double radius, double targetY){
		optimizingSpline.buildRisingLegOnly(angle, radius);
		computedPivotX = optimizingSpline.pivot_x();
		computedPivotY = optimizingSpline.pivot_y();
		return targetY - computedPivotY;
	}
	
	private double slopeFunction(double x, double f, double x_prev, double f_prev){
		return (f - f_prev)/(x - x_prev);
	}

}
