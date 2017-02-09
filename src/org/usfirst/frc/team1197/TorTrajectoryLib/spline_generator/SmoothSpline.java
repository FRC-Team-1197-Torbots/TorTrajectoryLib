package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;


public class SmoothSpline extends TorSpline {
	
	TorSpline inputSpline;
	static HalfSpiralSpline optimizingSpline = new HalfSpiralSpline();
	static double computedPivotX;
	static double computedPivotY;
	
	public SmoothSpline(TorSpline s) {
		super(s.internalTranslation().getEntry(0), s.internalTranslation().getEntry(1), s.internalRotation());
		inputSpline = s.clone();
//		optimizingSpline = new HalfSpiralSpline();
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
		double radius = secantMethod(angle, Math.abs(1.0/arc.curvatureAt(0.0)));
		optimizingSpline.buildRisingLegOnly(angle, radius);
		computedPivotX = optimizingSpline.pivot_x();
		computedPivotY = optimizingSpline.pivot_y();
//		System.out.println("end point = " + optimizingSpline.rawPositionAt(optimizingSpline.length()));
//		System.out.println("end heading = " + optimizingSpline.rawHeadingAt(optimizingSpline.length()));
//		System.out.println("end curvature = " + optimizingSpline.curvatureAt(optimizingSpline.length()));
//		System.out.println("computedPivotX = " + computedPivotX);
//		System.out.println("computedPivotY = " + computedPivotY);
//		System.out.println("radius = " + 1.0/arc.curvatureAt(0.0));
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
			System.out.println("x = " + x + ";\tf = " + f);
			if (Math.abs(f) <= accuracy)
				break;
			q = slopeFunction(x, f, x_prev, f_prev);
			x_prev = x;
			f_prev = f;
			x = x - f/q;
		}
		return x;

	}
	
	static int count = 1;
	private static double rootFunction(double angle, double radius, double targetY){
		optimizingSpline.buildRisingLegOnly(angle, radius);
		computedPivotX = optimizingSpline.pivot_x();
		computedPivotY = optimizingSpline.pivot_y();
		count++;
		return targetY - computedPivotY;
	}
	
//	private double rootFunction(HalfSpiralSpline spline, double targetY){
//		computedPivotX = spline.pivot_x();
//		computedPivotY = spline.pivot_y();
//		System.out.println(count);
//		System.out.println(spline);
//		count++;
//		return targetY - computedPivotY;
//	}
	
	private static double slopeFunction(double x, double f, double x_prev, double f_prev){
		return (f - f_prev)/(x - x_prev);
	}

}
