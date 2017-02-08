package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.SecantSolver;
import org.apache.commons.math3.analysis.solvers.UnivariateSolver;

public class SmoothSpline extends TorSpline {
	
	TorSpline inputSpline;
	HalfSpiralSpline optimizingSpline;
	UnivariateFunction radiusFunction;
	UnivariateSolver radiusSolver;
	double computedRadius;
	double computedXLength;
	
	public SmoothSpline(TorSpline s) {
		super(s.internalTranslation().getEntry(0), s.internalTranslation().getEntry(1), s.internalRotation());
		inputSpline = s.clone();
		optimizingSpline  = new HalfSpiralSpline();
		PathSegment segment, next_segment;
		for (int i = 0; i < inputSpline.path.size(); i++){
			segment = inputSpline.path.get(i);
			if (i + 1 < inputSpline.path.size()){
				next_segment = inputSpline.path.get(i+1);
				System.out.println("Hello");
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
		computeMinRadius(angle, 1.0/arc.curvatureAt(0.0));
		path.get(index-1).addToLength(-computedXLength);
		this.add(new SpiralSpline(angle, computedRadius));
		this.add(line.cloneTrimmedBy(computedXLength));
	}
	
	private void computeMinRadius(double angle, double radius){
		radiusFunction = new UnivariateFunction() {
			double x, y;
			public double value(double r) {
				optimizingSpline.buildRisingLegOnly(angle, r);
				x = optimizingSpline.rawPivotCoordinatesAt(optimizingSpline.length()).getEntry(0);
				y = optimizingSpline.rawPivotCoordinatesAt(optimizingSpline.length()).getEntry(1);
				computedXLength = Math.abs(x);
				return (radius - y);
			}
		};
		final double relativeAccuracy = 1.0e-6;
		final double absoluteAccuracy = 1.0e-8;
		radiusSolver = new SecantSolver(relativeAccuracy, absoluteAccuracy);
		computedRadius = radiusSolver.solve(10000, radiusFunction, 0, radius, radius);
	}

}
