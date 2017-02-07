package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class ArcSegment extends PathSegment {
	
	private double curvature;
	private double totalAngle;

	public ArcSegment(double radius, double angle){
		super(0.0, 0.0, 0.0);
		curvature = 1.0/radius;
		totalAngle = angle;
		length = radius * totalAngle;
	}
	@Override
	public String toString() {
		return "ArcSegment[curvature=".concat(String.valueOf(curvature))
				  .concat(", angle=")
				  .concat(String.valueOf(totalAngle))
				  .concat("]") ;
	}
	
	public ArcSegment clone(){
		ArcSegment arcCopy = new ArcSegment((1.0/curvature), totalAngle);
		arcCopy.translateInternally(internalTranslation());
		arcCopy.rotateInternally(internalRotation());
		return arcCopy;
	}

	@Override
	public RealVector rawPositionAt(double s) {
		double x = (1.0/curvature)*Math.sin(curvature*s);
		double y = (1.0/curvature)*(1-Math.cos(curvature*s));
		RealVector pos = new ArrayRealVector(new double[] {x, y});
		return internalTransform(pos);
	}

	@Override
	public double rawHeadingAt(double s) {
		return internalRotation()+curvature*s;
	}

	@Override
	public double curvatureAt(double s) {
		return curvature;
	}

}
