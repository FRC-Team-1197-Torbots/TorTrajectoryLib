package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class LineSegment extends PathSegment{
	
	public LineSegment(double l, double r){
		super();
		length = l;
		rotateInternally(r);
	}
	
	public String toString(){
		return "LineSegment[length=".concat(String.valueOf(length))
									  .concat(", rotation=")
									  .concat(String.valueOf(internalRotation()))
									  .concat("]") ;
	}

	@Override
	public RealVector positionAt(double s) {
		RealVector pos = new ArrayRealVector(new double[] {s, 0.0});
		return outputTransform(pos);
	}

	@Override
	public double headingAt(double s) {
		return totalRotation();
	}

	@Override
	public double curvatureAt(double s) {
		return 0.0;
	}

	@Override
	public LineSegment clone() {
		LineSegment lineCopy = new LineSegment(length, internalRotation());
		lineCopy.translateInternally(internalTranslation());
		lineCopy.rotateInternally(internalRotation());
		return lineCopy;
	}

}
