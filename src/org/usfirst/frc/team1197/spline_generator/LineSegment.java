package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class LineSegment extends PathSegment{
	
	public LineSegment(double l, double r){
		super();
		length = l;
		rotateTo(r);
		System.out.println(rotation);
	}
	
	public String toString(){
		return "LineSegment[length=".concat(String.valueOf(length))
									  .concat(", rotation=")
									  .concat(String.valueOf(rotation))
									  .concat("]") ;
	}

	@Override
	public RealVector positionAt(double s) {
		RealVector pos = new ArrayRealVector(new double[] {s, 0.0}, false);
		pos = rotationMatrix.operate(pos).add(translationVector);
		return pos;
	}

	@Override
	public double headingAt(double s) {
		return rotation;
	}

	@Override
	public double curvatureAt(double s) {
		return 0.0;
	}

}
