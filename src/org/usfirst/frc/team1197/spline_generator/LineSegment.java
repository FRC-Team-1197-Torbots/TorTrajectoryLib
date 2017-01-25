package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class LineSegment extends PathSegment{
	
	public LineSegment(double length, double t){
		super();
		this.length = length;
		rotation = t;
		rotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(t), Math.cos(t)}, {Math.sin(t), -Math.sin(t)}});
	}

	@Override
	public RealVector positionAt(double s) {
		RealVector pos = new ArrayRealVector(new double[] {s, 0.0}, false);
		pos = rotationMatrix.operate(pos).add(translationVector);
		return pos;
	}

	@Override
	public double headingAt(double s) {
		// TODO Auto-generated method stub
		return rotation;
	}

	@Override
	public double curvatureAt(double s) {
		// TODO Auto-generated method stub
		return 0.0;
	}

}
