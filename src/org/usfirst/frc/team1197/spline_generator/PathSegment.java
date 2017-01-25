package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public abstract class PathSegment {
	protected double length;
	protected double rotation;
	protected RealMatrix rotationMatrix;
	protected RealVector translationVector;
	public abstract String toString();
	public abstract RealVector positionAt(double s);
	public abstract double headingAt(double s);
	public abstract double curvatureAt(double s);
	
	
	public PathSegment(){
		translationVector = new ArrayRealVector(new double[] {0.0, 0.0}, false);
	}
	
	public double length(){
		return length;
	}
	
	public double rotation(){
		return rotation;
	}
	
	public void rotateTo(double r){
		rotation = r;
		rotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(r),  Math.cos(r)}, 
																  {Math.sin(r), -Math.sin(r)}});
	}
	
	public void rotateBy(double delta_theta){
		double r = rotation + delta_theta;
		rotateTo(r);
	}
	
	public void translateTo(RealVector v){
		translationVector = v;
	}
}
