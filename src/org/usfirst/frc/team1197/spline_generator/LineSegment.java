package org.usfirst.frc.team1197.spline_generator;

public class LineSegment extends PathSegment{
	
	private double x_cos;
	private double y_cos;

	public LineSegment(double x_length, double y_length){
		super();
		length = Math.sqrt(x_length*x_length + y_length*y_length);
		x_cos = x_length/length;
		y_cos = y_length/length;
	}
	
	@Override
	public double x(double t) {
		return x_cos*t;
	}

	@Override
	public double y(double t) {
		return y_cos*t;
	}

	@Override
	public Point pointAt(double t) {
		Point P = new Point(x_cos*t, y_cos*t);
		return P;
	}

}
