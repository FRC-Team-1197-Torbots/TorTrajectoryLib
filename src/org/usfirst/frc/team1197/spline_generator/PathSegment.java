package org.usfirst.frc.team1197.spline_generator;

public abstract class PathSegment {
	protected double length;
	protected Point P;
	public abstract Point pointAt(double t);
	public abstract double x(double t);
	public abstract double y(double t);
	
	public PathSegment(){
		length = 0.0;
	}
	
	public double length(){
		return length;
	}
}
