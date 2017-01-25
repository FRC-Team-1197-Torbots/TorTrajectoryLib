package org.usfirst.frc.team1197.spline_generator;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class Spline {
	private RealVector startPoint;
	private double startHeading;
	private double length;
	private List<PathSegment> path;
	
	public Spline(double start_x, double start_y, double start_head){
		startPoint = new ArrayRealVector(new double[] {start_x, start_y}, false);
		startHeading = start_head;
		path = new LinkedList<PathSegment>();
		length = 0.0;
	}
	
	public void add(PathSegment segment){
		if (path.size() > 0){
			RealVector nextStartingPoint = positionAt(length());
			PathSegment previousSegment = path.get(path.size()-1);
			double nextStartingHeading = previousSegment.headingAt(previousSegment.length());
			System.out.println(nextStartingHeading);
			segment.translateTo(nextStartingPoint);
			segment.rotateTo(nextStartingHeading);
			path.add(segment);
		} else {
			segment.translateTo(startPoint);
			segment.rotateTo(startHeading);
			path.add(segment);
		}
		length += segment.length();

	}
	
	public RealVector positionAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (s - lengthSoFar <= segment.length)
				return segment.positionAt(s-lengthSoFar);
			lengthSoFar += segment.length();
		}
		return null;
	}
	
	public double headingAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (s - lengthSoFar <= segment.length)
				return segment.headingAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		return 0.0;
	}
	
	public double length(){
		return length;
	}
}
