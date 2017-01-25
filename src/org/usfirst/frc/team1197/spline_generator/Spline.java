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
			PathSegment previousSegment = path.get(path.size()-1);
			RealVector nextStartingPoint = positionAt(length());
			double nextStartingHeading = previousSegment.headingAt(length());
			segment.translateTo(nextStartingPoint);
			segment.rotateBy(nextStartingHeading);
			path.add(segment);
		} else {
			segment.translateTo(startPoint);
			segment.rotateBy(startHeading);
			path.add(segment);
		}
		length += segment.length();

	}
	
	public RealVector positionAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (lengthSoFar + segment.length() >= s){
				return segment.positionAt(s-lengthSoFar);
			} else {
				lengthSoFar += segment.length();
			}
		}
		return null;
	}
	
	public double headingAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (lengthSoFar + segment.length() >= s){
				return segment.headingAt(s-lengthSoFar);
			} else {
				lengthSoFar += segment.length();
			}
		}
		return 0.0;
	}
	
	public double length(){
		return length;
	}
}
