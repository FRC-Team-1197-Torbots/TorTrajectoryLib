package org.usfirst.frc.team1197.spline_generator;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class Spline {
	private RealVector startPoint;
	private double startHeading;
	private List<PathSegment> path;
	
	public Spline(double start_x, double start_y, double start_head){
		startPoint = new ArrayRealVector(new double[] {start_x, start_y}, false);
		startHeading = start_head;
		path = new LinkedList<PathSegment>();
	}
	
	public void add(PathSegment segment){
		if (path.size() > 0){
			PathSegment previousSegment = path.get(path.size()-1);
			RealVector nextStartingPoint = previousSegment.positionAt(previousSegment.length());
			double nextStartingHeading = previousSegment.headingAt(previousSegment.length());
			segment.translateTo(nextStartingPoint);
			segment.rotateTo(nextStartingHeading);
			path.add(segment);
		} else {
			segment.translateTo(startPoint);
			segment.rotateTo(startHeading);
			path.add(segment);
		}

	}
	
	public RealVector positionAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (lengthSoFar + segment.length() >= s){
				return segment.positionAt(s);
			} else {
				lengthSoFar += segment.length();
			}
		}
		return null;
	}
}
