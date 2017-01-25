package org.usfirst.frc.team1197.spline_generator;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class Spline extends PathSegment{
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
	
	@Override
	public String toString() {
		String s = new String("Spline:\n");
		int i = 0;
		for(PathSegment segment:path){
			s.concat("\tsegment ")
			 .concat(String.valueOf(i))
			 .concat(" = ")
			 .concat(segment.toString())
			 .concat("\n");
			i++;
		}
		return s;
	}

	@Override
	public PathSegment clone() {
		Spline splineCopy = new Spline(startPoint.getEntry(0), startPoint.getEntry(1), startHeading);
		for(PathSegment segment:path){
			splineCopy.add(segment.clone());
		}
		return splineCopy;
	}
	
	public void add(PathSegment segment){
		if (segment == this)
			segment = segment.clone();
		if (path.size() > 0){
			RealVector nextStartingPoint = positionAt(length());
			double nextStartingHeading = headingAt(length);
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

	@Override
	public double curvatureAt(double s){
		double lengthSoFar = 0.0;
		for (PathSegment segment : path){
			if (s - lengthSoFar <= segment.length)
				return segment.curvatureAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		return 0.0;
	}
}
