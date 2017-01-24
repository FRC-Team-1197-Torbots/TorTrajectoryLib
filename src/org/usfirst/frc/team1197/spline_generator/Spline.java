package org.usfirst.frc.team1197.spline_generator;

import java.util.LinkedList;
import java.util.List;

public class Spline {
	private Point startPoint;
	private List<PathSegment> path;
	
	private Point P;
	
	public Spline(Point startPoint){
		this.startPoint = startPoint;
		path = new LinkedList<PathSegment>();
	}
	
	public void add(PathSegment segment){
		path.add(segment);
	}
	
	public Point pointAt(double t){
		P = startPoint;
		return P;
	}
}
