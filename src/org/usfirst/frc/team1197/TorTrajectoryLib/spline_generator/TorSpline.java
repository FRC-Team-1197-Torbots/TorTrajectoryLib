package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class TorSpline extends PathSegment {
	
	private List<PathSegment> path;

	public TorSpline(double start_x, double start_y, double start_head) {
		super(start_x, start_y, start_head);
		path = new ArrayList<PathSegment>();
		setLength(0.0);
	}

	@Override
	public String toString() {
		String s = new String("Spline:\n");
		int i = 0;
		for (PathSegment segment : path) {
			s = s.concat("Segment ").concat(String.valueOf(i)).concat(" = ").concat(segment.toString()).concat("\n");
			i++;
		}
		return s;
	}

	@Override
	public PathSegment clone() {
		TorSpline splineCopy = new TorSpline(internalTranslation().getEntry(0),
											 internalTranslation().getEntry(1),
											 internalRotation());
		for (PathSegment segment : path) {
			splineCopy.add(segment);
		}
		return splineCopy;
	}

	public void add(PathSegment s) {
		PathSegment segment = s.clone();
		if (path.size() > 0) {
			RealVector nextStartingPoint = rawPositionAt(length());
			double nextStartingHeading = rawHeadingAt(length());
			segment.translateExternally(nextStartingPoint);
			segment.rotateExternally(nextStartingHeading);
		}
		addToLength(segment.length());
		path.add(segment);
		System.out.println(segment);
		System.out.println(segment.length());
	}

	@Override
	public RealVector rawPositionAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length())
				return internalTransform(segment.positionAt(s - lengthSoFar));
			lengthSoFar += segment.length();
		}
		PathSegment lastSegment = path.get(path.size() - 1);
		return internalTransform(lastSegment.positionAt(lastSegment.length()));
	}

	@Override
	public double rawHeadingAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length())
				return internalRotation() + segment.headingAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		PathSegment lastSegment = path.get(path.size() - 1);
		return internalRotation() + lastSegment.headingAt(lastSegment.length());
	}

	@Override
	public double curvatureAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length())
				return segment.curvatureAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		PathSegment lastSegment = path.get(path.size() - 1);
		return internalRotation() + lastSegment.curvatureAt(lastSegment.length());
	}
}
