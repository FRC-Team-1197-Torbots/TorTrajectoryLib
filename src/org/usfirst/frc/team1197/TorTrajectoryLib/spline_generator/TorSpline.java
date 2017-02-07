package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class TorSpline extends PathSegment {
	private List<PathSegment> path;

	public TorSpline(double start_x, double start_y, double start_head) {
		super(start_x, start_y, start_head);
		path = new LinkedList<PathSegment>();
		length = 0.0;
	}

	@Override
	public String toString() {
		String s = new String("Spline:\n");
		int i = 0;
		for (PathSegment segment : path) {
			s.concat("\tsegment ").concat(String.valueOf(i)).concat(" = ").concat(segment.toString()).concat("\n");
			i++;
		}
		return s;
	}

	@Override
	public PathSegment clone() {
		RealVector translation = internalTranslation();
		TorSpline splineCopy = new TorSpline(translation.getEntry(0), translation.getEntry(1), internalRotation());
		for (PathSegment segment : path) {
			splineCopy.add(segment.clone());
		}
		return splineCopy;
	}

	public void add(PathSegment s) {
		PathSegment segment = s.clone();
		if (path.size() > 0) {
			RealVector nextStartingPoint = rawPositionAt(length);
			double nextStartingHeading = rawHeadingAt(length);
			segment.translateExternally(nextStartingPoint);
			segment.rotateExternally(nextStartingHeading);
		}
		path.add(segment);
		length += segment.length();
	}

	@Override
	public RealVector rawPositionAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return internalTransform(segment.positionAt(s - lengthSoFar));
			lengthSoFar += segment.length();
		}
		PathSegment segment = path.get(path.size() - 1);
		return internalTransform(segment.positionAt(s - lengthSoFar));
	}

	@Override
	public double rawHeadingAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return internalRotation() + segment.headingAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		PathSegment segment = path.get(path.size() - 1);
		return internalRotation() + segment.headingAt(s - lengthSoFar);
	}

	@Override
	public double curvatureAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return segment.curvatureAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		PathSegment segment = path.get(path.size() - 1);
		return segment.curvatureAt(s - lengthSoFar);
	}
}
