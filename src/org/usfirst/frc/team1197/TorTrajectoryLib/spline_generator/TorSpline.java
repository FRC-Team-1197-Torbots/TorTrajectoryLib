package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.linear.RealVector;

public class TorSpline extends PathSegment {
	private List<PathSegment> path;

	public TorSpline(double start_x, double start_y, double start_head) {
		translateInternally(start_x, start_y);
		rotateInternally(start_head);
		translateExternally(0.0, 0.0);
		rotateExternally(0.0);
		path = new LinkedList<PathSegment>();
		length = 0.0;
	}

	public TorSpline(double start_x, double start_y, double start_head, boolean isFirst) {
		if (!isFirst) {
			translateInternally(start_x, start_y);
			rotateInternally(start_head);
			translateExternally(0.0, 0.0);
			rotateExternally(0.0);
		} else {
			translateInternally(0.0, 0.0);
			rotateInternally(0.0);
			translateExternally(start_x, start_y);
			rotateExternally(start_head);
		}
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

	public void add(PathSegment segment) {
		segment = segment.clone();
		if (path.size() > 0) {
			RealVector nextStartingPoint = unrotateMatrix()
					.operate(positionAt(length()).subtract(externalTranslation()));
											  // .subtract(totalTranslation()));
			double nextStartingHeading = headingAt(length) - totalRotation();
			segment.translateExternally(nextStartingPoint);
			segment.rotateExternally(nextStartingHeading);
		}
		path.add(segment);
		length += segment.length();
	}

	public RealVector positionAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return outputTransform(segment.positionAt(s - lengthSoFar));
			lengthSoFar += segment.length();
		}
		return null;
	}

	public double headingAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return totalRotation() + segment.headingAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		return 0.0;
	}

	@Override
	public double curvatureAt(double s) {
		double lengthSoFar = 0.0;
		for (PathSegment segment : path) {
			if (s - lengthSoFar <= segment.length)
				return segment.curvatureAt(s - lengthSoFar);
			lengthSoFar += segment.length();
		}
		return 0.0;
	}
}
