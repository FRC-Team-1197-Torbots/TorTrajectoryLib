package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class LineSegment extends PathSegment{
	
	final RealVector position = new ArrayRealVector(new double[] {0.0, 0.0});
	
	public LineSegment(double l, double r){
		super(0.0, 0.0, r);
		setLength(l);
		setTotalAngle(0.0);
	}
	
	public String toString(){
		return "LineSegment[length=".concat(String.valueOf(length()))
									  .concat(", rotation=")
									  .concat(String.valueOf(internalRotation()))
									  .concat("]") ;
	}
	
	public SegmentType type(){
		return SegmentType.LINE;
	}

	@Override
	protected RealVector rawPositionAt(double s) {
		position.setEntry(0, s);
		position.setEntry(1, 0.0);
		return internalTransform(position);
	}

	@Override
	protected double rawHeadingAt(double s) {
		return internalRotation();
	}

	@Override
	public double curvatureAt(double s) {
		return 0.0;
	}

	@Override
	public LineSegment clone() {
		LineSegment lineCopy = new LineSegment(length(), internalRotation());
		lineCopy.translateInternally(internalTranslation());
		return lineCopy;
	}

}
