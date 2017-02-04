package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.text.DecimalFormat;

import org.apache.commons.math3.linear.RealVector;

public class CornuSpiral extends PathSegment {

	private double A, B, C;
	private double totalAngle;
	@Override
	public String toString() {
		DecimalFormat df = new DecimalFormat("0.####");
		return "Cornu Spiral[A=".concat(df.format(A))
		  .concat(", B=").concat(df.format(B))
		  .concat(", C=").concat(df.format(C))
		  .concat(", totalAngle=").concat(df.format(totalAngle))
		  .concat("]") ;
	}

	@Override
	public PathSegment clone() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public RealVector positionAt(double s) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double headingAt(double s) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double curvatureAt(double s) {
		// TODO Auto-generated method stub
		return 0;
	}
	
}
