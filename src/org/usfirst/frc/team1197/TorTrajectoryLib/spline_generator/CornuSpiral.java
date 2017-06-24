package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class CornuSpiral extends PathSegment {
	
	private double A, B, C;
	private double a, b, c;
	private double si, sf;

	UnivariateIntegrator x_integrator;
	UnivariateIntegrator y_integrator;
	UnivariateFunction x_integrand;
	UnivariateFunction y_integrand;
	
	private double xi, yi, startHeading;
	private double x_prev, y_prev, s_prev;

	public CornuSpiral(double A, double B, double C, double si, double sf) {
		super(0.0, 0.0, 0.0);
		build(A, B, C, si, sf);
	}

	private CornuSpiral() {
		super(0.0, 0.0, 0.0);
	}

	@Override
	public String toString() {
		DecimalFormat df = new DecimalFormat("0.####");
		return ("Cornu Spiral[A=").concat(df.format(A))
				   .concat(", B=").concat(df.format(B))
				   .concat(", C=").concat(df.format(C))
				   .concat("; si=").concat(df.format(si))
				   .concat(", sf=").concat(df.format(sf))
				   .concat("]");
	}
	
	public SegmentType type(){
		return SegmentType.CORNU_SPIRAL;
	}

	@Override
	public CornuSpiral clone() {
		// CornuSpiral cornuCopy = new Corn
		CornuSpiral cornuCopya = new CornuSpiral(); // hehe
		cornuCopya.build(A, B, C, si, sf);
		return cornuCopya;
	}

	@Override
	public RealVector rawPositionAt(double sb) {
		double sa, xa, ya, xb, yb;
		if (sb >= s_prev) {
			sa = s_prev;
			xa = x_prev;
			ya = y_prev;
		} else {
			sa = si;
			xa = xi;
			ya = yi;
		}
		if (sa == sb)
		{
			xb = xa;
			yb = ya;
		} else {
			if (sb < si)
			{
				System.out.printf("sb = %f, si = %f.\n", sb, si);
				System.out.println("What r u doin!!!");
			}
			xb = xa + x_integrator.integrate(10000, x_integrand, sa, sb);
			yb = ya + y_integrator.integrate(10000, y_integrand, sa, sb);
		}
		RealVector pos = new ArrayRealVector(new double[] {xb, yb});
		s_prev = sb;
		x_prev = xb;
		y_prev = yb;
		return internalTransform(pos);
	}

	@Override
	public double rawHeadingAt(double s) {
		return internalRotation() + a * s*s*s + b * s*s + c * s;
	}

	@Override
	public double curvatureAt(double s) {
		s = s + si;
		return A * s*s + B * s + C;
	}

	private void setConstants(double A, double B, double C, double si, double sf){
		this.A = A;
		this.B = B;
		this.C = C;
		this.si = si;
		this.sf = sf;
		
		a = A / 3.0;
		b = B / 2.0;
		c = C;
		setLength(sf - si);
		startHeading = rawHeadingAt(si);
		setTotalAngle(rawHeadingAt(sf)-startHeading);
	}
	
	private void build(double A, double B, double C, double si, double sf) {
		setConstants(A, B, C, si, sf);
		if (x_integrator == null)
		{
			x_integrator = new SimpsonIntegrator(relativeAccuracy, absoluteAccuracy, minIterations, maxIterations);
		}
		if (y_integrator == null)
		{
			y_integrator = new SimpsonIntegrator(relativeAccuracy, absoluteAccuracy, minIterations, maxIterations);
		}
		x_integrand = new UnivariateFunction() {
			public double value(double t) {
				return Math.cos(a * t*t*t + b * t*t + c * t);
			}
		};
		y_integrand = new UnivariateFunction() {
			public double value(double t) {
				return Math.sin(a * t*t*t + b * t*t + c * t);
			}
		};
		s_prev = 0.0;
		x_prev = 0.0;
		y_prev = 0.0;
		RealVector startPoint = rawPositionAt(si);
		xi = startPoint.getEntry(0);
		yi = startPoint.getEntry(1);
		translateInternally(startPoint.mapMultiply(-1.0));
		rotateInternally(-startHeading);
	}
}
