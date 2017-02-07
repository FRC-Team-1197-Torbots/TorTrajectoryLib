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
	
	private double ds = 0.0001;
	private List<RealVector> positionData;
	private List<Double> arcLength;

	UnivariateIntegrator x_integrator;
	UnivariateIntegrator y_integrator;
	UnivariateFunction x_integrand;
	UnivariateFunction y_integrand;

	public CornuSpiral(double A, double B, double C, double si, double sf) {
		super(0.0, 0.0, 0.0);
		arcLength = new ArrayList<Double>();
		positionData = new ArrayList<RealVector>();
		setConstants(A, B, C, si, sf);

		final double relativeAccuracy = 1.0e-6;
		final double absoluteAccuracy = 1.0e-8;
		final int minIterations = 8;
		final int maxIterations = 64;
		x_integrator = new SimpsonIntegrator(relativeAccuracy, absoluteAccuracy, minIterations, maxIterations);
		y_integrator = new SimpsonIntegrator(relativeAccuracy, absoluteAccuracy, minIterations, maxIterations);
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
		double x, y;
		for (double s = si; s <= sf; s += ds) {
			if(s == 0){
				x = 0.0;
				y = 0.0;
			} else {
				x = x_integrator.integrate(10000, x_integrand, 0, s);
				y = y_integrator.integrate(10000, y_integrand, 0, s);
			}
			arcLength.add(s - si);
			positionData.add(new ArrayRealVector(new double[] {x, y}));
		}
		RealVector startPoint = positionData.get(0);
		translateInternally(startPoint.mapMultiply(-1.0));
		double startHeading = rawHeadingAt(0.0);
		rotateInternally(-startHeading);
		
	}

	private CornuSpiral() {
		super(0.0, 0.0, 0.0);
		arcLength = new ArrayList<Double>();
		positionData = new ArrayList<RealVector>();
		arcLength.add(0.0);
		positionData.add(new ArrayRealVector(new double [] {0.0 , 0.0}));
	}

	@Override
	public String toString() {
		DecimalFormat df = new DecimalFormat("0.####");
		return ("Cornu Spiral[A=").concat(df.format(A))
				   .concat(", B=").concat(df.format(B))
				   .concat(", C=").concat(df.format(C))
				   .concat(" / si=").concat(df.format(si))
				   .concat(", sf=").concat(df.format(sf))
				   .concat("]");
	}

	@Override
	public CornuSpiral clone() {
		// CornuSpiral cornuCopy = new Corn
		CornuSpiral cornuCopya = new CornuSpiral(); // hehe
		cornuCopya.setConstants(A, B, C, si, sf);
		cornuCopya.setLists(positionData, arcLength);
		return cornuCopya;
	}

	@Override
	public RealVector rawPositionAt(double s) {
		int i_prev = getLowerNeighborIndex(s, arcLength);
		int i_next = i_prev+1;
		if (i_next >= positionData.size()){
			i_prev = positionData.size()-2;
			i_next = positionData.size()-1;
		}
		double s_prev = arcLength.get(i_prev);
		RealVector pos_prev = positionData.get(i_prev);
		RealVector pos_next = positionData.get(i_next);
		RealVector pos = linearInterpolate(pos_prev, pos_next, s - s_prev);

		return internalTransform(pos);
	}

	@Override
	public double rawHeadingAt(double s) {
		s = s + si;
		return internalRotation() + a * s*s*s + b * s*s + c * s;
	}

	@Override
	public double curvatureAt(double s) {
		s = s + si;
		return A * s*s + B * s + C;
	}

	private RealVector linearInterpolate(RealVector u, RealVector v, double s) {
		RealVector unit = (v.subtract(u)).unitVector();
		return u.add(unit.mapMultiply(s));
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
	}
	
	private void setLists(List<RealVector> p, List<Double> a) {
		arcLength.clear();
		positionData.clear();
		int i;
		for (double s : a) {
			i = a.indexOf(s);
			arcLength.add(s);
			positionData.add(p.get(i).copy());
		}
		RealVector startPoint = positionData.get(0);
		translateInternally(startPoint.mapMultiply(-1.0));
		double startHeading = headingAt(0.0);
		rotateInternally(-startHeading);
	}
	
	private int getLowerNeighborIndex(double s, List<Double> list){
		return (int)(s/ds);
	}

}
