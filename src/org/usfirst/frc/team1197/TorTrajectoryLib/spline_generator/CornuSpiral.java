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
	private List<RealVector> path;
	private List<Double> arcLength;

	UnivariateIntegrator x_integrator;
	UnivariateIntegrator y_integrator;
	UnivariateFunction x_integrand;
	UnivariateFunction y_integrand;
	
	private RealVector endPoint;

	public CornuSpiral(double A, double B, double C, double si, double sf) {
		super();
		arcLength = new ArrayList<Double>();
		path = new ArrayList<RealVector>();
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
			path.add(new ArrayRealVector(new double[] {x, y}));
		}
		RealVector startPoint = path.get(0);
		translateInternally(startPoint.mapMultiply(-1.0));
		double startHeading = headingAt(0.0);
		rotateInternally(-startHeading);
		
	}

	private CornuSpiral() {
		super();
		arcLength = new ArrayList<Double>();
		path = new ArrayList<RealVector>();
		arcLength.add(0.0);
		path.add(new ArrayRealVector(new double [] {0.0 , 0.0}));
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
		CornuSpiral cs = new CornuSpiral();
		cs.setConstants(A, B, C, si, sf);
		cs.setLists(path, arcLength);
		return cs;
	}

	@Override
	public RealVector positionAt(double s) {
		int i_prev = getLowerNeighborIndex(s, arcLength);
		int i_next = i_prev+1;
		if (i_next >= path.size()){
			i_prev = path.size()-2;
			i_next = path.size()-1;
		}
		double s_prev = arcLength.get(i_prev);
		RealVector pos_prev = path.get(i_prev);
		RealVector pos_next = path.get(i_next);
		RealVector pos = linearInterpolate(pos_prev, pos_next, s - s_prev);

		return outputTransform(pos);
	}

	@Override
	public double headingAt(double s) {
		s = s + si;
		return totalRotation() + a * s*s*s + b * s*s + c * s;
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
		length = sf - si;
	}
	
	private void setLists(List<RealVector> p, List<Double> a) {
		arcLength.clear();
		path.clear();
		int i;
		for (double s : a) {
			i = a.indexOf(s);
			arcLength.add(s);
			path.add(p.get(i).copy());
		}
		RealVector startPoint = path.get(0);
		translateInternally(startPoint.mapMultiply(-1.0));
		double startHeading = headingAt(0.0);
		rotateInternally(-startHeading);
	}
	
	private int getLowerNeighborIndex(double s, List<Double> list){
		return (int)(s/ds);
	}

}
