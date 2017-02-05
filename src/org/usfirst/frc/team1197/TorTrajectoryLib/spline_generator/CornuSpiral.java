package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.List;
import java.util.ArrayList;
import java.text.DecimalFormat;

import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.apache.commons.math3.linear.ArrayRealVector;

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

	public CornuSpiral(double A, double B, double C, double si, double sf) {
		super();
		arcLength = new ArrayList<Double>();
		path = new ArrayList<RealVector>();
		setConstants(A, B, C, si, sf);
		
		final double relativeAccuracy = 1e-4;
		final double absoluteAccuracy = 1e-8;
		final int minIterations = 3;
		final int maxIterations = 32;
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
			x = x_integrator.integrate(100000, x_integrand, 0, s);
			y = y_integrator.integrate(100000, y_integrand, 0, s);
			arcLength.add(s - si);
			path.add(new ArrayRealVector(new double[] {x, y}));
		}
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
		double s_previous = roundDown(s);
		double s_next = s_previous + ds;
		RealVector pos_previous = path.get(arcLength.indexOf(s_previous));
		RealVector pos_next = path.get(arcLength.indexOf(s_next));
		return linearInterpolate(pos_previous, pos_next, s - s_previous);
	}

	@Override
	public double headingAt(double s) {
		return a * s*s*s + b * s*s + c * s;
	}

	@Override
	public double curvatureAt(double s) {
		return A * s*s + B * s + C;
	}

	private RealVector linearInterpolate(RealVector u, RealVector v, double s) {
		RealVector unit = (v.subtract(u)).unitVector();
		return u.add(unit.mapMultiply(s));
	}

	private double roundDown(double x) {
		x = x / ds;
		x = (double) (int) x;
		return x * ds;
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
	}

}
