package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public abstract class PathSegment {
	protected double length;
	private double internalRotation;
	private double externalRotation;
	private RealMatrix internalRotationMatrix;
	private RealMatrix externalRotationMatrix;
	private RealVector internalTranslation;
	private RealVector externalTranslation;
	public abstract String toString();
	public abstract PathSegment clone();
	public abstract double curvatureAt(double s);
	protected abstract RealVector rawPositionAt(double s);
	protected abstract double rawHeadingAt(double s);
	
	
	public PathSegment(RealVector internalTranslation, double internalRotation){
		translateInternally(internalTranslation);
		rotateInternally(internalRotation);
		rotateExternally(0.0);
		translateExternally(0, 0);
	}
	
	public PathSegment(double x, double y, double internalRotation){
		translateInternally(x, y);
		rotateInternally(internalRotation);
		rotateExternally(0.0);
		translateExternally(0, 0);
	}
	
	public RealVector positionAt(double s){
		return externalTransform(rawPositionAt(s));
	}
	
	public double headingAt(double s){
		return externalRotation + rawHeadingAt(s);
	}
	
	public double length(){
		return length;
	}
	
	// Output transformation stuff:
	public RealVector internalTransform(RealVector pos){
		return internalRotationMatrix.operate(pos.add(internalTranslation));
	}
	public RealVector externalTransform(RealVector pos){
		return externalRotationMatrix.operate(pos).add(externalTranslation);
	}
	
	// Rotation
	public double internalRotation(){
		return internalRotation;
	}
	public double externalRotation(){
		return externalRotation;
	}
	public void rotateInternally(double r){
		internalRotation = r;
		internalRotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  															  {Math.sin(r), Math.cos(r)}});
	}
	public void rotateExternally(double r){
		externalRotation = r;
		internalRotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  															  {Math.sin(r), Math.cos(r)}});		
	}
	public RealMatrix internalRotationMatrix(){
		return internalRotationMatrix;
	}
	public RealMatrix externalRotationMatrix(){
		return externalRotationMatrix;
	}
	public RealMatrix internalUnrotateMatrix(){
		double r = -internalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	public RealMatrix externalUnrotateMatrix(){
		double r = -externalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	
	// Translation
	public RealVector internalTranslation(){
		return internalTranslation;
	}
	public RealVector externalTranslation(){
		return externalTranslation;
	}
	public void translateInternally(RealVector i){
		internalTranslation = i;
	}
	public void translateInternally(double x, double y){
		internalTranslation = new ArrayRealVector(new double[] {x, y});
	}
	public void translateExternally(RealVector e){
		externalTranslation = e;
	}
	public void translateExternally(double x, double y){
		externalTranslation = new ArrayRealVector(new double[] {x, y});
	}
}
