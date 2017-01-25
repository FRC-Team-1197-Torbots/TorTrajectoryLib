package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public abstract class PathSegment {
	protected double length;
	private double internalRotation;
	private double externalRotation;
	private double totalRotation;
	private RealMatrix rotationMatrix;
	private RealVector internalTranslation;
	private RealVector externalTranslation;
	private RealVector translationVector;
	public abstract String toString();
	public abstract PathSegment clone();
	public abstract RealVector positionAt(double s);
	public abstract double headingAt(double s);
	public abstract double curvatureAt(double s);
	
	
	public PathSegment(){
		internalTranslation = new ArrayRealVector(new double[] {0.0, 0.0});
		externalTranslation = new ArrayRealVector(new double[] {0.0, 0.0});
		translationVector = new ArrayRealVector(new double[] {0.0, 0.0});
		rotationMatrix = new Array2DRowRealMatrix(new double[][] {{0.0, 0.0}, 
																  {0.0, 0.0}});
	}
	
	public double length(){
		return length;
	}
	
	// Output transformation stuff:
	public RealVector outputTransform(RealVector pos){
		return rotationMatrix.operate(pos).add(translationVector);
	}
	
	// Rotation
	public double internalRotation(){
		return internalRotation;
	}
	public double externalRotation(){
		return externalRotation;
	}
	public double totalRotation(){
		totalRotation = internalRotation + externalRotation;
		return totalRotation;
	}
	public RealMatrix externalUnrotateMatrix(){
		double r = -externalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	public void rotateInternally(double i){
		internalRotation = i;
		totalRotation = internalRotation + externalRotation;
		rotateTo(totalRotation);
	}
	public void rotateExternally(double e){
		externalRotation = e;
		totalRotation = internalRotation + externalRotation;
		rotateTo(totalRotation);
	}
	private void rotateTo(double r){
		rotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													  {Math.sin(r), Math.cos(r)}});
	}
	public RealMatrix rotationMatrix(){
		return rotationMatrix;
	}
	public RealMatrix unrotateMatrix(){
		double r = -totalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	public RealMatrix internalRotationMatrix(){
		double r = internalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	public RealMatrix internalUnrotateMatrix(){
		double r = -internalRotation;
		RealMatrix m = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
			  													{Math.sin(r), Math.cos(r)}});
		return m;
	}
	public RealMatrix externalRotationMatrix(){
		double r = externalRotation;
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
	public RealVector totalTranslation(){
		translateTo(internalTranslation.add(externalTranslation));
		return translationVector;
	}
	public void translateInternally(RealVector i){
		internalTranslation = i;
		translateTo(internalTranslation.add(externalTranslation));
	}
	public void translateInternally(double x, double y){
		internalTranslation = new ArrayRealVector(new double[] {x, y});
		translateTo(internalTranslation.add(externalTranslation));
	}
	public void translateExternally(RealVector e){
		externalTranslation = e;
		translateTo(internalTranslation.add(externalTranslation));
	}
	public void translateExternally(double x, double y){
		externalTranslation = new ArrayRealVector(new double[] {x, y});
		translateTo(internalTranslation.add(externalTranslation));
	}
	private void translateTo(RealVector v){
		translationVector = v;
	}
}
