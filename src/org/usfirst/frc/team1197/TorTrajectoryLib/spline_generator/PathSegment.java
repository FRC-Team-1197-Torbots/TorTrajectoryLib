package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public abstract class PathSegment {
	private double length;
	private double totalAngle;
	private double internalRotation;
	private double externalRotation;
	private RealMatrix internalRotationMatrix;
	private RealMatrix externalRotationMatrix;
	private RealVector internalTranslation;
	private RealVector externalTranslation;
	public enum SegmentType {LINE, ARC, CORNU_SPIRAL, SPLINE};
	public abstract SegmentType type();
	public abstract String toString();
	public abstract PathSegment clone();
	public abstract double curvatureAt(double s);
	protected abstract RealVector rawPositionAt(double s);
	protected abstract double rawHeadingAt(double s);
	
	public PathSegment(RealVector internalTranslation, double internalRotation) {
		translateInternally(internalTranslation);
		rotateInternally(internalRotation);
		rotateExternally(0.0);
		translateExternally(0.0, 0.0);
		length = 0.0;
		totalAngle = 0.0;
	}
	
	public PathSegment(double x, double y, double internalRotation) {
		translateInternally(x, y);
		rotateInternally(internalRotation);
		rotateExternally(0.0);
		translateExternally(0.0, 0.0);
		length = 0.0;
		totalAngle = 0.0;
	}
	
	public RealVector positionAt(double s) {
		return externalTransform(rawPositionAt(s));
	}
	
	public double headingAt(double s) {
		return externalRotation + rawHeadingAt(s);
	}
	
	public double length() {
		return length;
	}
	protected void setLength(double l) {
		length = l;
	}
	protected void addToLength(double increment) {
		length += increment;
	}
	
	public double totalAngle(){
		return totalAngle;
	}
	protected void setTotalAngle(double a){
		totalAngle = a;
	}
	protected void addToTotalAngle(double increment){
		totalAngle += increment;
	}
	
	// Output transformation stuff:
	public RealVector internalTransform(RealVector pos){
		return internalRotationMatrix.operate(pos.add(internalTranslation));
	}
	public RealVector externalTransform(RealVector pos){
		return externalRotationMatrix.operate(pos).add(externalTranslation);
	}
	public RealVector reverseInternalTransform(RealVector pos){
		return internalUnrotateMatrix().operate(pos).subtract(internalTranslation);
	}
	public RealVector reverseExternalTransform(RealVector pos){
		return externalUnrotateMatrix().operate(pos.subtract(externalTranslation));
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
		externalRotationMatrix = new Array2DRowRealMatrix(new double[][] {{Math.cos(r), -Math.sin(r)}, 
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
	
	public PathSegment cloneTrimmedBy(double decrement){
		PathSegment segment = this.clone();
		segment.addToLength(-decrement);
		segment.setTotalAngle(segment.rawHeadingAt(segment.length()));
		return segment;
	}
}
