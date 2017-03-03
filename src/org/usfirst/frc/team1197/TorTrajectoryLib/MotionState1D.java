package org.usfirst.frc.team1197.TorTrajectoryLib;

public class MotionState1D {
	public double pos;
	public double vel;
	public double acc;
	public MotionState1D(){
		pos = 0.0;
		vel = 0.0;
		acc = 0.0;
	}
	
	public MotionState1D(double p, double v, double a){
		pos = p;
		vel = v;
		acc = a;
	}
	
	public double get(int n){
		switch(n){
		case 0:
			return pos;
		case 1:
			return vel;
		case 2:
			return acc;
		default:
			return Double.NaN;	
		}
	}
	
	public void set(double p, double v, double a){
		pos = p;
		vel = v;
		acc = a;
	}
	
	public void set(int n, double x){
		switch(n){
		case 0:
			pos = x;
			return;
		case 1:
			vel = x;
			return;
		case 2:
			acc = x;
			return;
		default:
			return;	
		}
	}
	
}
