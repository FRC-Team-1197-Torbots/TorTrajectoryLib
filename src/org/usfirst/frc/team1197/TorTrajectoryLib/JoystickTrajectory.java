package org.usfirst.frc.team1197.TorTrajectoryLib;

public class JoystickTrajectory extends TorTrajectory{
	
	private MotionState1D translation;
	private MotionState1D rotation;
	private double tgt_vel;
	private double tgt_omg;
	private double tgt_acc;
	
	public JoystickTrajectory(){
		super(0.0, 0.0);
		max_alf = 1000.0;
		translation = new MotionState1D(0.0, 0.0, 0.0);
		rotation = new MotionState1D(0.0, 0.0, 0.0);
	}
	
	public double lookUpPosition(long t){
		return translation.pos;
	}
	public double lookUpVelocity(long t){
		return translation.vel;
	}
	public double lookUpAcceleration(long t){
		return translation.acc;
	}
	
	public double lookUpHeading(long t){
		return rotation.pos;
	}
	public double lookUpOmega(long t){
		return rotation.vel;
	}
	public double lookUpAlpha(long t){
		return rotation.acc;
	}
	
	public boolean lookUpIsLast(long t){
		return true;
	}
	
	public void setTargets(double v, double w){
		tgt_vel = v;
		tgt_omg = w;
	}
	
	public void execute(double p_init, double v_init, double h_init, double w_init){
		translation.pos = p_init;
		translation.vel = v_init;
		translation.acc = 0.0;
		
		rotation.pos = h_init;
		rotation.vel = w_init;
		rotation.acc = 0.0;
		
//		TODO: find a better way
//		TorMotionProfile.INSTANCE.loadTrajectory(this);
	}
	
	public void update(double tgt_vel, MotionState1D m, double max_acc){
		// Target (requested) acceleration:
		tgt_acc = (tgt_vel - m.vel) / dt;
		// Actual acceleration:
		m.acc = sign(tgt_acc)*Math.min(Math.abs(tgt_acc), max_acc);
		// Velocity:
		m.vel = m.vel + m.acc*dt;
		// Position:
		m.pos = m.pos + m.vel*dt + 0.5*m.acc*dt*dt;	
	}
	
	public void updateVelocity(){
		update(tgt_vel, translation, max_acc);
	}
	
	public void updateOmega(){
		update(tgt_omg, rotation, max_alf);
	}
	
	public void updateDt(double dt){
		if(dt == 0.0){
			this.dt = 0.005;
		}
		else{
			this.dt = dt;
		}
	}
	
	//TODO: make this global
	public static double sign(double x){
		if (x > 0.0)
			return 1.0;
		else if (x < 0.0)
			return -1.0;
		else
			return 0.0;
	}
}
