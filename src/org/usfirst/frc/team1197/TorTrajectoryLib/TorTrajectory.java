
package org.usfirst.frc.team1197.TorTrajectoryLib;

import java.util.ArrayList;

import java.util.List;

public abstract class TorTrajectory {

	protected double goal_pos = 0.0;
	protected double goal_head = 0.0;
	
//	protected double max_vel = 4.5; //4.5
//	protected double max_acc = 3.5; //6.0 
//	protected double max_jerk = 20.0; //20.0
//	
//	protected double max_omg = 8.0; //8.0
//	protected double max_alf = 9.0; //10.0
//	protected double max_jeta = 40.0; //40.0
	protected double max_vel = 0.0; //4.5
	protected double max_acc = 0.0; //6.0 
	protected double max_jerk = 0.0; //20.0
	
	protected double max_omg = 0.0; //8.0
	protected double max_alf = 0.0; //10.0
	protected double max_jeta = 0.0; //40.0
	
	protected List<Long> time;
	protected List<MotionState1D> translation;
	protected List<MotionState1D> rotation;
	
	protected boolean isComplete;
	protected double dt = 0.005;
	
	public TorTrajectory(double goal_pos, double goal_head){
		this.goal_pos = goal_pos;
		this.goal_head = goal_head;
		
		time = new ArrayList<Long>();
		translation = new ArrayList<MotionState1D>();
		rotation = new ArrayList<MotionState1D>();
		time.add((long) 0);
		translation.add(new MotionState1D(0.0, 0.0, 0.0));
		rotation.add(new MotionState1D(0.0, 0.0, 0.0));
		isComplete = false;
	}
	
	public TorTrajectory(){
		time = new ArrayList<Long>();
		translation = new ArrayList<MotionState1D>();
		rotation = new ArrayList<MotionState1D>();
		time.add((long) 0);
		translation.add(new MotionState1D(0.0, 0.0, 0.0));
		rotation.add(new MotionState1D(0.0, 0.0, 0.0));
		isComplete = false;
	}
	
	// The following magic was adapted from 254's TrajectoryLib.
	protected void build(double goal_pos, double max_vel, double max_acc, double max_jerk, 
						List<MotionState1D> motion){
		// This guarantees that if we don't have the runway to accelerate up to top speed
		// or to "jerk" up to top acceleration, we set more realistic goals:
		
		double disp = Math.abs(goal_pos); // Worry about sign later.
		double adjusted_max_acc = Math.min(max_acc,
										   Math.min(Math.sqrt(max_vel * max_jerk),
										            Math.pow((0.5 * disp * max_jerk * max_jerk), (1.0/3.0))));
		double adjusted_max_vel = Math.min( max_vel,
				(-adjusted_max_acc * adjusted_max_acc + Math.sqrt(adjusted_max_acc
						* adjusted_max_acc * adjusted_max_acc * adjusted_max_acc
						+ 4 * max_jerk * max_jerk * adjusted_max_acc * disp)) 
				/ (2 * max_jerk) );
		
		// Since each filter has a discrete number of slots, we need this "fancy rounding code" 
		// to make sure the robot always drives exactly the correct distance, and always
		// moves in a way that does not exceed the maximum velocity, acceleration, or
		// jerk. Without this, the use of a digital filter to produce motion profiles can
		// give unexpected results that violate these constraints, especially when
		// the path is short or the maximum velocity, acceleration, and/or jerk are high.

		int f0_length = (int) (1 + Math.round(disp / (adjusted_max_vel * dt)));
		adjusted_max_vel = disp/(f0_length * dt);
		int f1_length = (int) Math.ceil(adjusted_max_vel / (adjusted_max_acc * dt));
		int f2_length = (int) Math.ceil(adjusted_max_acc / (max_jerk * dt));
		int tot_length = f0_length + f1_length + f2_length;
		if(goal_pos < 0.0){
			adjusted_max_vel = -adjusted_max_vel;
		}
		secondOrderFilter(f0_length, f1_length, f2_length, dt, adjusted_max_vel, tot_length, motion);
	}
	
	protected void secondOrderFilter(int f0_length, int f1_length, int f2_length, double dt, double max_vel, int tot_length,
			List<MotionState1D> motion) {
		// Why no "f0"? Because the zero-filter can be equivalently implemented more
		// simply by just feeding a constant velocity value into the first filter for
		// the correct length of time. The "real" filters, on the other hand, MUST be 
		// implemented as lists of numbers:
		List<Double> f1 = new ArrayList<Double>();
		for(int i = 0; i < f1_length; i++){
			f1.add(new Double(0));
		}
		List<Double> f2 = new ArrayList<Double>();
		for(int i = 0; i < f2_length; i++){
			f2.add(new Double(0));
		}
		
		// Inputs and outputs of the filters:
		double input;
		double FL1out;
		double FL2out;
		// Individual data values:
		long t = 0;
		double pos = 0.0;
		double vel = 0.0;
		double acc = 0.0;
		// record previous values so we can do integration/differentiation:
		double last_pos = 0.0;
		double last_vel = 0.0;
		
		for (int i = 0; i < tot_length; ++i) {
			last_pos = pos;
			last_vel = vel;
			// As long as the zero filter is not empty, feed-forward the target velocity:
			if(f0_length > 0){
				input = max_vel;
				f0_length--; // Decrement the zero filter.
			}
			else{
				input = 0.0;
			}

			// Input goes at the beginning of the filter (end of the list):
			f1.add(new Double(input));
			// Throw away the element at the end of the filter (beginning of the list)
			f1.remove(0);
			//  Output is the average of the elements in the first filter:
			FL1out = average(f1);
			// Repeat this procedure for the second filter:
			f2.add(new Double(FL1out));
			f2.remove(0);
			FL2out = average(f2);
			
			// The output of the filter is velocity:
			vel = FL2out;
		    // We have to integrate to get position. This uses trapezoidal integration,
		    // but the choice of integration strategy probably doesn't matter:
			pos = last_pos + 0.5 * (last_vel + vel) * dt;
			// We have to differentiate to get acceleration:
			acc = (vel - last_vel) / dt;
			// Add the new motion and time data to their respective lists:
			motion.add(new MotionState1D(pos, vel, acc));
			t += (long)(dt * 1000);
			time.add(new Long(t));
		}
	}
	
	public double lookUpPosition(long t){
		if(t < time.get(0)){
			return 0.0;
		}
		int i = time.indexOf(t);
		if(i == -1){
			return goal_pos;
		}
		return translation.get(i).pos;
	}
	public double lookUpVelocity(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return translation.get(i).vel;
	}
	public double lookUpAcceleration(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return translation.get(i).acc;
	}
	public double lookUpHeading(long t){
		if(t < time.get(0)){
			return 0.0;
		}
		int i = time.indexOf(t);
		if(i == -1){
			return goal_head;
		}
		return rotation.get(i).pos;
	}
	public double lookUpOmega(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return rotation.get(i).vel;
	}
	public double lookUpAlpha(long t){
		int i = time.indexOf(t);
		if(i == -1){
			return 0.0;
		}
		return rotation.get(i).acc;
	}
	
	public boolean lookUpIsLast(long t){
		if(t < time.get(0)){
			return false;
		}
		int i = time.indexOf(t);
		if(i == -1){
			return true;
		}
		return (i+1 == time.size());
	}
	
	//TODO put this in TorMath:
	public double average(List<Double> list){
		double avg = 0;
		for(Double element : list){
			avg += element;
		}
		avg /= list.size();
		return avg;
	}
	
	public double goalPos(){
		return goal_pos;
	}
	public double maxVelocity(){
		return max_vel;
	}
	public double maxAcceleration(){
		return max_acc;
	}
	public double maxJerk(){
		return max_jerk;
	}
	public double adjustedMaxVelocity(){
		double max = 0.0;
		for (MotionState1D m:translation){
			if(Math.abs(m.vel)>max)
				max = Math.abs(m.vel);
		}
		return max;
	}
	public double adjustedMaxAcceleration(){
		double max = 0.0;
		for (MotionState1D m:translation){
			if(Math.abs(m.acc)>max)
				max = Math.abs(m.acc);
		}
		return max;
	}
	
	public double goalHead(){
		return goal_head;
	}
	public double maxOmega(){
		return max_omg;
	}
	public double maxAlpha(){
		return max_alf;
	}
	public double maxJeta(){
		return max_jeta;
	}
	public double adjustedMaxOmega(){
		double max = 0.0;
		for (MotionState1D m : rotation){
			if(Math.abs(m.vel) > max)
				max = Math.abs(m.vel);
		}
		return max;
	}
	public double adjustedMaxAlpha(){
		double max = 0.0;
		for (MotionState1D m : rotation){
			if(Math.abs(m.acc) > max)
				max = Math.abs(m.acc);
		}
		return max;
	}
	
	public long totalTime(){
		return time.get(time.size()-1);
	}
	
	public void setComplete(boolean b){
		isComplete = b;
	}
	public boolean isComplete(){
		return isComplete;
	}
	public void flipSign(List<MotionState1D> motion){
		for(MotionState1D element : motion){
			element.set(-element.pos, -element.vel, -element.acc);
		}
	}
}
