package org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter;

import org.usfirst.frc.team1197.TorTrajectoryLib.LinearTrajectory;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph.motionType;

public class VelocityPlotter {
	final static double dt = 0.005;
	final static LinearTrajectory trajectory = new LinearTrajectory(1.0);
	final static VelocityGraph graph = new VelocityGraph(trajectory, motionType.Translation);

	public static void main(String[] args) {
		long time;
		double pos;
		double vel;
		double acc;

		for (double t = 0; t <= trajectory.totalTime(); t+=dt){
			time = (long)(t*1000);
			pos = trajectory.lookUpPosition(time);
			vel = trajectory.lookUpVelocity(time);
			acc = trajectory.lookUpVelocity(time);
			graph.position.add(t, pos);
			graph.velocity.add(t, vel);
			graph.acceleration.add(t, acc);
	    }
	}

}
