package org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter;

import org.usfirst.frc.team1197.TorTrajectoryLib.LinearTrajectory;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph.motionType;

public class VelocityPlotter {

	public static void main(String[] args) {
		LinearTrajectory linTrajectory = new LinearTrajectory(1.0);
		VelocityGraph graph = new VelocityGraph(linTrajectory, motionType.Translation);
        graph.display();
        graph.plotData();
	}

}
