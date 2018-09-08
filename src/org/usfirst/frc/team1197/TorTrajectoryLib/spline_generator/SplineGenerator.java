package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.io.IOException;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
//import org.usfirst.frc.team1197.TorTrajectoryLib.FileTrajectory;
import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;
import org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer.TrajectoryWriter;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph.motionType;

public class SplineGenerator {

	private static final double ds = 0.005;
	private static TorSpline inputSpline;
	private static TorSpline outputSpline;
	private static TorTrajectory trajectory;
	private static VelocityGraph translationGraph;
	private static VelocityGraph rotationGraph;
	private static TrajectoryWriter writer;

	public static void main(final String[] args) {

		final PathGraph graph = new PathGraph();
		graph.display();
		
		// false = forward, true = backward
		boolean forOrBack = false;

		/**
		 * TEST SPLINES
		 */
		
		//this is cool

		// A figure-8:
//		 inputSpline = new TorSpline(8.0, 4.0, 36.87*(Math.PI/180.0));
//		 inputSpline.add(new LineSegment(1.0, 0.0));
//		 inputSpline.add(new ArcSegment(0.75, -253.74*(Math.PI/180.0)));
//		 inputSpline.add(new LineSegment(2.0, 0.0));
//		 inputSpline.add(new ArcSegment(0.75, 253.74*(Math.PI/180.0)));
//		 inputSpline.add(new LineSegment(1.0, 0.0));

		// // An Illegal spline:
		// inputSpline = new TorSpline(1.0, 1.0, 0.0);
		// inputSpline.add(new LineSegment(2.0, 0.0));
		// inputSpline.add(new ArcSegment(2.0, 60 * (Math.PI / 180.0)));
		// inputSpline.add(new LineSegment(2.0, 0.0));
		// inputSpline.add(new ArcSegment(2.0, -30 * (Math.PI / 180.0)));
		// inputSpline.add(new ArcSegment(2.0, 30 * (Math.PI / 180.0)));

		RealVector P = new ArrayRealVector(new double[] { 0.0, 0.0 });
		for (double s = 0.0; s <= inputSpline.length(); s += ds) {
			P = inputSpline.positionAt(s);
			graph.inputPath.add(P.getEntry(0), P.getEntry(1));
		}
		
		outputSpline = new SmoothSpline(inputSpline); // TODO: Make it work for
														// splines of splines?
		
		if (Math.abs(outputSpline.length()) > 0.0) {
			for (double s = 0.0; s <= outputSpline.length(); s += 0.005) {
				P = outputSpline.positionAt(s);
				graph.outputPath.add(P.getEntry(0), P.getEntry(1));
			}

			trajectory = new SplineTrajectory(outputSpline, forOrBack);
//			trajectory = new SpeedySplineTrajectory(outputSpline, forOrBack);
			
			/* Uncomment the next 4 lines if you want a SpeedySpline: */
			// trajectory = new SpeedySplineTrajectory(outputSpline, true);
			// TorTrajectory splineTraj = new SplineTrajectory(outputSpline, true);
			// System.out.println(" Speedy Spline Trajectory length: " + trajectory.goalPos());
			// System.out.println("Regular Spline Trajectory length: " + splineTraj.goalPos());

			translationGraph = new VelocityGraph(trajectory, motionType.Translation);
			rotationGraph = new VelocityGraph(trajectory, motionType.Rotation);
			translationGraph.display();
			rotationGraph.display();
			translationGraph.plotData();
			rotationGraph.plotData();
			
			writer = new TrajectoryWriter(trajectory, new String("GeneratedTrajectory"));
			try {
				writer.writeDotJava(new String("org.usfirst.frc.team1197.generatedSplines"));
			} catch (IOException x) {
				System.err.format("IOException: %s%n", x);
			}
		}
	}
}
