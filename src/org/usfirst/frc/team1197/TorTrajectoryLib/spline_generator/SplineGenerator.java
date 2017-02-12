package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.io.IOException;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
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
		
//		inputSpline = new TorSpline(0.375, 6.070, 0.0);
//		inputSpline.add(new LineSegment(1.697, 0.0));
//		inputSpline.add(new ArcSegment(0.943, -60*(Math.PI/180.0)));
//		inputSpline.add(new LineSegment(0.5, 0.0));

//		inputSpline = new TorSpline(5.139, 6.165, 0.0);
//		inputSpline.add(new LineSegment(1.0, 2.0*Math.PI/3.0));
//		// Note: 0.5 was baaarely too short, so this path is garbage for competition purposes.
//		inputSpline.add(new ArcSegment(0.75, 150.0*(Math.PI/180.0)));
//		inputSpline.add(new LineSegment(3.025, 0.0));
//		inputSpline.add(new ArcSegment(1.654, -40.0*(Math.PI/180.0)));
//		inputSpline.add(new LineSegment(0.5, 0.0));
		
		inputSpline = new TorSpline(8.0, 4.0, 0.0);
		inputSpline.add(new LineSegment(1.0, 0.0));
		inputSpline.add(new ArcSegment(0.75, -253.74*(Math.PI/180.0)));
		inputSpline.add(new LineSegment(2.0, 0.0));
		inputSpline.add(new ArcSegment(0.75, 253.74*(Math.PI/180.0)));
		inputSpline.add(new LineSegment(2.0, 0.0));
		inputSpline.add(new ArcSegment(0.75, -253.74*(Math.PI/180.0)));
		inputSpline.add(new LineSegment(2.0, 0.0));
		inputSpline.add(new ArcSegment(0.75, 253.74*(Math.PI/180.0)));
		inputSpline.add(new LineSegment(1.0, 0.0));
		
		RealVector P = new ArrayRealVector(new double[] { 0.0, 0.0 });
		for (double s = 0.0; s <= inputSpline.length(); s += ds) {
			P = inputSpline.positionAt(s);
			graph.inputPath.add(P.getEntry(0), P.getEntry(1));
		}
		
		outputSpline = new SmoothSpline(inputSpline); // TODO: make it work for splines of splines
		for (double s = 0.0; s <= outputSpline.length(); s += ds) {
			P = outputSpline.positionAt(s);
			graph.outputPath.add(P.getEntry(0), P.getEntry(1));
		}

		trajectory = new SplineTrajectory(outputSpline);
		translationGraph = new VelocityGraph(trajectory, motionType.Translation);
		rotationGraph = new VelocityGraph(trajectory, motionType.Rotation);
		translationGraph.display();
		rotationGraph.display();
		translationGraph.plotData();
		rotationGraph.plotData();

		writer = new TrajectoryWriter(trajectory, new String("testSpline"));
		try {
			writer.writeDotJava(new String("org.usfirst.frc.team1197.robot"));
//			writer.writeDotCSV();
		} catch (IOException x) {
			System.err.format("IOException: %s%n", x);
		}
	}

}
