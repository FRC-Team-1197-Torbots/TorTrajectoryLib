package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.io.IOException;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.usfirst.frc.team1197.TorTrajectoryLib.TrajectoryWriter;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph;
import org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter.VelocityGraph.motionType;

public class SplineGenerator {
	
	private static final double ds = 0.005;
	private static Spline inputSpline;
	private static SplineTrajectory trajectory;
	private static VelocityGraph translationGraph;
	private static VelocityGraph rotationGraph;
	private static TrajectoryWriter writer;

    public static void main(final String[] args) {
        
        final PathGraph graph = new PathGraph();
        graph.display();
        PathSegment line = new LineSegment(1.0, 0.0);
        PathSegment arc = new ArcSegment(1.0, Math.PI/3.0);
        Spline basicSpline = new Spline(0.0, 0.0, 0.0);
        basicSpline.add(line);
        basicSpline.add(arc);
        inputSpline = new Spline(6.0, 2.0, 0.0, true);
        inputSpline.add(basicSpline);
        inputSpline.add(basicSpline);
        inputSpline.add(line);
        
        RealVector P = new ArrayRealVector(new double[] {0.0, 0.0});
        for (double s = 0.0; s <= inputSpline.length(); s += ds){
        	P = inputSpline.positionAt(s);
        	graph.inputPath.add(P.getEntry(0), P.getEntry(1));  	
        }
        
        trajectory = new SplineTrajectory(inputSpline);
        translationGraph = new VelocityGraph(trajectory, motionType.Translation);
        rotationGraph = new VelocityGraph(trajectory, motionType.Rotation);
        translationGraph.display();
        rotationGraph.display();
        translationGraph.plotData();
        rotationGraph.plotData();
        
        writer = new TrajectoryWriter(trajectory, new String("testSpline"));
        try {
			writer.writeDotCSV();
		} catch (IOException x) {
			System.err.format("IOException: %s%n", x);
		}
        
    }
}
