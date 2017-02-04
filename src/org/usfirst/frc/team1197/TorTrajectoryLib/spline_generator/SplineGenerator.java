package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SplineGenerator {
	
	private static final double ds = 0.005;
	private static Spline inputSpline;

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
    }
}
