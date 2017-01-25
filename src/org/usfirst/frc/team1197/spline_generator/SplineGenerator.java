package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SplineGenerator {
	
	private static final double ds = 0.005;
	private static Spline inputSpline;

    public static void main(final String[] args) {
        
        final PathGraph graph = new PathGraph();
        graph.display();
        PathSegment line1 = new LineSegment(2, 0);
        PathSegment line2 = new LineSegment(2, Math.PI/6);
        PathSegment line3 = new LineSegment(2, -Math.PI/6);
        inputSpline = new Spline(1.0, 4.0, 0.0);
        inputSpline.add(line1);
        inputSpline.add(line2);
        inputSpline.add(line3);
        
        RealVector P = new ArrayRealVector(new double[] {0.0, 0.0});
        for (double s = 0.0; s <= inputSpline.length(); s += ds){
        	P = inputSpline.positionAt(s);
        	graph.inputPath.add(P.getEntry(0), P.getEntry(1));
        	// graph.outputPath.add(x(t), 0.0);        	
        }
    }
}
