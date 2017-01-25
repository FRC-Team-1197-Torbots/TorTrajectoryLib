package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SplineGenerator {
	
	private static final double ds = 0.005;
	private static RealVector P;
	private static PathSegment line;

    public static void main(final String[] args) {
        
        final PathGraph graph = new PathGraph();
        graph.display();
        P = new ArrayRealVector(new double[] {0.0, 0.0}, false);
        line = new LineSegment(5*1.414, Math.PI/6);
        
        for (double s = 0.0; s <= line.length(); s += ds){
        	P = line.positionAt(s);
        	graph.inputPath.add(P.getEntry(0), P.getEntry(1));
        	// graph.outputPath.add(x(t), 0.0);        	
        }
    }
}
