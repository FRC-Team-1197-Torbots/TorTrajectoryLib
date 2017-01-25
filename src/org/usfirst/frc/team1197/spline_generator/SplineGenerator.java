package org.usfirst.frc.team1197.spline_generator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class SplineGenerator {
	
	private static final double ds = 0.005;
	private static Spline inputSpline;

    public static void main(final String[] args) {
        
        final PathGraph graph = new PathGraph();
        graph.display();
        PathSegment line1 = new LineSegment(1.0, 0.0);
        PathSegment arcA = new ArcSegment(1.0, Math.PI/3.0);
        PathSegment line2 = new LineSegment(1.0, 0.0);
        PathSegment arcB = new ArcSegment(1.0, Math.PI/3.0);
        Spline baseSpline = new Spline(0.0, 0.0, Math.PI/6.0, true);
        baseSpline.add(line1);
        baseSpline.add(arcA);
        baseSpline.add(line2);
        baseSpline.add(arcB);
        Spline inputSpline = new Spline(5.0, 3.0, Math.PI/6.0, true);
        inputSpline.add(baseSpline);
        inputSpline.add(baseSpline);
        inputSpline.add(baseSpline);
        
        RealVector P = new ArrayRealVector(new double[] {0.0, 0.0});
        for (double s = 0.0; s <= inputSpline.length(); s += ds){
        	P = inputSpline.positionAt(s);
        	graph.inputPath.add(P.getEntry(0), P.getEntry(1));
        	// graph.outputPath.add(x(t), 0.0);        	
        }
    }
}
