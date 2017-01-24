package org.usfirst.frc.team1197.spline_generator;

public class SplineGenerator {
	
	private static final double dt = 0.005;
	private static Point P;
	private static PathSegment line;

    public static void main(final String[] args) {
        
        final PathGraph graph = new PathGraph();
        graph.display();
        P = new Point(0.0, 0.0);
        line = new LineSegment(3.0, 4.0);
        
        for (double t = 0.0; t <= line.length(); t += dt){
        	P = line.pointAt(t);
        	graph.inputPath.add(P.x, P.y);
        	// graph.outputPath.add(x(t), 0.0);        	
        }
    }
}
