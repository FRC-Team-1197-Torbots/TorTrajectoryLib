package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.List;

public class SmoothSpline extends TorSpline {

	private TorSpline inputSpline;
	private HalfSpiralSpline optimizingSpline = new HalfSpiralSpline();
	private double computedPivotX;
	private double computedPivotY;

	public SmoothSpline(TorSpline s) {
		super(s.externalTranslation().getEntry(0), s.externalTranslation().getEntry(1), s.externalRotation());
		inputSpline = s.clone();
		int input_index, output_index = 1;
		if (!SplineErrMsg.pathIllegalAlert(inputSpline.path)) {
			this.add(inputSpline.path.get(0).clone());
			for (input_index = 1; input_index < inputSpline.path.size()-1; input_index++) {
				if (inputSpline.path.get(input_index-1).type() == SegmentType.LINE 
				 && inputSpline.path.get(input_index).type() == SegmentType.ARC
				 && inputSpline.path.get(input_index+1).type() == SegmentType.LINE){
					replaceArc(inputSpline.path, this.path, input_index, output_index);
					input_index++;
					output_index++;
				}
				else if (inputSpline.path.get(input_index-1).type() == SegmentType.LINE 
						&& inputSpline.path.get(input_index).type() == SegmentType.LINE) {
					spliceLines(inputSpline.path, path, input_index, output_index);
					output_index++; // since we added an extra path segment on the output path, but not the input
				} 
				else {
					SplineErrMsg.unexpectedSegmentAlert(inputSpline.path.get(input_index).type());
					break;
				}
			}
		}
	}

	private void replaceArc(List<PathSegment> inputPath, List<PathSegment> outputPath, int in_i, int out_i) {
		double angle = inputPath.get(in_i).totalAngle();
		double curvature = inputPath.get(in_i).curvatureAt(0.0);
		double radius = secantMethod(Math.abs(angle), Math.abs(1.0 / curvature));
		if (!SplineErrMsg.tooShortAlert(outputPath, out_i - 1, computedPivotX)
				&& !SplineErrMsg.tooShortAlert(inputPath, in_i + 1, computedPivotX)) {
			outputPath.get(out_i - 1).addToLength(-computedPivotX);
			this.addToLength(-computedPivotX);
			this.add(new SpiralSpline(angle, radius));
			this.add(inputPath.get(in_i + 1).cloneTrimmedBy(computedPivotX));
		}
	}
	
	private void spliceLines(List<PathSegment> inputPath, List<PathSegment> outputPath, int in_i, int out_i) {
		double angle = inputPath.get(in_i).internalRotation();
		SpiralSpline newSpline = new SpiralSpline(angle);
		double length_to_cut = Math.abs(newSpline.pivot_x()) 
				+ Math.abs(newSpline.pivot_y())/Math.tan(0.5*(Math.PI - Math.abs(angle)));
		//TODO: Handle segments with 0 internal rotation--------------------------^^^
		//Probably add a trivial spiral spline, i.e. a short line segment
		if (!SplineErrMsg.tooShortAlert(outputPath, out_i - 1, length_to_cut)
				&& SplineErrMsg.tooShortAlert(inputPath, in_i, length_to_cut)) {
			outputPath.get(out_i - 1).addToLength(-length_to_cut);
			this.addToLength(-length_to_cut);
			this.add(newSpline);
			this.add(inputPath.get(in_i).cloneTrimmedBy(length_to_cut));
		}
	}
	
	

	private double secantMethod(double angle, double radius) {
		double accuracy = 1.0e-8;
		int max_iterations = 100;
		double x = radius, x_prev = radius * 1.001;
		double f = rootFunction(angle, x, radius);
		double f_prev = rootFunction(angle, x_prev, radius);
		double q;
		for (int i = 0; i < max_iterations; i++) {
			f = rootFunction(angle, x, radius);
			// System.out.println("x = " + x + ";\tf = " + f);
			if (Math.abs(f) <= accuracy)
				break;
			q = slopeFunction(x, f, x_prev, f_prev);
			x_prev = x;
			f_prev = f;
			x = x - f / q;
		}
		return x;
	}

	private double rootFunction(double angle, double radius, double targetY) {
		optimizingSpline.buildFirstHalfOnly(angle, radius);
		computedPivotX = optimizingSpline.pivot_x();
		computedPivotY = optimizingSpline.pivot_y();
		return targetY - computedPivotY;
	}

	private double slopeFunction(double x, double f, double x_prev, double f_prev) {
		return (f - f_prev) / (x - x_prev);
	}

}
