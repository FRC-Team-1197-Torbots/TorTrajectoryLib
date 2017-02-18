package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.util.List;

import org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator.PathSegment.SegmentType;

class SplineError {
	protected static boolean checkPathIsLegal(List<PathSegment> path){
		if (path.size() == 0) {
			System.err.println("Error creating SmoothSpline: input path is empty.");
			return false;
		}
		boolean isLegal = false;
		PathSegment badSeg1 = null, badSeg2 = null, badSeg3 = null;
		if (path.size() == 1) {
			isLegal = true;
		}
		if (path.size() == 2) {
			if (path.get(0).type() == SegmentType.LINE && path.get(1).type() == SegmentType.LINE) {
				isLegal = true;
			} else {
				isLegal = false;
				badSeg1 = path.get(0);
				badSeg2 = path.get(1);
			}
		} else {
			for (int i = 1; i < path.size() - 1 ; i++){
				if (path.get(i-1).type() == SegmentType.LINE 
				 && path.get(i).type()   == SegmentType.LINE){
					isLegal = true;
				} else if (path.get(i-1).type() == SegmentType.LINE 
						&& path.get(i).type() == SegmentType.ARC 
						&& path.get(i+1).type() == SegmentType.LINE) {
					isLegal = true;
					i++;
				} else {
					isLegal = false;
					badSeg1 = path.get(i-1);
					badSeg2 = path.get(i);
					badSeg3 = path.get(i+1);
					break;
				}
			}
		}
		if (!isLegal) {
			String s = "Error creating SmoothSpline: PathSegment sequence ";
			if(badSeg1 != null){
				s = s.concat(badSeg1.type().toString());
				if(badSeg2 != null || badSeg3 != null){
					s = s.concat("->");
				}
			}
			if(badSeg2 != null){
				s = s.concat(badSeg2.type().toString());
				if(badSeg3 != null){
					s = s.concat("->");
				}
			}
			if(badSeg3 != null){
				s = s.concat(badSeg3.type().toString());
			}
			s = s.concat(" is not allowed. (Must be LINE->LINE or LINE->ARC->LINE.)");
			System.err.println(s);
			return false;
		}
		return true;
	}
}
