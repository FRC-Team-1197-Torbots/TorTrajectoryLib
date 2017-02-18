package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.text.DecimalFormat;
import java.util.List;

import org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator.PathSegment.SegmentType;

class SplineErrMsg {
	
	private static DecimalFormat df = new DecimalFormat("0.####");
	private static final String basic_msg = "Error creating SmoothSpline: ";
	
	protected static boolean pathIllegalAlert(List<PathSegment> path) {
		if (path.size() == 0) {
			System.err.println(basic_msg + "input path is empty.");
			return false;
		}
		boolean isIllegal = true;
		PathSegment badSeg1 = null, badSeg2 = null, badSeg3 = null;
		if (path.size() == 1) {
			isIllegal = false;
		}
		if (path.size() == 2) {
			if (path.get(0).type() == SegmentType.LINE && path.get(1).type() == SegmentType.LINE) {
				isIllegal = false;
			} else {
				isIllegal = true;
				badSeg1 = path.get(0);
				badSeg2 = path.get(1);
			}
		} else {
			for (int i = 1; i < path.size() - 1; i++) {
				if (path.get(i - 1).type() == SegmentType.LINE && path.get(i).type() == SegmentType.LINE) {
					isIllegal = false;
				} else if (path.get(i - 1).type() == SegmentType.LINE && path.get(i).type() == SegmentType.ARC
						&& path.get(i + 1).type() == SegmentType.LINE) {
					isIllegal = false;
					i++;
				} else {
					isIllegal = true;
					badSeg1 = path.get(i - 1);
					badSeg2 = path.get(i);
					badSeg3 = path.get(i + 1);
					break;
				}
			}
		}
		if (isIllegal) {
			String s = basic_msg + "PathSegment sequence ";
			if (badSeg1 != null) {
				s = s + badSeg1.type();
				if (badSeg2 != null || badSeg3 != null) {
					s = s + "->";
				}
			}
			if (badSeg2 != null) {
				s = s + badSeg2.type();
				if (badSeg3 != null) {
					s = s + "->";
				}
			}
			if (badSeg3 != null) {
				s = s + badSeg3.type();
			}
			s = s + " is not allowed. (Must be LINE->LINE or LINE->ARC->LINE.)";
			System.err.println(s);
			return true;
		}
		return false;
	}

	protected static boolean tooShortAlert(List<PathSegment> path, int i_use, int i_disp, double needed_length) {
		double actual_length = path.get(i_use).length();
		if ((actual_length < 0) || ((actual_length - needed_length) < 0)) {
			String s = basic_msg + "The ";
			s = s + ordinal(i_disp+1);
			s = s + " input path segment is too short by ";
			s = s + df.format(needed_length - actual_length); // If this comes out negative, something is really weird.
			s = s + "m.";
			System.err.println(s);
			return true;
		}
		return false;
	}
	
	protected static boolean unexpectedSegmentAlert(SegmentType type) {
		System.err.println(basic_msg + "Unexpected path segment (" + type + ").");
		return true;
	}

	private static String ordinal(int i) {
		String[] sufixes = new String[] { "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th" };
		switch (i % 100) {
		case 11:
		case 12:
		case 13:
			return i + "th";
		default:
			return i + sufixes[i % 10];

		}
	}
}
