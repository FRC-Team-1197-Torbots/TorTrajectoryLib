package org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer;

import java.io.IOException;
import java.text.DecimalFormat;

import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class DotJavaWriter extends MotionDataWriter {
	private String packageName;
	private String className;
	private final DecimalFormat df = new DecimalFormat("0.####");

	public DotJavaWriter(TorTrajectory t, String p, String c) {
		super(t);
		packageName = p;
		className = c;

	}

	@Override
	public void write() throws IOException {
		s = "package ".concat(packageName).concat(";\n\n");
		s = s.concat("public class ").concat(className).concat(" extends TorTrajectory {\n\n");
		fileWriter.write(s, 0, s.length());
		s = "\tpublic ".concat(className).concat("() {\n");
		s = s.concat("\t\tsuper(").concat(df.format(trajectory.goalPos())).concat(", ")
				.concat(df.format(trajectory.goalHead())).concat(");\n");
		s = s.concat("\t\ttime.clear();\n\t\ttranslation.clear();\n\t\trotation.clear();\n");
		fileWriter.write(s, 0, s.length());

		for (long time = 0; time <= trajectory.totalTime(); time += dt) {
			lookUpData(time);
			s = "\t\ttestMethod(".concat(String.valueOf(time)).concat(", ");
			
			s = s.concat(df.format(pos)).concat(", ");
			s = s.concat(df.format(vel)).concat(", ");
			s = s.concat(df.format(acc)).concat(", ");
			s = s.concat(df.format(head)).concat(", ");
			s = s.concat(df.format(omg)).concat(", ");
			s = s.concat(df.format(alf)).concat(");\n");
			
			fileWriter.write(s, 0, s.length());
		}
		s = s.concat("\t}\n");
		s = s.concat("\tpublic void ").concat("testMethod").concat("(int t, double pos, double vel, double acc, double hed, double omg, double alf) {\n");
		s = s.concat("\t\ttime.add((long) ").concat("t").concat(");\n");
		
		s = s.concat("\t\ttranslation.add(new MotionState1D(");
		s = s.concat("pos, ");
		s = s.concat("vel, ");
		s = s.concat("acc));\n");

		s = s.concat("\t\trotation.add(new MotionState1D(");
		s = s.concat("hed, ");
		s = s.concat("omg, ");
		s = s.concat("alf));\n");
		
		s = s.concat("\t}");
		s = s.concat("\n\n}");
		fileWriter.write(s, 0, s.length());
	}
}
