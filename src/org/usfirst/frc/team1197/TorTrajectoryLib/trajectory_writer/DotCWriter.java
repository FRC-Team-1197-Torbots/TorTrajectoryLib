package org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer;

import java.io.IOException;
import java.text.DecimalFormat;

import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class DotCWriter extends MotionDataWriter {
	private final DecimalFormat df = new DecimalFormat("######");

	public DotCWriter(TorTrajectory t) {
		super(t);
	}

	@Override
	public void write() throws IOException {
		s = "Trajectory t = {\n\t";
		s = s.concat(df.format((trajectory.totalTime()/50)+2)).concat(",\n\t");
		s = s.concat("{");
		fileWriter.write(s, 0, s.length());

		for (long time = 0; time < trajectory.totalTime(); time += 10*dt) {
			lookUpData(time);
			s = "{";
			s = s.concat(df.format(vel*1000)).concat(", ");
			s = s.concat(df.format(acc*1000)).concat(", ");
			s = s.concat(df.format(omg*1000)).concat(", ");
			s = s.concat(df.format(alf*1000)).concat("},\n\t");
			fileWriter.write(s, 0, s.length());
		}
		s = "{0, 0, 0, 0}}\n};\n";
		fileWriter.write(s, 0, s.length());
	}
}
