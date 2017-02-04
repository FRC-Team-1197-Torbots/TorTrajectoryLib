package org.usfirst.frc.team1197.TorTrajectoryLib;

import java.io.IOException;

public class CSVWriter extends StringWriter {

	private String s;

	public CSVWriter(TorTrajectory t) {
		super(t);
		// TODO Auto-generated constructor stub
	}

	public void write() throws IOException {
		for (long time = 0; time <= trajectory.totalTime(); time += dt) {
			lookUpData(time);
			s = new String("");
			s = s.concat(String.valueOf(time)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(pos)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(vel)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(acc)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(head)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(omg)).concat(String.valueOf(", "));
			s = s.concat(String.valueOf(alf)).concat(String.valueOf('\n'));
			fileWriter.write(s, 0, s.length());
		}
	}
}