package org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer;

import java.io.IOException;

import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class DotCSVWriter extends MotionDataWriter {

	public DotCSVWriter(TorTrajectory t) {
		super(t);
		// TODO Auto-generated constructor stub
	}

	@Override
	public void write() throws IOException {
		for (long time = 0; time <= trajectory.totalTime(); time += dt) {
			lookUpData(time);
			s = "";
			s = s + String.valueOf(time) + ", ";
			s = s + String.valueOf(pos) + ", ";
			s = s + String.valueOf(vel) + ", ";
			s = s + String.valueOf(acc) + ", ";
			s = s + String.valueOf(head) + ", ";
			s = s + String.valueOf(omg) + ", ";
			s = s + String.valueOf(alf);
			if (time < trajectory.totalTime()){
				s = s + '\n';
			}
			fileWriter.write(s, 0, s.length());
		}
	}
}
