package org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer;

import java.io.BufferedWriter;
import java.io.IOException;

import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public abstract class StringWriter {
	protected BufferedWriter fileWriter;
	protected TorTrajectory trajectory;
	protected long dt = 5;
	protected double pos, vel, acc;
	protected double head, omg, alf;
	protected String s;
	
	public StringWriter(TorTrajectory t){
		trajectory = t;
	}
	
	public void setFileWriter(BufferedWriter w){
		fileWriter = w;
	}
	
	public abstract void write() throws IOException;
	
	protected void lookUpData(long time){
		pos = trajectory.lookUpPosition(time);
		vel = trajectory.lookUpVelocity(time);
		acc = trajectory.lookUpAcceleration(time);
		head = trajectory.lookUpHeading(time);
		omg = trajectory.lookUpOmega(time);
		alf = trajectory.lookUpAlpha(time);
	}
}
