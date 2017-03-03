package org.usfirst.frc.team1197.TorTrajectoryLib;

import java.io.FileReader;
import java.io.IOException;

public class FileTrajectory extends TorTrajectory {

	public FileTrajectory(String fileName) {
		super();
		time.clear();
		translation.clear();
		rotation.clear();
		try (FileReader fileReader = new FileReader(fileName)) {
//			String s = new String("");
			char c;
//			InputType inputType;
//			MotionOrder motionOrder;
			while ((c = (char) fileReader.read()) != -1) {
				time.add((long) c);
			}
		} catch (IOException e) {
			System.err.println("The trajectory file \"" + fileName + "\" could not be found.");
		}
	}
	
	private enum InputType{TIME, TRANSLATION, ROTATION};
	private enum MotionOrder{POS, VEL, ACC};

}
