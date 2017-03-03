package org.usfirst.frc.team1197.TorTrajectoryLib;

import java.io.FileReader;
import java.io.IOException;

public class FileTrajectory extends TorTrajectory {

	String s;
	boolean endOfFileReached;

	public FileTrajectory(String fileName) {
		super();
		time.clear();
		translation.clear();
		rotation.clear();
		try (FileReader fileReader = new FileReader(fileName)) {
			endOfFileReached = false;
			while (!endOfFileReached) {
				time.add(readLong(fileReader));
				translation.add(readMotion(fileReader));
				rotation.add(readMotion(fileReader));
			}
			goal_pos = translation.get(translation.size()-1).pos;
			goal_head = rotation.get(rotation.size()-1).pos;
		} catch (IOException e) {
			System.err.println("The trajectory file \"" + fileName + "\" could not be accessed.");
		}
	}

	private MotionState1D readMotion(FileReader fileReader) throws IOException {
		MotionState1D m = new MotionState1D();
		for (int i = 0; i < 3; i++) {
			m.set(i, readDouble(fileReader));
		}
		return m;
	}

	private double readDouble(FileReader fileReader) throws IOException {
		String s = "";
		int cUTF16;
		char cUTF8;
		while((cUTF16 = fileReader.read()) != -1){
			cUTF8 = (char) cUTF16;
			if (cUTF8 == ',' || cUTF8 == '\n'){
				break;
			}
			if (isValidDoubleChar(cUTF8)) {
				s = s + cUTF8;
			}
		}
		if(cUTF16 == -1){
			endOfFileReached = true;
		}
		return Double.parseDouble(s);
	}
	
	private long readLong(FileReader fileReader) throws IOException {
		String s = "";
		int cUTF16;
		char cUTF8;
		while((cUTF16 = fileReader.read()) != -1){
			cUTF8 = (char) cUTF16;
			if (cUTF8 == ',' || cUTF8 == '\n'){
				break;
			}
			if (isValidLongChar(cUTF8)) {
				s = s + cUTF8;
			}
		}
		if(cUTF16 == -1){
			endOfFileReached = true;
		}
		System.out.println(s);
		return Long.parseLong(s);
	}

	private boolean isValidDoubleChar(char c) {
		return ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E');
	}

	private boolean isValidLongChar(char c) {
		return ((c >= '0' && c <= '9') || c == '-' || c == '+');
	}

}
