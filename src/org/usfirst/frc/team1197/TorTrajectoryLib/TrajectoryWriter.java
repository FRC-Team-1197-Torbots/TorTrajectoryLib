package org.usfirst.frc.team1197.TorTrajectoryLib;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;

public class TrajectoryWriter {
	private File outputFile;
	private String fileName;
	private TorTrajectory trajectory;
	public TrajectoryWriter(String s){
		fileName = s;
	}
	public void setTrajectory(TorTrajectory t){
		trajectory = t;
	}
	public void setFile(String fileName){
		outputFile = new File(fileName);
	}
	public void writeDotJava() throws IOException{
		String fileNameDotJava = fileName.concat(".java");
		outputFile = new File(fileNameDotJava);
		Charset charset = Charset.forName("US-ASCII");
		String s = "test";
		BufferedWriter writer = null;
		try {
		    writer = Files.newBufferedWriter(outputFile.toPath(), charset);
		    writer.write(s, 0, s.length());
		} catch (IOException x) {
		    System.err.format("IOException: %s%n", x);
		} finally {
		    if (writer != null)
		    	writer.close();
		}
		
	}
	public void writeCSV(){
		
	}
	public void writeJavaObject(){
		
	}
}
