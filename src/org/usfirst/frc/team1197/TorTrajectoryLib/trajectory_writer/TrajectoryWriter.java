package org.usfirst.frc.team1197.TorTrajectoryLib.trajectory_writer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;

import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class TrajectoryWriter {
	private File outputFile;
	private String fileName;
	private TorTrajectory trajectory;
	private MotionDataWriter stringWriter;
	
	public TrajectoryWriter(TorTrajectory t, String name){
		trajectory = t;
		fileName = name;
	}
	
	public TrajectoryWriter() {
		// TODO Auto-generated constructor stub
	}
	
	public void setFile(String fileName){
		outputFile = new File(fileName);
	}
	
	public void writeDotJava(String p) throws IOException{
		String fileNameDotJava = fileName.concat(".java");
		outputFile = new File(fileNameDotJava);
		stringWriter = new DotJavaWriter(trajectory, p, fileName);
		writeTextFile(outputFile, stringWriter);
	}
	
	public void writeDotCSV() throws IOException{
		String fileNameDotCSV = fileName.concat(".CSV");
		outputFile = new File(fileNameDotCSV);
		stringWriter = new DotCSVWriter(trajectory);
		writeTextFile(outputFile, stringWriter);
	}
	
	private void writeTextFile(File outputFile, MotionDataWriter stringWriter) throws IOException{
		Charset charset = Charset.forName("US-ASCII");
		BufferedWriter fileWriter = null;
		try {
		    fileWriter = Files.newBufferedWriter(outputFile.toPath(), charset);
		    stringWriter.setFileWriter(fileWriter);
		    stringWriter.write();
		} catch (IOException x) {
		    System.err.format("IOException: %s%n", x);
		} finally {
		    if (fileWriter != null)
		    	fileWriter.close();
		}
	}
	
	public void writeJavaObject(){
		
	}
}
