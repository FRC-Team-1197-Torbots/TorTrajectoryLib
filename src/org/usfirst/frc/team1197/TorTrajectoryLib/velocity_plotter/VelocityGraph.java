package org.usfirst.frc.team1197.TorTrajectoryLib.velocity_plotter;

import java.awt.Color;
import java.awt.image.BufferedImage;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;
import org.usfirst.frc.team1197.TorTrajectoryLib.TorTrajectory;

public class VelocityGraph extends ApplicationFrame {
	private static final long serialVersionUID = -3149991762921866114L;
	public enum motionType{Translation, Rotation;}
	private motionType graphType;
	private TorTrajectory trajectory;
	XYSeries position;
	XYSeries velocity;
	XYSeries acceleration;
	static ChartPanel chartPanel;
	BufferedImage backgroundImage;
	double xMin;
	double xMax;
	double yMax;
	double yMin;
	NumberAxis rangeAx;
	NumberAxis domainAx;

	public VelocityGraph(TorTrajectory trajectory, motionType graphType) {
		super(graphType.toString());
		this.graphType = graphType;
		this.trajectory = trajectory;
		xMin = 0.0;
		xMax = trajectory.totalTime()*0.001;
		if (xMax == 0.0){
			xMax = 1.0; // so we don't try to make an x-axis with zero length.
		}
		if (graphType == motionType.Rotation){
			position = new XYSeries("Heading");
			velocity = new XYSeries("Omega");
			acceleration = new XYSeries("Alpha");
			yMax = 1.25*Math.abs(Math.max(0.0,  
								 		  Math.max(trajectory.goal_head(),
								 				   Math.max(trajectory.max_omg(),
								 						    trajectory.max_alf()))));
			yMax *= 180.0/Math.PI;
		} else {
			position = new XYSeries("Position");
			velocity = new XYSeries("Velocity");
			acceleration = new XYSeries("Acceleration");
			yMax = 1.25*Math.abs(Math.max(0.0,
										  Math.max(trajectory.goal_pos(),
												   Math.max(trajectory.max_vel(),
						    							    trajectory.max_acc()))));

		}
		if (yMax == 0.0){
			yMax = 1.0; // so we don't try to make a y-axis with zero length
		}
		yMin = -yMax;
		final XYDataset dataset = createDataset();
		// Configure axes:
		NumberAxis domainAx = new NumberAxis();
		domainAx.setRange(new Range(xMin, xMax));
//		domainAx.setStandardTickUnits(NumberAxis.createStandardTickUnits());
		NumberAxis rangeAx = new NumberAxis();
		rangeAx.setRange(new Range(yMin, yMax));
//		rangeAx.setStandardTickUnits(NumberAxis.createStandardTickUnits());
		// Configure series rendering options:
		XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
		renderer.setSeriesLinesVisible(0, true);
		renderer.setSeriesShapesVisible(0, false);
		renderer.setSeriesPaint(0, new Color(0x00, 0x00, 0x00));
		renderer.setSeriesLinesVisible(1, true);
		renderer.setSeriesShapesVisible(1, false);
		renderer.setSeriesPaint(1, new Color(0xFF, 0x00, 0x00));
		renderer.setSeriesLinesVisible(2, true);
		renderer.setSeriesShapesVisible(2, false);
		renderer.setSeriesPaint(2, new Color(0x00, 0x00, 0xFF));
		// Create and configure the plot:
		XYPlot plot = new XYPlot(dataset, domainAx, rangeAx, renderer);
//		plot.setOrientation(PlotOrientation.VERTICAL);
		plot.setBackgroundPaint(new Color(0xFF, 0xFF, 0xFF));
		plot.setDomainGridlinePaint(new Color(0xCC, 0xCC, 0xCC));
		plot.setRangeGridlinePaint(new Color(0xCC, 0xCC, 0xCC));
		// Create the chart:
		JFreeChart chart = new JFreeChart("", JFreeChart.DEFAULT_TITLE_FONT, plot, false);
		chartPanel = new ChartPanel(chart);
//		chartPanel.setPreferredSize(new Dimension(824,420));
		setContentPane(chartPanel);
	}

	public void display() {
		this.pack();
		RefineryUtilities.centerFrameOnScreen(this);
		this.setVisible(true);
	}
	
	public void plotData(){
		double dt = 5;
		double pos;
		double vel;
		double acc;
		if (graphType == motionType.Rotation){
			for (long time = 0; time <= trajectory.totalTime(); time+=dt){
				pos = trajectory.lookUpHeading(time);
				vel = trajectory.lookUpOmega(time);
				acc = trajectory.lookUpAlpha(time);
				position.add(time*0.001, pos*(180.0/Math.PI));
				velocity.add(time*0.001, vel*(180.0/Math.PI));
				acceleration.add(time*0.001, acc*(180.0/Math.PI));
		    }
		}else{
			for (long time = 0; time <= trajectory.totalTime(); time+=dt){
				pos = trajectory.lookUpPosition(time);
				vel = trajectory.lookUpVelocity(time);
				acc = trajectory.lookUpAcceleration(time);
				position.add(time*0.001, pos);
				velocity.add(time*0.001, vel);
				acceleration.add(time*0.001, acc);
		    }
		}
	}
	
	private XYDataset createDataset() {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		dataset.addSeries(position);
		dataset.addSeries(velocity);
		dataset.addSeries(acceleration);
//		border.add(0.0,8.2296);
//		border.add(16.5608,8.2296);
//		border.add(16.5608,0.0);
		return dataset;
	}

}
