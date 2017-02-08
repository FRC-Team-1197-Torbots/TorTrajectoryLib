package org.usfirst.frc.team1197.TorTrajectoryLib.spline_generator;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.Range;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.ApplicationFrame;
import org.jfree.ui.RefineryUtilities;

public class PathGraph extends ApplicationFrame {

	private static final long serialVersionUID = 4784689670538172011L;
	final XYSeries outputPath = new XYSeries("Output Path", false);	// "false" to disable auto-sort
	final XYSeries inputPath = new XYSeries("Input Path", false);	// so points plot in the order
	private final XYSeries border = new XYSeries("Border", false);	// they were added.
	static ChartPanel chartPanel;
	BufferedImage backgroundImage;
	NumberAxis rangeAx;
	NumberAxis domainAx;

	public PathGraph() {
		super("Path"); // window title = "Path"
		final XYDataset dataset = createDataset();
		// Configure axes:
		NumberAxis domainAx = new NumberAxis();
		domainAx.setRange(new Range(0.0, 16.5608));
		domainAx.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
		NumberAxis rangeAx = new NumberAxis();
		rangeAx.setRange(new Range(0.0, 8.2296));
		rangeAx.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
		// Configure series rendering options:
		XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
		renderer.setSeriesLinesVisible(0, true);
		renderer.setSeriesShapesVisible(0, false);
		renderer.setSeriesPaint(0, new Color(0x00, 0xFF, 0x00));
		renderer.setSeriesLinesVisible(1, true);
		renderer.setSeriesShapesVisible(1, false);
		renderer.setSeriesPaint(1, new Color(0x00, 0x00, 0x00));
		renderer.setSeriesLinesVisible(2, true);
		renderer.setSeriesShapesVisible(2, false);
		renderer.setSeriesPaint(2, new Color(0x00, 0x00, 0x00));
		// Create and configure the plot:
		SquaredXYPlot plot = new SquaredXYPlot(dataset, domainAx, rangeAx, renderer);
		plot.setOrientation(PlotOrientation.VERTICAL);
		plot.setSquaredToRange(true); 		// Define x-axis, and square y-axis to it.
		readBackgroundImage();
		plot.setBackgroundImage(backgroundImage);
		plot.setBackgroundPaint(Color.white);
		plot.setDomainGridlinePaint(Color.white);
		plot.setRangeGridlinePaint(Color.white);
		// Create the chart:
		JFreeChart chart = new JFreeChart("", JFreeChart.DEFAULT_TITLE_FONT, plot, false);
		chartPanel = new ChartPanel(chart);
		chartPanel.setMouseZoomable(true, false);
		chartPanel.setPreferredSize(new Dimension(824,420));
		setContentPane(chartPanel);
	}

	public void readBackgroundImage() {
		try {
			// the line that reads the image file
			backgroundImage = ImageIO.read(new File("backgroundImage.jpg"));
			// work with the image here ...
		} catch (IOException e) {
			// log the exception
			// re-throw if desired
		}
	}

	public void display() {
		this.pack();
		RefineryUtilities.centerFrameOnScreen(this);
		this.setVisible(true);
	}

	private XYDataset createDataset() {
		final XYSeriesCollection dataset = new XYSeriesCollection();
		dataset.addSeries(outputPath);
		dataset.addSeries(inputPath);
		dataset.addSeries(border);
		border.add(0.0,8.2296);
		border.add(16.5608,8.2296);
		border.add(16.5608,0.0);
		return dataset;
	}

}
