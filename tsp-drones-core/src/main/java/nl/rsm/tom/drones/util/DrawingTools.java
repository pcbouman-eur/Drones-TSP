/* Copyright 2017 Paul Bouman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the  
 * "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * In case you use this software for research purposes, it is appreciated if you provide a citation of the following paper:
 * 
 * N.A.H. Agatz, P.C. Bouman & M.E. Schmidt. Optimization Approaches for the Traveling Salesman Problem with Drone. Transportation Science.
 * 
 * The paper still has to appear, but was accepted for publication. This notice will be updated with a more detailed reference if that
 * information is available.
 */

package nl.rsm.tom.drones.util;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Stroke;
import java.awt.image.BufferedImage;
import java.awt.image.RenderedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Utility class that provides a number of methods to make drawing of instances and solutions
 * @author Paul Bouman
 *
 */

public class DrawingTools
{
	
	/**
	 * Interface that is used to map from the double coordinate space to the drawing coordinate space
	 * @author Paul Bouman
	 *
	 */
	@FunctionalInterface
	public static interface Mapping
	{
		public Point transform(double x, double y);
	}
	
	/**
	 * Creates a mapping based on a bounding box
	 * @param minX The minimum x coordinate of the input
	 * @param minY The minimum y coordinate of the input
	 * @param maxX The maximum x coordinate of the input
	 * @param maxY The maximum y coordinate of the input
	 * @param width The width of the output
	 * @param height The height of the output
	 * @param margin The margin of the output
	 * @return The resulting mapping
	 */
	public static Mapping defaultMapping(double minX, double minY, double maxX, double maxY, int width, int height, int margin)
	{
		
		double xScale = (1d*(width-(2*margin)))/((maxX-minX));
		double yScale = (1d*(height-(2*margin)))/((maxY-minY));
		return (x,y) -> new Point( margin + (int)Math.round((x-minX)*xScale),
								   margin + (int)Math.round((y-minY)*(yScale)));
	}
	
	/**
	 * Creates a mapping for an instance
	 * @param gi The instance for which this mapping is produced
	 * @param width Then width of the output
	 * @param height The height of the output
	 * @param margin The margin of the output
	 * @return The resulting mapping
	 */
	public static Mapping defaultMapping(Instance<Vec2D> gi, int width, int height, int margin)
	{
		double minX = Double.POSITIVE_INFINITY;
		double maxX = Double.NEGATIVE_INFINITY;
		double minY = Double.POSITIVE_INFINITY;
		double maxY = Double.NEGATIVE_INFINITY;
		for (Vec2D v : gi)
		{
			minX = Math.min(minX, v.x);
			maxX = Math.max(maxX, v.x);
			minY = Math.min(minY, v.y);
			maxY = Math.max(maxY, v.y);
		}
		return defaultMapping(minX, minY, maxX, maxY, width, height, margin);
	}
	
	/**
	 * Creates a mapping based on an instance with 3% of the output box as its margin
	 * @param gi The instance
	 * @param width The width of the output
	 * @param height The height of the output
	 * @return The resulting mapping
	 */
	public static Mapping defaultMapping(Instance<Vec2D> gi, int width, int height)
	{
		int margin = (int) Math.ceil(0.03 * Math.max(width,height));
		return defaultMapping(gi, width, height, margin);
	}
	
	/**
	 * Draws an instance on a Graphics object
	 * @param gi The instance to be drawn
	 * @param m The mapping to be used
	 * @param gr The graphics object to draw on
	 * @param labels Whether or not to print location labels
	 */
	private static void draw(Instance<Vec2D> gi, Mapping m, Graphics2D gr, boolean labels)
	{
		for (Vec2D v : gi)
		{
			Point p = m.transform(v.x, v.y);
			gr.drawRect(p.x-1, p.y-1, 3, 3);
			if (labels)
			{
				gr.drawString(v.getName(), p.x, p.y);
			}
		}
	}
	
	/**
	 * Draws the truck itinerary on the graphics object
	 * @param sol The solution to be drawn
	 * @param m The mapping to use to drawn
	 * @param gr The graphics object to draw on
	 */
	private static void drawDrive(Solution<Vec2D> sol, Mapping m, Graphics2D gr)
	{
		for (Operation<Vec2D> op : sol)
		{
			Point prev = null;
			for (Vec2D v : op.getDrivePath())
			{
				Point p = m.transform(v.x, v.y);
				if (prev != null)
				{
					int x1 = prev.x;
					int y1 = prev.y;
					int x2 = p.x;
					int y2 = p.y;
					gr.drawLine(x1, y1, x2, y2);
				}
				prev = p;
			}
		}
	}
	
	/**
	 * Draws the drone itinerary
	 * @param sol The solution to drawn
	 * @param m The mapping to use
	 * @param gr The graphics object to draw on
	 */
	
	public static void drawFly(Solution<Vec2D> sol, Mapping m, Graphics2D gr)
	{
		for (Operation<Vec2D> op : sol)
		{
			if (op.hasFly())
			{
				Vec2D vs = op.getStart();
				Vec2D ve = op.getEnd();
				Vec2D vf = op.getFly();
				Point p1 = m.transform(vs.x, vs.y);
				Point p2 = m.transform(vf.x, vf.y);
				Point p3 = m.transform(ve.x, ve.y);
				gr.drawLine(p1.x, p1.y, p2.x, p2.y);
				gr.drawLine(p2.x, p2.y, p3.x, p3.y);
			}
		}
	}
	
	/**
	 * Renders an image where the instance is drawn
	 * @param gi The instance to render
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels Whether to print labels or not
	 * @return The rendered image of the instance
	 */
	public static RenderedImage drawInstance(Instance<Vec2D> gi, int w, int h, boolean labels)
	{
		Mapping m = defaultMapping(gi, w, h);
		BufferedImage bi = new BufferedImage(w,h, BufferedImage.TYPE_INT_RGB);
		Graphics2D gr = (Graphics2D) bi.getGraphics();
		gr.setBackground(Color.WHITE);
		gr.clearRect(0, 0, w, h);
		gr.setColor(Color.BLACK);
		draw(gi, m, gr, labels);
		return bi;
	}
	
	/**
	 * Draws an instance to a file as a PNG file
	 * @param gi The instance to be drawn
	 * @param f The file it should be written to
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels The labels to be drawn
	 * @throws IOException can occur while attempting to write the image to a file
	 */
	
	public static void drawInstance(Instance<Vec2D> gi, File f, int w, int h, boolean labels) throws IOException
	{
		RenderedImage i = drawInstance(gi,w,h,labels);
		ImageIO.write(i, "PNG", f);
	}
	
	/**
	 * Draws an instance to file as a PNG file
	 * @param gi The instance to be drawn
	 * @param file The name of the file it should be written to
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels Whether labels should be drawn
	 * @throws IOException can occur while attempting to write the image to a file
	 */
	public static void drawInstance(Instance<Vec2D> gi, String file, int w, int h, boolean labels) throws IOException
	{
		drawInstance(gi, new File(file), w, h, labels);
	}
	
	/**
	 * Renders a solution as an image file
	 * @param sol The solution to be rendered
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels Whether to draw labels or not
	 * @return The rendered image
	 */
	public static RenderedImage drawSolution(Solution<Vec2D> sol, int w, int h, boolean labels)
	{
		Instance<Vec2D> gi = sol.getInstance();
		Mapping m = defaultMapping(sol.getInstance(), w, h);
		BufferedImage bi = new BufferedImage(w,h, BufferedImage.TYPE_INT_RGB);
		Graphics2D gr = (Graphics2D) bi.getGraphics();
		gr.setBackground(Color.WHITE);
		gr.clearRect(0, 0, w, h);
		gr.setColor(Color.BLACK);
		draw(gi, m, gr, labels);
		gr.setColor(Color.GREEN);
        drawDrive(sol,m,gr);
        Stroke oldstr = gr.getStroke();
		Stroke dashed = new BasicStroke(1, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{9}, 0);
        gr.setStroke(dashed);
        gr.setColor(Color.CYAN);
		drawFly(sol,m,gr);
        gr.setStroke(oldstr);
        return bi;
	}
	
	/**
	 * Writes a solution to an image file as PNG
	 * @param sol The solution to be written
	 * @param f The image file to write to 
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels Whether to draw labels or not
	 * @throws IOException can occur while attempting to write the image to a file
	 */
	public static void drawSolution(Solution<Vec2D> sol, File f, int w, int h, boolean labels) throws IOException
	{
		RenderedImage i = drawSolution(sol,w,h,labels);
		ImageIO.write(i, "PNG", f);
	}
	
	/**
	 * Writes a solution to an image file as PNG
	 * @param sol The solution to be written
	 * @param file The name of the image file to write to 
	 * @param w The width of the output image
	 * @param h The height of the output image
	 * @param labels Whether to draw labels or not
	 * @throws IOException can occur while attempting to write the image to a file
	 */public static void drawSolution(Solution<Vec2D> sol, String file, int w, int h, boolean labels) throws IOException
	{
		drawSolution(sol, new File(file), w, h, labels);
	}
	
	
}
