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

package nl.rsm.tom.drones.main;

import java.io.IOException;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.io.InstanceIO;
import nl.rsm.tom.drones.data.io.SolutionIO;
import nl.rsm.tom.drones.solver.heuristic.RandomSolver;
import nl.rsm.tom.drones.util.DrawingTools;

/**
 * Simple main class example
 * @author Paul Bouman
 *
 */
public class Main
{
	/**
	 * Main method that accepts three or four arguments and uses
	 * the RandomSolver to solve an instance from a file.
	 * @param args the commandline arguments
	 */
	public static void main(String [] args)
	{
		if (args.length < 3 || args.length > 4) 
		{
			System.out.println("This program requires three or four arguments: seed input-instance output-solution [output-drawing.png]");
			System.exit(0);
		}
		
		long seed = Long.parseLong(args[0]);
		String input = args[1];
		String output = args[2];
		String image = args.length == 4 ? args[3] : null;
		
		try
		{
			GeometricInstance instance = InstanceIO.readGeometricInstance(input);
			RandomSolver<Vec2D> solver = new RandomSolver<>(seed);
			Solution<Vec2D> solution = solver.solve(instance);
			SolutionIO.writeSolution(solution, output);
			if (image != null)
			{
				DrawingTools.drawSolution(solution, image, 800, 800, true);
			}
		}
		catch (IOException e)
		{
			System.out.println("An IOException occurred");
			e.printStackTrace();
		}
	}
}
