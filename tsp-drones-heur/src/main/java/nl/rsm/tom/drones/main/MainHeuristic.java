package nl.rsm.tom.drones.main;

/* Copyright 2018 Paul Bouman
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

import java.io.IOException;
import java.util.Random;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.io.InstanceIO;
import nl.rsm.tom.drones.data.io.SolutionIO;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.Solver;
import nl.rsm.tom.drones.solver.fixedorder.FixedOrderDPSolver;
import nl.rsm.tom.drones.solver.fixedorder.FixedOrderHeuristicSolver;
import nl.rsm.tom.drones.solver.fixedorder.IterativeImprovementSolver;
import nl.rsm.tom.drones.solver.heuristic.ConcordeTSPSolver;
import nl.rsm.tom.drones.solver.heuristic.MSTSolver;
import nl.rsm.tom.drones.solver.heuristic.RandomSolver;
import nl.rsm.tom.drones.util.DrawingTools;

/**
 * Simple main class example for heuristics
 * @author Paul Bouman
 *
 */
public class MainHeuristic
{
	
	private static final String MST = "mst";
	private static final String CONCORDE = "concorde";
	private static final String RANDOM = "random";
	
	private static final String HEUR = "heur";
	private static final String DP = "dp";
	
	/**
	 * Main method that accepts three or four arguments and uses
	 * the RandomSolver to solve an instance from a file.
	 * @param args the commandline arguments
	 */
	public static void main(String [] args)
	{
		if (args.length < 5 || args.length > 6) 
		{
			System.out.println("This program requires five or six arguments: input output bootstrap fixedOrder neighborhood [output-drawing.png]");
			System.out.println("Example arguments are: myinstance.txt mysolution.sol mst dp 7");
			System.exit(0);
		}
		
		String input = args[0];
		String output = args[1];
		String bootstrap = args[2];
		
		if (!bootstrap.equals(MST) && !bootstrap.equals(CONCORDE) && !bootstrap.equals(RANDOM)) {
			System.out.println("The bootstrap argument must be either '"+MST+"', '"+CONCORDE+"', or '"+RANDOM+"'");
		}
		
		String fixedOrder = args[3];
		
		if (!fixedOrder.equals(HEUR) && !fixedOrder.equals(DP)) {
			System.out.println("The fixedOrder argument must be either '"+HEUR+"' or '"+DP+"'");
			System.exit(0);
		}
		
		int neighborhood = Integer.parseInt(args[4]);
		if (neighborhood < 0 || neighborhood > 7) {
			System.out.println("The neighborhood must be a whole number in the range 1-7.");
			System.out.println("It is the sum of the neighborhoods you want to include");
			System.out.println(" 1 : swap neighborhood");
			System.out.println(" 2 : 2-opt neighborhood");
			System.out.println(" 4 : insertion neighborhood");
			System.exit(0);
		}
		boolean swap = (neighborhood & 1) != 0;
		boolean twoopt = (neighborhood & 2) != 0;
		boolean ins = (neighborhood & 4) != 0;
		
		
		String image = args.length == 6 ? args[5] : null;
		
		try
		{
			GeometricInstance instance = InstanceIO.readGeometricInstance(input);			
			Solver<Vec2D> bootstrapSolver;
			if (bootstrap.equals(MST)) {
				bootstrapSolver = new MSTSolver<>();
			}
			else if (bootstrap.equals(CONCORDE)) {
				bootstrapSolver = new ConcordeTSPSolver<>("concorde", "concorde_temp_file");
			}
			else if (bootstrap.equals(RANDOM)) {
				Random rnd = new Random();
				bootstrapSolver = new RandomSolver<>(rnd);
			}
			else {
				throw new IllegalStateException("The 'bootstrap' argument was not recognized");
			}
			
			FixedOrderSolver<Vec2D> fos;
			if (fixedOrder.equals(HEUR)) {
				fos = new FixedOrderHeuristicSolver<>();
			}
			else if (fixedOrder.equals(DP)) {
				fos = new FixedOrderDPSolver<>();
			}
			else {
				throw new IllegalStateException("The 'fixedOrder' argument was not recognized");
			}
			IterativeImprovementSolver<Vec2D> iis = new IterativeImprovementSolver<>(fos,swap,twoopt,ins);
			Solver<Vec2D> solver = iis.bootstrap(bootstrapSolver);
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
