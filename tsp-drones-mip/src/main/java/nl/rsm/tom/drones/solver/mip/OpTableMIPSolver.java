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
package nl.rsm.tom.drones.solver.mip;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import ilog.concert.IloException;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.optable.IntOpTable;
import nl.rsm.tom.drones.optable.constraints.ConstraintTools;
import nl.rsm.tom.drones.optable.constraints.OpConstraint;
import nl.rsm.tom.drones.solver.Solver;

/**
 * Solver that construct a table of operations and then uses a MIP model to 
 * find the optimal solution to the TSP-D.
 * This solve can not solve instances larger than 25 because integers are used
 * to encode the possible operations. Furthermore, this will probably take too
 * much memory and time anyway.
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the instance
 */
public class OpTableMIPSolver<E> implements Solver<E>
{
	private int maxCardinality;
	private double maxRange;
	private Map<String,Object> stats;
	
	/**
	 * Constructor without any constraints
	 */
	public OpTableMIPSolver()
	{
		this(-1,2.0);
	}
	
	/**
	 * Constructor for a solver that includes a maximum number of truck-only locations
	 * in a single operation, and a maximum relative drone range (i.e. a factor of the
	 * maximum drone-distance in the instance). Any maximum relative drone range of 2
	 * or greater poses no restriction on the possible solutions.
	 * @param maxCardinality the maximum allowed number of truck-only locations
	 * @param maxFlyRange the relative drone range
	 */
	public OpTableMIPSolver(int maxCardinality, double maxFlyRange)
	{
		this.maxCardinality = maxCardinality;
		this.maxRange = maxFlyRange;
		stats = new TreeMap<>();
		stats.put("OpTable Size", "");
		stats.put("OpTable DP Time (ms)", "");
		stats.put("Solution MIP Time (ms)", "");
		stats.put("Maximum Drone Range", maxRange);
		stats.put("Maximum Truck Nodes", maxCardinality >= 0 ? maxCardinality : "Unrestricted");
	}
	
	/**
	 * Constructor for a solver that limits the maximum number of truck-only locations.
	 * @param maxInternal the maximum allowed number of truck only locations.
	 */
	public OpTableMIPSolver(int maxInternal)
	{
		this(maxInternal,Double.POSITIVE_INFINITY);
	}
	
	/**
	 * Constructor for a solver that limits the relative maximum range of the drone,
	 * indicates as a factor of the maximum single-leg instance in the graph. Thus,
	 * a relative maximum range of 1 means that the drone can at most fly the maximum
	 * single leg dinstance in the graph. Since the drone always has to fly two legs,
	 * this will restrict some operations. A relative maximum range of 2 allows all
	 * possible operations.
	 * @param maxFlyRange the maximum relative range of the drone
	 */
	public OpTableMIPSolver(double maxFlyRange)
	{
		this(-1,maxFlyRange);
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		if (instance.getNodeCount() > 25)
		{
			throw new IllegalArgumentException("Cannot enumerate all operation for instances of size larger than 25");
		}
		List<OpConstraint<Object, Object>> cons = ConstraintTools.buildConstraints(instance, maxRange, maxCardinality);
		long time = System.currentTimeMillis();
		IntOpTable<E> iot = new IntOpTable<E>(instance,cons);
		time = System.currentTimeMillis() - time;
		stats.put("OpTable Size",iot.getSize());
		stats.put("OpTable DP Time (ms)", time);
		time = System.currentTimeMillis();
		try
		{
			OpTableMIP<E,Integer> otm = new OpTableMIP<>(iot);
			otm.solve();
			Solution<E> sol = otm.getSolution();
			otm.clear();
			stats.put("Solution MIP Time (ms)", (System.currentTimeMillis() - time));
			return sol.simplify();
		}
		catch (IloException ioe)
		{
			throw new RuntimeException("A problem occurred while running CPLEX",ioe);
		}
	}

	@Override
	public Map<String,Object> getLastSolverStatistics()
	{
		return new TreeMap<>(stats);
	}
	
}
