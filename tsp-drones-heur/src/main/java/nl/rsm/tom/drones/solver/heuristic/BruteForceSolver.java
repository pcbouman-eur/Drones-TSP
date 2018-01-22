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
package nl.rsm.tom.drones.solver.heuristic;

import java.util.ArrayList;

import org.paukov.combinatorics.CombinatoricsVector;
import org.paukov.combinatorics.ICombinatoricsVector;
import org.paukov.combinatorics.permutations.PermutationGenerator;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.Solver;
import nl.rsm.tom.drones.solver.algorithms.FixedOrderDP;

/**
 * Solver which runs the FixedOrderDPSolver on all permutations. Note that this approach
 * may not result in the optimal solution, as optimal solution may require the truck to
 * visit the same location multiple times, and the FixedOrderDPSolver is not able to
 * detect such situation.
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instances to solve
 */
public class BruteForceSolver<E> implements Solver<E>
{
	
	private boolean _report;

	/**
	 * Default constructor
	 */
	public BruteForceSolver()
	{
		this(false);
	}
	
	/**
	 * Constructor which allows to specify whether reports should be written to standard out.
	 * @param report Whether reports should be written to standard out
	 */
	public BruteForceSolver(boolean report)
	{
		_report = report;
	}
	
	/**
	 * Factorial to compute progress
	 * @param n The factorial input
	 * @return The factorial
	 */
	public static long fac(int n)
	{
		long res = 1;
		for (int t=n; t > 1; t--)
		{
			res *= t;
		}
		return res;
	}

	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		Solution<E> best = null;
		ArrayList<E> list = new ArrayList<E>(instance.getNodeCount() - 1);
		list.addAll(instance.getLocations());
		CombinatoricsVector<E> v  = new CombinatoricsVector<>(list);
		long count = 0;
		long total = fac(instance.getNodeCount()-1);
		for (ICombinatoricsVector<E> p : new PermutationGenerator<E>(v))
		{
			count++;
			if (_report && count % 1000 == 0)
			{
				System.out.println("Running permutation "+count+" of "+total+" ...");
			}
			ArrayList<E> order = new ArrayList<>(list.size()+2);
			order.add(instance.getDepot());
			for (E e : p)
			{
				order.add(e);
			}
			order.add(instance.getDepot());
			FixedOrderDP<E> fodp = new FixedOrderDP<E>(instance, order);
			Solution<E> sol = fodp.getSolution();
			if (best == null || sol.getTotalCost() < best.getTotalCost())
			{
				best = sol;
			}
		}
		return best;
	}
}
