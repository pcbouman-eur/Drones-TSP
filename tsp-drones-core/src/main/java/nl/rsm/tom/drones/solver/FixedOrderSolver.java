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

package nl.rsm.tom.drones.solver;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Solver interface that assumes the locations need to be visited
 * in a particular order. Thus these type of solvers need an initial
 * tour, and make decisions which locations are covered by the drone
 * and which locations are covered by the truck.
 * @author Paul Bouman
 *
 * @param <E> the type of location a solver can deal with
 */

public interface FixedOrderSolver<E>
{
	/**
	 * The main method used for solving an instance
	 * @param instance The instance to be solved
	 * @param order The fixed order in which the truck can visit nodes.
	 * @return The solution found, or null if no solution was found.
	 */
	default public Solution<E> solve(Instance<E> instance, List<E> order)
	{
		return solve(instance, new Solution<E>(order, instance));
	}
	
	/**
	 * The main method used for solving an instance
	 * @param instance The instance to be solved
	 * @param order The fixed order in which the truck can visit nodes as a solution.
	 * @return The solution found, or null if no solution was found.
	 */
	Solution<E> solve(Instance<E> instance, Solution<E> order);
	
	/**
	 * When the solver is running, statistics can be collected.
	 * These should be stored in a map for reporting purposes.
	 * @return The map containing solver statistics
	 */
	default public Map<String,String> getLastSolverStatistics()
	{
		return Collections.emptyMap();
	}
	
	/**
	 * Converts this FixedOrderSolver into a regular Solver using another solver
	 * to bootstrap the solving process
	 * @param bootstrapSolver the bootstrapping solver
	 * @return a regular Solver that first applies the bootstrapSolver, and then this FixedOrderSolver
	 */
	default public Solver<E> bootstrap(Solver<E> bootstrapSolver)
	{
		return i -> solve(i, bootstrapSolver.solve(i));
	}
}
