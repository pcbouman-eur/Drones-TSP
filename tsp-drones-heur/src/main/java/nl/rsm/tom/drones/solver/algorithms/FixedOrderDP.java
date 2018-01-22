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
package nl.rsm.tom.drones.solver.algorithms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.util.Array3;

/**
 * Class that applies Dynamic Programming to compute the best way to divide
 * locations between the truck and drone, given that the order in which they
 * have to be visited is fixed.
 * 
 * This dynamic programming algorithm is introduced and explained in the following paper:
 * 
 * N.A.H. Agatz, P.C. Bouman & M.E. Schmidt. Optimization Approaches for the Traveling Salesman Problem with Drone. Transportation Science.
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the instance
 */

public class FixedOrderDP<E>
{
	private Instance<E> _instance;
	private List<E> _list;
	private int _maxOpLength;
	private double [][] _dist;
	private Array3 _ops;
	
	/**
	 * Initializes the solver for an instance and an order on the nodes
	 * @param i The instance this solver will work for
	 * @param order The order of the nodes
	 */
	public FixedOrderDP(Instance<E> i, List<E> order)
	{
		this(i, order, order.size() + 1);
	}
	
	/**
	 * Initializes the solver based on a node order and a maximum number
	 * of drive nodes per operation.
	 * TODO: currently this is not supported; should build a clever version of Array3 to do this
	 * TODO: When implemented correctly, this constructor should be made public
	 * @param i The instance for which we will fix an order
	 * @param order The order that will be considered
	 * @param maxOpLength The maximum number of drive nodes per operation
	 */
	private FixedOrderDP(Instance<E> i, List<E> order, int maxOpLength)
	{
		_instance = i;
		_list = new ArrayList<>(order);
		_maxOpLength = maxOpLength;
	}
	
	/**
	 * Initializes this solver based on a fixed order read from the solution
	 * @param sol The solution that contains the order to fix
	 */
	public FixedOrderDP(Solution<E> sol)
	{
		this(sol, sol.getInstance().getNodeCount() + 1);
	}
	
	/**
	 * Constructor for a non-optimal Dynamic Programming solver.
	 * TODO: currently this is not supported; should build a clever version of Array3 to do this
	 * TODO: When implemented correctly, this constructor should be made public
	 * @param sol The initial solution that fixes the order of the nodes
	 * @param maxOpLength The maximum number of drive nodes in an operation
	 */
	private FixedOrderDP(Solution<E> sol, int maxOpLength)
	{
		_maxOpLength = maxOpLength;
		_instance = sol.getInstance();
		List<E> l = new ArrayList<>();
		for (Operation<E> op : sol)
		{
			if (!op.getInternalNodes(true).isEmpty())
			{
				throw new IllegalArgumentException("The solution contains non-atomic operations!");
			}
			l.add(op.getStart());
		}
		l.add(_instance.getDepot());
		_list = l;
	}
	
	/**
	 * Builds a distance table that enforces the fact that the order of nodes must be maintained
	 */
	private void buildDist()
	{
		int n = _list.size();
		Distance<E> d = _instance.getDriveDistance();
		_dist = new double[n][n];
		for (int i=0; i < n; i++)
		{
			for (int j=i+1; j < n; j++)
			{
				E f = _list.get(j-1);
				E t = _list.get(j);
				_dist[i][j] = _dist[i][j-1] + d.getContextFreeDistance(f, t, _dist[i][j-1]);
			}
		}
	}
	
	/**
	 * Builds a table of Operations and their corresponding cost
	 */
	private void buildOps()
	{
		int n = _list.size();
		_ops = new Array3(n,_maxOpLength,_maxOpLength);
		Distance<E> dd = _instance.getDriveDistance();
		Distance<E> df = _instance.getFlyDistance();
		for (int i=0; i < n; i++)
		{
			for (int j=i+1; j < n; j++)
			{
				_ops.set(i, j, i, _dist[i][j]);
				for (int k=i+1; k < j; k++)
				{
					E from = _list.get(i);
					E to = _list.get(j);
					E fly = _list.get(k);
					E flyPrev = _list.get(k-1);
					E flyNext = _list.get(k+1);
					//Note: this can be made context dependent with a case analysis
					double drive = _dist[i][j]
							     - dd.getContextFreeDistance(flyPrev, fly)
							     - dd.getContextFreeDistance(fly, flyNext)
							     + dd.getContextFreeDistance(flyPrev, flyNext);
					double flyCost = df.getFlyDistance(from, to, fly);
					//double flyCost = df.getDistance(from, fly)
					//		       + df.getDistance(fly, to);
					double cost = Math.max(drive,flyCost);
					_ops.set(i, j, k, cost);
				}
			}
		}		
	}
	
	/**
	 * Builds the Dynamic Programming table and constructs the solution
	 * @return The best solution found based on the DP-table
	 */
	private List<Operation<E>> runDP()
	{
		int n = _list.size();
		double [] val = new double[n];
		int [] is = new int[n];
		int [] ks = new int[n];

		// Phase 1: Build the Dynamic Programming Table
		for (int j=1; j < n; j++)
		{
			double best = Double.POSITIVE_INFINITY;
			int bestI = -1;
			int bestK = -1;
			for (int i=0; i < j; i++)
			{
				for (int k=i; k < j; k++)
				{
					double cost = val[i] + _ops.get(i, j, k);
					if (cost < best)
					{
						best = cost;
						bestI = i;
						bestK = k;
					}
				}
			}
			is[j] = bestI;
			ks[j] = bestK;
			val[j] = best;
		}

		// Phase 2: Walk back from the final node to construct the solution
		ArrayList<Operation<E>> result = new ArrayList<>();
		int cur = n-1;
		while (cur != 0)
		{
			E fly = is[cur] == ks[cur] ? null : _list.get(ks[cur]);
			List<E> locs = new ArrayList<>();
			for (int i=is[cur]; i <= cur; i++)
			{
				E loc = _list.get(i);
				if (loc != fly)
				{
					locs.add(loc);
				}
			}
			Operation<E> op;
			if (fly != null)
			{
				op = new Operation<>(locs,fly);
			}
			else
			{
				op = new Operation<>(locs);
			}
			result.add(op);
			cur = is[cur];
		}
		Collections.reverse(result);
		return result;
	}

	/**
	 * Performs the Dynamic Programming computations to obtain a solution
	 * @return
	 */
	public Solution<E> getSolution()
	{
		buildDist();
		buildOps();
		List<Operation<E>> ops = runDP();
		Solution<E> sol = new Solution<>(_instance, ops);
		return sol;
	}
	
}
