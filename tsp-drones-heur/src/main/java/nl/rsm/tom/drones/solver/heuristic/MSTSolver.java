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
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.LowerBoundSolver;
import nl.rsm.tom.drones.solver.Solver;
import nl.rsm.tom.drones.util.UnionFind;

/**
 * The MTSSolver produces a solution which only
 * generates a TSP tour for the Truck, based on
 * the Minimum Spanning Tree heuristic.
 * 
 * The Minimum Spanning Tree is generated using a
 * UnionFind Data Structure. It is roughly linear
 * in the number of edges. By default, all edges
 * are generated, but in case of a Graph Instance
 * only the edges of the graph are used.
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the instance
 */

public class MSTSolver<E> implements Solver<E>, LowerBoundSolver<E>
{

	private boolean _nn;
	private double lastMST;
	
	private Instance<E> _lastInstance;
	private Solution<E> _lastSolution;
	
	private boolean tighterLB = false;
	
	public MSTSolver()
	{
		this(false);
	}
	
	public MSTSolver(boolean nearestNeighbour)
	{
		_nn = nearestNeighbour;
		tighterLB = false;
	}
	
	public MSTSolver(boolean nearestNeighbour, boolean betterLB) {
		_nn = nearestNeighbour;
		tighterLB = betterLB;
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		if (instance == _lastInstance)
		{
			return _lastSolution;
		}
		_lastInstance = null;
		_lastSolution = null;
		
		List<Pair> l = null;
		if (instance instanceof GraphInstance<?>)
		{
			try
			{
				GraphInstance<E> gi = (GraphInstance<E>) instance;
				l = generateList(gi);
			}
			catch (Exception e)
			{
				// Do nothing, revert to the default situation
			}
		}
		if (l == null)
		{
			l = generateList(instance);
		}
		Map<E,List<E>> mst = mst(instance, l);
		List<E> seq = mstToSequence(instance,mst);
		E cur = null;
		List<Operation<E>> ops = new ArrayList<>(seq.size()-1);
		for (E e : seq)
		{
			if (cur != null)
			{
				Operation<E> op = new Operation<>(cur,e);
				ops.add(op);
			}
			cur = e;
		}
		Solution<E> sol = new Solution<E>(instance,ops);
		_lastInstance = instance;
		_lastSolution = sol;
		return sol;
	}

	/**
	 * Returns the cost of the most recently computed MST
	 * @return The value of the most recently computer MST
	 */
	public double getLastMSTCost()
	{
		return lastMST;
	}
	
	private List<Pair> generateList(Instance<E> instance)
	{
		List<Pair> res = new ArrayList<>();
		int nc = instance.getNodeCount();
		for (int i=0; i < nc; i++)
		{
			for (int j=i+1; j < nc; j++)
			{
				E u = instance.getLocation(i);
				E v = instance.getLocation(j);
				Pair p = new Pair(u,v,instance.getDriveDistance().getContextFreeDistance(u, v));
				res.add(p);
			}
		}
		return res;
	}

	private List<Pair> generateList(GraphInstance<E> instance)
	{
		List<Pair> res = new ArrayList<>();
		for (GraphInstance<E>.Edge e : instance.getEdges())
		{
			Pair p = new Pair(e.left,e.right,e.drive);
			res.add(p);
		}
		return res;
	}

	
	private List<E> mstToSequence(Instance<E> i, Map<E,List<E>> mst)
	{
		Set<E> visited = new HashSet<E>();
		Set<E> processed = new HashSet<E>();
		ArrayList<List<E>> queue = new ArrayList<>();
		ArrayList<E> res = new ArrayList<E>();
		E depot = i.getDepot();
		E cur = depot;
		visited.add(cur);
		res.add(cur);
		for (E child : mst.get(cur))
		{
			visited.add(child);
			List<E> l = new ArrayList<>();
			l.add(child);
			queue.add(l);
		}
		processed.add(cur);
		while (!queue.isEmpty())
		{
			List<E> l = queue.get(queue.size()-1);
			if (l.isEmpty())
			{
				queue.remove(queue.size()-1);
				continue;
			}
			if (_nn)
			{
				Collections.sort(l, distanceOrder(cur, i.getDriveDistance()));
			}
			E node = l.remove(0);
			res.add(node);
			List<E> nl = new ArrayList<>();
			for (E child : mst.get(node))
			{
				if (!visited.contains(child))
				{
					visited.add(child);
					nl.add(child);
				}
			}
			queue.add(nl);
			processed.add(node);
		}
		res.add(depot);
		return res;
	}
	
	private Comparator<E> distanceOrder(E o, Distance<E> d)
	{
		return (E o1, E o2) -> (int)Math.signum(d.getContextFreeDistance(o, o1) - d.getContextFreeDistance(o, o2));
	}
	
	private Map<E,List<E>> mst(Iterable<E> locs, List<Pair> pairs)
	{
		Map<E,List<E>> res = new HashMap<>();
		UnionFind<E> uf = new UnionFind<>();
		for (E loc : locs)
		{
			uf.createSet(loc);
			res.put(loc, new ArrayList<>());
		}
		
		ArrayList<Pair> list = new ArrayList<Pair>(pairs);
		Collections.sort(list);
		lastMST = 0;
		for (Pair p : list)
		{
			if (!uf.sameSet(p.left, p.right))
			{
				res.get(p.left).add(p.right);
				res.get(p.right).add(p.left);
				uf.union(p.left, p.right);
				lastMST += p.cost;
			}
		}
		
		return res;
	}
	
	
	private class Pair implements Comparable<Pair>
	{
		public final E left;
		public final E right;
		public final double cost;
		
		public Pair(E l, E r, double c)
		{
			left = l;
			right = r;
			cost = c;
		}
		
		@Override
		public int compareTo(Pair arg0)
		{
			return (int)Math.signum(cost-arg0.cost);
		}
	}


	@Override
	public double lowerbound(Instance<E> instance)
	{
		if (instance instanceof GeometricInstance)
		{
			if (!tighterLB)
			{
				GeometricInstance gi = (GeometricInstance) instance;
				solve(instance);
				double sum = gi.getDriveSpeed() + gi.getFlySpeed();
				double frac = Math.min(gi.getDriveSpeed(), gi.getFlySpeed()) / sum;
				return frac * lastMST;
			}
			else
			{
				GeometricInstance gi = (GeometricInstance) instance;
				solve(instance);
				double smaller = Math.min(gi.getDriveSpeed(), gi.getFlySpeed());
				double bigger = Math.max(gi.getDriveSpeed(),  gi.getFlySpeed());
				double alpha = bigger/smaller;
				double frac = (2 / (2 + alpha));
				return frac * lastMST;
			}
		}
		return 0;
	}	
}
