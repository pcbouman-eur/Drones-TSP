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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * This class provides utility methods that can be used to compute a proper
 * Euler walk on a list of operations for a given instance.
 * @author Paul Bouman
 *
 */
public class EulerTourTools
{
	
	/**
	 * Compute an actual solution to the TSP-D for a given list of operations and
	 * an instance of the TSP-D, by computing a Eulerian walk on the graph
	 * spawned by the provided list of operations.
	 * @param ops a list of operations that form a Eulerian graph when the end-points of
	 *            every operation are considered vertices and the operations themselves
	 *            are considered as arcs between their endpoints.
	 * @param instance the instance for which a solution must be computed
	 * @return a solution based on a Eulerian walk over all operations.
	 */
	public static <E> Solution<E> buildSolution(List<Operation<E>> ops, Instance<E> instance)
	{
		Map<E, List<Operation<E>>> map = buildMap(ops, Operation::getStart);
		List<Operation<E>> res = buildTours(map, instance.getDepot(), Operation::getEnd);
		return new Solution<>(instance,res);
	}
	
	/**
	 * Construct a map with all outgoing arcs provided by some function. 
	 * @param arcs A list of arcs that must be included in the graph
	 * @param from a method that produce the origin location of a given arc
	 * @return a map from locations to a list of outgoing arcs
	 */
	public static <V,A> Map<V,List<A>> buildMap( List<? extends A> arcs, Function<? super A, ? extends V> from)
	{
		Map<V,List<A>> map = new HashMap<>();
		for (A a : arcs)
		{
			V key = from.apply(a);
			if (!map.containsKey(key))
			{
				List<A> l = new ArrayList<>();
				l.add(a);
				map.put(key, l);
			}
			else
			{
				map.get(key).add(a);
			}
		}
		return map;
	}
	
	/**
	 * The algorithm that constructs an Euler tour from a provided starting point. In principle, it
	 * generates a first tour. If some arcs are not covered by that tour, the algorithm computes a
	 * sub-tour that starts at one of the origin locations of an arc that was not yet covered, but
	 * which is on the current tour. The original tour is than expanded by this subtour. This process
	 * is repeated as long as not all arcs are covered. In the end, this will yield a Eulerian walk
	 * where all arcs are covered.
	 * @param map a map that provides a list of out-arcs for every location.
	 * @param start the starting location where the Eulerian walk should originate.
	 * @param to a function that can provide the destination location of a given arc.
	 * @return a list of all the arcs in the order of a valid Eulerian walk
	 */
	private static <V,A> List<A> buildTours( Map<V,List<A>> map, V start, Function<? super A, ? extends V> to)
	{
		Map<V,List<A>> tourMap = new HashMap<>();
		
		List<A> tour = buildTour(map, start, to);
		tourMap.put(start, tour);
		
		while (map.size() > 0)
		{
			V val = null;
			
			innerLoop:
			for (List<A> candidate : tourMap.values())
			{
				for (A a : candidate)
				{
					V end = to.apply(a);
					if (map.containsKey(end))
					{
						val = end;
						break innerLoop;
					}
				}
			}
			
			tour = buildTour(map, val, to);
			tourMap.put(val, tour);
		}
		
		List<A> res = new ArrayList<>();
	
		List<A> subTour = tourMap.remove(start);
		res.addAll(subTour);
		
		while (!tourMap.isEmpty())
		{
			V key = null;
			int index = 0;
			innerLoop:
			for (A a : res)
			{
				V v = to.apply(a);
				if (tourMap.containsKey(v))
				{
					key = v;
					break innerLoop;
				}
				index++;
			}
			if (key == null)
			{
				throw new IllegalStateException("This should never happen...");
			}
			subTour = tourMap.remove(key);
			res.addAll(index+1, subTour);
		}
		
		return res;
	}  
	
	/**
	 * This method computes a tour given a Map that provides a list of out-arcs for every location,
	 * a starting location where the tour should start, and a method that given an arc, provides
	 * its endpoint. Note that this tour does not necessary visit all arcs in the maps: it just
	 * takes a random walk from the start location and stops the first time it returns to that location.
	 * @param map a Map that provides a list of out-arcs for every possible location.
	 * @param start a starting point for the tour.
	 * @param to a function that can provided the destination of an arc.
	 * @return a tour starting and ending at the starting location, not necessarily covering all arcs.
	 */
	public static <V,A> List<A> buildTour( Map<V,List<A>> map, V start, Function<? super A, ? extends V> to)
	{
		List<A> res = new ArrayList<>();
		
		V cur = start;
		while (!cur.equals(start) || (map.get(start) != null && !map.get(start).isEmpty()))
		{
			List<A> l = map.get(cur);
			
			A first = l.remove(0);
			
			if (l.isEmpty())
			{
				map.remove(cur);
			}
			
			res.add(first);
			cur = to.apply(first);
			
			if (!cur.equals(start) && (!map.containsKey(cur) || map.get(cur).isEmpty()))
			{
				throw new IllegalArgumentException("The structure of the graph is not Eulerian!");
			}

		}
		return res;
	}
						
}
