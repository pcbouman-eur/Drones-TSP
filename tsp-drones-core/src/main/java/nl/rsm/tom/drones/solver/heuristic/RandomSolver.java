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
import java.util.Random;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.Solver;

/**
 * The RandomSolver generates a permutation of the locations as a solution
 * and then assigns the locations with an odd index in the permutation to
 * be fly nodes, and the other locations drive nodes.
 * 
 * @author Paul Bouman
 * 
 * @param <E> The type of location
 */

public class RandomSolver<E> implements Solver<E>
{
	private Random _ran;
	private int _repeat;
	
	/**
	 * Default constructor based on a seed with as single try
	 * @param seed The seed used for the permutations
	 */
	public RandomSolver(long seed)
	{
		this(seed,1);
	}
	
	/**
	 * Default constructor based on a seed
	 * @param seed The seed used for the permutations
	 * @param repeat The number of trials before the best one is returned
	 */
	public RandomSolver(long seed, int repeat)
	{
		_ran = new Random(seed);
		_repeat = repeat;
	}
	
	/**
	 * Default constructor based on a RNG with a single trial
	 * @param ran The RNG to use
	 */
	public RandomSolver(Random ran)
	{
		this(ran,1);
	}
	
	/**
	 * Default constructor based on a RNG
	 * @param r The RNG to be used for the permutations
	 * @param repeat The number of trials before the best one is returned
	 */
	public RandomSolver(Random r, int repeat)
	{
		_ran = r;
		_repeat = repeat;
	}

	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		Solution<E> best = null;
		for (int t=0; t < _repeat; t++)
		{
			Solution<E> sol = randomSolution(instance);
			if (best == null || best.getTotalCost() > sol.getTotalCost())
			{
				best = sol;
			}
		}
		return best;
	}
	
	private Solution<E> randomSolution(Instance<E> instance)
	{
		ArrayList<Operation<E>> ops = new ArrayList<>();
		ArrayList<E> locs = new ArrayList<E>(instance.getLocations());
		Collections.shuffle(locs,_ran);
		E d = instance.getDepot();
		locs.add(d);
		E cur = d;
		for (int t=0; t < locs.size(); t += 2)
		{
			Operation<E> op;
			E to;
			if (t < locs.size()-1)
			{
				E fly = locs.get(t);
				to = locs.get(t+1);
				op = new Operation<E>(cur,to,fly);
				
			}
			else
			{
				to = locs.get(t);
				op = new Operation<E>(cur,to);
			}
			ops.add(op);
			cur = to;
		}
		return new Solution<E>(instance,ops);
	}
}
