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
package nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.algorithms.localsearch.Action;
import nl.rsm.tom.drones.solver.algorithms.localsearch.AllActionProvider;

/**
 * Provides 2-opt actions in which the order of a subsequence
 * of the lised is reversed.
 * @author Paul Bouman
 *
 * @param <E> the types of the locations in the instance
 */

public class TwoOptProvider<E> implements AllActionProvider<E>
{

	@Override
	public List<Action<E>> getActions(ArrayList<E> lst, Instance<E> e)
	{
		int n = lst.size();
		ArrayList<Action<E>> result = new ArrayList<>((n*(n+1))/2);
		for (int i=0; i < n; i++)
		{
			for (int j=i+1; j < n; j++)
			{
				result.add(new TwoOptAction(i,j));
			}
		}
		return result;
	}
	
	/**
	 * A TwoOptAction reverses the order of a subsequence in the list.
	 * @author Paul Bouman
	 *
	 */
	public class TwoOptAction implements Action<E>
	{
		public final int from;
		public final int to;
		
		/**
		 * Constructor which specifies the range of indices that should
		 * be reversed in order
		 * @param f the start of the range
		 * @param t the end of the range
		 */
		public TwoOptAction(int f, int t)
		{
			from = f;
			to = t;
		}
		
		@Override
		public void doAction(ArrayList<E> list)
		{
			for (int i=0; from+i < to-i; i++)
			{
				E temp = list.get(to-i);
				list.set(to-i, list.get(from+i));
				list.set(from+i, temp);
			}
		}
		
		@Override
		public void undoAction(ArrayList<E> list)
		{
			doAction(list);
		}
		
	}

}
