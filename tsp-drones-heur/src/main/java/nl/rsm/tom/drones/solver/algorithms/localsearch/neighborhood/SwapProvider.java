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
 * Provides actions where two items in the list are swapped
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instances
 */

public class SwapProvider<E> implements AllActionProvider<E>
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
				result.add(new SwapAction(i,j));
			}
		}
		return result;
	}

	/**
	 * A swap action swaps two items in the list
	 * @author Paul Bouman
	 *
	 */
	public class SwapAction implements Action<E>
	{
		private final int from;
		private final int to;
		
		/**
		 * Constructor that specifies which items are swaps
		 * @param f the first index to swap
		 * @param t the second index to swap
		 */
		public SwapAction(int f, int t)
		{
			from = f;
			to = t;
		}
		
		@Override
		public void doAction(ArrayList<E> list)
		{
			E temp = list.get(from);
			list.set(from, list.get(to));
			list.set(to, temp);
		}

		@Override
		public void undoAction(ArrayList<E> list)
		{
			doAction(list);
		}
		
	}

}
