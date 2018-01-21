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
