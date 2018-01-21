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
