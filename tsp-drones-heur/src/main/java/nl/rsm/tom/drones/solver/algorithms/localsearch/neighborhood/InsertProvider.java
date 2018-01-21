package nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.algorithms.localsearch.Action;
import nl.rsm.tom.drones.solver.algorithms.localsearch.AllActionProvider;

/**
 * Provides insertion actions, i.e. take out an element of the list
 * and insert it at another position in the list
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */

public class InsertProvider<E> implements AllActionProvider<E>
{
	@Override
	public List<Action<E>> getActions(ArrayList<E> lst, Instance<E> e)
	{
		int n = lst.size();
		ArrayList<Action<E>> result = new ArrayList<>();
		for (int i=0; i < n; i++)
		{
			for (int j=0; j < n; j++)
			{
				if (i!=j)
				{
					result.add(new InsertAction(i,j));
				}
			}
		}
		return result;
	}
	
	/**
	 * Models an insert action, where an element is taken from the
	 * list and inserted at another position in the list.
	 * @author Paul Bouman
	 *
	 */
	public class InsertAction implements Action<E>
	{
		private final int from;
		private final int to;

		/**
		 * Creates an insert action.
		 * @param f the position to remove an element
		 * @param t the position to insert that element
		 */
		public InsertAction(int f, int t)
		{
			from = f;
			to = t;
		}
		
		@Override
		public void doAction(ArrayList<E> list)
		{
			E element = list.remove(from);
			if (to > from)
			{
				list.add(to-1, element);
			}
			else
			{
				list.add(to, element);
			}
		}

		@Override
		public void undoAction(ArrayList<E> list)
		{
			E element;
			if (to > from)
			{
				element = list.remove(to-1);
			}
			else
			{
				element = list.remove(to);
			}
			list.add(from, element);
		}		
	}
}
