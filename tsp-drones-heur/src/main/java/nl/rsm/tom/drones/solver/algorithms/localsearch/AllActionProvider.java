package nl.rsm.tom.drones.solver.algorithms.localsearch;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Interface that describes that all action can be obtains. This is used
 * by search methods that intend to evaluate a full neighborhood.
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in an instance
 */
public interface AllActionProvider<E>
{
	/**
	 * Can be called to generate all actions that can be applied given a
	 * current sequence of locations. 
	 * @param lst the current sequence of locations
	 * @param e the instance from which these locations were obtained
	 * @return a list of actions than can be applied on the current state of lst
	 */
	public List<Action<E>> getActions(ArrayList<E> lst, Instance<E> e);
}
