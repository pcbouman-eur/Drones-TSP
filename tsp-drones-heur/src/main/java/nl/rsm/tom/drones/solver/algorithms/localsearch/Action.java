package nl.rsm.tom.drones.solver.algorithms.localsearch;

import java.util.ArrayList;

/**
 * An action can modify a list, and also can do the inverse action
 * on the the list. If we have a <pre>list</pre> that contains a certain
 * order of locations, and an <pre>action</pre> the following code example
 * should have the same order after executing as it has before execution 
 * 
 * <pre>{@code
 * action.doAction(list);
 * action.undoAction(list);
 * }</pre>
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of elements in the list of which the position can be changed
 */
public interface Action<E>
{
	public void doAction(ArrayList<E> list);
	public void undoAction(ArrayList<E> list);
}
