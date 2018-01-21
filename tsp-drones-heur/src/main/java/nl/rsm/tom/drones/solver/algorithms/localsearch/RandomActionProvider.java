package nl.rsm.tom.drones.solver.algorithms.localsearch;

import java.util.ArrayList;
import java.util.Random;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * 
 * @author Paul Bouman
 *
 * @param <E>
 */

public interface RandomActionProvider<E>
{
	public Action<E> getAction(ArrayList<E> lst, Instance<E> e, Random ran, Solution<E> current);
}
