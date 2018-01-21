package nl.rsm.tom.drones.solver.fixedorder;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.algorithms.MurrayChuHeuristic;

/**
 * Murray and Chu based solver that takes an initial solution and then
 * applies the Murray and Chu heuristic on it.
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */
public class MurrayChuFixedOrderSolver<E> implements FixedOrderSolver<E>
{

	@Override
	public Solution<E> solve(Instance<E> instance, Solution<E> order)
	{
		MurrayChuHeuristic<E> mch = new MurrayChuHeuristic<E>(instance,order);
		mch.runHeuristic();
		return mch.getSolution();
	}

}
