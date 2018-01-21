package nl.rsm.tom.drones.solver.fixedorder;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.algorithms.FixedOrderDP;

/**
 * A class that runs the Dynamic Programming algorithm for cases where
 * the order in which locations have to be visited is fixed.
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the instance
 */

public class FixedOrderDPSolver<E> implements FixedOrderSolver<E>
{

	@Override
	public Solution<E> solve(Instance<E> instance, Solution<E> order)
	{
		FixedOrderDP<E> fdp = new FixedOrderDP<E>(instance, order.getOrder());
		return fdp.getSolution();
	}

}
