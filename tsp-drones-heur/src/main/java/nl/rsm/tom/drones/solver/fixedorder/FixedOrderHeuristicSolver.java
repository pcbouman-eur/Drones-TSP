package nl.rsm.tom.drones.solver.fixedorder;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.algorithms.FixedOrderHeuristic;

/**
 * Solver class which runs the FixedOrderHeuristic on an instance
 * @author Paul Bouman
 *
 * @param <E>
 */
public class FixedOrderHeuristicSolver<E> implements FixedOrderSolver<E>
{

	private boolean nonNegative;
	private boolean twoPass;
	
	public FixedOrderHeuristicSolver()
	{
		this(false,true);
	}
	
	public FixedOrderHeuristicSolver(boolean nonNegative, boolean twoPass)
	{
		this.nonNegative = nonNegative;
		this.twoPass = twoPass;
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance, Solution<E> order)
	{
		FixedOrderHeuristic<E> foh = new FixedOrderHeuristic<>(instance, order, nonNegative, twoPass);
		return foh.getSolution();
	}

	
}
