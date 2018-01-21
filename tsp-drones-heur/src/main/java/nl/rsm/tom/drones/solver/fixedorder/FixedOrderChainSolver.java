package nl.rsm.tom.drones.solver.fixedorder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;

/**
 * A FixedOrderChainSolver is a FixedOrderSolver that runs multiple FixedOrderSolvers
 * sequentially. The solution found by one solver in the chain is fed to the next
 * solver in the chain. This can for example be used to determine whether certain
 * FixedOrderSolvers can improve upon the results of other FixedOrderSolvers.
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of locations in instances that have to be solved
 */
public class FixedOrderChainSolver<E> implements FixedOrderSolver<E>
{
	private double threshold = 10e-10;
	private boolean allowDecrease;
	private List<FixedOrderSolver<E>> solvers;

	/**
	 * Produces the trivial FixedOrderSolver that does not change the initial solution.
	 */
	
	public FixedOrderChainSolver()
	{
		this(false, Collections.emptyList());
	}
	
	/**
	 * Produces a FixedOrderSolver that applies the solvers in the list sequentially.
	 * Assumes that the solvers never produce a solution that is worse than the initial
	 * solution fed to them.
	 * @param solvers
	 */
	public FixedOrderChainSolver(List<FixedOrderChainSolver<E>> solvers)
	{
		this(false,solvers);
	}
	
	/**
	 * Produces a trivial FixedOrderSolver that does not change the initial solution.
	 * If solvers are added, these solvers are allowed to produce worse solution
	 * that the solution initially provided to them if the boolean is set to true.
	 * @param allowDecrease whether solvers are allowed to produce worse solutions than fed to them
	 */
	
	public FixedOrderChainSolver(boolean allowDecrease)
	{
		this(allowDecrease, Collections.emptyList());
	}
	
	/**
	 * Produces a FixedOrderSolver that executes the solver in the list sequentially.
	 * Solvers in this sequence are only allowed to produce worse solutions if the
	 * allowDecrease boolean is set to true
	 * @param allowDecrease whether solves are allowed to produce worse solutions than those
	 *                      fed to them
	 * @param solvers a list of solvers that will be executed sequentially
	 */
	public FixedOrderChainSolver(boolean allowDecrease, List<FixedOrderChainSolver<E>> solvers)
	{
		this.allowDecrease = allowDecrease;
		this.solvers = new ArrayList<>(solvers);
	}
	
	/**
	 * Adds a fixed order solver to the end of the list of solvers that are executed sequentially.
	 * @param fos the solver to be added to the end of the sequence
	 */
	public void addSolver(FixedOrderSolver<E> fos)
	{
		solvers.add(fos);
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance, Solution<E> order)
	{
		List<E> currentOrder = order.getOrder();
		Solution<E> currentSol = order;
		
		for (FixedOrderSolver<E> solver : solvers)
		{
			Solution<E> newSol = solver.solve(instance, currentOrder);
			if (!allowDecrease && newSol.getTotalCost() - threshold > currentSol.getTotalCost())
			{
				throw new IllegalStateException("Quality should not decrease during the chain.");
			}
			currentSol = newSol;
			currentOrder = newSol.getOrder();
		}
		return currentSol;
	}
	

}
