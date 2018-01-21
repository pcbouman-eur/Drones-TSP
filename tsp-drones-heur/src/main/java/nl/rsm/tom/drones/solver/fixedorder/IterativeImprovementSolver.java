package nl.rsm.tom.drones.solver.fixedorder;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.algorithms.IterativeImprovement;
import nl.rsm.tom.drones.solver.algorithms.localsearch.AllActionProvider;
import nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood.CombinedProvider;
import nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood.InsertProvider;
import nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood.SwapProvider;
import nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood.TwoOptProvider;

/**
 * Solver that applies iterative improvement procedures to instances
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */
public class IterativeImprovementSolver<E> implements FixedOrderSolver<E>
{
	private FixedOrderSolver<E> solver;
	private AllActionProvider<E> provider;

	/**
	 * Constructor that accepts a FixedOrderSolver to apply to the different orders generated
	 * during the local search, and a general AllActionProvider that can generate neighborhoods
	 * @param solver a FixedOrderSolver that translates the orders of the local search into
	 *               actual truck-and-drone tours
	 * @param provider an AllActionProvider that generates actions that represent the neighborhood
	 *                 of the current order
	 */
	public IterativeImprovementSolver(FixedOrderSolver<E> solver, AllActionProvider<E> provider)
	{
		this.solver = solver;
		this.provider = provider;
	}
	
	/**
	 * Constructor in which we can select which neighborhood functions are combined, and then uses
	 * the provided FixedOrderSolver to obtain truck-and-drone tours for the explored orders
	 * @param solver the solver that will generate truck-and-drone tours.
	 * @param swap whether to consider the swapping neighborhoods during search 
	 * @param twoopt whether to consider the 2-opt neighborhoods during search
	 * @param ins whether to consider the insertion neighborhoods during search
	 */
	public IterativeImprovementSolver(FixedOrderSolver<E> solver, boolean swap, boolean twoopt, boolean ins)
	{
		this.solver = solver;
		CombinedProvider<E> cp = new CombinedProvider<>();
		if (swap)
		{
			cp.addProvider(new SwapProvider<>());
		}
		if (twoopt)
		{
			cp.addProvider(new TwoOptProvider<>());
		}
		if (ins)
		{
			cp.addProvider(new InsertProvider<>());
		}
		provider = cp;
			
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance, Solution<E> order)
	{
		IterativeImprovement<E> ii = new IterativeImprovement<>(instance,order,provider,solver);
		ii.solve();
		return ii.getCurrentSolution();
	}

}
