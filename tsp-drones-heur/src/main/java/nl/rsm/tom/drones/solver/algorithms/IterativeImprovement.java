package nl.rsm.tom.drones.solver.algorithms;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.FixedOrderSolver;
import nl.rsm.tom.drones.solver.algorithms.localsearch.Action;
import nl.rsm.tom.drones.solver.algorithms.localsearch.AllActionProvider;

/**
 * Implements an iterative improvement procedure to find good solutions
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */
public class IterativeImprovement<E> extends FixedOrderLocalSearchBase<E>
{

	/**
	 * Initialize an objectthat can perform an iterative improvement procedure
	 * @param instance the instance to solve
	 * @param initial an initial solution from which a starting order can be obtained
	 * @param allActions a provider of neighborhoods of the order 
	 * @param solver a solver that takes an order of locations and divides work over truck and drone
	 */
	public IterativeImprovement(Instance<E> instance, Solution<E> initial,
			AllActionProvider<E> allActions, FixedOrderSolver<E> solver)
	{
		super(instance, initial, allActions, solver);
	}
	
	public void solve()
	{
		Solution<E> best = getCurrentSolution();
		boolean improve = true;
		while (improve)
		{	
			Action<E> a = getBestAction();
			if (a != null)
			{
				super.doAction(a);
				Solution<E> newSol = getCurrentSolution();
				if (newSol.getTotalCost() >= best.getTotalCost())
				{
					super.undoAction(a);
					improve = false;
				}
				else
				{
					best = newSol;
				}
			}
			else
			{
				improve = false;
			}
		}
		
	}

}
