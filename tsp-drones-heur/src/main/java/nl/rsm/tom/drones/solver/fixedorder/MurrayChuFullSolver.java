package nl.rsm.tom.drones.solver.fixedorder;

import java.util.List;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.Solver;

/**
 * A solver that uses some initial solver and then performs the
 * Murray and Chu heuristic.
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in an instance
 */
public class MurrayChuFullSolver<E> implements Solver<E>
{
	private Solver<E> initSolver;
	private MurrayChuFixedOrderSolver<E> solver;
	
	/**
	 * Constructor which specifies 
	 * @param initSolver
	 */
	public MurrayChuFullSolver(Solver<E> initSolver)
	{
		this.initSolver = initSolver;
		solver = new MurrayChuFixedOrderSolver<E>();
		
	}
	
	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		Solution<E> init = initSolver.solve(instance);
		List<E> initOrder = init.getOrder();
		Solution<E> result = solver.solve(instance, initOrder);
		return result;
	}

}
