package nl.rsm.tom.drones.solver.algorithms.localsearch.neighborhood;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.solver.algorithms.localsearch.Action;
import nl.rsm.tom.drones.solver.algorithms.localsearch.AllActionProvider;

/**
 * This provides the actions of multiple actionproviders combined
 * @author Paul Bouman
 *
 * @param <E> the type of location in an instance
 */
public class CombinedProvider<E> implements AllActionProvider<E>
{
	private List<AllActionProvider<E>> providers;

	/**
	 * Creates an empty combined provider
	 */
	public CombinedProvider()
	{
		providers = new ArrayList<>();
	}
	
	/**
	 * Add an action provider to this provider the actions from
	 * that provider will then be provided by this provider
	 * @param provider the provider to add
	 */
	public void addProvider(AllActionProvider<E> provider)
	{
		providers.add(provider);
	}
	
	@Override
	public List<Action<E>> getActions(ArrayList<E> lst, Instance<E> e)
	{
		ArrayList<Action<E>> result = new ArrayList<>();
		for (AllActionProvider<E> prov : providers)
		{
			result.addAll(prov.getActions(lst, e));
		}
		return result;
	}
	
}
