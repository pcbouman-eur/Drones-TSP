/* Copyright 2017 Paul Bouman
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the  
 * "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
 * THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * In case you use this software for research purposes, it is appreciated if you provide a citation of the following paper:
 * 
 * N.A.H. Agatz, P.C. Bouman & M.E. Schmidt. Optimization Approaches for the Traveling Salesman Problem with Drone. Transportation Science.
 * 
 * The paper still has to appear, but was accepted for publication. This notice will be updated with a more detailed reference if that
 * information is available.
 */

package nl.rsm.tom.drones.data.instance;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.function.Predicate;

import nl.rsm.tom.drones.data.Distance;

/**
 * Interface that determines how information can be extracted from an
 * instance of the problem.
 * @author Paul Bouman
 *
 * @param <E> the location type of this instance
 */

public interface Instance<E> extends Iterable<E>
{
	/**
	 * The NodeCount is the number of locations including the depot.
	 * @return the number of nodes in the instance
	 */
	default public int getNodeCount()
	{
		return getLocationCount() + 1;
	}
	
	/**
	 * The number of locations that need to be visited (excluding the depot)
	 * @return The number of locations without the depot
	 */
	public int getLocationCount();
	
	/**
	 * The depot
	 * @return the depot
	 */
	public E getDepot();
	
	default public boolean isDepot(E e)
	{
		return e.equals(getDepot());
	}
	
	/**
	 * The locations that need to visited, excluding the depot
	 * @return an iterable containing the locations that need to be visited
	 */
	public List<E> getLocations();

	/**
	 * Returns a location based on an index.
	 * An index of 0 corresponds to the depot, while the other indices
	 * correspond to the locations
	 * @param index The index to be returned
	 * @return The corresponding location.
	 */
	default public E getLocation(int index)
	{
		if (index == 0)
		{
			return getDepot();
		}
		List<E> locs = getLocations();
		if (index > locs.size() || index < 0)
		{
			throw new IndexOutOfBoundsException();
		}
		return locs.get(index-1);
	}
	
	/**
	 * Create an instance that only contains the nodes for which
	 * the supplied predicate returns true
	 * The depot will always be retained
	 * @param retain a predicate indicating which nodes should remain
	 * @return the resulting smaller instance
	 */
	public Instance<E> getSubInstance(Predicate<? super E> retain);
	
	/**
	 * Create an instance that only contains the nodes which are
	 * also contained in the supplied Collection
	 * The depot will always be retained
	 * @param retain the Collection of nodes that should be retained
	 * @return the resulting smaller instance
	 */
	public default Instance<E> getSubInstance(Collection<? super E> retain)
	{
		return getSubInstance(retain::contains);
	}
	
	/**
	 * A distance metric that defines drive distances
	 * @return a distance metric for driving
	 */
	public Distance<E> getDriveDistance();
	
	/**
	 * A distance metric that defines fly distances
	 * @return a distance metric for flying
	 */
	public Distance<E> getFlyDistance();
	
	/**
	 * An iterator which iterates over all locations that need
	 * to be visited, including the depot.
	 */
	@Override
	default public Iterator<E> iterator()
	{
		return new Iterator<E>()
		{

			Iterator<E> it = getLocations().iterator();
			
			@Override
			public boolean hasNext()
			{
				return it != null;
			}

			@Override
			public E next()
			{
				if (it == null)
				{
					return null;
				}
				if (!it.hasNext())
				{
					it = null;
					return getDepot();
				}
				return it.next();
			}
			
		};
	}
	
	/**
	 * Generates a list of locations that are accepted by a certain predicate
	 * @param pred The predicate used to test whether a location needs to be included
	 * @return a list with matching locations
	 */
	public default List<E> select(Predicate<E> pred)
	{
		List<E> result = new ArrayList<>();
		for (E loc : this)
		{
			if (pred.test(loc))
			{
				result.add(loc);
			}
		}
		return result;
	}
	
	
}
