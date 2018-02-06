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
package nl.rsm.tom.drones.optable;

import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * General interface for a table of operations
 * @author Paul Bouman
 *
 * @param <E> the type of locations 
 * @param <S> the type that is used to encode sets of locations
 */
public interface OpTable<E,S>
{
	/**
	 * Retrieve the instance for which this table was built
	 * @return the instance
	 */
	public Instance<E> getInstance();
	
	/**
	 * Retrieve an operation with a given origin, a given destination and
	 * the given set of locations covered
	 * @param from the origin of the operation
	 * @param to the destination of the operation
	 * @param covered the set of covered locations
	 * @return the associated operation stored in the table
	 */
	public Operation<E> getOperation(E from, E to, Collection<E> covered);
	
	/**
	 * Retrieve an operation with a given origin, a given destination and
	 * the given set of locations covered
	 * @param from the origin of the operation
	 * @param to the destination of the operation
	 * @param covered the set of covered locations
	 * @return the associated operation stored in the table
	 */
	public OperationEntry<E,S> getOpEntry(E from, E to, S covered);
	
	/**
	 * Add an element to a set and return the new set
	 * @param s the set to add the element to
	 * @param e the element to add
	 * @return the expanded set
	 */
	public S expandSet(S s, E e);
	
	/**
	 * Get the tools that can be used to manipulate sets.
	 * @return the set tools
	 */
	public SetOperators<S> getTools();
	
	/**
	 * Compute the complement of the given set, based on the instance
	 * associated with this table 
	 * @param s the set to take the complement of
	 * @return the complete of the set
	 */
	public S complement(S s);
	
	/**
	 * Convert an encoded set to a regular Java Collection
	 * @param s an encoded set
	 * @return the set as a Java Collection
	 */
	public Collection<E> convert(S s);
	
	/**
	 * Converts a single location into an encoded singleton set
	 * @param singleton the element to put into the singleton set
	 * @return an encoded set containing only the location
	 */
	public default S makeSet(E singleton)
	{
		return makeSet(Collections.singleton(singleton));
	}
	
	/**
	 * Converts a Java Collection into a coded set
	 * @param covered the Java Collection of locations
	 * @return an encoded set of these locations
	 */
	public default S makeSet(Collection<E> covered)
	{
		S res = getTools().empty();
		for (E elem : covered)
		{
			res = expandSet(res, elem);
		}
		return res;
	}
	
	/**
	 * Enumerates useful subsets of the locations in the instance such that
	 * no locations in the provided set are visited by only the truck or
	 * only the drone.
	 * @param set the set of locations that have been visisted
	 * @param from the origin of the operation under consideration
	 * @param to the destination of the operaton under consideration
	 * @return an Iterable over the sets for which it is still necessary
	 *         to cover a number of locations.
	 */
	public default Iterable<S> enumerateUsefulSubsets(S set, E from, E to)
	{
		SetOperators<S> st = getTools();
		HashSet<S> result = new HashSet<>();
		HashSet<S> queue = new HashSet<>();
		S basicSet = expandSet(expandSet(st.empty(), from), to);
		S emptySet = st.empty();
		
		if (getOpEntry(from, to, basicSet) != null)
		{
			result.add(emptySet);
			queue.add(emptySet);
		}
		
		while (!queue.isEmpty())
		{
			HashSet<S> newQueue = new HashSet<>();
			for (S curSet : queue)
			{
				for (Integer index : st.elems(set))
				{
					if (!st.elem(index, curSet))
					{
						S expSet = st.add(curSet, index);
						if (getOpEntry(from, to, st.union(expSet, basicSet)) != null)
						{
							result.add(expSet);
							newQueue.add(expSet);
						}
					}
				}
			}
			queue = newQueue;
		}
		return result;
	}
}
