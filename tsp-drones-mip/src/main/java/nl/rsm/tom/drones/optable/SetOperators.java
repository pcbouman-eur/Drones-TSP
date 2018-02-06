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

import java.util.Set;

/**
 * An interface that explains how encoded sets can be manipulated.
 * Elements are assumed to live at fixed indices, and instead of
 * manipulating elements directly, they should be manipulated via
 * their indices.
 * 
 * In a sense, this interface is similar to the interface of the BitSet available in Java. 
 * @author Paul Bouman
 *
 * @param <S> the encoding of the set
 */
public interface SetOperators<S>
{
	/**
	 * Provides an empty set
	 * @return an empty set
	 */
	public S empty();
	
	/**
	 * Provides a set with a single element
	 * @param i the index of the element
	 * @return a singleton set with the provided element
	 */
	public S singleton(Integer i);
	
	/**
	 * Provides a set with the first n element
	 * @param n the number of elements to include
	 * @return a set with the first n elements
	 */
	public S fullSet(Integer n);
	
	/**
	 * Provides the union of two sets
	 * @param a one set
	 * @param b another set
	 * @return the union of the two sets
	 */
	public S union(S a, S b);
	
	/**
	 * Provides an Iterable of the subsets of the given set
	 * @param set a set to generate the subsets of
	 * @return an iterable over subsets
	 */
	public Iterable<S> subsets(S set);
	
	/**
	 * Check whether an element is in a set
	 * @param i the index of the element
	 * @param set the set
	 * @return whether the element is in the set
	 */
	public boolean elem(Integer i, S set);
	
	/**
	 * Provides a set of indices of a given encoded set
	 * @param set the encoded set
	 * @return a set of indices of the elements in the set
	 */
	public Set<Integer> elems(S set);
	
	/**
	 * Provides the compliment assuming the full set contains n elements
	 * @param set the set
	 * @param n the number of elements in the 'full set'
	 * @return the complement of the set
	 */
	public S complement(S set, Integer n);
	
	/**
	 * Provides the number of elements in a set
	 * @param set the set
	 * @return the number of elements in the set
	 */
	public int size(S set);
	
	/**
	 * Provides a bitstring representation of the set
	 * @param set the set
	 * @return a bitstring representation
	 */
	public String makeString(S set);
	
	/**
	 * Expand a set with a single element
	 * @param a the base set
	 * @param i the index of the element to add
	 * @return the set a with the element added
	 */
	public default S add(S a, Integer i)
	{
		return union(a, singleton(i));
	}
}
