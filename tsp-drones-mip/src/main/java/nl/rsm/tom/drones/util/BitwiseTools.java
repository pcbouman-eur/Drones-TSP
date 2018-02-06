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
package nl.rsm.tom.drones.util;


import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class BitwiseTools
{
	/**
	 * Generates integer representations of subsets given a set as an integer based representation of the set
	 * @param set The set to generate subsets of
	 * @return An iterable which generates subsets of the given set
	 */
	public static Iterable<Integer> subsets(final int set)
	{
		return () -> new Iterator<Integer>()
		{
			private int current = set;

			@Override
			public boolean hasNext()
			{
				return current > 0;
			}

			@Override
			public Integer next()
			{
				int result = current;
				current = (current - 1) & set;
				return result;
			}
		};
	}
	
	/**
	 * Checks whether the element with the given index is in the set
	 * @param index The index of the element for which presence should be checked
	 * @param set The integer representation of the set
	 * @return Whether the bit at the index is set to 1
	 */
	public static boolean elem(int index, int set)
	{
		return ((1 << index) & set) != 0;
	}
	

	/**
	 * Returns the integer representation of a set containing all elements up to the given size.
	 * @param size The number of elements in the set
	 * @return An integer representation with the bits for 'size' elements set to 1
	 */
	public static int fullSet(int size)
	{
		return (1 << size) - 1;
	}
	
	/**
	 * Computes the union of two sets using integer representations
	 * @param set1 Integer representation of the first set
	 * @param set2 Integer representation of the second set
	 * @return Integer representation of the union of the two sets
	 */
	public static int union(int set1, int set2)
	{
		return set1 | set2;
	}
	
	/**
	 * Set subtraction using integer representations
	 * @param set1 The integer representation of the set from which the other set will be subtracted
	 * @param set2 The integer representation of the set which will be subtracted from the other set
	 * @return Integer representation of the resulting set of the subtraction
	 */
	public static int subtract(int set1, int set2)
	{
		return set1 & ~set2;
	}
	
	/**
	 * Adds the element with a given index to the set
	 * @param index The index of the element to be added
	 * @param set The set which it should be added to
	 * @return Integer representation of the resulting set
	 */

	public static int add(int index, int set)
	{
		return set | (1 << index);
	}
	
	/**
	 * Removes an element with a given index from the given set
	 * @param index The index of the element to be removed
	 * @param set The set it should be removed from
	 * @return Integer representation of the resulting set
	 */
	public static int remove(int index, int set)
	{
		return set & ~(1 << index);
	}
	
	/**
	 * Generates a list with the indices of the elements present in a given set
	 * @param set Integer representation of the set which should be converted
	 * @return The list of indices contained in the provided set
	 */
	public static Set<Integer> indexSet(int set)
	{
		LinkedHashSet<Integer> result = new LinkedHashSet<>();
		int cur = set;
		for (int t=0; t < 32; t++)
		{
			if ((1 & cur) == 1)
			{
				result.add(t);
			}
			cur >>= 1;
		}
		return result;
	}
		
	/**
	 * This method provides an element set based on a list of elements
	 * at their associated indices and an integer which has bit-wise
	 * encoding of the set
	 * @param data the list of elements at their associated indices
	 * @param set an int which provides a bitwise encoding of a set
	 * @return a set which the provided elements.
	 */
	public static <E> Set<E> makeSet(List<E> data, int set)
	{
		HashSet<E> result = new HashSet<>();
		for (int t=0; t < data.size(); t++)
		{
			if (elem(t,set))
			{
				result.add(data.get(t));
			}
		}
		return result;
	}
	
	/**
	 * This counts the number of elements in the set by counting the nubmer of
	 * bits set to 1. 
	 * The bit counting algorithm is obtained from http://stackoverflow.com/a/109025
	 * Apparently it is called the 'parallel' or 'variable-precision SWAR algorithm'.
	 * 
	 * @param set an int providing a bit-wise encoding of some set
	 * @return the number of element in the set (or bits set to 1)
	 */

	public static int size(int set)
	{
		set = set - ((set >>> 1) & 0x55555555);
		set = (set & 0x33333333) + ((set >>> 2) & 0x33333333);
		return (((set + (set >>> 4) & 0x0F0F0F0F) * 0x01010101) >>> 24);
	}

}
