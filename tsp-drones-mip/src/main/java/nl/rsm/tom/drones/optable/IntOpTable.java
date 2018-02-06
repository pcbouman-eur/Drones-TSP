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

import java.util.Collections;
import java.util.List;
import java.util.Set;

import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.optable.constraints.OpConstraint;
import nl.rsm.tom.drones.util.BitwiseTools;

/**
 * Class that builds a table of operations by encoding sets of indices
 * into integers. As a consequence, this class can only deal with instances
 * that have a number of location that is smaller than the number of bits
 * in an integer (i.e. 32). 
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */
public class IntOpTable<E> extends AbstractOpTable<E,Integer>
{
	private Int2ObjectArrayMap<Int2ObjectArrayMap<Int2ObjectArrayMap<OpEntry>>> table;
	private int size;
	
	/**
	 * Constructs an operations table for the given instance
	 * @param instance the instance for which to build a table
	 */
	public IntOpTable(Instance<E> instance)
	{
		super(instance, new IntegerSetTools());
	}

	/**
	 * Constructs a table of operations for the given instance subject to the provided constraints
	 * @param instance
	 * @param constraints
	 */
	public IntOpTable(Instance<E> instance, List<? extends OpConstraint<? super E, ? super Integer>> constraints)
	{
		super(instance, new IntegerSetTools(), constraints);
	}
	
	/**
	 * Returns the size of this operations table
	 * @return
	 */
	public int getSize()
	{
		return size;
	}
	
	@Override
	public Iterable<OpEntry> getOperations(Integer from, Integer to)
	{
		Int2ObjectArrayMap<Int2ObjectArrayMap<OpEntry>> subTab = table.get(from);
		if (subTab == null)
		{
			return Collections.emptySet();
		}
		Int2ObjectArrayMap<OpEntry> subsubTab = subTab.get(to);
		if (subsubTab == null)
		{
			return Collections.emptySet();
		}
		return Collections.unmodifiableCollection(subsubTab.values());
	}

	
	/**
	 * Prints the operations table to the standard out.
	 */
	public void printOpTable()
	{
		IntegerSetTools st = new IntegerSetTools();
		for (Integer i : table.keySet())
		{
			Int2ObjectArrayMap<Int2ObjectArrayMap<OpEntry>> subTab = table.get(i);
			for (Integer j : subTab.keySet())
			{
				Int2ObjectArrayMap<OpEntry> subSubTab = subTab.get(j);
				for (OpEntry o : subSubTab.values())
				{
					String setString = st.elems(o.set).toString();
					String entryString = "[ f:"+o.first+", t:"+o.last+", s:"+setString+" ]";
					System.out.println(entryString + " - > ["+o.getCost()+"] "+o.getOperation());
				}
			}
		}
	}
	
	@Override
	protected void initOpTable()
	{
		table = new Int2ObjectArrayMap<>();
	}

	@Override
	protected void storeEntry(OpEntry entry)
	{
		if (!table.containsKey(entry.first))
		{
			table.put(entry.first, new Int2ObjectArrayMap<>());
		}
		Int2ObjectArrayMap<Int2ObjectArrayMap<OpEntry>> subtab = table.get(entry.first);
		if (!subtab.containsKey(entry.last))
		{
			subtab.put(entry.last, new Int2ObjectArrayMap<>());
		}
		Int2ObjectArrayMap<OpEntry> subsubtab = subtab.get(entry.last);
		if (!subsubtab.containsKey(entry.set))
		{
			size++;
		}
		subsubtab.put(entry.set, entry);
	}

	@Override
	protected OpEntry getEntry(int first, int last, Integer set) 
	{
		if (!table.containsKey(first))
		{
			return null;
		}
		Int2ObjectArrayMap<Int2ObjectArrayMap<OpEntry>> subtab = table.get(first);
		if (!subtab.containsKey(last))
		{
			return null;
		}
		Int2ObjectArrayMap<OpEntry> subsubtab = subtab.get(last);
		OpEntry res = subsubtab.get(set);
		return res;
	}
	
	/**
	 * Class that provides set operations on a set of indices encoded using the bits
	 * of an integer.
	 * @author Paul Bouman
	 *
	 */
	public static class IntegerSetTools implements SetOperators<Integer>
	{

		@Override
		public Integer singleton(Integer i)
		{
			return BitwiseTools.add(i, 0);
		}

		@Override
		public Integer fullSet(Integer n)
		{
			return BitwiseTools.fullSet(n);
		}

		@Override
		public Integer union(Integer a, Integer b)
		{
			return BitwiseTools.union(a, b);
		}

		@Override
		public Iterable<Integer> subsets(Integer set)
		{
			return BitwiseTools.subsets(set);
		}

		@Override
		public boolean elem(Integer i, Integer set)
		{
			return BitwiseTools.elem(i, set);
		}

		@Override
		public Set<Integer> elems(Integer set)
		{
			return BitwiseTools.indexSet(set);
		}

		@Override
		public Integer complement(Integer set, Integer n)
		{
			return BitwiseTools.subtract(fullSet(n), set);
		}

		@Override
		public Integer empty()
		{
			return 0;
		}

		@Override
		public int size(Integer set)
		{
			return BitwiseTools.size(set);
		}

		@Override
		public String makeString(Integer set)
		{
			return Integer.toBinaryString(set);
		}	
	}

	
	@Override
	public OperationEntry<E, Integer> getOpEntry(E from, E to, Integer covered)
	{
		return getEntry(typeMap.get(from), typeMap.get(to), covered);
	}
	

}
