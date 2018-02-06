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

import it.unimi.dsi.fastutil.ints.Int2ObjectArrayMap;
import it.unimi.dsi.fastutil.objects.Object2IntArrayMap;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.optable.constraints.OpConstraint;

/**
 * This class contains some of the logic that is needed for constructing a table of operations.
 * 
 * Be warned: the current setup of tabulating operations is probably more complicated than it
 *            needs to be. 
 *            
 * It is assumed that the locations of type T in the instance also have integer indices,
 * and that a set of these indices can be encoded in a set of type S. This class works
 * with an instance and an SetOperators object to construct a table of operations.
 * It can also take constraints on valid operations in account. Since operations are
 * constructed start small, the tabulation algorithm stops expanding a certain operation
 * when one of the constraints is violated. 
 * 
 * @author Paul Bouman
 *
 * @param <T> the type of the locations in the instance
 * @param <S> a type in which sets of integer indices can be encoded
 */

public abstract class AbstractOpTable<T, S> implements OpTable<T,S>
{
	private List<? extends OpConstraint<? super T, ? super S>> constraints;
	
	private Instance<T> instance;
	private Distance<T> driveDist;
	private Distance<T> flyDist;
	
	private SetOperators<S> st;
	
	protected Map<T, Integer> typeMap;
	protected Map<Integer, T> intMap;

	/**
	 * Initializes an implementation agnostic data structure for the construction
	 * of a table which contains only efficient operations. Calling this constructor
	 * automatically triggers the construction of building the table of operations.
	 * @param instance the instance for which a table of operations must be constructed
	 * @param tools tools that can operate on the type in which location sets are encoded
	 */
	public AbstractOpTable(Instance<T> instance, SetOperators<S> tools)
	{
		this(instance, tools, Collections.emptyList());
	}
	
	/**
	 * Initializes an implementation agnostic data structure for the constructor of a
	 * table which contains only efficient operations that adhere to the provided list
	 * of constraints. Calling this constructor automatically triggers the construction
	 * of the the table of operations. The more restricted the set of constraints, the
	 * faster the constructor will occur.
	 * @param instance the instance for which a table of operations must be constructed
	 * @param tools tools that can operate on the type in which location sets are encoded
	 * @param cons a list of constraints that must be taken into account while
	 */
	public AbstractOpTable(Instance<T> instance, SetOperators<S> tools,
			List<? extends OpConstraint<? super T,? super S>> cons)
	{
		this.constraints = cons;
		this.instance = instance;
		this.driveDist = instance.getDriveDistance();
		this.flyDist = instance.getFlyDistance();
		
		this.intMap = new Int2ObjectArrayMap<>();
		this.typeMap = new Object2IntArrayMap<>();
		
		this.st = tools;
		
		for (T node : instance)
		{
			int index = typeMap.size();
			typeMap.put(node, index);
			intMap.put(index, node);
		}
		
		buildTable();
	}

	/**
	 * Initializes the table
	 */
	protected abstract void initOpTable();
	/**
	 * Store an entry in the table
	 * @param entry the entry to story
	 */
	protected abstract void storeEntry(OpEntry entry);
	/**
	 * Retrieve an entry from the table
	 * @param first the index of the origin location
	 * @param last the index of the destination location
	 * @param set the set of locations covered
	 * @return an operation entry stored in the table
	 */
	protected abstract OpEntry getEntry(int first, int last, S set);
	
	/**
	 * Retrieve the instance for which this operations table was constructed
	 */
	public Instance<T> getInstance()
	{
		return instance;
	}
	
	/**
	 * Retrieve an operation with the specified origin and destination,
	 * which covers the given collection of locations. Note that this
	 * method may return null if no such operation is valid.
	 * @param from the origin location
	 * @param to the destination location
	 * @param covered a collection of locations that must be covered by the operation
	 * @return an efficient operation according to the specification, or null if none exists
	 */
	public Operation<T> getOperation(T from, T to, Collection<T> covered)
	{
		Integer f = typeMap.get(from);
		Integer t = typeMap.get(to);
		S s = st.union(st.singleton(f), st.singleton(t));
		for (T e : covered)
		{
			s = st.add(s,  typeMap.get(e));
		}
		OpEntry oe = getEntry(f,t,s);
		if (oe == null)
		{
			return null;
		}
		return oe.getOperation();
	}

	
	private boolean isValid(OpEntry entry)
	{
		if (constraints.size() < 1)
		{
			return true;
		}
		for (OpConstraint<? super T, ? super S> con : constraints)
		{
			if (!con.isValid(entry))
			{
				return false;
			}
		}
		return true;
	}
	
	private void buildTable()
	{
		initOpTable();
		ArrayList<OpEntry> expand = new ArrayList<>();
		for (Integer index=0; index < typeMap.size(); index++)
		{
			OpEntry start = new OpEntry(index, index, st.singleton(index), null, 0);
			expand.add(start);
			storeEntry(start);
		}
		while (!expand.isEmpty())
		{
			ArrayList<OpEntry> newExpand = new ArrayList<>();
			for (OpEntry e : expand)
			{
				//S set = st.complement(e.set, instance.getNodeCount());
				//set = st.union(set, st.singleton(depot));
				S set = st.fullSet(instance.getNodeCount());
				for (Integer node : st.elems(set))
				{
					T from = intMap.get(e.last);
					T to = intMap.get(node);
					S newSet = st.add(e.set, node);
					double newValue = e.driveValue + driveDist.getContextFreeDistance(from, to, e.driveValue);
					OpEntry old = getEntry(e.first,node,newSet);
					if (old == null || newValue < old.driveValue)
					{
						OpEntry newEntry = new OpEntry(e.first, node, newSet, e, newValue);
						if (isValid(newEntry))
						{
							storeEntry(newEntry);
							// If an operation ends at the depot, don't expand it further
							if (!instance.isDepot(to) && !newEntry.repetition)
							{
								newExpand.add(newEntry);
							}
						}
					}					
				}
			}
			expand = newExpand;
		}
		addFlyNodes();
	}
	
	private void addFlyNodes()
	{
		//System.out.println("Adding Fly Nodes");
		List<OpEntry> flyEntries = new ArrayList<OpEntry>();
		for (Integer f=0; f < typeMap.size(); f++)
		{
			T from = intMap.get(f);
			for (Integer t=0; t < typeMap.size(); t++)
			{
				T to = intMap.get(t);
				for (OpEntry oe : getOperations(f,t))
				{
					if (oe.drone == null)
					{
						for (Integer fly : st.elems(st.complement(oe.set, instance.getNodeCount())))
						{
							T flyNode = intMap.get(fly);
							if (instance.isDepot(flyNode))
							{
								continue;
							}
							S newSet = st.add(oe.set, fly);
							OpEntry old = getEntry(f,t,newSet);
							
							double flyValue = flyDist.getFlyDistance(from, to, flyNode);
							OpEntry newEntry = new OpEntry(f,t,fly,newSet,oe,oe.driveValue,flyValue);
							if (isValid(newEntry) && (old == null || old.value > newEntry.value))
							{
								flyEntries.add(newEntry);
							}
						}
					}
				}
			}
		}
		for (OpEntry oe : flyEntries)
		{
			OpEntry old = getEntry(oe.first, oe.last, oe.set);
			if (old == null || old.value > oe.value)
			{
				storeEntry(oe);
			}
		}
	}
	
	

	@Override
	public S expandSet(S s, T e)
	{
		return st.add(s, typeMap.get(e));
	}
	
	@Override
	public SetOperators<S> getTools()
	{
		return st;
	}
	
	@Override
	public S complement(S s)
	{
		return st.complement(s, instance.getNodeCount());
	}
	
	@Override
	public Collection<T> convert(S s)
	{
		List<T> result = new ArrayList<>();
		for (Integer i : st.elems(s))
		{
			result.add(intMap.get(i));
		}
		return result;
	}
	
	/**
	 * Index based retrievel of operations that start at the location with index from,
	 * and end at locations with index to. Must be 
	 * @param from the index of the origin location
	 * @param to the index of the destination location
	 * @return an iterable of OpEntry that start in the origin and end at the destination
	 */
	public abstract Iterable<OpEntry> getOperations(Integer from, Integer to);
	
	/**
	 * Generates a list of operations that start at a given origin and end at the
	 * given destination. All efficient operations that were constructed are 
	 * returned.
	 * @param from the origin
	 * @param to the destination
	 * @return an iterable of efficient operations that start at the origin and end at the destination
	 */
	public Iterable<Operation<T>> getOperations(T from, T to)
	{
		Integer f = typeMap.get(from);
		Integer t = typeMap.get(to);
		final Iterable<OpEntry> it = getOperations(f, t);
		
		return new Iterable<Operation<T>>()
		{

			@Override
			public Iterator<Operation<T>> iterator()
			{
				Iterator<AbstractOpTable<T, S>.OpEntry> iterator = it.iterator();
				return new Iterator<Operation<T>>()
				{

					@Override
					public boolean hasNext()
					{
						return iterator.hasNext();
					}

					@Override
					public Operation<T> next()
					{
						return iterator.next().getOperation();
					}
					
				};
			}
			
		};
		
	}

	/**
	 * Class that models an entry in an operations table
	 * @author Paul Bouman
	 *
	 */
	public class OpEntry implements Iterable<T>, OperationEntry<T,S>
	{
		public final Integer first;
		public final Integer last;
		public final S set;
		public final OpEntry prev;
		public final double driveValue;
		public final double flyValue;
		public final double value;
		public final Integer drone;
		public final int nodeCount;
		public final boolean repetition;
		
		/**
		 * Creates an operation entry with an origin, destination, set
		 * of covered nodes, a predecessor entry in the table and
		 * the value of the entry (assuming that no drone is active)
		 * @param f an origin
		 * @param l a destination
		 * @param s a set of covered nodes
		 * @param p a predecessor entry
		 * @param v the (truck-only) value of the entry
		 */
		public OpEntry(Integer f, Integer l, S s, OpEntry p, double v)
		{
			this(f,l,null,s,p,v,0);
		}
		
		/**
		 * Creates an operation entry with an origin, destination, set
		 * of covered nodes, a predecessor entry in the table and
		 * values of the entry for truck and drone
		 * @param f an origin
		 * @param l a destination
		 * @param s a set of covered nodes
		 * @param p a predecessor entry
		 * @param dv the truck value of the entry
		 * @param fv the drone value of the entry
		 */
		public OpEntry(Integer f, Integer l, Integer d, S s, OpEntry p, double dv, double fv)
		{
			boolean rep = false;
			if (p != null)
			{
				rep = p.repetition;
			}
			if (p != null && p.drone != null)
			{
				throw new IllegalArgumentException("Cannot extend an entry which has a drone");
			}
			if (p != null && d == null && st.elem(l, p.set))
			{
				if (p.repetition)
				{
					throw new IllegalArgumentException("Cannot repeat a node in a single operation.");
				}
				else if (st.size(s) > 1)
				{
					rep = true;
				}
			}
			first = f;
			last = l;
			set = s;
			driveValue = dv;
			prev = p;
			drone = d;
			flyValue = fv;
			value = Math.max(dv,fv);
			nodeCount = st.size(s);
			repetition = rep;
		}

		@Override
		public Operation<T> getOperation()
		{
			T fly = null;
			if (drone != null)
			{
				fly = intMap.get(drone);
			}
			ArrayList<T> list = new ArrayList<>();
			forEach(list::add);
			Collections.reverse(list);
			Operation<T> op;
			if (list.size() == 1)
			{
				op = new Operation<>(list.get(0), list.get(0), fly);
			}
			else
			{
				op = new Operation<>(list,fly);	
			}
			if (Math.abs(op.getCost(instance) - value) > 1e-8)
			{
				throw new IllegalStateException("Table value and operation value do not match!");
			}
			return op;
		}
		
		@Override
		public Iterator<T> iterator()
		{
			return new EntryIterator(this);
		}

		@Override
		public S getSet()
		{
			return set;
		}

		@Override
		public T getFrom()
		{
			return intMap.get(first);
		}

		@Override
		public T getTo()
		{
			return intMap.get(last);
		}

		@Override
		public T getFly()
		{
			if (drone == null)
			{
				return null;
			}
			return intMap.get(drone);
		}

		@Override
		public double getDriveCost()
		{
			return driveValue;
		}

		@Override
		public double getFlyCost()
		{
			return flyValue;
		}

		@Override
		public double getCost()
		{
			return value;
		}

		@Override
		public int nodeCount() 
		{
			return nodeCount;
		}

		@Override
		public int getTruckOnlyCount()
		{
			int minus = 0;
			if (getFly() != null)
			{
				minus++;
			}
			if (!getFrom().equals(getFly()))
			{
				minus++;
			}
			if (!getTo().equals(getFrom())&&!getTo().equals(getFly()))
			{
				minus++;
			}
			return nodeCount - minus;
		}
		
		@Override
		public OperationEntry<T, S> getPrev()
		{
			return prev;
		}
	}
	
	/**
	 * Iterator that walks back from an entry in the table
	 * @author Paul Bouman
	 *
	 */
	public class EntryIterator implements Iterator<T>
	{
		private OpEntry cur;
		private T last;

		/**
		 * Initializes an iterator that walks back from the provided entry
		 * @param entry the entry to walk back from
		 */
		public EntryIterator(OpEntry entry)
		{
			cur = entry;
			if (cur.drone != null && cur.prev.last == cur.last)
			{
				cur = cur.prev;
			}
		}
		
		@Override
		public boolean hasNext()
		{
			return cur != null;
		}

		@Override
		public T next()
		{
			T result = null;
			if (cur != null)
			{
			  result = intMap.get(cur.last);
			  if (cur.prev == null)
			  {
				  last = intMap.get(cur.first);
			  }
			  cur = cur.prev;			  
			}
			else
			{
				if (last == null)
				{
					result = last;
					last = null;
				}
			}
			return result;
		}
	}
	
}
