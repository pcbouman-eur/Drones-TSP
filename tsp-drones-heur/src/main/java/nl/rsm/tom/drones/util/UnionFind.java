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

import java.util.HashMap;
import java.util.Map;

/**
 * Union-Find Data Structure for the purpose of MST computations
 * @author Paul Bouman
 *
 * @param <E> The type of data stored in this Data Structure
 */
public class UnionFind<E>
{
	private Map<E,SetNode> _pointers;
		
	/**
	 * Instantiates a Union Find Datastructure
	 */
	public UnionFind()
	{
		_pointers = new HashMap<>();
	}
	
	/**
	 * Checks whether two elements are in the same set
	 * @param e1 The first element
	 * @param e2 The second element
	 * @return Whether they are currently in the same set
	 */
	public boolean sameSet(E e1, E e2)
	{
		SetNode sn1 = _pointers.get(e1);
		SetNode sn2 = _pointers.get(e2);
		if (sn1 == null)
		{
			throw new IllegalArgumentException("First argument was never registered as a set!");
		}
		if (sn2 == null)
		{
			throw new IllegalArgumentException("Second argument was never registered as a set!");
		}
		return sn1.getRoot().element.equals(sn2.getRoot().element);
	}
	
	/**
	 * Creates a singleton set with this value
	 * @param e
	 */
	public void createSet(E e)
	{
		if (!_pointers.containsKey(e))
		{
			SetNode sn = new SetNode(e);
			_pointers.put(e, sn);
		}
		else
		{
			throw new IllegalArgumentException("There is already a set for element "+e);
		}
	}

	/**
	 * Unions two sets which contain the two supplied arguments
	 * @param e1 The first element identifying a set to be unioned 
	 * @param e2 The second element identifying a set to be unioned
	 */
	public void union(E e1, E e2)
	{
		SetNode sn1 = _pointers.get(e1);
		SetNode sn2 = _pointers.get(e2);
		if (sn1 == null)
		{
			throw new IllegalArgumentException("First argument was never registered as a set!");
		}
		if (sn2 == null)
		{
			throw new IllegalArgumentException("Second argument was never registered as a set!");
		}
		sn1.getRoot().setParent(sn2);
	}
	
	private class SetNode
	{
		public final E element;
		private SetNode _parent;
		
		public SetNode(E e)
		{
			element = e;
		}
		
		public SetNode getRoot()
		{
			if (_parent == null)
			{
				return this;
			}
			SetNode cur = _parent;
			while (cur._parent != null)
			{
				cur = cur._parent;
			}
			_parent = cur;
			return cur;
		}
		
		public void setParent(SetNode parent)
		{
			if (_parent != null)
			{
				throw new IllegalArgumentException("Cannot set a parent, as one was already set!");
			}
			_parent = parent;
		}
	}
}
