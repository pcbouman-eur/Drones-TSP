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
package nl.rsm.tom.drones.solver.algorithms;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.util.MaxHeap;
import nl.rsm.tom.drones.util.MaxHeap.HeapIndexChangedListener;

/**
 * This class implements the greedy heuristic that greedily decides
 * which locations have to be visited by the drone and which ones by
 * the truck. The algorithm is introduced and explained in the paper
 * 
 * N.A.H. Agatz, P.C. Bouman & M.E. Schmidt. Optimization Approaches for the Traveling Salesman Problem with Drone. Transportation Science.
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the instance
 */

public class FixedOrderHeuristic<E>
{
	private final Instance<E> _instance;
	private final Solution<E> _sol;
	private final boolean _nonNegative;
	private final Distance<E> _drive;
	private final Distance<E> _fly;
	private final boolean _twoPass;

	private MaxHeap<SolutionNode> _heap;
	private ArrayList<SolutionNode> _list;
	
	/**
	 * Constructor for the Fixed Order Heuristic
	 * @param i The instance to which the heuristic will be applied
	 * @param sol The basic solution which describes the fixed order
	 * @param nonNegative should the algorithm stop when a nonNegative savings is encountered?
	 */
	public FixedOrderHeuristic(Instance<E> i, Solution<E> sol, boolean nonNegative, boolean twoPass)
	{
		_instance = i;
		_drive = _instance.getDriveDistance();
		_fly = _instance.getFlyDistance();
		_sol = sol;
		_nonNegative = nonNegative;
		_twoPass = twoPass;
		
		init();
		double res = solve();
		if (_twoPass && res >= 0)
		{
			solve(res);
		}

	}
	
	private void init()
	{
		_heap = new MaxHeap<SolutionNode>(_instance.getNodeCount()+2);
		_list = new ArrayList<>(_instance.getNodeCount()+1);
		for (Operation<E> op : _sol)
		{
			if (!op.getInternalNodes(true).isEmpty())
			{
				throw new IllegalArgumentException("The provided solution contains complex operations.");
			}
			_list.add(new SolutionNode(op.getStart()));
		}
		_list.add(new SolutionNode(_instance.getDepot()));
		_list.get(0).makeLink(null, _list.get(1));
		for (int t=1; t < _list.size() - 1; t++)
		{
			_list.get(t).makeLink(_list.get(t-1), _list.get(t+1));
		}
		_list.get(_list.size()-1).makeLink(_list.get(_list.size()-2), null);
		for (SolutionNode sn : _list)
		{
			_heap.add(sn, sn.getMaxSavings());
		}
	}
	
	private double solve()
	{
		return solve(Double.POSITIVE_INFINITY);
	}
	
	private double solve(double target)
	{
		double curSavings = 0;
		double bestTarget = 0;
		while (_heap.size() > 0)
		{
			SolutionNode sn = _heap.getKey(0);
			if (sn == null)
			{
				throw new IllegalStateException("Heap is broken...");
			}
			
			// Check if we should proceed
			double maxSavings = sn.getMaxSavings();
			if (   (_nonNegative && Double.isInfinite(maxSavings))
				|| (!_nonNegative && maxSavings < 0) 
				|| curSavings + maxSavings >= target)
			{
				for (SolutionNode node : _list)
				{
					node._label = Label.TERMINAL;
				}
				break;
			}
			
			curSavings += maxSavings;
			bestTarget = Math.max(bestTarget, curSavings);
			if (sn.canPushLeft() || sn.canPushRight() || sn.canMakeFly())
			{
				sn.doBestMutation();
			}
			else
			{
				throw new IllegalStateException("The node should not be in the heap anymore!");
			}
		}
		if (curSavings == bestTarget)
		{
			return -1;
		}
		return bestTarget;
	}

	/**
	 * The solution computed by this heuristic
	 * @return
	 */
	public Solution<E> getSolution()
	{
		List<Operation<E>> ops = new ArrayList<>();
		List<E> curList = null;
		E fly = null;
		
		for (SolutionNode sn : _list)
		{
			if (sn._label == Label.TERMINAL)
			{
				if (curList != null)
				{
					curList.add(sn.element);
					Operation<E> op = new Operation<E>(curList,fly);
					ops.add(op);
				}
				curList = new ArrayList<>();
				curList.add(sn.element);
				fly = null;
			}
			else if (sn._label == Label.INTERNAL)
			{
				curList.add(sn.element);
			}
			else if (sn._label == Label.FLY)
			{
				if (fly != null)
				{
					throw new IllegalStateException("Cannot have two flight nodes in one operation!");
				}
				fly = sn.element;	
			}
			else
			{
				throw new IllegalStateException("All simple nodes should have been processed!");
			}
		}
		if (curList.size() > 1)
		{
			Operation<E> op = new Operation<E>(curList,fly);
			ops.add(op);
		}
		return new Solution<E>(_instance,ops);
	}
		
	private enum Label
	{
		SIMPLE, TERMINAL, INTERNAL, FLY;
	}
	
	/**
	 * Solution Node. Combines the logic of the heuristic with the MaxHeap interaction in order
	 * to achieve an efficient speed for the heuristic
	 * @author Paul Bouman
	 *
	 */
	public class SolutionNode implements HeapIndexChangedListener
	{
		public final E element;
		
		private SolutionNode _left;
		private SolutionNode _right;

		protected int _heapIndex;
		
		private Label _label;
		private SolutionNode _flyTo;
		private SolutionNode _flyFrom;
		private double _driveBeforeCost;
		private double _driveAfterCost;
		private double _flyBeforeCost;
		private double _flyAfterCost;
		
		
		/**
		 * Instantiates the Solution Node
		 * @param e the location for this solution node
		 */
		public SolutionNode(E e)
		{
			element = e;
			_label = Label.SIMPLE;
		}
		
		/**
		 * Sets the order of this node in the linked list
		 * @param left the solution node to the left of this one
		 * @param right the solution node to the right of thise one
		 */
		public void makeLink(SolutionNode left, SolutionNode right)
		{
			if (_left != null || _right != null)
			{
				throw new IllegalStateException("Can only set the links once!");
			}
			_left = left;
			_right = right;
		}
		
		/**
		 * Whether this node is currently part of a larger operation
		 * @return whether this is not a simple solution node
		 */
		public boolean isOperation()
		{
			return _label != Label.SIMPLE;
		}
		
		/**
		 * Whether this node can currently be transformed into a fly node
		 * @return whether this can be a fly node
		 */
		public boolean canMakeFly()
		{
			return _label == Label.SIMPLE && _left != null && _right != null && !_instance.isDepot(element);
		}
		
		/**
		 * Whether this node can currently be merged into the larger operation before it
		 * @return whether this node can be merged into the operation to the left of it
		 */
		public boolean canPushRight()
		{
			return _label == Label.SIMPLE && _right != null && _right._label == Label.TERMINAL;
		}
		
		/**
		 * Whether this node can currently be merged into the larger operation after it
		 * @return whether this node can be merged into the operation to the right of it
		 */
		public boolean canPushLeft()
		{
			return _label == Label.SIMPLE && _left != null && _left._label == Label.TERMINAL;
		}
		
		/**
		 * The savings if this node is currently transformed into a fly node
		 * @return the savings if this node is transformed into a fly node
		 */
		public double makeFlySavings()
		{
			if (isOperation())
			{
				throw new IllegalStateException();
			}
			if (element == null || _left == null || _right == null)
			{
				throw new IllegalStateException();
			}
			double cur =  _drive.getContextFreeDistance(_left.element,element)
						+ _drive.getContextFreeDistance(element, _right.element);
			double fly = _fly.getFlyDistance(_left.element, _right.element, element);
			double dr  = _drive.getContextFreeDistance(_left.element, _right.element);
			return cur - Math.max(fly, dr);
		}
		
		/**
		 * Transforms this node into a fly node
		 */
		public void makeFly()
		{
			if (!canMakeFly())
			{
				throw new IllegalStateException();
			}
			
			_label = Label.FLY;
			
			double d = _drive.getContextFreeDistance(_left.element, _right.element);
			_left._driveAfterCost = d;
			_right._driveBeforeCost = d;
			double f = _fly.getFlyDistance(_left.element, _right.element, element);
			_left._flyAfterCost = f;
			_right._flyBeforeCost = f;
			_left._flyTo = this;
			_right._flyFrom = this;
			
			_left._right = _right;
			_right._left = _left;
			
			_flyFrom = _left;
			_flyTo = _right;
			
			_heap.remove(_heapIndex);
			if (_left._label == Label.SIMPLE)
			{
				_heap.remove(_left._heapIndex);
				_left._label = Label.TERMINAL;
			}
			if (_right._label == Label.SIMPLE)
			{
				_heap.remove(_right._heapIndex);
				_right._label = Label.TERMINAL;
			}
			if (_left._left != null && !_left._left.isOperation() && _left._left._label == Label.SIMPLE)
			{
				_heap.update(_left._left._heapIndex, _left._left.getMaxSavings());
			}
			if (_right._right != null && !_right._right.isOperation() && _right._right._label == Label.SIMPLE)
			{
				_heap.update(_right._right._heapIndex, _right._right.getMaxSavings());
			}
		}
		
		
		
		/**
		 * The current cost of the operation before this terminal node
		 * @return the cost of the operation left of this node
		 */
		public double getLeftOperationCost()
		{
			if (_label != Label.TERMINAL || _flyFrom == null)
			{
				throw new IllegalStateException();
			}
			return Math.max(_driveBeforeCost, _flyBeforeCost);
		}
		
		/**
		 * The current cost of the operation after this terminal node
		 * @return the cost of the operation right of this node
		 */
		public double getRightOperationCost()
		{
			if (_label != Label.TERMINAL || _flyTo == null)
			{
				throw new IllegalStateException();
			}
			return Math.max(_driveAfterCost, _flyAfterCost);
		}
		
		/**
		 * The current savings if this node is merged into the operation before it
		 * @return the savings if this node is merge into the operation left of it
		 */
		public double pushLeftSavings()
		{
			double newDrive = _left._driveBeforeCost + _drive.getContextFreeDistance(_left.element, element);
			if (Double.isInfinite(_left._flyBeforeCost))
			{
				throw new IllegalStateException("An operation with infinite costs was created in a "
						+"previous step, although this should never happen.");
			}
			double newFly = _fly.getFlyDistance(_left._flyFrom._flyFrom.element,
					                            element,
					                            _left._flyFrom.element);
			return _left.getLeftOperationCost() - Math.max(newDrive, newFly);
		}
		
		/**
		 * Merges this node into the operation before it
		 */
		public void pushLeft()
		{
			if (!canPushLeft())
			{
				throw new IllegalStateException();
			}
			
			
			double driveBefore = _left._driveBeforeCost + _drive.getContextFreeDistance(_left.element, element);
			double flyBefore = _fly.getFlyDistance(_left._flyFrom._flyFrom.element,
								                    element,
								                    _left._flyFrom.element);

			_left._label = Label.INTERNAL;
			_label = Label.TERMINAL;
			_driveBeforeCost = driveBefore;
			_flyBeforeCost = flyBefore;
			_flyFrom = _left._flyFrom;
			_flyFrom._flyTo = this;
			
			_flyFrom._flyFrom._flyAfterCost = flyBefore;
			_flyFrom._flyFrom._driveAfterCost = driveBefore;
			
			_heap.remove(_heapIndex);
			if (_right != null && !_right.isOperation())
			{
				_heap.update(_right._heapIndex, _right.getMaxSavings());
			}
		}
		
		/**
		 * The current savings if this operation is merged into the operation after it
		 * @return the savings if this operation is merged into the operation right of it
		 */
		public double pushRightSavings()
		{
			double newDrive = _right._driveAfterCost + _drive.getContextFreeDistance(element, _right.element);
			double newFly = _fly.getFlyDistance(element, _right._flyTo._flyTo.element, _right._flyTo.element);
			return _right.getRightOperationCost() - Math.max(newDrive, newFly);
		}

		/**
		 * Merges this node into the operation after it
		 */
		public void pushRight()
		{
			if (!canPushRight())
			{
				throw new IllegalStateException();
			}
			
			double driveAfter = _right._driveAfterCost + _drive.getContextFreeDistance(element, _right.element);
			double flyAfter = _fly.getFlyDistance(element, _right._flyTo._flyTo.element, _right._flyTo.element);
			
			_right._label = Label.INTERNAL;
			_label = Label.TERMINAL;
			_flyTo = _right._flyTo;
			_flyTo._flyFrom = this;
			_flyTo._flyTo._flyBeforeCost = flyAfter;
			_flyTo._flyTo._driveBeforeCost = driveAfter;
			_driveAfterCost = driveAfter;
			_flyAfterCost = flyAfter;
			
			
			_heap.remove(_heapIndex);
			if (_left != null && !_left.isOperation())
			{
				_heap.update(_left._heapIndex, _left.getMaxSavings());
			}
		}

		/**
		 * Computes the maximum possible savings
		 * @return the maximum savings
		 */
		public double getMaxSavings()
		{
			double saving = Double.NEGATIVE_INFINITY;
			if (canMakeFly())
			{
				saving = Math.max(saving, makeFlySavings());
			}
			if (canPushRight())
			{
				saving = Math.max(saving, pushRightSavings());
			}
			if (canPushLeft())
			{
				saving = Math.max(saving, pushLeftSavings());
			}
			return saving;
		}
		
		/**
		 * Performs the best possible operation
		 */
		public void doBestMutation()
		{
			double fs = Double.NEGATIVE_INFINITY;
			double pls = Double.NEGATIVE_INFINITY;
			double prs = Double.NEGATIVE_INFINITY;
			
			if (canMakeFly())
			{
				fs = makeFlySavings();
			}
			if (canPushLeft())
			{
				pls = pushLeftSavings();
			}
			if (canPushRight())
			{
				prs = pushRightSavings();
			}
			
			if (fs >= pls && fs >= prs)
			{
				makeFly();
			}
			else if (Double.isFinite(pls) && pls >= prs)
			{
				pushLeft();
			}
			else if (Double.isFinite(prs))
			{
				pushRight();
			}
			else
			{
				throw new IllegalStateException();
			}
		}
		
		@Override
		public void notifyHeapIndex(int index)
		{
			_heapIndex = index;
		}
		
		@Override
		public String toString()
		{
			return element.toString()+" ["+_label+"]";
		}
	}
	
}
