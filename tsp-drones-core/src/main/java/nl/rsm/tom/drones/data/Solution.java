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

package nl.rsm.tom.drones.data;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import nl.rsm.tom.drones.data.instance.Instance;

/**
 * A Solution consists of a list of operations defined for a specific instance.
 * This class supports a number of operations that allow us to check whether
 * a solution is feasible for the instance and whether the solution is strict
 * (i.e. visits every location exactly once)
 * @author Paul Bouman
 * @param <E> The type of the locations
 */
public class Solution<E> implements Iterable<Operation<E>>
{
	private Instance<E> _instance;
	private List<Operation<E>> _operations;
	private double _totalCost;
	private double _totalTruckCost;
	private double _totalDroneCost;
	private double _totalTruckWait;
	private double _totalDroneWait;
	private double _maxTruckCost;
	private double _maxDroneCost;
	private double _maxTruckWait;
	private double _maxDroneWait;
	private Set<E> _fly;
	private Set<E> _drive;
	
	/**
	 * A constructor which builds a Solution for a specific instance
	 * based on a list of operations
	 * @param i The instance for which this is a solution
	 * @param ops The list of operations which make up the solution
	 */
	
	public Solution(Instance<E> i, List<Operation<E>> ops)
	{
		_operations = new ArrayList<>(ops.size());
		_instance = i;
		_totalCost = 0;
		_totalTruckCost = 0;
		_totalDroneCost = 0;
		_totalTruckWait = 0;
		_totalDroneWait = 0;
		_maxTruckCost = 0;
		_maxDroneCost = 0;
		_maxTruckWait = 0;
		_maxDroneWait = 0;
		_fly = new HashSet<E>();
		_drive = new HashSet<E>();
		
		for (Operation<E> op : ops)
		{
			_totalCost += op.getCost(_instance);			
			_totalTruckCost += op.getDriveCost(_instance);
			_totalDroneCost += op.getFlyCost(_instance);
			_totalTruckWait += op.getTruckWait(_instance);
			_totalDroneWait += op.getDroneWait(_instance);
			_maxTruckCost = Math.max(_maxTruckCost, op.getDriveCost(_instance));
			_maxDroneCost = Math.max(_maxDroneCost, op.getFlyCost(_instance));
			_maxTruckWait = Math.max(_maxTruckWait, op.getTruckWait(_instance));
			_maxDroneWait = Math.max(_maxDroneWait, op.getDroneWait(_instance));
			_operations.add(op);
			if (op.hasFly())
			{
				_fly.add(op.getFly());
			}
			_drive.addAll(op.getDrivePath());
		}
	}

	/**
	 * Builds a Truck-only solution for a certain order of the locations
	 * @param order The order that will be used
	 * @param i The instance for which this is a solution
	 */
	public Solution(List<E> order, Instance<E> i)
	{
		this(i, convert(order));
	}
	
	/**
	 * Converts a list of locations to a list of operations
	 * @param order The order of the locations
	 * @return A list of operations
	 */
	private static <F> List<Operation<F>> convert(List<F> order)
	{
		ArrayList<Operation<F>> ops = new ArrayList<>();
		F prev = null;
		for (F loc : order)
		{
			if (prev != null)
			{
				Operation<F> op = new Operation<F>(prev,loc);
				ops.add(op);
			}
			prev = loc;
		}
		return ops;
	}

	
	/**
	 * The instance for which this solution was defined
	 * @return The instance
	 */
	public Instance<E> getInstance()
	{
		return _instance;
	}
	
	/**
	 * Whether the operations in this solution actually form a tour
	 * @return Whether this is a tour
	 */
	public boolean isTour()
	{
		E f = _operations.get(0).getStart();
		E cur = f;
		for (Operation<E> o : _operations)
		{
			if (!o.getStart().equals(cur))
			{
				return false;
			}
			cur = o.getEnd();
		}
		return cur.equals(f);
	}
	
	/**
	 * Checks whether all locations in the instance are covered by
	 * this solution
	 * @return Whether all locations are covered.
	 */
	public boolean coversAll()
	{
		 return getNonCovered().isEmpty();
	}
	
	/**
	 * Checks whether this solution starts and ends at the depot of the instance
	 * @return Whether this solution starts and ends at the depot
	 */
	public boolean depotCorrect()
	{
		Operation<E> first = _operations.get(0);
		Operation<E> last = _operations.get(_operations.size()-1);
		E d = _instance.getDepot();
		return first.getStart().equals(d) && last.getEnd().equals(d);
	}
	
	/**
	 * Checks whether this solution is a tour, covers all locations
	 * and starts and ends at the depot
	 * @return Whether this solution is feasible
	 */
	public boolean isFeasible()
	{
		return isTour() && coversAll() && depotCorrect();
	}
	
	/**
	 * Checks whether this solution is strict. This means that every
	 * location is visited precisely once, either by the drone or by
	 * the car. For this purpose, only the start location of every
	 * operation is considered.
	 * @return Whether this solution is strict
	 */
	public boolean isStrict()
	{
		HashSet<E> covered = new HashSet<E>();
		for (Operation<E> op : _operations)
		{
			if (covered.contains(op.getStart()))
			{
				return false;
			}
			covered.add(op.getStart());
			for (E e : op.getInternalNodes(true))
			{
				if (covered.contains(e))
				{
					return false;
				}
				covered.add(e);
			}
		}
		return true;
	}
	
	/**
	 * Generates the set of locations in the instance which are not
	 * covered by this solution (i.e. they are not visited at all)
	 * @return The set of operations which are not covered.
	 */
	public Collection<E> getNonCovered()
	{
		HashSet<E> res = new HashSet<E>(_instance.getLocations());
		res.add(_instance.getDepot());
		res.removeAll(_fly);
		res.removeAll(_drive);
		return res;
	}

	/**
	 * The list of operations which make up this solution
	 * @return The list of operations
	 */
	public List<Operation<E>> getOperations()
	{
		return Collections.unmodifiableList(_operations);
	}

	/**
	 * Returns the total cost of this solution
	 * @return the total cost
	 */
	public double getTotalCost()
	{
		return _totalCost;
	}
	
	/**
	 * Returns the maximum cost for the drone
	 * @return the maximum cost for the drone
	 */
	public double getMaxDroneCost()
	{
		return _maxDroneCost;
	}
	
	/**
	 * Returns the maximum cost for the truck
	 * @return the maximum cost for the truck
	 */
	public double getMaxTruckCost()
	{
		return _maxTruckCost;
	}
	
	/**
	 * Returns the cost for the drone
	 * @return The cost for the drone
	 */
	public double getDroneCost()
	{
		return _totalDroneCost;
	}
	
	/**
	 * Returns the cost for the truck
	 * @return The cost for the truck
	 */
	public double getTruckCost()
	{
		return _totalTruckCost;
	}
	
	/**
	 * Returns the waiting time for the drone
	 * @return the waiting time for the drone
	 */
	public double getDroneWait()
	{
		return _totalDroneWait;
	}

	/**
	 * Returns the waiting time for the truck
	 * @return the waiting time for the truck
	 */
	public double getTruckWait()
	{
		return _totalTruckWait;
	}
	
	/**
	 * Returns the maximum waiting time for the truck
	 * @return the maximum waiting time for the truck
	 */
	public double getMaxTruckWait()
	{
		return _maxTruckWait;
	}

	/**
	 * Returns the maximum waiting time for the truck
	 * @return the maximum waiting time for the truck
	 */
	public double getMaxDroneWait()
	{
		return _maxDroneWait;
	}

	
	/**
	 * Returns the order in which nodes are visited in this solution.
	 * @return The order of the nodes
	 */
	public List<E> getOrder()
	{
		ArrayList<E> order = new ArrayList<>();
		for (Operation<E> op : _operations)
		{
			if (op.hasFly())
			{
				throw new IllegalStateException("Cannot get the order if there are flight nodes...");
			}
			if (order.isEmpty())
			{
				order.add(op.getStart());
			}
			for (E loc : op.getInternalNodes(true))
			{
				order.add(loc);
			}
			order.add(op.getEnd());
		}
		return order;
	}
	
	/**
	 * Generates a list of all flight nodes in this solution
	 * @return a list of flight nodes
	 */
	public List<E> getFlightNodes()
	{
		ArrayList<E> result = new ArrayList<>();
		for (Operation<E> op : _operations)
		{
			if (op.hasFly())
			{
				result.add(op.getFly());
			}
		}
		return result;
	}
	
	/**
	 * Generates a list of all internal nodes in this solution
	 * @param includeFly Whether a flight node should be considered internal
	 * @return The list of the nodes
	 */
	public List<E> getInternalNodes(boolean includeFly)
	{
		ArrayList<E> result = new ArrayList<>();
		for (Operation<E> op : _operations)
		{
			result.addAll(op.getInternalNodes(includeFly));
		}
		return result;
	}
	
	/**
	 * Returns the number of operations
	 * @return the number of operations in this solution
	 */
	public int getOperationCount()
	{
		return _operations.size();
	}
	
	public Solution<E> simplify()
	{
		List<Operation<E>> res = new ArrayList<>();
		Set<E> covered = new HashSet<>();
		
		for (Operation<E> op : this)
		{
			List<E> drive = op.getDrivePath();
			Iterator<E> it = drive.iterator();
			boolean first = true;
			while (it.hasNext() && drive.size() > 2)
			{
				E loc = it.next();
				if (!first && it.hasNext() && covered.contains(loc))
				{
					it.remove();
				}
				else
				{
					covered.add(loc);
				}
				first = false;
			}
			if (op.hasFly() && !covered.contains(op.getFly()))
			{
				Operation<E> newOp = new Operation<>(drive,op.getFly());
				res.add(newOp);
			}
			else
			{
				E prev = null;
				for (E loc : drive)
				{
					if (prev != null)
					{
						Operation<E> newOp = new Operation<E>(prev,loc);
						res.add(newOp);
					}
					prev = loc;
				}
			}
		}
		return new Solution<E>(_instance,res);
	}
	
	/**
	 * Creates a list of the location in the order they are visited by the truck.
	 * @return the list of locations visited by the truck
	 */
	public List<E> getDriveOrderList()
	{
		List<E> result = new ArrayList<E>();
		result.add(_operations.get(0).getStart());
		for (Operation<E> op : _operations)
		{
			result.addAll(op.getInternalNodes(false));
			result.add(op.getEnd());
		}
		return result;
	}
	
	/**
	 * Builds a map with statistics of the current solution.
	 * The map is built in such a way that in can be easily
	 * serialized as json data for future analysis.
	 * @return a map with statistics related to this solution
	 */
	public Map<? extends String, ? extends Object> getStatsMap()
	{
		TreeMap<String,Object> res = new TreeMap<>();
		res.put("SolutionValue", getTotalCost());
		res.put("MaxDroneCost", getMaxDroneCost());
		res.put("MaxTruckCost", getMaxTruckCost());
		res.put("DroneCost", getDroneCost());
		res.put("TruckCost", getTruckCost());
		res.put("DroneWait", getDroneWait());
		res.put("TruckWait", getTruckWait());
		res.put("MaxTruckWait", getMaxTruckWait());
		res.put("MaxDroneWait", getMaxDroneWait());
		res.put("OperationCount", getOperationCount());
		res.put("TruckNodes", getInternalNodes(false).size());
		res.put("DroneNodes", getFlightNodes().size());
		return res;
	}
	
	@Override
	public Iterator<Operation<E>> iterator()
	{
		return Collections.unmodifiableList(_operations).iterator();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((_drive == null) ? 0 : _drive.hashCode());
		result = prime * result + ((_fly == null) ? 0 : _fly.hashCode());
		result = prime * result
				+ ((_instance == null) ? 0 : _instance.hashCode());
		result = prime * result
				+ ((_operations == null) ? 0 : _operations.hashCode());
		long temp;
		temp = Double.doubleToLongBits(_totalCost);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		@SuppressWarnings("rawtypes")
		Solution other = (Solution) obj;
		if (_drive == null) {
			if (other._drive != null)
				return false;
		} else if (!_drive.equals(other._drive))
			return false;
		if (_fly == null) {
			if (other._fly != null)
				return false;
		} else if (!_fly.equals(other._fly))
			return false;
		if (_instance == null) {
			if (other._instance != null)
				return false;
		} else if (!_instance.equals(other._instance))
			return false;
		if (_operations == null) {
			if (other._operations != null)
				return false;
		} else if (!_operations.equals(other._operations))
			return false;
		if (Double.doubleToLongBits(_totalCost) != Double
				.doubleToLongBits(other._totalCost))
			return false;
		return true;
	}
	
	@Override
	public String toString()
	{
		return "Solution, total cost: "+_totalCost+", operations: "+_operations;
	}


}
