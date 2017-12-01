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
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import nl.rsm.tom.drones.data.instance.Instance;

/**
 * An operation is a joint movement of the truck and the drone.
 * At the start of an operation, the drone departs from the truck
 * and goes its own way, covering a single location, before it
 * meets the truck again at the end of the operation.
 * While the drone is in flight, the truck may move directly to
 * the final destination of the operation, or visit any number of
 * locations on the way.
 * It is assumed the truck and the drone wait for each other at the
 * end of the operation. When they are joined again, the operation
 * ends.
 *
 * @author Paul Bouman
 * 
 * @param <E> The type of location for which this is an operation.
 */

public class Operation<E>
{
	private final E _start;
	private final E _end;
	private final List<E> _drive;
	private final E _fly;
	
	private Distance<E> _lastDriveDist;
	private Distance<E> _lastFlyDist;
	
	private double _lastDrive;
	private double _lastFly;

	/**
	 * Constructor without internal drive nodes
	 * @param start The start node
	 * @param end The end node
	 * @param fly The fly node
	 */
	public Operation(E start, E end, E fly)
	{
		this(start,Collections.emptyList(),end,fly);
	}
	
	/**
	 * Constructor with no internal drive or fly nodes
	 * @param start The start node
	 * @param end The fly node
	 */
	public Operation(E start, E end)
	{
		this(start,Collections.emptyList(),end);
	}
	
	/**
	 * Constructor with no fly nodes where the start and end nodes
	 * are included in the drive nodes list.
	 * @param drive a list of locations the truck visits
	 */
	public Operation(List<E> drive)
	{
		this(drive,null);
	}
	
	/**
	 * Constructor whereh the start and end nodes are included in
	 * the drive nodes list.
	 * @param drive The list of drive nodes
	 * @param fly The fly node
	 */
	public Operation(List<E> drive, E fly)
	{
		this(drive.get(0), drive.subList(1,drive.size()-1), drive.get(drive.size()-1), fly);
	}
	
	/**
	 * Constructor without a fly node, where the start and end nodes
	 * are not included in the list of internal drive nodes
	 * @param start The start node
	 * @param drive A list of drives nodes (excluding start and end)
	 * @param end The end node
	 */
	public Operation(E start, List<E> drive, E end)
	{
		this(start,drive,end,null);
	}
	
	/**
	 * Constructor which specificies the start and end seperately
	 * from the internal drive nodes.
	 * @param start The start node
	 * @param drive The list of drive nodes excluding the start and end nodes
	 * @param end The end node
	 * @param fly The fly node
	 */
	public Operation(E start, List<E> drive, E end, E fly)
	{
		_start = start;
		_drive = new ArrayList<E>(drive);
		_end = end;
		_fly = fly;
	}
	
	/**
	 * Computes cost based on the distance measures in the instance
	 * @param i The instance which distance measures will be used
	 * @return The computed cost
	 */
	public double getCost(Instance<E> i)
	{
		return getCost(i.getDriveDistance(), i.getFlyDistance());
	}
	
	/**
	 * Computes cost based on two distance measures
	 * @param drive The distance measure for driving
	 * @param fly The distance measure for flying
	 * @return The computed cost
	 */
	public double getCost(Distance<E> drive, Distance<E> fly)
	{
		getDriveCost(drive);
		getFlyCost(fly);
		_lastDriveDist = drive;
		_lastFlyDist = fly;
		return Math.max(_lastDrive,_lastFly);
	}
	
	/**
	 * Computes the cost for the truck based on an instance
	 * @param i The instance
	 * @return The computed cost
	 */
	public double getDriveCost(Instance<E> i)
	{
		return getDriveCost(i.getDriveDistance());
	}
	
	/**
	 * Computes the cost for the truck
	 * @param drive The drive distance
	 * @return The computed cost
	 */
	public double getDriveCost(Distance<E> drive)
	{
		
		if (_lastDriveDist != drive)
		{
			/*
			_lastDrive = 0;
			E cur = _start;
			for (E next : _drive)
			{
				_lastDrive += drive.getDistance(cur, next);
				cur = next;
			}
			_lastDrive += drive.getDistance(cur, _end);
			_lastDriveDist = drive;
			*/
			_lastDrive = drive.getPathDistance(_start, _end, _drive);
			_lastDriveDist = drive;
		}
		return _lastDrive;
	}
	
	/**
	 * Computes the cost for the drone based on an instance
	 * @param i The instance
	 * @return The computed costs
	 */
	public double getFlyCost(Instance<E> i)
	{
		return getFlyCost(i.getFlyDistance());
	}
	
	/**
	 * Computes the cost for the drone
	 * @param fly The fly distance
	 * @return The computed cost
	 */
	public double getFlyCost(Distance<E> fly)
	{
		if (_lastFlyDist != fly)
		{
			_lastFly = 0;
			if (_fly != null)
			{
				_lastFly = fly.getFlyDistance(_start, _end, _fly);
			}
		}
		_lastFlyDist = fly;
		return _lastFly;
	}
	
	/**
	 * Computes the waiting costs for the truck based on an instance
	 * @param i The instance
	 * @return The waiting cost for the truck
	 */
	public double getTruckWait(Instance<E> i)
	{
		return getTruckWait(i.getDriveDistance(), i.getFlyDistance());
	}
	
	/**
	 * Computes the waiting costs for the truck
	 * @param drive The drive distance
	 * @param fly The fly distance
	 * @return The waiting cost for the truck
	 */
	public double getTruckWait(Distance<E> drive, Distance<E> fly)
	{
		getCost(drive,fly);
		return Math.max(_lastDrive, _lastFly) - _lastDrive;
	}
	
	/**
	 * Computer the waiting costs for the drone given an instance
	 * @param i The instance
	 * @return The waiting costs for the drone
	 */
	public double getDroneWait(Instance<E> i)
	{
		return getDroneWait(i.getDriveDistance(), i.getFlyDistance());
	}
	
	/**
	 * Computes the waiting costs for the drone
	 * @param drive The drive distance
	 * @param fly The fly distance
	 * @return The waiting cost for the drone
	 */
	public double getDroneWait(Distance<E> drive, Distance<E> fly)
	{
		if (!hasFly())
		{
			return 0;
		}
		getCost(drive,fly);
		return Math.max(_lastDrive, _lastFly) - _lastFly;
	}
	
	/**
	 * Whether this operation contains a fly node
	 * @return True if this operation contains a drone action 
	 */
	public boolean hasFly()
	{
		return _fly != null;
	}
	
	/**
	 * The fly node, if it is defined. Null otherwise
	 * @return When this operation has no fly action, this is null. Otherwise it is the fly node.
	 */
	public E getFly()
	{
		return _fly;
	}
	
	/**
	 * The path that will be performed by the drone
	 * @return A list of three nodes (start, end and fly) in case of flying. Empty otherwise
	 */
	public List<E> getFlyPath()
	{
		if (_fly == null)
		{
			return Collections.emptyList();
		}
		List<E> res = new ArrayList<E>(3);
		res.add(_start);
		res.add(_fly);
		res.add(_end);
		return res;
	}
	
	/**
	 * A set of all nodes covered by this operation
	 * @return a set of nodes covered by this operation
	 */
	public Set<E> getNodeSet()
	{
		Set<E> result = new LinkedHashSet<>();
		result.add(_start);
		if (_fly != null)
		{
			result.add(_fly);
		}
		result.addAll(_drive);
		result.add(_end);
		return result;
	}
	
	/**
	 * The path of nodes that will be visited (in order) by the truck
	 * @return The list containing the driving path.
	 */
	public List<E> getDrivePath()
	{
		List<E> res = new ArrayList<E>(_drive.size()+2);
		res.add(_start);
		res.addAll(_drive);
		res.add(_end);
		return res;
	}
	
	/**
	 * The internal nodes are those excluding the start and end node.
	 * The fly node may or may not be included
	 * @param includeFly Whether the fly node should be included.
	 * @return A list of the internal nodes.
	 */
	public List<E> getInternalNodes(boolean includeFly)
	{
		ArrayList<E> res = new ArrayList<>(_drive);
		if (includeFly && hasFly())
		{
			res.add(_fly);
		}
		return res;
	}
	
	/**
	 * The start node; this is where this operation starts
	 * @return The start node
	 */
	public E getStart()
	{
		return _start;
	}

	/**
	 * The end node; this is where this operation ends
	 * @return The end node
	 */
	public E getEnd()
	{
		return _end;
	}
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((_drive == null) ? 0 : _drive.hashCode());
		result = prime * result + ((_end == null) ? 0 : _end.hashCode());
		result = prime * result + ((_fly == null) ? 0 : _fly.hashCode());
		result = prime * result + ((_start == null) ? 0 : _start.hashCode());
		return result;
	}

	@SuppressWarnings("rawtypes")
	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Operation other = (Operation) obj;
		if (_drive == null) {
			if (other._drive != null)
				return false;
		} else if (!_drive.equals(other._drive))
			return false;
		if (_end == null) {
			if (other._end != null)
				return false;
		} else if (!_end.equals(other._end))
			return false;
		if (_fly == null) {
			if (other._fly != null)
				return false;
		} else if (!_fly.equals(other._fly))
			return false;
		if (_start == null) {
			if (other._start != null)
				return false;
		} else if (!_start.equals(other._start))
			return false;
		return true;
	}
	
	@Override
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		sb.append("{");
		sb.append("start=");
		sb.append(_start);
		sb.append(", drive=");
		sb.append(_drive.toString());
		sb.append(", fly=");
		sb.append(_fly);
		sb.append(", end=");
		sb.append(_end);
		sb.append("}");
		return sb.toString();
	}

}