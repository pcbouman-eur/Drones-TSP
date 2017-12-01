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
import java.util.HashMap;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import nl.rsm.tom.drones.data.SimpleDistance;

/**
 * Instance type that depends on a full matrix of distances. This allows for
 * in depth control over all distances in the instance.
 * @author Paul Bouman
 *
 * @param <E> the type of locations between which the distances are defined.
 */
public class MatrixInstance<E> implements Instance<E>
{
	
	private final HashMap<E,Integer> _indexMap;
	private final ArrayList<E> _locs;
	
	private final double [][] _driveDistances;
	private final double [][] _flyDistances;
	

	/**
	 * Converts any instance to a MatrixInstance.
	 * @param instance The instance to be converted
	 */
	public MatrixInstance(Instance<E> instance)
	{
		this(instance.getDepot(), instance.getLocations(), 
				(SimpleDistance<E>) instance.getDriveDistance(),
				(SimpleDistance<E>) instance.getFlyDistance());
	}
	
	/**
	 * Instance which stores a full matrix for both the drive and fly distances.
	 * @param depot The depot
	 * @param locations The locations that need to be visited
	 * @param drive The distance measure for driving. 
	 * @param fly The distance measure for flying.
	 */
	
	public MatrixInstance(E depot, Iterable<E> locations, SimpleDistance<E> drive, SimpleDistance<E> fly)
	{
		_indexMap = new HashMap<>();
		_locs = new ArrayList<>();
		
		_indexMap.put(depot, 0);
		_locs.add(depot);
		int n = 1;
		for (E loc : locations)
		{
			int index = _indexMap.size();
			_indexMap.put(loc, index);
			_locs.add(loc);
			n++;
		}

		_driveDistances = new double[n][n];
		_flyDistances = new double[n][n];

		for (int x=0; x < n; x++)
		{
			for (int y=0; y < n; y++)
			{
				E f = _locs.get(x);
				E t = _locs.get(y);
				double dd = drive.getSimpleDistance(f,t);
				_driveDistances[x][y] = dd;
				double fd = fly.getSimpleDistance(f,t);
				_flyDistances[x][y] = fd;
			}
		}
	}
	

	@Override
	public int getLocationCount()
	{
		return _locs.size() - 1;
	}

	@Override
	public E getDepot()
	{
		return _locs.get(0);
	}

	@Override
	public List<E> getLocations() {
		return new ArrayList<E>(_locs.subList(1, _locs.size()));
	}

	@Override
	public SimpleDistance<E> getDriveDistance()
	{
		return (E f, E t) -> _driveDistances[_indexMap.get(f)][_indexMap.get(t)];
	}

	@Override
	public SimpleDistance<E> getFlyDistance()
	{
		return (E f, E t) -> _flyDistances[_indexMap.get(f)][_indexMap.get(t)];
	}

	@Override
	public Instance<E> getSubInstance(Predicate<? super E> retain)
	{
		List<E> newLocs = _locs.stream().filter(retain).collect(Collectors.toList());
		return new MatrixInstance<E>(getDepot(), newLocs, getDriveDistance(), getFlyDistance());
	}
}
