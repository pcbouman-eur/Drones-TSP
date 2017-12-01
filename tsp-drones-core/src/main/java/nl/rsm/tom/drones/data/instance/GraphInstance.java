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
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.SimpleDistance;

/**
 * Instance type that uses nodes in a graph to model the locations,
 * and edges or arcs between these locations for travel options.
 * The instances uses an all-pairs shortest path computation to
 * determine the distances between arbitrary pairs of locations.
 * @author Paul Bouman
 *
 * @param <E> the type of locations/nodes in the graph.
 */
public class GraphInstance<E> implements Instance<E>
{
	private boolean _bidirectional;
	private E _depot;
	private List<E> _locations;
	private Map<E,Map<E,Edge>> _edges;
	private HashSet<Edge> _edgeSet;
	private List<Edge> _edgeList;
	
	private Map<E,Integer> _indices;
	private double [][] _driveDistances;
	private double [][] _flyDistances;
	private boolean _recalcDistances;
	
	private Distance<E> _customDriveDistance;
	private Distance<E> _customFlyDistance;
	
	/**
	 * Constructor for a sparse instance.
	 * Sparse instances are mutable, so only the locations are fixed in advance.
	 * @param depot The depot node
	 * @param locations The location node
	 */
	public GraphInstance(E depot, List<E> locations)
	{
		this(depot,locations,true);
	}
	
	/**
	 * Constructor for a sparse instance.
	 * Sparse instances are mutable, so only the locations are fixed in advance.
	 * @param depot The depot node
	 * @param locations The location node
	 * @param bidirectional Whether distances are defined in a bidirectional manner
	 */
	public GraphInstance(E depot, List<E> locations, boolean bidirectional)
	{
		init(depot,locations);
		_bidirectional = bidirectional;
		int n = _locations.size() + 1;
		_flyDistances = new double[n][n];
		_driveDistances = new double[n][n];
		
		_recalcDistances = true;
	}
	
	/**
	 * Constructor which allows for the definition of custom distance measures.
	 * @param depot The depot of this instance
	 * @param locations The list of locations to be visited
	 * @param drive The distance measure for the driving
	 * @param fly The distance measure for flying
	 */
	public GraphInstance(E depot, List<E> locations, Distance<E> drive, Distance<E> fly)
	{
		init(depot,locations);
		_customDriveDistance = drive;
		_customFlyDistance = fly;
	}
	
	private void init(E depot, List<E> locations)
	{
		_depot = depot;
		_locations = new ArrayList<E>(locations);
		_indices = new HashMap<>();
		_edges = new HashMap<>();
		_edgeSet = new HashSet<>();
		_edgeList = new ArrayList<>();
		_indices.put(depot, 0);
		for (E loc : _locations)
		{
			_edges.put(loc, new HashMap<>());
			_indices.put(loc, _indices.size());
		}
		_edges.put(depot, new HashMap<>());
		_recalcDistances = true;		
	}
	
	/**
	 * Adds a bidirectional distance between two nodes
	 * @param left One node
	 * @param right Another node
	 * @param drive The driving distance
	 * @param fly The flying distance
	 */
	
	public void addDistance(E left, E right, double drive, double fly)
	{
		if (_edges.containsKey(left) && _edges.get(left).containsKey(right))
		{
			throw new IllegalStateException("Cannot define the same distance twice!");
		}

		Edge e = new Edge(left,right,drive,fly,_bidirectional);
		_edges.get(left).put(right, e);
		if (!_edgeSet.contains(e))
		{
			_edgeList.add(e);
		}
		_edgeSet.add(e);
		if (_bidirectional)
		{
			if (_edges.containsKey(right) && _edges.get(right).containsKey(left))
			{
				throw new IllegalStateException("The opposite direction is already defined!");
			}	
			_edges.get(right).put(left, e);
		}
		_recalcDistances = true;
	}	
	
	/**
	 * Gives the set of all edges which are defined in this model
	 * @return the set of defined edges
	 */
	public List<Edge> getEdges()
	{
		return Collections.unmodifiableList(_edgeList);
	}
	
	/**
	 * @return Whether distances in this instance are defined in a bidirectional manner.
	 */
	public boolean isBidirectional()
	{
		return _bidirectional;
	}
	
	@Override
	public int getLocationCount()
	{
		return _locations.size();
	}

	@Override
	public E getDepot()
	{
		return _depot;
	}

	@Override
	public List<E> getLocations()
	{
		return Collections.unmodifiableList(_locations);
	}

	@Override
	public Distance<E> getDriveDistance()
	{
		if (_customDriveDistance != null)
		{
			return _customDriveDistance;
		}
		computeDistances();
		SimpleDistance<E> result = (E e1, E e2) -> _driveDistances[_indices.get(e1)][_indices.get(e2)];
		return result;
	}

	@Override
	public Distance<E> getFlyDistance()
	{
		if (_customFlyDistance != null)
		{
			return _customFlyDistance;
		}
		computeDistances();
		SimpleDistance<E> result = (E e1, E e2) -> _flyDistances[_indices.get(e1)][_indices.get(e2)];
		return result;
	}
	
	// Does an all pairs shortest path computation;
	private void computeDistances()
	{
		if (!_recalcDistances)
		{
			return;
		}
		int n = _driveDistances.length;
		for (int i=0; i < n; i++)
		{
			for (int j=0; j < n; j++)
			{
				if (i==j)
				{
					_driveDistances[i][j] = 0;
					_flyDistances[i][j] = 0;
				}
				else
				{
					_driveDistances[i][j] = Double.POSITIVE_INFINITY;
					_flyDistances[i][j] = Double.POSITIVE_INFINITY;
				}
			}
		}
		for (Edge e : _edgeSet)
		{
			int i = _indices.get(e.left);
			int j = _indices.get(e.right);
			_driveDistances[i][j] = e.drive;
			_flyDistances[i][j] = e.fly;
			if (e.bidirectional)
			{
				_driveDistances[j][i] = e.drive;
				_flyDistances[j][i] = e.fly;	
			}
		}
		allPairsShortestPath(_driveDistances);
		allPairsShortestPath(_flyDistances);
		_recalcDistances = false;
	}
	
	/**
	 * Applies the Floyd-Warshall algorithm to find the shortest
	 * distance between every pair of nodes.
	 * @param matrix a matrix of edge distances
	 */
	
	private void allPairsShortestPath(double [][] matrix)
	{
		int n = matrix.length;
		for (int k=0; k < n; k++)
		{
			boolean improve = false;
			for (int i=0; i < n; i++)
			{
				for (int j=0; j < n; j++)
				{
					if (matrix[i][j] > matrix[i][k] + matrix[k][j])
					{
						matrix[i][j] = matrix[i][k] + matrix[k][j];
						improve = true;
					}
				}
			}
			if (!improve)
			{
				break;
			}
		}
	}

	/**
	 * Edge inner class
	 */
	public class Edge
	{
		public final E left;
		public final E right;
		public final double fly;
		public final double drive;
		public final boolean bidirectional;
		
		private Edge(E l, E r, double d, double f, boolean b)
		{
			left = l;
			right = r;
			fly = f;
			drive = d;
			bidirectional = b;
		}
		
		/**
		 * Given one end of this edge, returns the other end.
		 * @param first The end of this edge that should not be returned.
		 * @return The other end of this edge.
		 */
		public E getOther(E first)
		{
			if (left.equals(first))
			{
				return right;
			}
			if (right.equals(first))
			{
				return left;
			}
			throw new IllegalArgumentException("The argument should be at least one of the vertices!");
		}
	}

	@Override
	public Instance<E> getSubInstance(Predicate<? super E> retain)
	{
		List<E> newLocs = _locations.stream().filter(retain).collect(Collectors.toList());
		return new GraphInstance<E>(_depot, newLocs, getDriveDistance(), getFlyDistance());
	}
}
