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

import java.util.Collections;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.SimpleDistance;
import nl.rsm.tom.drones.data.Vec2D;

/**
 * Class that defines an instance where locations are
 * points in a two dimensional plane, and the distances
 * are Euclidean, multiplied with a constant speed
 * factor for either vehicle type
 * @author Paul Bouman
 *
 */

public class GeometricInstance implements Instance<Vec2D>
{
	private final Vec2D _depot;
	private final List<Vec2D> _locations;
	
	private final double _driveSpeed;
	private final double _flySpeed;
	
	private final SimpleDistance<Vec2D> _drive;
	private final SimpleDistance<Vec2D> _fly;
	
	/**
	 * A constructor for a geometric instance which only specifies the relative fly speed.
	 * A geometric instance is defined by a set of points in a plane.
	 * @param depot The depot of the instance
	 * @param locs The locations that need to be visited
	 * @param fly The relative speed of the drone.
	 */
	
	public GeometricInstance(Vec2D depot, List<Vec2D> locs, double fly)
	{
		this(depot,locs,1,fly);
	}
	
	/**
	 * A constructor for a geometric instance with both speeds defined.
	 * A geometric instance instance is define by a set of points in a plane.
	 * @param depot The depot of instance
	 * @param locs The locations that need to be visited
	 * @param drive The speed of the truck
	 * @param fly The speed of the drone
	 */
	public GeometricInstance(Vec2D depot, List<Vec2D> locs, double drive, double fly)
	{
		_depot = depot;
		_locations = locs;
		
		_drive = (Vec2D p1, Vec2D p2) -> euclideanDistance(p1,p2)*drive; 
		_fly = (Vec2D p1, Vec2D p2) -> euclideanDistance(p1,p2)*fly;
		
		_driveSpeed = drive;
		_flySpeed = fly;
	}
	
	/**
	 * Derives a new geometric instance by modifying only the drive and fly speeds
	 * @param i The original instance
	 * @param drive The new drive speed
	 * @param fly The new fly speed
	 */
	public GeometricInstance(GeometricInstance i, double drive, double fly)
	{
		this(i._depot,i._locations,drive,fly);
	}
	
	/**
	 * @return The speed of the drone
	 */
	public double getFlySpeed()
	{
		return _flySpeed;
	}
	
	/**
	 * @return The speed of the truck
	 */
	public double getDriveSpeed()
	{
		return _driveSpeed;
	}
	
	
	@Override
	public int getLocationCount()
	{
		return _locations.size();
	}

	@Override
	public Vec2D getDepot()
	{
		return _depot;
	}

	@Override
	public List<Vec2D> getLocations()
	{
		return Collections.unmodifiableList(_locations);
	}

	@Override
	public Distance<Vec2D> getDriveDistance()
	{
		return _drive;
	}

	@Override
	public Distance<Vec2D> getFlyDistance()
	{
		return _fly;
	}
	
	/**
	 * Uses the Pythagoras Theorem to compute the distance between two points.
	 * @param p1 the first point
	 * @param p2 the second point
	 * @return the euclidean distance
	 */
	
	public static double euclideanDistance(Vec2D p1, Vec2D p2)
	{
		double dx = p1.x - p2.x;
		double dy = p1.y - p2.y;
		return Math.sqrt(dx*dx + dy*dy);
	}

	@Override
	public Instance<Vec2D> getSubInstance(Predicate<? super Vec2D> retain)
	{
		List<Vec2D> newLocs = _locations.stream().filter(retain).collect(Collectors.toList());
		return new GeometricInstance(_depot,newLocs,_driveSpeed,_flySpeed);
	}
	
}
