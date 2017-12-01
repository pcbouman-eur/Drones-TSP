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

import java.util.List;

/**
 * This is a general distance measure interface that can be provided to algorithms
 * that require a distance measure. It is always assumed that there are seperate
 * distance measures for the truck and the drone, so a seperate implementation
 * of this interface can be provided for each vehicle type.
 * 
 * @author Paul Bouman
 *
 * @param <E> The type of the locations this distance measure operates on.
 */
public interface Distance<E>
{
	/**
	 * This function should return the distance between two locations.
	 * Typically, it is assumed that getDistance(a,b) == getDistance(b,a)
	 * and that the triangle inequality holds, so
	 * getDistance(a,c) &lt;= getDistance(a,b) + getDistance(b,c)
	 * for any a, b and c.
	 * In general, distance implementations can ignore the actions, unless
	 * the time for delivery, or combining the truck and drone, or the
	 * launch of drone needs to be modeled.
	 * @param from the origin location
	 * @param to the destination location
	 * @param fromAction the action performed at the beginning of this trip
	 * @param toAction the action performed at the end of this trip
	 * @param priorDistance the distance prior to this trip
	 * @return the distance
	 */
	
	public double getDistance(E from, E to, Action fromAction, Action toAction, double priorDistance);
	
	/**
	 * Get the distance assuming the vehicle departs and the visits a location
	 * @param from the location the vehicle departs from
	 * @param to the location the vehicle will visit
	 * @return the distance
	 */
	
	public default double getDepartVisit(E from, E to)
	{
		return getDistance(from, to, Action.DEPARTURE, Action.VISIT, 0);
	}

	/**
	 * Get the distance assuming the vehicle departs from one location and arrives at 
	 * another location
	 * @param from the departure location
	 * @param to the arrival location
	 * @return the distance
	 */
	public default double getDepartArrive(E from, E to)
	{
		return getDistance(from, to, Action.DEPARTURE, Action.ARRIVAL, 0);
	}

	/**
	 * Get the distance assuming the vehicle performs two visits in a row.
	 * @param from The first location to visit
	 * @param to The second location to visit
	 * @param prior The prior distance travelled since the last departure
	 * @return the distance
	 */
	public default double getVisitTwice(E from, E to, double prior)
	{
		return getDistance(from, to, Action.VISIT, Action.VISIT, prior);
	}

	/**
	 * Get the distance for the vehicle moving from a visit location to
	 * join another vehicle
	 * @param from the location that was visited
	 * @param to the location where the vehicle will meet another vehicle
	 * @param prior the distance traveled before
	 * @return the distance
	 */
	public default double getVisitArrive(E from, E to, double prior)
	{
		return getDistance(from, to, Action.VISIT, Action.ARRIVAL, prior);
	}
	
	/**
	 * Get the distance if the actions are unknown
	 * @param from the origin location 
	 * @param to the destination location 
	 * @return the distance
	 */
	public default double getContextFreeDistance(E from, E to)
	{
		return getDistance(from, to, Action.UNDEFINED, Action.UNDEFINED, -1);
	}
	
	/**
	 * Get the distance assuming the actions are unknown,
	 * but the vehicle has travelled a prior distance.
	 * @param from the origin location
	 * @param to the destination location 
	 * @param prior distance travel prior to this movement
	 * @return the distance of this movement
	 */
	public default double getContextFreeDistance(E from, E to, double prior)
	{
		return getDistance(from, to, Action.UNDEFINED, Action.UNDEFINED, prior);
	}
	
	/**
	 * Convenience method for drone operations, that involve a departure,
	 * a visit and an arrival location.	
	 * @param from the location to depart from
	 * @param to the location to arrive at
	 * @param fly the location to visit
	 * @return the distance
	 */
	public default double getFlyDistance(E from, E to, E fly)
	{
		double step1 = getDepartVisit(from, fly);
		return step1 + getVisitArrive(fly, to, step1);
	}
	
	/**
	 * Compute the distance for a path for a path of with an arbitrary number
	 * of intermediate locations which are to be visited
	 * @param start the departure location
	 * @param end the arrival location
	 * @param intermediate the intermediate locations to be visited
	 * @return the total path distance
	 */
	public default double getPathDistance(E start, E end, List<E> intermediate)
	{
		double result = 0;
		if (intermediate.isEmpty())
		{
			return getDepartArrive(start, end);
		}
		E prev = null;
		for (E e : intermediate)
		{
			if (prev == null)
			{
				result += getDepartVisit(start, e);
			}
			else
			{
				result += getVisitTwice(prev, e, result);
			}
			prev = e;
		}
		result += getVisitArrive(prev, end, result);
		return result;
	}
}
