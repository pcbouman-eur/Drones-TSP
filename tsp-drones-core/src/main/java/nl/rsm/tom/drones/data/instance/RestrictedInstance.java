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
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Predicate;

import nl.rsm.tom.drones.data.Action;
import nl.rsm.tom.drones.data.Distance;

/**
 * Restricted instances are derivatives of a regular instance that adds
 * some rules related to the maximum flight time of the drone before
 * it has to return to the truck. It also allows for the specification
 * of locations that can not be visited/covered by the drone (called
 * no-visit locations) and locations that can not be visited/covered
 * by the drone and where the drone is also not allowed to depart or
 * arrive while leaving or joining the truck (called forbidden
 * locations). The no-visit locations can be considered as locations
 * where a person needs to be present to deliver a package (e.g. if
 * it requires a signature). The forbidden locations can be considered
 * as no fly zones, where it is forbidden by law for drones to fly.
 * 
 * While most methods of the instance interface are deligated to
 * the original instance, the fly distance is modified to return
 * an infinite distance in case the drone violates one of the
 * restrictions that apply to the drone. 
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of the locations within this instance
 * @param <I> the type of the instance that will be restricted.
 */
public class RestrictedInstance<E,I extends Instance<E>> implements Instance<E>
{
	private double maximum;
	private Predicate<E> forbidden;
	private Predicate<E> noVisit;
	
	private I originalInstance;

	/**
	 * Constructor for a restricted instance
	 * @param instance the instance that will be restricted in the new instance
	 * @param droneMax the maximum distance the drone may travel before returning to the truck
	 * @param forbidden a predicate indicating whether a certain location is forbidden to the drone
	 * @param noVisit a predicate indicating whether a certain location can not be covered by the drone.
	 */
	public RestrictedInstance(I instance, double droneMax, Predicate<E> forbidden, Predicate<E> noVisit)
	{
		super();
		this.originalInstance = instance;
		this.maximum = droneMax;
		this.forbidden = forbidden;
		this.noVisit = noVisit;
	}
	
	/**
	 * Derives a restricted instance from an instance such that a maximum flight distance can not be
	 * exceed by the drone without returning to the truck first. The maximum flight distance is
	 * given as a factor of the longest distance between a pair of locations in the network. Since
	 * the drone always flies at most two times before returning to the truck, a maxFactor of 2
	 * implies that all drone flights are possible.
	 * 
	 * @param <E> the type of the locations
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param instance the instance that will be restricted
	 * @param maxFactor a factor that is used to determine the actual maximum distance a drone can fly.
	 *                  this factor is multiplied with the longest distance between a single pair of
	 *                  locations the drone can fly.
	 * @return the restricted instance
	 */
	public static <E,F extends Instance<E>> RestrictedInstance<E,F> restrict(F instance, double maxFactor)
	{
		
		return restrict(instance, maxFactor, 0, true, null);
	}
	
	/**
	 * Derives a restricted instance where random locations are selected to be either forbidden or
	 * no-visit locations for the drone, based on a given probability. If the noFly boolean is
	 * true, all these locations are forbidden, otherwise they are all novisit locations. 
	 * The procedure used to determine which locations are forbidden is to randomly shuffle the
	 * locations and then forbid the first Math.round(forbidFrac * locations.size()) locations.
	 * Thus, the probability is not applied on a per location basis.
	 * 
	 * @param <E> the type of the locations in the instance
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param instance the instance that will be restricted
	 * @param forbidFrac the fraction of locations that will be forbidden or no-visit
	 * @param noFly if true locations are made forbidden, and made no visit if false.
	 * @param ran the random number generator used to shuffle the locations in the process.
	 * @return the restricted instance
	 */
	public static <E,F extends Instance<E>> RestrictedInstance<E,F> restrict(F instance, double forbidFrac,
			boolean noFly, Random ran)
	{
		return restrict(instance,2,forbidFrac,noFly,ran);
	}
	
	/**
	 * Derives a retricted instance where both the maximum flight distance of the drone before it
	 * has to return to the truck is defined, as well as which locations can not be visited by
	 * the drone, or where the drone is not allowed to fly at all, i.e. no-visit and forbidden
	 * locations. The maximum flight distance is expressed as a factor of the longest distance
	 * for a pair of locations in the instance. The number of locations that are deemed no-visit
	 * or forbidden to the drone is determined by a fraction. After shuffling the locations, the
	 * first Math.round(forbidFrac * locations.size()) locations are forbidden.
	 * 
	 * @param <E> the type of the locations in the instance
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param instance the instance that will be restricted
	 * @param maxFactor a factor that is used to determine the actual maximum distance a drone can fly.
	 *                  this factor is multiplied with the longest distance between a single pair of
	 *                  locations the drone can fly.
	 * @param forbidFrac the fraction of locations that will be forbidden or no-visit
	 * @param noFly if true locations are made forbidden, and made no visit if false.
	 * @param ran the random number generator used to shuffle the locations in the process.
	 * @return the restricted instance
	 */
	public static <E,F extends Instance<E>> RestrictedInstance<E,F> restrict(F instance,
			double maxFactor, double forbidFrac, boolean noFly, Random ran)
	{
		if (maxFactor < 0)
		{
			throw new IllegalArgumentException("The maximum factor should be in the range [0,2]");
		}
		
		double maxFlight = Double.POSITIVE_INFINITY;
		if (maxFactor < 2)
		{
			Distance<E> fly = instance.getFlyDistance();
			double max = 0;
			for (E loc1 : instance)
			{
				for (E loc2 : instance)
				{
					double dist = fly.getContextFreeDistance(loc1, loc2);
					max = Math.max(max, dist);
				}
			}
			maxFlight = maxFactor * max;
		}
		Set<E> forbid = Collections.emptySet();
		if (forbidFrac > 0)
		{
			ArrayList<E> locs = new ArrayList<>(instance.getLocations());
			Collections.shuffle(locs, ran);
			int maxIndex = (int)Math.round(locs.size() * forbidFrac);
			forbid = new HashSet<>(locs.subList(0, Math.min(locs.size(), maxIndex)));
		}
		if (noFly)
		{
			return new RestrictedInstance<>(instance, maxFlight, forbid::contains, s -> false);
		}
		else
		{
			return new RestrictedInstance<>(instance, maxFlight, s -> false, forbid::contains);
		}
	}
	
	
	@Override
	public Distance<E> getFlyDistance()
	{
		final Distance<E> dist = originalInstance.getFlyDistance();
		return (from, to, action1, action2, cumul) -> {
			if (action1 == Action.UNDEFINED || action2 == Action.UNDEFINED)
			{
				throw new IllegalArgumentException("The drone distance for a restricted instance requires "
						+ "the purpose of a drone flight.");
			}
			if (forbidden.test(from) || forbidden.test(to) ||       // drone in no flight zone
				(action1 == Action.VISIT && noVisit.test(from)) ||  // visiting a non-visit node
				(action2 == Action.VISIT && noVisit.test(to)))      // visiting a non-visit node
			{
				return Double.POSITIVE_INFINITY;
			}
			double d = dist.getDistance(from, to, action1, action2, cumul);
			if (cumul + d > maximum) // does the new cumulative distance exceed the maximum?
			{
				return Double.POSITIVE_INFINITY;
			}
			return d;
		};
	}
	
	@Override
	public Distance<E> getDriveDistance()
	{
		// No restrictions apply to the drive instance
		return originalInstance.getDriveDistance();
	}

	@Override
	public Instance<E> getSubInstance(Predicate<? super E> retain)
	{
		return new RestrictedInstance<>(originalInstance.getSubInstance(retain), maximum, forbidden, noVisit);
	}

	/**
	 * Check whether it is forbidden to fly near a location. This means that
	 * the location can not be covered by the drone, and must be covered
	 * by the truck. It also means that the drone can not depart from the truck
	 * at this location, or arrive to join the truck at this location. The only
	 * way the drone can be at a location that is forbidden, is when it is
	 * attached to the truck.
	 * @param loc the location for which we want to check if it is forbidden 
	 * @return whether a location is a no-fly zone
	 */
	public boolean isForbidden(E loc)
	{
		return forbidden.test(loc);
	}
	
	/**
	 * Check whether it is not possible for the drone to visit
	 * (deliver a package at) a location. This means that this
	 * location needs to be covered by the truck.
	 * @param loc the location for which we want to perform the check
	 * @return whether the location cannot accept a package via the drone
	 */
	public boolean isNoVisit(E loc)
	{
		return noVisit.test(loc);
	}
	
	/**
	 * Get the maximum distance that the drone can fly before it has to return to the truck for a recharge.
	 * @return the maximum distance to be flown by the drone.
	 */
	public double getMaximumDistance()
	{
		return maximum;
	}
	
	/**
	 * Get the original instance on top of which the restrictions are added
	 * @return the original instance
	 */
	public I getOriginalInstance()
	{
		return originalInstance;
	}
	
	@Override
	public int getNodeCount() {
		return originalInstance.getNodeCount();
	}

	@Override
	public int getLocationCount() {
		return originalInstance.getLocationCount();
	}

	@Override
	public E getDepot() {
		return originalInstance.getDepot();
	}

	@Override
	public boolean isDepot(E e) {
		return originalInstance.isDepot(e);
	}

	@Override
	public List<E> getLocations() {
		return originalInstance.getLocations();
	}

	@Override
	public E getLocation(int index) {
		return originalInstance.getLocation(index);
	}

	@Override
	public Iterator<E> iterator() {
		return originalInstance.iterator();
	}
}
