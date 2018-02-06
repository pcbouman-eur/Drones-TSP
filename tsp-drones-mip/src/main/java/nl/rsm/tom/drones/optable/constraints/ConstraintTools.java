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
package nl.rsm.tom.drones.optable.constraints;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.data.instance.RestrictedInstance;

public class ConstraintTools
{
	/**
	 * Utility method that can be used to define a maximum range and maximum cardinality
	 * @param i the instance for which to generate the constraints
	 * @param maxRange a maximum range that the drone is allowed to fly, given as a factor
	 *                 of the maximum distance in the instance. Thus, if maxRange is 1.5
	 *                 and the maximum single leg distance in the instance is 20, the 
	 *                 actual maximum range the drone is allowed to fly will be 30. Thus,
	 *                 a maxRange factor of 2 or greater will not impose a restriction on
	 *                 the drone.
	 * @param maxCardinality the maximum number of truck-only locations that can be in a
	 *                       single operation. A negative value for maxCardinality will
	 *                       result in no restriction on the number of truck-only locations.
	 * @return a list of OpConstraints based on the provided arguments.
	 */
	public static <E> List<OpConstraint<Object,Object>> buildConstraints(Instance<E> i, double maxRange, int maxCardinality)
	{
		List<OpConstraint<Object,Object>> res = new ArrayList<>();
		if (Double.isFinite(maxRange) && !(i instanceof RestrictedInstance))
		{
			Distance<E> fd = i.getFlyDistance();
			double maxDist = 0;
			for (E f : i)
			{
				for (E t : i)
				{
					//double dist = fd.getContextFreeDistance(f, t, 0);
					double dist1 = fd.getDepartVisit(f, t);
					double dist2 = fd.getVisitArrive(f, t, 0);
					if (Double.isFinite(dist1))
					{
						maxDist = Math.max(maxDist, dist1);
					}
					if (Double.isFinite(dist2))
					{
						maxDist = Math.max(maxDist, dist2);
					}
				}
			}
			res.add(new MaxFlyConstraint(maxDist * maxRange));
		}
		if (maxCardinality >= 0)
		{
			OpConstraint<Object,Object> con = new CardinalityConstraint(maxCardinality);
			res.add(con);
		}
		return res;
	}
}
