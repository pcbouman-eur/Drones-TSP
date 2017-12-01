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

import nl.rsm.tom.drones.data.Vec2D;

/**
 * This is a utility class that can be used to generate instances
 * that useful in testcases.
 * @author Paul Bouman
 */

public class TestCaseGenerator
{
	/**
	 * Generates a Geometric Instance with 1+2n points on a line, with the depot in the middle
	 * @param n The number of points to be added both left and right to the depot
	 * @param alpha The relative speed of the drone
	 * @return The instance
	 */
	public static GeometricInstance buildLineInstance(int n, double alpha)
	{
		Vec2D depot = new Vec2D(0,0,"0");
		ArrayList<Vec2D> locs = new ArrayList<>(2*n);
		for (int t=1; t <= n; t++)
		{
			Vec2D neg = new Vec2D(-t,0,"-"+t);
			Vec2D pos = new Vec2D(t,0,""+t);
			locs.add(neg);
			locs.add(pos);
		}
		GeometricInstance gi = new GeometricInstance(depot,locs,alpha);
		return gi;
	}
}
