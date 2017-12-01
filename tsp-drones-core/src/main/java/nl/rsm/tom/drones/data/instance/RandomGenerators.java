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
import java.util.List;
import java.util.Random;

import org.apache.commons.math3.random.MersenneTwister;
import org.apache.commons.math3.random.RandomDataGenerator;

import nl.rsm.tom.drones.data.Vec2D;

/**
 * This is a utility class that contains a number of ways to generate random instances.
 * @author Paul Bouman
 *
 */

public class RandomGenerators
{
	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,1] x [0,1] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param seed the random seed of the prng
	 * @param fly the speed of the drone
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, long seed, double fly)
	{
		return randomInstance(n, new Random(seed), fly);
	}

	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,k] x [0,k] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param seed the random seed of the prng
	 * @param fly the speed of the drone
	 * @param k the size of the grid
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, long seed, double fly, int k)
	{
		return randomInstance(n, new Random(seed), fly, k);
	}

	
	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,1] x [0,1] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param ran the random number generator to be used
	 * @param fly the relative speed of the drone
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, Random ran, double fly)
	{
		return randomInstance(n,ran,1,fly);
	}

	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,k] x [0,k] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param ran the random number generator to be used
	 * @param fly the relative speed of the drone
	 * @param k the size of the grid
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, Random ran, double fly, int k)
	{
		return randomInstance(n,ran,1,fly,k);
	}

	
	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,k] x [0,k] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param ran the random number generator to be used
	 * @param car the random number generator to be used
	 * @param fly the relative speed of the drone
	 * @param k the size of the grid
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, Random ran, double car, double fly, int k)
	{
		Vec2D depot = new Vec2D(ran.nextDouble(), ran.nextDouble(), "depot");
		ArrayList<Vec2D> locations = new ArrayList<>();
		for (int t=0; t < n; t++)
		{
			Vec2D p;
			if (k > 1)
			{
				p = new Vec2D(ran.nextInt(k), ran.nextInt(k), "loc"+(t+1));
			}
			else
			{
				p = new Vec2D(ran.nextDouble()*k, ran.nextDouble()*k, "loc"+(t+1));
			}
			locations.add(p);
		}
		return new GeometricInstance(depot,locations,car,fly);		
	}
	
	/**
	 * Creates a random geometric instance with points spread uniformly in the
	 * [0,1] x [0,1] plane.
	 * @param n the number of locations (excluding the depot) to be generated
	 * @param ran the random number generator to be used
	 * @param car the random number generator to be used
	 * @param fly the relative speed of the drone
	 * @return a random instance
	 */
	public static GeometricInstance randomInstance(int n, Random ran, double car, double fly)
	{
		return randomInstance(n,ran,car,fly,1);		
	}
	
	/**
	 * Generates a graph with random edges with random drive and fly distances.
	 * To make sure the graph is always connected, a hamiltonion path is generated first.
	 * Then, on top of that edges are added with a given probability.
	 * @param n The number of locations to be visited
	 * @param prob The probability that an edge between a pair of nodes is generated
	 * @param ran a random number generator to used
	 * @param fly the relative speed of the drone compare to the truck
	 * @return a graph instance
	 */
	
	public static GraphInstance<Integer> randomInstance(int n, double prob, Random ran, double fly)
	{
		Integer depot = 0;
		List<Integer> list = new ArrayList<Integer>();
		for (int k=1; k < n; k++)
		{
			list.add(k);
		}
		GraphInstance<Integer> gi = new GraphInstance<>(depot,list,true);
		for (int k=1; k < n; k++)
		{
			gi.addDistance(k-1, k, ran.nextDouble(), ran.nextDouble()*fly);
		}
		for (int i=0; i < n; i++)
		{
			for (int j=i+2; j < n; j++)
			{
				if (ran.nextDouble()<=prob)
				{
					gi.addDistance(i, j, ran.nextDouble(), ran.nextDouble()*fly);
				}
			}
		}
		return gi;
	}
	
	/**
	 * Random 1 Center instance; uses polar coordinates to generate points
	 * @param n The number of nodes to be generated
	 * @param ran The random number generator to be used
	 * @param stddev The standard deviation around the center
	 * @param fly The speed of the drone
	 * @return A geometric instance
	 */
	public static GeometricInstance random1CenterInstance(int n, Random ran, double stddev, double fly)
	{
		RandomDataGenerator rdg = new RandomDataGenerator(new MersenneTwister(ran.nextLong()));
		ArrayList<Vec2D> list = new ArrayList<>();
		for (int t=1; t < n; t++)
		{
			double r = rdg.nextGaussian(0, stddev);
			double ang = rdg.nextUniform(0, 2 * Math.PI);
			double x = r * Math.cos(ang);
			double y = r * Math.sin(ang);
			Vec2D vec = new Vec2D(x,y,"v"+t);
			list.add(vec);
		}
		double r = rdg.nextGaussian(0, stddev);
		double ang = rdg.nextUniform(0, 2 * Math.PI);
		double x = r * Math.cos(ang);
		double y = r * Math.sin(ang);
		Vec2D depot = new Vec2D(x,y,"depot");
		return new GeometricInstance(depot, list, fly);
		}
	
	
	/**
	 * Random 2 Center instance; uses polar coordinates to generate points
	 * @param n The number of nodes to be generated
	 * @param ran The random number generator to be used
	 * @param centerProb The probability that a node is generated in center 1
	 * @param stddev1 The standard deviation of center 1
	 * @param stddev2 The standard deviation of center 2
	 * @param centerDist The distance of center 2 from center 1
	 * @param fly The speed of the drone
	 * @return A random geometric instance
	 */
	public static GeometricInstance random2CenterInstance(int n, Random ran, double centerProb,
						double stddev1, double stddev2, double centerDist, double fly)
	{
		RandomDataGenerator rdg = new RandomDataGenerator(new MersenneTwister(ran.nextLong()));
		ArrayList<Vec2D> list = new ArrayList<>();
		for (int t=1; t < n; t++)
		{
			boolean b = rdg.nextUniform(0, 1) < centerProb;
			double stddev = b ? stddev1 : stddev2;
			double r = rdg.nextGaussian(0, stddev);
			double ang = rdg.nextUniform(0, 2 * Math.PI);
			double x = r * Math.cos(ang);
			double y = r * Math.sin(ang);
			if (!b)
			{
				x += centerDist;
			}
			String str = b ? "v"+t : "u"+t;
			Vec2D vec = new Vec2D(x,y,str);
			list.add(vec);
		}
		double r = rdg.nextGaussian(0, stddev1);
		double ang = rdg.nextUniform(0, 2 * Math.PI);
		double x = r * Math.cos(ang);
		double y = r * Math.sin(ang);
		Vec2D depot = new Vec2D(x,y,"depot");
		return new GeometricInstance(depot, list, fly);
	}
}
