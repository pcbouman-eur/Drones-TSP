package nl.rsm.tom.drones.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import nl.rsm.tom.drones.data.instance.RandomGenerators;

/**
 * Class that can be used to generate instances for the purpose of running unit tests.
 * @author Paul Bouman
 *
 */
public class TestInstances
{
	
	/**
	 * Generates a number of random instances of size 20, where the drone is twice
	 * as fast as the truck. This method is deterministic (i.e. the random seed is fixed)
	 * @param count the number of instances to genereate
	 * @return a list of geometric instances
	 */
	public static List<GeometricInstance> getGeometricInstances(int count)
	{
		Random ran = new Random(1234);
		List<GeometricInstance> result = new ArrayList<>();
		for (int t=0; t < count; t++)
		{
			GeometricInstance instance = RandomGenerators.randomInstance(20, ran, 0.5);
			result.add(instance);
		}
		return result;
	}

	/**
	 * Genereates a number of random graph instances, with 20 nodes, a 20% probability that
	 * an edge is generated between two locations and the drone twice as fast as the truck.
	 * This method is deterministic (i.e. the random seed is fixed)
	 * @param count the number of instances to generate.
	 * @return a list of instances
	 */
	public static List<GraphInstance<Integer>> getGraphInstances(int count)
	{
		Random ran = new Random(1234);
		List<GraphInstance<Integer>> result = new ArrayList<>();
		for (int t=0; t < count; t++)
		{
			GraphInstance<Integer> instance = RandomGenerators.randomInstance(20, 0.2, ran, 0.5);
			result.add(instance);
		}
		return result;
	}
	
	/**
	 * Genereates a number of matrix instances, of which half are geometric instances converted to
	 * a matrix, and half are graph instances converted to a matrix. All instances have 20 locations.
	 * This method is deterministic (i.e. the random seed is fixed)
	 * @param count the number of instances to generate
	 * @return a list of instances
	 */
	public static List<MatrixInstance<?>> getMatrixInstances(int count)
	{
		List<MatrixInstance<?>> result = new ArrayList<>();
		result.addAll(getMatrixGraphInstances((int)Math.ceil(count/2d)));
		result.addAll(getMatrixGeometricInstances(count/2));
		return result;
	}
	
	/**
	 * Generates a number of graph instances and converts them to matrix instances
	 * This method is deterministic (i.e. the random seed is fixed)
	 * @param count the number of instances to generate
	 * @return a list of instances
	 */
	public static List<MatrixInstance<Integer>> getMatrixGraphInstances(int count)
	{
		Random ran = new Random(1234);
		List<MatrixInstance<Integer>> result = new ArrayList<>();
		for (int t=0; t < count; t++)
		{
			GraphInstance<Integer> instance = RandomGenerators.randomInstance(20, 0.2, ran, 0.5);
			MatrixInstance<Integer> mi = new MatrixInstance<Integer>(instance);
			result.add(mi);
		}
		return result;
	}


	/**
	 * Genereates a number of geometric instances and converts them to matrix instances
	 * This method is deterministic (i.e. the random seed is fixed) 
	 * @param count  the number of instances to generate
	 * @return a list of instances
	 */
	public static List<MatrixInstance<Vec2D>> getMatrixGeometricInstances(int count)
	{
		Random ran = new Random(1234);
		List<MatrixInstance<Vec2D>> result = new ArrayList<>();

		for (int t=0; t < count; t++)
		{
			GeometricInstance instance = RandomGenerators.randomInstance(20, ran, 0.5);
			MatrixInstance<Vec2D> mi = new MatrixInstance<Vec2D>(instance);
			result.add(mi);
		}
		return result;
	}
}
