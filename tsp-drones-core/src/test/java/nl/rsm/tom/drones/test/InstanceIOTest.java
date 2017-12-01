package nl.rsm.tom.drones.test;


import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import nl.rsm.tom.drones.data.instance.RandomGenerators;
import nl.rsm.tom.drones.data.io.InstanceIO;

import org.junit.Test;

public class InstanceIOTest {

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
	
	public static List<MatrixInstance<?>> getMatrixInstances(int count)
	{
		List<MatrixInstance<?>> result = new ArrayList<>();
		result.addAll(getMatrixGraphInstances((int)Math.ceil(count/2d)));
		result.addAll(getMatrixGeometricInstances(count/2));
		return result;
	}
	
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
	
	
	@Test
	public void testGeometricInstances()
	{
		for (GeometricInstance i : getGeometricInstances(100))
		{
			GeometricInstance instance = i;
			String firstDump = InstanceIO.instanceToString(instance);
			instance = InstanceIO.readGeometricInstanceFromData(firstDump);
			String secondDump = InstanceIO.instanceToString(instance);
			assertEquals("Testing Geometric Instance IO", firstDump,secondDump);
		}
	}
	
	@Test
	public void testGraphInstances()
	{
		for (GraphInstance<Integer> i : getGraphInstances(100))
		{
			GraphInstance<Integer> instance = i;
			String firstDump = InstanceIO.instanceToString(instance);
			instance = InstanceIO.readGraphInstanceFromData(firstDump);
			String secondDump = InstanceIO.instanceToString(instance);
			assertEquals("Testing Graph Instance IO", firstDump,secondDump);
		}
	}
	
	@Test
	public void testMatrixInstances()
	{
		for (MatrixInstance<?> i : getMatrixInstances(100))
		{
			MatrixInstance<?> mi = i;
			String firstDump = InstanceIO.instanceToString(mi);
			mi = InstanceIO.readMatrixInstanceFromData(firstDump);
			String secondDump = InstanceIO.instanceToString(mi);
			assertEquals("Testing Matrix Instance IO", firstDump,secondDump);
		}

	}

}
