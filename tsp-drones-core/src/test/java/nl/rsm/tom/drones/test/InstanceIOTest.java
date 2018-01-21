package nl.rsm.tom.drones.test;


import static nl.rsm.tom.drones.util.TestInstances.getGeometricInstances;
import static nl.rsm.tom.drones.util.TestInstances.getGraphInstances;
import static nl.rsm.tom.drones.util.TestInstances.getMatrixInstances;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import nl.rsm.tom.drones.data.io.InstanceIO;

/**
 * Unit tests for input and output, which check whether instances can be
 * written to file and then read back again.
 * @author Paul Bouman
 *
 */
public class InstanceIOTest {

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
