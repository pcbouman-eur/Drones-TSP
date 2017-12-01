package nl.rsm.tom.drones.test;


import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import nl.rsm.tom.drones.data.io.SolutionIO;
import nl.rsm.tom.drones.solver.heuristic.RandomSolver;

import org.junit.Test;



public class SolutionIOTest
{
	
	@Test
	public void testRandomSolutionIO()
	{
		long seed = 1234;
		for (GeometricInstance gi : InstanceIOTest.getGeometricInstances(100))
		{
			RandomSolver<Vec2D> rs = new RandomSolver<>(seed);
			Solution<Vec2D> s = rs.solve(gi);
			runIOTest(s);
		}
		for (GraphInstance<Integer> gi : InstanceIOTest.getGraphInstances(100))
		{
			RandomSolver<Integer> rs = new RandomSolver<>(seed);
			Solution<Integer> s = rs.solve(gi);
			runIOTest(s);
		}
		for (MatrixInstance<Integer> mi : InstanceIOTest.getMatrixGraphInstances(100))
		{
			RandomSolver<Integer> rs = new RandomSolver<Integer>(seed);
			Solution<Integer> s = rs.solve(mi);
			runIOTest(s);
		}
		for (MatrixInstance<Vec2D> mi : InstanceIOTest.getMatrixGeometricInstances(100))
		{
			RandomSolver<Vec2D> rs = new RandomSolver<Vec2D>(seed);
			Solution<Vec2D> s = rs.solve(mi);
			runIOTest(s);
		}
	}
	
	@Test
	public void testSingleOperationSolutionIO()
	{
		for (GeometricInstance gi : InstanceIOTest.getGeometricInstances(100))
		{
			Vec2D depot = gi.getDepot();
			List<Vec2D> points = gi.getLocations();
			Operation<Vec2D> op = new Operation<>(depot,points,depot);
			List<Operation<Vec2D>> ops = new ArrayList<>();
			ops.add(op);
			Solution<Vec2D> sol = new Solution<Vec2D>(gi,ops);
			runIOTest(sol);
		}		
	}
	
	@Test
	public void testAtomicOperationSolutionIO()
	{
		for (GeometricInstance gi : InstanceIOTest.getGeometricInstances(100))
		{
			Vec2D depot = gi.getDepot();
			List<Operation<Vec2D>> ops = new ArrayList<>();
			Vec2D cur = depot;
			for (Vec2D p : gi.getLocations())
			{
				Operation<Vec2D> op = new Operation<>(cur,p);
				ops.add(op);
				cur = p;
			}
			ops.add(new Operation<>(cur,depot));
			Solution<Vec2D> sol = new Solution<Vec2D>(gi,ops);
			runIOTest(sol);
		}		
		
	}
	
	public void runIOTest(Solution<?> s)
	{
		String sol = SolutionIO.solutionToString(s);
		Solution<?> read = SolutionIO.readSolutionFromData(s.getInstance(), sol);
		assertEquals(s,read);
	}
}
