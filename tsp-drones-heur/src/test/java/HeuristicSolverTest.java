

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import nl.rsm.tom.drones.data.instance.RandomGenerators;
import nl.rsm.tom.drones.solver.algorithms.FixedOrderDP;
import nl.rsm.tom.drones.solver.fixedorder.FixedOrderDPSolver;
import nl.rsm.tom.drones.solver.fixedorder.IterativeImprovementSolver;
import nl.rsm.tom.drones.solver.fixedorder.MurrayChuFixedOrderSolver;
import nl.rsm.tom.drones.solver.heuristic.MSTSolver;
import nl.rsm.tom.drones.solver.heuristic.RandomSolver;
import nl.rsm.tom.drones.util.DelaunayTriangulation;
import nl.rsm.tom.drones.util.TestInstances;

/**
 * Unit tests for a number of heuristic solvers
 * @author Paul Bouman
 *
 */
public class HeuristicSolverTest
{
	@Test
	public void testMSTSolutions()
	{
		MSTSolver<Vec2D> msts = new MSTSolver<>();
		for (GeometricInstance i : TestInstances.getGeometricInstances(100))
		{
			GraphInstance<Vec2D> gi = DelaunayTriangulation.convert(i);
			Solution<Vec2D> sol = msts.solve(gi);
			assertTrue("Checking feasibility of MST solution for a Graph Instance",sol.isFeasible());			
		}
		MSTSolver<Integer> mstsi = new MSTSolver<>();
		for (GraphInstance<Integer> i : TestInstances.getGraphInstances(100))
		{
			Solution<Integer> sol = mstsi.solve(i);
			assertTrue("Checking feasibility of MST solution for a Graph Instance",sol.isFeasible());
		}
		for (MatrixInstance<Integer> mi : TestInstances.getMatrixGraphInstances(100))
		{
			Solution<Integer> sol = mstsi.solve(mi);
			assertTrue("Checking feasibility of MST solution for a Matrix Instance",sol.isFeasible());	
		}
	}
	
	@Test
	public void testFixedOrderDP()
	{
		MSTSolver<Vec2D> msts = new MSTSolver<>();
		for (GeometricInstance i : TestInstances.getGeometricInstances(100))
		{
			GraphInstance<Vec2D> gi = DelaunayTriangulation.convert(i);
			Solution<Vec2D> sol = msts.solve(gi);
			FixedOrderDP<Vec2D> fodp = new FixedOrderDP<Vec2D>(sol);
			Solution<Vec2D> sol2 = fodp.getSolution();
			assertTrue("Checking feasibility of heuristic solution", sol2.isFeasible());
			assertTrue("Checking whether the heuristic did not make things worse", sol2.getTotalCost() <= sol.getTotalCost());
		}
	}
	
	
	@Test
	public void testIterativeImprovementHeuristic()
	{
		MSTSolver<Vec2D> msts = new MSTSolver<>();
		FixedOrderDPSolver<Vec2D> dp = new FixedOrderDPSolver<Vec2D>();
		IterativeImprovementSolver<Vec2D> iis = new IterativeImprovementSolver<Vec2D>(dp, false, true, false);
		for (GeometricInstance i : TestInstances.getGeometricInstances(100))
		{
			GraphInstance<Vec2D> gi = DelaunayTriangulation.convert(i);
			Solution<Vec2D> sol = msts.solve(gi);
			Solution<Vec2D> sol2 = iis.solve(gi, sol);
			assertTrue("Checking feasibility of heuristic solution", sol2.isFeasible());
			assertTrue("Checking whether the heuristic did not make things worse", sol2.getTotalCost() <= sol.getTotalCost());
		}
	}

	@Test
	public void testMurrayChuHeuristic()
	{
		int n = 100;
		
		MSTSolver<Vec2D> msts = new MSTSolver<>();
		MurrayChuFixedOrderSolver<Vec2D> mch = new MurrayChuFixedOrderSolver<>();
		int improveCount = 0;
		for (GeometricInstance i : TestInstances.getGeometricInstances(n))
		{
			GraphInstance<Vec2D> gi = DelaunayTriangulation.convert(i);
			Solution<Vec2D> sol = msts.solve(gi);
			
			Solution<Vec2D> sol2 = mch.solve(gi, sol);
			assertTrue("Checking feasibility of heuristic solution", sol2.isFeasible());
			assertTrue("Checking whether the heuristic did not make things worse", sol2.getTotalCost() <= sol.getTotalCost());
			if (sol2.getTotalCost() < sol.getTotalCost() && sol2.getFlightNodes().size() > 0)
			{
				improveCount++;
			}
		}
		
		assertTrue("The MurrayChu Heuristic provides better solutions than just the initial tour", improveCount >= n * 0.8);
	}
	
	@Test
	public void testRandomSolutions()
	{
		long seed = 1234;
		Random r = new Random(seed);
		for (int t=0; t < 100; t++)
		{
			GeometricInstance i = RandomGenerators.randomInstance(10, r, 0.5);
			for (int k=0; k < 10; k++)
			{
				RandomSolver<Vec2D> rs = new RandomSolver<>(r.nextLong());
				Solution<Vec2D> sol = rs.solve(i);
				assertTrue("Checking feasibility of random solution",sol.isFeasible());
			}
		}
	}
	

	
}
