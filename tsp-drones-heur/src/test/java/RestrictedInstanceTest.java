

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.RandomGenerators;
import nl.rsm.tom.drones.data.instance.RestrictedInstance;
import nl.rsm.tom.drones.data.io.InstanceIO;
import nl.rsm.tom.drones.data.io.RestrictedInstanceIO;
import nl.rsm.tom.drones.solver.fixedorder.FixedOrderDPSolver;
import nl.rsm.tom.drones.solver.fixedorder.FixedOrderHeuristicSolver;
import nl.rsm.tom.drones.solver.fixedorder.IterativeImprovementSolver;
import nl.rsm.tom.drones.solver.fixedorder.MurrayChuFixedOrderSolver;
import nl.rsm.tom.drones.solver.heuristic.MSTSolver;

/**
 * Some unit tests that check whether the restricted instances work as expected
 * @author Paul Bouman
 *
 */

public class RestrictedInstanceTest
{
	
	private static List<RestrictedInstance<Vec2D, GeometricInstance>> generate(final int [] ns, final int reps)
	{
		final Random ran = new Random(54321);
		final double [] flys = {0.5, 0.8};
		
		final double [] facs = { 0.5, 1.5};
		final double [] probs = { 0.2, 0.6 };
		
		List<RestrictedInstance<Vec2D, GeometricInstance>> result = new ArrayList<>();
		for (int n : ns)
		{
			for (double fly : flys)
			{
				GeometricInstance instance = RandomGenerators.randomInstance(n, ran, fly);
				for (int rep=0; rep < reps; rep++)
				{
					for (double fac : facs)
					{
						for (double prob : probs)
						{
							RestrictedInstance<Vec2D, GeometricInstance> ri;
							ri = RestrictedInstance.restrict(instance, fac, prob, true, ran);
							result.add(ri);
							ri = RestrictedInstance.restrict(instance, fac, prob, false, ran);
							result.add(ri);
						}
					}
					
				}
			}
		}
		return result;
	}
	
	@Test
	public void testRandomInstances()
	{
		final int [] ns = {10, 25};
		for (RestrictedInstance<Vec2D, GeometricInstance> ri : generate(ns,3))
		{
			String instanceData = RestrictedInstanceIO.writeRestrictions(ri) + "\n"
					            + InstanceIO.writeInstance(ri.getOriginalInstance());
			RestrictedInstance<Vec2D, GeometricInstance> ri2;
			ri2 = RestrictedInstanceIO.readRestrictedInstance(instanceData,
						InstanceIO::readGeometricInstanceFromData);
			String instanceData2 = RestrictedInstanceIO.writeRestrictions(ri2) + "\n"
		            + InstanceIO.writeInstance(ri2.getOriginalInstance());
			assertTrue("Reading and writing a restricted instance works correctly", instanceData.equals(instanceData2));
		}	
	}

	@Test
	public void testSolvers()
	{
		MSTSolver<Vec2D> mst = new MSTSolver<>();
		IterativeImprovementSolver<Vec2D> dpHeur = new IterativeImprovementSolver<>(new FixedOrderDPSolver<>(), true, true, true);
		IterativeImprovementSolver<Vec2D> greedyHeur = new IterativeImprovementSolver<>(new FixedOrderHeuristicSolver<>(), true, true, true);
		IterativeImprovementSolver<Vec2D> mcHeur = new IterativeImprovementSolver<>(new MurrayChuFixedOrderSolver<>(), true, true, true);
		
		final int [] ns = {4,5,6};
		List<RestrictedInstance<Vec2D, GeometricInstance>> list = generate(ns,3);
		for (RestrictedInstance<Vec2D, GeometricInstance> ri : list)
		{
			Solution<Vec2D> mstSol, dpHeurSol, greedyHeurSol, mcSol;
			
			mstSol = mst.solve(ri);
			dpHeurSol = dpHeur.solve(ri, mstSol);
			mcSol = mcHeur.solve(ri, mstSol);
			greedyHeurSol = greedyHeur.solve(ri, mstSol);
			
			assertTrue("MST yields a feasible solution for a restricted instance", mstSol.isFeasible());
			assertTrue("DP Heuristic yields a feasible solution for a restricted instance", dpHeurSol.isFeasible());
			assertTrue("MC Heuristic yields a feasible solution for a restricted instance", mcSol.isFeasible());
			assertTrue("Greedy Heuristic yields a feasible solution for a restricted instance", greedyHeurSol.isFeasible());			
		}
	}
	
}
