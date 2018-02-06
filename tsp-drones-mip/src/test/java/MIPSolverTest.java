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
import static org.junit.Assert.assertEquals;
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
import nl.rsm.tom.drones.solver.mip.OpTableMIPSolver;

public class MIPSolverTest
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
	public void testSolvers()
	{
		OpTableMIPSolver<Vec2D> mip = new OpTableMIPSolver<>();
		
		final int [] ns = {4,5,6};
		List<RestrictedInstance<Vec2D, GeometricInstance>> list = generate(ns,3);
		for (RestrictedInstance<Vec2D, GeometricInstance> ri : list)
		{
			Solution<Vec2D> mipSol;
			mipSol = mip.solve(ri);
			assertTrue("MIP yields feasible solution for a restricted instance", mipSol.isFeasible());
		}
	}
	
	@Test
	public void testUnrestrictedRestricted()
	{
		final int n = 20;
		OpTableMIPSolver<Vec2D> mip = new OpTableMIPSolver<>();
		Random ran = new Random(65432);
		for (int t=0; t < n; t++)
		{
			GeometricInstance gi = RandomGenerators.randomInstance(6, ran, 0.5);
			RestrictedInstance<Vec2D,GeometricInstance> ri;
			ri = RestrictedInstance.restrict(gi, Double.POSITIVE_INFINITY, 0, true, ran);
			Solution<Vec2D> gSol = mip.solve(gi);
			Solution<Vec2D> rSol = mip.solve(ri);
			assertTrue(gSol.isFeasible());
			assertTrue(rSol.isFeasible());
			assertEquals("MIP solution for lenient restricted and unrestricted instance is equal",
					gSol.getTotalCost(), rSol.getTotalCost(), 10e-8);
		}
	}
}
