package nl.rsm.tom.drones.solver.heuristic;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Map;

import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.data.instance.RestrictedInstance;
import nl.rsm.tom.drones.solver.LowerBoundSolver;
import nl.rsm.tom.drones.solver.Solver;
import nl.rsm.tom.drones.util.ProcessStreamConsumer;
import nl.rsm.tom.drones.util.TSPTools;

/**
 * Class which utilizes an external process to call the Concorde TSP solver,
 * and generates a truck-only tour based on the solution provided by Concorde.
 * 
 * Note that Concorde uses a different way to compute distances that our Geometric
 * instances. Concorde rounds the distance down to an integer, while we use floating
 * point arithmetic. As such, there can be minor differences in what is optimal
 * according to Concorde and to our solver. However, it is typically a good starting
 * solution for heuristic searches.
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */

public class ConcordeTSPSolver<E> implements Solver<E>, LowerBoundSolver<E>
{
	private String _pathToConcorde;
	private String _tempfileName;
	
	private Instance<E> _lastInstance;
	private Solution<E> _lastSol;
		
	/**
	 * Constructor. Requires the path to the Concorde executable and a prefix
	 * for (temporary) filenames that are used while Concorde does it work.
	 * 
	 * @param pathToConcorde the path to the Concorde executable
	 * @param tempfileName a prefix for temporary files. Typically, prefix+'.tsp'
	 *                     is the file that has to be solved by Concorde, and
	 *                     prefix+'.sol' is the solution produced by concorde.
	 */
	public ConcordeTSPSolver(String pathToConcorde, String tempfileName)
	{
		_pathToConcorde = pathToConcorde;
		_tempfileName = tempfileName;
	}
	
	@SuppressWarnings("unchecked")
	@Override
	public Solution<E> solve(Instance<E> instance)
	{
		if (instance == _lastInstance)
		{
			return _lastSol;
		}
		_lastInstance = null;
		_lastSol = null;
		File tempIn, tempOut;
		Map<Integer,E> map;
		try
		{
			tempIn = new File(_tempfileName+".tsp");
			tempOut = new File(_tempfileName+".sol");
			PrintWriter pw = new PrintWriter(new FileOutputStream(tempIn.toString()));
			Instance<E> instanceToTest = instance;
			if (instance instanceof RestrictedInstance)
			{
				RestrictedInstance<?, Instance<?>> ri = (RestrictedInstance<?,Instance<?>>) instance;
				Instance<?> i = ri.getOriginalInstance();
				if (i instanceof GeometricInstance)
				{
					GeometricInstance gi = (GeometricInstance) i;
					// now we should check if there are no restrictions on the truck
					if (gi.getDriveDistance() == i.getDriveDistance())
					{
						instanceToTest = (Instance<E>) gi;
					}
				}
			}
			if (instanceToTest instanceof GeometricInstance)
			{
				GeometricInstance gi = (GeometricInstance) instanceToTest;
				map = (Map<Integer,E>)TSPTools.writeGeom(gi, pw);
			}
			else
			{
				map = TSPTools.writeMatrix(instanceToTest, pw);
			}
			pw.flush();
			pw.close();
		}
		catch (Exception e)
		{
			throw new Error("Error while creating temporary instance file",e);
		}
		try
		{
			Process p = Runtime.getRuntime().exec(_pathToConcorde+" -x "+tempIn);
			new ProcessStreamConsumer(p, false);
			p.waitFor();
		}
		catch (Exception e)
		{
			throw new Error("Error while executing concorde",e);
		}
		if (!tempIn.exists())
		{
			throw new IllegalStateException("Concorde exited without writing a solution.");
		}
		try
		{
			FileInputStream fis = new FileInputStream(tempOut);
			Solution<E> sol = TSPTools.readSolution(instance, fis, map);
			fis.close();
			tempOut.delete();
			_lastInstance = instance;
			_lastSol = sol;
			return sol;
		}
		catch (Exception e)
		{
			throw new Error("Error while reading output file",e);
		}
	}

	@Override
	public double lowerbound(Instance<E> instance)
	{
		if (instance instanceof GeometricInstance)
		{
			GeometricInstance gi = (GeometricInstance) instance;
			Solution<E> sol = solve(instance);
			double sum = gi.getDriveSpeed() + gi.getFlySpeed();
			double frac = Math.min(gi.getDriveSpeed(), gi.getFlySpeed()) / sum;
			return frac * sol.getTotalCost();
		}
		return 0;
	}
	
	
}
