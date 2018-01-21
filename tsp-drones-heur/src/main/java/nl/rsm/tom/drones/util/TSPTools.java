package nl.rsm.tom.drones.util;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Locale;
import java.util.Map;
import java.util.Scanner;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Class that provides utility methods that can write and read
 * data to be processed by the Concorde TSP solver
 * 
 * The Concorde TSP solver can be found at: http://www.math.uwaterloo.ca/tsp/concorde.html
 * 
 * @author Paul Bouman
 */

public class TSPTools
{
	
	/**
	 * Writes a Geometric instance to a Geometrically defined .tsp file 
	 * @param gi The instance to be written
	 * @param pw The printwriter it should be written to
	 */
	public static Map<Integer,Vec2D> writeGeom(GeometricInstance gi, PrintWriter pw)
	{
		int dim = gi.getNodeCount();
		pw.println("NAME : temp");
		pw.println("TYPE : TSP");
		pw.println("DIMENSION : "+dim);
		pw.println("EDGE_WEIGHT_TYPE : EUC_2D");
		pw.println("NODE_COORD_SECTION");
		int count = 1;
		HashMap<Integer,Vec2D> res = new HashMap<>();
		for (Vec2D vec : gi)
		{
			pw.println(count+" "+(vec.x)+" "+(vec.y));
			res.put(count-1, vec);
			count++;
		}
		pw.println("EOF");
		return res;
	}
	
	
	/**
	 * Writes an instance to a .tsp file with a lower triangular matrix
	 * based on the driving distances
	 * @param i The instance to be written
	 * @param pw The PrintWriter it should be written to
	 */
	public static <E> Map<Integer,E> writeMatrix(Instance<E> i, PrintWriter pw)
	{
		int dim = i.getNodeCount();
		pw.println("NAME : temp");
		pw.println("TYPE : TSP");
		pw.println("DIMENSION : "+dim);
		pw.println("EDGE_WEIGHT_TYPE : EXPLICIT");
		pw.println("EDGE_WEIGHT_FORMAT : LOWER_DIAG_ROW");
		pw.println("EDGE_WEIGHT_SECTION");
		Distance<E> d = i.getDriveDistance();
		ArrayList<E> l = new ArrayList<>(dim);
		Map<Integer,E> res = new HashMap<>();
		for (E loc : i)
		{
			l.add(loc);
			res.put(res.size(), loc);
		}
		int tokens = 0;
		for (int x=0; x < dim; x++)
		{
			for (int y=0; y <= x; y++)
			{
				if (tokens > 10)
				{
					pw.println();
					tokens = 0;
				}
				E from = l.get(x);
				E to = l.get(y);
				double dist = d.getContextFreeDistance(from, to);
				String str = String.format(Locale.US, "%.5f", dist);
				pw.print(" "+str);
				tokens++;
			}
		}
		if (tokens > 0)
		{
			pw.println();
		}
		pw.println("EOF");
		return res;
	}
	
	/**
	 * Reads the solution for a .tsp file
	 * @param i The instance that was used to generate the problem
	 * @param input The instance for which this is a solution
	 * @return The solution that only contains drive operations.
	 */
	public static <E> Solution<E> readSolution(Instance<E> i, InputStream input, Map<Integer,E> map)
	{
		Scanner s = new Scanner(new BufferedReader(new InputStreamReader(input)));

		int dim = s.nextInt();
		if (dim != i.getNodeCount())
		{
			s.close();
			throw new IllegalArgumentException("The solution specifies "+dim+" locations, while the instance has "+i.getNodeCount());
		}
		
		LinkedList<E> tour = new LinkedList<E>();
		for (int j=0; j < dim; j++)
		{
			int index = s.nextInt();
			E loc = map.get(index);
			tour.add(loc);
		}
		
		while (!i.isDepot(tour.getFirst()))
		{
			tour.addFirst(tour.removeLast());
		}
		tour.addLast(i.getDepot());
		
		
		E prev = null;
		ArrayList<Operation<E>> result = new ArrayList<>();
		for (E cur : tour)
		{
			if (prev != null)
			{
				result.add(new Operation<E>(prev,cur));
			}
			prev = cur;
		}
		s.close();
		Solution<E> res = new Solution<E>(i, result);
		return res;
	}
}
