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

package nl.rsm.tom.drones.data.io;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Utility class that contains static methods that can be used to read and write solutions
 * @author Paul Bouman
 *
 */

public class SolutionIO
{
	/**
	 * Reads a solution from a file
	 * @param <E> the type of the locations in the instance
	 * @param i The instance for which the file contains a solution
	 * @param filename The file to be read
	 * @return The solution obtained from the file
	 * @throws IOException When an error occurs during reading.
	 */
	public static <E> Solution<E> readSolution(Instance<E> i, String filename) throws IOException
	{
		return readSolution(i,new File(filename));
	}
	
	/**
	 * Reads a solution from a file
	 * @param <E> the type of the locations in the instance
	 * @param i The instance for which the file constains a solution
	 * @param f The name of the file to be read
	 * @return The solution obtained from the file
	 * @throws IOException When an error occurs during reading.
	 */
	public static <E> Solution<E> readSolution(Instance<E> i, File f) throws IOException
	{
		StringBuilder sb = new StringBuilder();
		BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(f)));
		String line = null;
		while ((line = br.readLine()) != null)
		{
			sb.append(line);	
			sb.append("\n");
		}
		br.close();
		return readSolutionFromData(i,sb.toString());
	}
	
	/**
	 * Parse a string containing a solution
	 * @param <E> the type of the locations in the instance
	 * @param i The instance for which the String contains a solution
	 * @param solData The data of the instance
	 * @return The solution
	 */
	public static <E> Solution<E> readSolutionFromData(Instance<E> i, String solData)
	{
		TokenScanner ts = new TokenScanner(solData);
		List<Operation<E>> res = new ArrayList<>();
		int ops = ts.nextInt();
		for (int t=0; t < ops; t++)
		{
			E start = i.getLocation(ts.nextInt());
			E end = i.getLocation(ts.nextInt());
			int flyIndex = ts.nextInt();
			int internalCount = ts.nextInt();
			List<E> internal = new ArrayList<>();
			for (int k=0; k < internalCount; k++)
			{
				E loc = i.getLocation(ts.nextInt());
				internal.add(loc);
			}
			Operation<E> op;
			if (flyIndex == -1)
			{
				op = new Operation<E>(start,internal,end);
			}
			else
			{
				E fly = i.getLocation(flyIndex);
				op = new Operation<E>(start,internal,end,fly);
			}
			res.add(op);
		}
		return new Solution<E>(i,res);
	}
	

	/**
	 * Converts a solution to a String representation
	 * @param <E> the type of the locations in the instance
	 * @param sol The solution to be converted
	 * @return The String representation of the solution
	 */
	public static <E> String solutionToString(Solution<E> sol)
	{
		StringWriter sw = new StringWriter();
		PrintWriter pw = new PrintWriter(sw);
		writeSolution(sol,pw);
		pw.flush();
		pw.close();
		return sw.toString();
	}
	
	/**
	 * Writes a solution to a file
	 * @param <E> the type of the locations in the instance
	 * @param sol The solution to be written
	 * @param filename The name of the file the solution needs to be written to
	 * @throws IOException When an error occurs during writing.
	 */
	public static <E> void writeSolution(Solution<E> sol, String filename) throws IOException
	{
		writeSolution(sol, new File(filename));
	}
	
	/**
	 * Writes a solution to a file
	 * @param <E> the type of the locations in the instance
	 * @param sol The solution to be written
	 * @param f The file it should be written to
	 * @throws IOException When an error occurs during writing.
	 */
	public static <E> void writeSolution(Solution<E> sol, File f) throws IOException
	{
		PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(f)));
		writeSolution(sol, pw);
		pw.flush();
		pw.close();
	}
	
	/**
	 * Writes a solution to a PrintWriter
	 * @param <E> the type of the locations in the instance
	 * @param sol The solution to be written
	 * @param pw The PrintWriter it should be written to
	 */
	public static <E> void writeSolution(Solution<E> sol, PrintWriter pw)
	{
		Instance<E> i = sol.getInstance();
		Map<E,Integer> indices = new HashMap<>();
		for (int t=0; t <= i.getLocationCount(); t++)
		{
			indices.put(i.getLocation(t), t);
		}
		
		List<Operation<E>> ops = sol.getOperations();
		pw.println("/* Number of Operations */");
		pw.println(ops.size());
		pw.println("/* List of Operations. */");
		pw.println("/* Start\tEnd\tFly\t#Internal\tLocations...*/");
		for (Operation<E> op : ops)
		{
			pw.print(indices.get(op.getStart()));
			pw.print("\t");
			pw.print(indices.get(op.getEnd()));
			pw.print("\t");
			if (op.hasFly())
			{
				pw.print(indices.get(op.getFly()));
			}
			else
			{
				pw.print("-1");
			}
			pw.print("\t");
			List<E> internal = op.getInternalNodes(false);
			pw.print(internal.size());
			for (E e : internal)
			{
				pw.print("\t");
				pw.print(indices.get(e));
			}
			pw.print("\t/* Operation cost : " + op.getCost(i) + "*/");
			pw.println();
		}
		pw.println("/* Total cost : "+sol.getTotalCost()+" */");
		pw.flush();
	}
}
