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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.FileInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import nl.rsm.tom.drones.data.Distance;
import nl.rsm.tom.drones.data.SimpleDistance;
import nl.rsm.tom.drones.data.Vec2D;
import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.GraphInstance;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.data.instance.MatrixInstance;
import static nl.rsm.tom.drones.data.io.TokenScanner.transform;

/**
 * Utility class that provides many static methods that can
 * be used to read and write instances to and from files
 * @author Paul Bouman
 *
 */

public class InstanceIO
{
	/**
	 * Converts a Geometric instance to String data
	 * @param instance The instance to be converted
	 * @return The instance represented as a String.
	 */
	public static String instanceToString(GeometricInstance instance)
	{
		StringWriter sw = new StringWriter();
		PrintWriter pw = new PrintWriter(sw);
		writeInstance(instance,pw);
		pw.flush();
		pw.close();
		return sw.toString();
	}
	
	/**
	 * Writes a Geometric instance to a file
	 * @param instance The instance to be written
	 * @param filename The name of the file it will be written to
	 * @throws IOException Whether something occurred during writing
	 */
	public static void writeInstance(GeometricInstance instance, String filename) throws IOException
	{
		writeInstance(instance, new File(filename));
	}
	
	/**
	 * Writes a Geometric instance to a file
	 * @param instance The instance to be written
	 * @param f The name of the file it will be written to
	 * @throws IOException Whether something occurred during writing
	 */
	public static void writeInstance(GeometricInstance instance, File f) throws IOException
	{
		PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(f)));
		writeInstance(instance, pw);
		pw.flush();
		pw.close();
	}
	
	/**
	 * Writes a Geometric instance to a PrintWriter
	 * @param instance The instance to be written
	 * @param pw The PrintWriter it should be written to
	 */
	public static void writeInstance(GeometricInstance instance, PrintWriter pw)
	{
		pw.println("/*The speed of the Truck*/");
		pw.println(instance.getDriveSpeed());
		pw.println("/*The speed of the Drone*/");
		pw.println(instance.getFlySpeed());
		int nodes = instance.getNodeCount();
		pw.println("/*Number of Nodes*/");
		pw.println(nodes);
		pw.println("/*The Depot*/");
		Vec2D d = instance.getDepot();
		pw.println(d.x+" "+d.y+" "+transform(d.getName()));
		pw.println("/*The Locations (x_coor y_coor name)*/");
		for (Vec2D p : instance.getLocations())
		{
			pw.println(p.x+" "+p.y+" "+transform(p.getName()));
		}
		pw.flush();
	}
	
	/**
	 * Reads a Geometric instance from a file
	 * @param filename The file which contains the Geometric instance
	 * @return The instance which was read
	 * @throws IOException When an error occurs during reading
	 */
	public static GeometricInstance readGeometricInstance(String filename) throws IOException
	{
		return readGeometricInstance(new File(filename));
	}
	
	/**
	 * Reads a Geometric instance from a file
	 * @param f The file which contains the Geometric instance
	 * @return The instace which was read
	 * @throws IOException When an error occurred during reading
	 */
	public static GeometricInstance readGeometricInstance(File f) throws IOException
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
		return readGeometricInstanceFromData(sb.toString());
	}
	
	/**
	 * Converts a String to a Geometric instance
	 * @param inputData The String which contains the instance data
	 * @return The parsed instance
	 */
	public static GeometricInstance readGeometricInstanceFromData(String inputData)
	{
		TokenScanner ts = new TokenScanner(inputData);
		double drive = ts.nextDouble();
		double fly = ts.nextDouble();
		int n = ts.nextInt();
		ArrayList<Vec2D> points = new ArrayList<>();
		Vec2D depot = null;
		for (int t=0; t < n; t++)
		{
			double x = ts.nextDouble();
			double y = ts.nextDouble();
			String name = ts.nextIdentifier();
			Vec2D p = new Vec2D(x,y,name);
			if (depot == null)
			{
				depot = p;
			}
			else
			{
				points.add(p);
			}
		}
		return new GeometricInstance(depot,points,drive,fly);
	}
	
	/**
	 * Converts a Matrix instance to a String
	 * @param <E> the type of the locations in the instance
	 * @param instance The instance to be converted
	 * @return The resulting String representation of the instance
	 */
	public static <E> String instanceToString(MatrixInstance<E> instance)
	{
		StringWriter sw = new StringWriter();
		PrintWriter pw = new PrintWriter(sw);
		writeInstance(instance,pw);
		pw.flush();
		pw.close();
		return sw.toString();
	}

	/**
	 * Writes a Matrix instance to a file
	 * @param <E> the type of the locations in the instance
	 * @param mi The instance to be written
	 * @param filename The name of the file the instance will be written to
	 * @throws IOException When an error occurred during writing.
	 */
	public static <E> void writeInstance(MatrixInstance<E> mi, String filename) throws IOException
	{
		writeInstance(mi, new File(filename));
	}
	
	/**
	 * Writes a Matrix instance to a file
	 * @param <E> the type of the locations in the instance
	 * @param mi The instance to be written
	 * @param f The file to which it should be written
	 * @throws IOException When an error occurred during writing.
	 */
	public static <E> void writeInstance(MatrixInstance<E> mi, File f) throws IOException
	{
		PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(f)));
		writeInstance(mi,pw);
		pw.flush();
		pw.close();
	}
	
	/**
	 * Writes a Matrix instance to a PrintWriter
	 * @param <E> the type of the locations in the instance
	 * @param mi The instance to be written
	 * @param pw The PrintWriter it should be written to
	 */
	public static <E> void writeInstance(MatrixInstance<E> mi, PrintWriter pw)
	{
		pw.println("/* Number of Locations including the Depot */");
		pw.println(mi.getNodeCount());
		List<E> objects = new ArrayList<>();
		objects.add(mi.getDepot());
		for (E o : mi.getLocations())
		{
			objects.add(o);
		}

		pw.println("/* The distance matrix for driving */");
		Distance<E> dist = mi.getDriveDistance();
		for (E o1 : objects)
		{
			for (E o2 : objects)
			{
				//NOTE: assumption that a matrix instance ignores actions
				double d = dist.getDistance(o1, o2, null, null, 0);
				pw.print(d);
				pw.print(" ");
			}
			pw.println();
		}
		
		pw.println("/* The distance matrix for flying */");
		dist = mi.getFlyDistance();
		for (E o1 : objects)
		{
			for (E o2 : objects)
			{
				//NOTE: assumption that a matrix instance ignores actions
				double d = dist.getDistance(o1, o2, null, null, 0);
				pw.print(d);
				pw.print(" ");
			}
			pw.println();
		}
		pw.flush();
	}
	
	/**
	 * Reads a Matrix instance from a file
	 * @param filename The name of the file containing the instance
	 * @return The instance which was read from the file
	 * @throws IOException When an error occurs during reading
	 */
	public static MatrixInstance<Integer> readMatrixInstance(String filename) throws IOException
	{
		return readMatrixInstance(new File(filename));
	}
	
	/**
	 * Reads a Matrix instance from a file	
	 * @param f The file from which the instance will be read
	 * @return The instance which was read from the file
	 * @throws IOException When an error occurs during reading
	 */
	public static MatrixInstance<Integer> readMatrixInstance(File f) throws IOException
	{
		BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(f)));
		StringBuilder sb = new StringBuilder();
		String line = null;
		while ((line = br.readLine()) != null)
		{
			sb.append(line);
			sb.append("\n");
		}
		br.close();
		return readMatrixInstanceFromData(sb.toString());
	}

	/**
	 * Converts a String to a Matrix instace
	 * @param input The String which contains the input data
	 * @return The instance which was read from the String
	 */
	public static MatrixInstance<Integer> readMatrixInstanceFromData(String input)
	{
		TokenScanner ts = new TokenScanner(input);
		int count = ts.nextInt();
		List<Integer> locs = new ArrayList<>();
		for (int t=1; t < count; t++)
		{
			locs.add(t);
		}
		double [][] drive = new double[count][count];
		double [][] fly = new double[count][count];
		for (int x=0; x < count; x++)
		{
			for (int y=0; y < count; y++)
			{
				drive[x][y] = ts.nextDouble();
			}
		}
		for (int x=0; x < count; x++)
		{
			for (int y=0; y < count; y++)
			{
				fly[x][y] = ts.nextDouble();
			}
		}
		SimpleDistance<Integer> dDist = (Integer x, Integer y) -> drive[x][y];
		SimpleDistance<Integer> fDist = (Integer x, Integer y) -> fly[x][y];
		
		return new MatrixInstance<Integer>(0,locs,dDist,fDist);
	}

	/**
	 * Converts a Graph instance to a String representation
	 * @param <E> the type of the locations in the instance
	 * @param instance The instance to be converted
	 * @return A representation of the instance as a String
	 */
	public static <E> String instanceToString(GraphInstance<E> instance)
	{
		StringWriter sw = new StringWriter();
		PrintWriter pw = new PrintWriter(sw);
		writeInstance(instance,pw);
		pw.flush();
		pw.close();
		return sw.toString();
	}
	
	/**
	 * Writes a Graph instance to a file
	 * @param <E> the type of the locations in the instance
	 * @param si The instance to be written
	 * @param filename The name of the file the instance will be written to
	 * @throws IOException When an error occurs during writing
	 */
	public static <E> void writeInstance(GraphInstance<E> si, String filename) throws IOException
	{
		writeInstance(si, new File(filename));
	}
	
	/**
	 * Writes a Graph instance to a file
	 * @param <E> the type of the locations in the instance
	 * @param si The instance to be written
	 * @param f The file the instance will be written to
	 * @throws IOException When an error occurs during writing
	 */
	public static <E> void writeInstance(GraphInstance<E> si, File f) throws IOException
	{
		PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(f)));
		writeInstance(si,pw);
		pw.flush();
		pw.close();
	}
	
	/**
	 * Writes a Graph instance to a PrintWriter
	 * @param <E> the type of the locations in the instance
	 * @param si The instance to be written
	 * @param pw The PrintWriter the instance will be written to
	 */
	public static <E> void writeInstance(GraphInstance<E> si, PrintWriter pw)
	{
		ArrayList<E> locations = new ArrayList<>();
		Map<E,Integer> indices = new HashMap<>();
		locations.add(si.getDepot());
		indices.put(si.getDepot(),0);
		for (E loc : si.getLocations())
		{
			locations.add(loc);
			indices.put(loc,indices.size());
		}
		pw.println("/*Number of Locations including Depot*/");
		pw.println(indices.size());
		pw.println("/*Is this a bidirectional instance?*/");
		pw.println(si.isBidirectional());
		pw.println("/*Edges (from to drive_distance fly_distance)*/");
		for (GraphInstance<E>.Edge e : si.getEdges())
		{
			int f = indices.get(e.left);
			int t = indices.get(e.right);
			pw.println(f+" "+t+" "+e.drive+" "+e.fly);
		}
		pw.flush();
	}
	
	/**
	 * Reads a Graph instance from a file
	 * @param filename The name of the file which contains the instance
	 * @return The instance which was read from the file
	 * @throws IOException When an error occurs during reading
	 */
	public static GraphInstance<Integer> readGraphInstance(String filename) throws IOException
	{
		return readGraphInstance(new File(filename));
	}
	
	/**
	 * Reads a Graph instance from a file
	 * @param f The file from which the instance will be read 
	 * @return The instance which was read from the file
	 * @throws IOException When an error occurs during reading
	 */
	public static GraphInstance<Integer> readGraphInstance(File f) throws IOException
	{
		BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(f)));
		StringBuilder sb = new StringBuilder();
		String line = null;
		while ((line = br.readLine()) != null)
		{
			sb.append(line);
			sb.append("\n");
		}
		br.close();
		return readGraphInstanceFromData(sb.toString());
	}
	
	/**
	 * Converts a Graph instance represented as a String to a parsed instance
	 * @param inputData The instance data as a String
	 * @return The parsed instance
	 */
	public static GraphInstance<Integer> readGraphInstanceFromData(String inputData)
	{
		TokenScanner ts = new TokenScanner(inputData);
		int count = ts.nextInt();
		boolean bidirectional = ts.nextBoolean();
		List<Integer> list = new ArrayList<>();
		for (int t=1; t < count; t++)
		{
			list.add(t);
		}
		GraphInstance<Integer> res = new GraphInstance<>(0,list,bidirectional);
		while (ts.hasNext())
		{
			int from = ts.nextInt();
			int to = ts.nextInt();
			double drive = ts.nextDouble();
			double fly = ts.nextDouble();
			res.addDistance(from, to, drive, fly);
		}
		return res;
	}
	
	/**
	 * Takes an instance and a writer method for that instance to produce a String
	 * @param <E> the type of the locations in the instance
	 * @param <F> the instance type that is written
	 * @param instance The instance to be written
	 * @param write A method that takes an instance and writes it to a PrintWriter
	 * @return the resulting String
	 */
	public static <E,F extends Instance<E>> String writeInstanceData(F instance, BiConsumer<F,PrintWriter> write)
	{
		StringWriter sw = new StringWriter();
		try (PrintWriter pw = new PrintWriter(sw))
		{
			write.accept(instance, pw);
		}
		return sw.toString();
	}
	
	/**
	 * Converts a GeometricInstance to a String
	 * @param gi the geometric instance
	 * @return a string representing the instance
	 */
	public static String writeInstance(GeometricInstance gi)
	{
		return writeInstanceData(gi, InstanceIO::writeInstance);
	}
	
	/**
	 * Converts a GraphInstance to a String
	 * @param <E> the type of the locations in the instance
	 * @param gi the graph instance
	 * @return a string representing the instance
	 */
	public static <E> String writeInstance(GraphInstance<E> gi)
	{
		return writeInstanceData(gi, InstanceIO::writeInstance);
	}
	
	/**
	 * Converts a MatrixInstance to a String
	 * @param <E> the type of the locations in the instance
	 * @param mi the matrix instance
	 * @return a string representing the instance
	 */
	public static <E> String writeInstance(MatrixInstance<E> mi)
	{
		return writeInstanceData(mi, InstanceIO::writeInstance);
	}
}
