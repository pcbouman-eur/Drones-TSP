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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.data.instance.RestrictedInstance;

/**
 * Utility class that contains a number of methods that can be used to read and write
 * restricted instances. These methods typically rely on another method that can read
 * or write the underlying unrestricted instance
 * @author Paul Bouman
 *
 */

public class RestrictedInstanceIO
{
	private final static String MAXFLY = "#MAXFLY";
	private final static String FORBID = "#FORBID";
	private final static String NOVISIT = "#NOVISIT";
	
	private final static List<String> TOKENS = Arrays.asList(MAXFLY, FORBID, NOVISIT);
	
	/**
	 * Read a restricted instance from a string containing the instance data,
	 * using a function that can read an underlying unrestricted instance
	 * from String data.
	 * @param <E> the type of the locations in the instance
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param data a string containing the raw instance data
	 * @param reader a function that can convert a string containing the underlying
	 *               instance data into a instance
	 * @return the restricted instance
	 */
	
	public static <E,F extends Instance<E>> RestrictedInstance<E,F> readRestrictedInstance(String data, Function<String,F> reader)
	{
		List<String> restrict = Arrays.stream(data.split("\n"))
				                      .filter(s -> TOKENS.stream().anyMatch(t -> s.startsWith(t)))
				                      .collect(Collectors.toList());
		
		String instanceData = Arrays.stream(data.split("\n"))
				                    .filter( s -> !TOKENS.stream().anyMatch(t -> s.startsWith(t)))
				                    .collect(Collectors.joining("\n"));
		
		F instance = reader.apply(instanceData);
		
		double maxFlight = Double.POSITIVE_INFINITY;
		Set<E> forbid = new HashSet<>();
		Set<E> noVisit = new HashSet<>();
		
		for (String s : restrict)
		{
			if (s.startsWith(MAXFLY))
			{
				double mf = Double.parseDouble(s.substring(MAXFLY.length()).trim());
				maxFlight = Math.min(maxFlight, mf);
			}
			if (s.startsWith(FORBID))
			{
				int index = Integer.parseInt(s.substring(FORBID.length()).trim());
				forbid.add(instance.getLocation(index));
			}
			if (s.startsWith(NOVISIT))
			{
				int index = Integer.parseInt(s.substring(NOVISIT.length()).trim());
				noVisit.add(instance.getLocation(index));
			}
		}
		
		return new RestrictedInstance<>(instance, maxFlight, forbid::contains, noVisit::contains);
	}
	
	/**
	 * Write a restricted instance to a file
	 * @param <E> the type of the locations in the instance
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param ri the restricted instance that needs to be written to a file
	 * @param convert a function that can convert the underlying unrestricted instance to a string
	 * @param file the file to which the restricted instance must be written
	 * @throws FileNotFoundException occurs if the file to which the instance must be written can not be created.
	 */
	
	public static <E,F extends Instance<E>> void writeRestrictedInstance(RestrictedInstance<E,F> ri, Function<F,String> convert, File file) throws FileNotFoundException
	{
		try (PrintWriter pw = new PrintWriter(new FileOutputStream(file)))
		{
			pw.println(writeRestrictions(ri));
			pw.println(convert.apply(ri.getOriginalInstance()));
		}
	}
	
	/**
	 * Converts the restrictions of a restricted instance to a textual representation
	 * @param <E> the type of the locations in the instance
	 * @param <F> the type of the unrestricted instance to which the restrictions are applied
	 * @param instance the restricted instance from which the restrictions must be converted
	 * @return a String representing the restrictions.
	 */
	public static <E,F extends Instance<E>> String writeRestrictions(RestrictedInstance<E,F> instance)
	{
		StringBuilder sb = new StringBuilder();
		sb.append(MAXFLY+" "+instance.getMaximumDistance()+"\n");
		for (int index = 0; index < instance.getNodeCount(); index++)
		{
			E loc = instance.getLocation(index);
			if (instance.isForbidden(loc))
			{
				sb.append(FORBID+" "+index+"\n");
			}
			if (instance.isNoVisit(loc))
			{
				sb.append(NOVISIT+" "+index+"\n");
			}
		}
		return sb.toString();
	}
}
