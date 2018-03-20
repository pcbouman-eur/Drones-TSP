/* Copyright 2018 Paul Bouman
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

package nl.rsm.tom.drones.main.generator;

import java.io.File;
import java.io.IOException;
import java.time.LocalDate;
import java.util.Locale;
import java.util.Random;
import java.util.function.Function;

import org.springframework.shell.standard.ShellComponent;
import org.springframework.shell.standard.ShellMethod;
import org.springframework.shell.standard.ShellOption;

import nl.rsm.tom.drones.data.instance.GeometricInstance;
import nl.rsm.tom.drones.data.instance.RandomGenerators;
import nl.rsm.tom.drones.data.io.InstanceIO;

/**
 * Annotated class to be utilized by the Spring Shell framework
 * @author Paul Bouman
 *
 */

@ShellComponent
public class GeneratorShell
{
	private double alpha = 2;
	private boolean useDate = false;
	private boolean overwrite = false;

	private boolean makeFolder = false;
	
	private int offset = 0;
	private File output = new File(".");
	private long seed = 54321;
	
	@ShellMethod(value="Set whether the current date should be part of the output filename (default: false)", key="toggle_date")
	public boolean setDate()
	{
		this.useDate = !this.useDate;
		return this.useDate;
	}
	
	@ShellMethod(value="Set whether existing output files can be overwritten (default: false)", key="toggle_overwrite")
	public boolean setOverwrite()
	{
		this.overwrite = !this.overwrite;
		return this.overwrite;
	}
	
	@ShellMethod(value="Set the seed used to generate instances", key="set_seed")
	public void setSeed(long seed)
	{
		this.seed = seed;
	}
	
	@ShellMethod(value="Show the current seed", key="show_seed")
	public long getSeed()
	{
		return this.seed;
	}
	
	@ShellMethod(value="Set whether to create subfolders based on the instance types generated (default: false)", key="toggle_subfolder")
	public boolean setSubfolder()
	{
		this.makeFolder = !this.makeFolder;
		return this.makeFolder;
	}

	@ShellMethod(value="Set the relative speed of the drone compare to the truck", key="set_alpha")
	public void setAlpha(double alpha)
	{
		if (alpha <= 0 || !Double.isFinite(alpha) || Double.isNaN(alpha))
		{
			throw new IllegalArgumentException("The value of alpha must be a positive number");
		}
		this.alpha = alpha;
	}
	
	@ShellMethod(value="Shows the current value of alpha", key="show_alpha")
	public double getAlpha()
	{
		return this.alpha;
	}
	
	
	@ShellMethod(value="Set the directory where instances should be written", key="set_output")
	public void setOutput(File f)
	{
		if (f.exists() && !f.isDirectory())
		{
			throw new IllegalArgumentException("The output must be a directory");
		}
		else if (!f.exists())
		{
			if (!f.mkdirs())
			{
				throw new IllegalArgumentException("The output directory could not be created");
			}
		}
		this.output = f;
	}
	
	@ShellMethod(value="Shows the current directory where instances will be written", key="show_output")
	public File getOutput() throws IOException
	{
		return this.output.getCanonicalFile();
	}
	
	@ShellMethod(value="Generates a number of uniform instances", key="gen_uniform")
	public void generateUniform(int instances, int locations, @ShellOption(defaultValue="100") int gridSize) throws IOException
	{
		// The -1 is used because the uniform generator does not include the depot in the number of locations.
		generate(instances, "uniform", alpha, locations, r -> RandomGenerators.randomInstance(locations-1, r, 1, 1d/alpha, 100));
	}
	
	@ShellMethod(value="Generates a number of single center instances", key="gen_singlecenter")
	public void generateSingleCenter(int instances, int locations, @ShellOption(defaultValue="50") double stddev) throws IOException
	{
		generate(instances, "singlecenter", alpha, locations, r -> RandomGenerators.random1CenterInstance(locations,r, stddev, 1d/alpha));
	}
	
	@ShellMethod(value="Generates a number of double center instances", key="gen_doublecenter")
	public void generateDoublepCenter(int instances, int locations, @ShellOption(defaultValue="50") double stddev1,
			@ShellOption(defaultValue="50") double stddev2, @ShellOption(defaultValue="200") double distance,
			@ShellOption(defaultValue="0.5") double prob) throws IOException
	{
		generate(instances, "doublecenter", alpha, locations, r -> RandomGenerators.random2CenterInstance(locations,r,alpha, stddev1, stddev2, distance, prob));
	}
	
	private void generate(int instances, String type, double alpha, int locs, Function<Random,GeometricInstance> generator) throws IOException
	{
		String prefix = type+"-";
		if (useDate)
		{
			LocalDate ld = LocalDate.now();
			prefix += ld.toString().replaceAll("-", "_")+"-";
		}
		
		String alphaStr = "";
		if (alpha % 1.0 < 0.001)
		{
			alphaStr = String.format(Locale.US, "%d", (int)Math.round(alpha));
		}
		else
		{
			alphaStr = String.format(Locale.US, "%.3f", alpha);
		}
		String postfix = "-alpha_"+alphaStr+"-n"+locs+".txt";
	
		File outDir = output;
		if (makeFolder)
		{
			String dirname = "inputs"+"-n"+locs+"-alpha_"+alphaStr+"-"+type;
			outDir = new File(outDir, dirname);
			outDir.mkdirs();	
		}


		Random ran = new Random(seed);
		for (int i=1; i <= instances; i++)
		{
			String filename = prefix+(offset+i)+postfix;
			File outFile = new File(outDir, filename);
			if (outFile.exists() && !overwrite)
			{
				throw new IOException("File "+filename+" already exists and overwrite mode is not enabled. Aborting.");
			}
			GeometricInstance gi = generator.apply(ran);
			InstanceIO.writeInstance(gi, outFile);
		}
		offset += instances;

	}
}
