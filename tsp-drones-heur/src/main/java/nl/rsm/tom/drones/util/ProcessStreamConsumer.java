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

package nl.rsm.tom.drones.util;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * Since an external process may block if the buffer of either the
 * standard out or the error stream is filled, we implement a
 * consumer which makes sure these buffers are kept clean.
 * @author Paul Bouman
 *
 */

public class ProcessStreamConsumer
{
	private final BufferedReader _stdout;
	private final BufferedReader _stderr;
	
	private final StringBuilder _out;
	private final StringBuilder _err;
	
	private boolean _outClosed;
	private boolean _errClosed;
	
	/**
	 * Default constructor. 
	 * @param p The process of which the streams should be consumed
	 */
	public ProcessStreamConsumer(Process p)
	{
		this(p,true);
	}
	
	/**
	 * Constructor
	 * @param p The process of which the streams should be consumed
	 * @param store Whether or not the output should be stored in this object. Default is false.
	 */
	public ProcessStreamConsumer(Process p, boolean store)
	{
		if (store)
		{
			_out = new StringBuilder();
			_err = new StringBuilder();
		}
		else
		{
			_out = null;
			_err = null;
		}
		
		_stdout = new BufferedReader(new InputStreamReader(p.getInputStream()));
		_stderr = new BufferedReader(new InputStreamReader(p.getErrorStream()));
		
		Thread t1 = new Thread()
		{
			@Override public void run()
			{
				String line;
				try
				{
					while ((line = _stdout.readLine()) != null)
					{
						if (_out != null)
						{
							_out.append(line);
							_out.append("\n");
						}
					}
				}
				catch (Exception e) {}
				_outClosed = true;
			}
		};
		
		Thread t2 = new Thread()
		{
			@Override public void run()
			{
				String line;
				try
				{
					while ((line = _stderr.readLine()) != null)
					{
						if (_err != null)
						{
							_err.append(line);
							_err.append("\n");
						}
					}
				}
				catch (Exception e) {}
				_errClosed = true;
			}
		};
		
		t1.setDaemon(true);
		t2.setDaemon(true);
		t1.start();
		t2.start();
	}
	
	public boolean isOutClosed()
	{
		return _outClosed;
	}
	
	public boolean isErrClosed()
	{
		return _errClosed;
	}
	
	public String getOutString()
	{
		if (_out == null)
		{
			throw new IllegalStateException("Standard out not recorded!");
		}
		if (!_outClosed)
		{
			throw new IllegalStateException("Standard out still open!");
		}
		return _out.toString();
	}
	
	public String getErrString()
	{
		if (_out == null)
		{
			throw new IllegalStateException("Standard error not recorded!");
		}
		if (!_outClosed)
		{
			throw new IllegalStateException("Standard error still open!");
		}
		return _out.toString();
	}
}
