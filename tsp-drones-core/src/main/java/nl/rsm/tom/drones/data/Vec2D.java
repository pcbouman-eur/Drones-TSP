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

package nl.rsm.tom.drones.data;

/**
 * Simple class for two dimensional points.
 * @author Paul Bouman
 */

public class Vec2D implements Comparable<Vec2D>
{
	private final String name;
	
	/**
	 * The x coordinate of this point.
	 */
	public final double x;
	/**
	 * The y coordinate of this point
	 */
	public final double y;
	
	/**
	 * Constructor that specifies an x coordinate, and y coordinate and a name
	 * @param x the x coordinate
	 * @param y the y coordinate
	 * @param name the name
	 */
	public Vec2D(double x, double y, String name)
	{
		this.x = x;
		this.y = y;
		this.name = name;
	}
	
	/**
	 * Constructor where a name is not specified
	 * @param x the x coordinate
	 * @param y the y coordinate
	 */
	public Vec2D(double x, double y)
	{
		this(x,y,"Point_x=" + x + "_y=" + y + "");
	}
	
	/**
	 * @return the name of this point
	 */
	public String getName()
	{
		return name;
	}
	
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		long temp;
		temp = Double.doubleToLongBits(x);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		temp = Double.doubleToLongBits(y);
		result = prime * result + (int) (temp ^ (temp >>> 32));
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vec2D other = (Vec2D) obj;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		if (Double.doubleToLongBits(x) != Double.doubleToLongBits(other.x))
			return false;
		if (Double.doubleToLongBits(y) != Double.doubleToLongBits(other.y))
			return false;
		return true;
	}

	@Override
	public String toString()
	{
		return name;
	}
	
	public Vec2D sub(Vec2D o)
	{
		return new Vec2D(x-o.x, y-o.y, name+" - "+o.name);
	}
	
	public Vec2D add(Vec2D o)
	{
		return new Vec2D(x+o.x, y+o.y, name+" + "+o.name);
	}
	
	public double dot(Vec2D o)
	{
		return x*o.x + y*o.y;
	}
	
	public double length()
	{
		return Math.sqrt(x*x + y*y);
	}
	
	public double angle(Vec2D o)
	{
		double dot = dot(o);
		return Math.acos(dot / (length() * o.length()));
		
	}

	@Override
	public int compareTo(Vec2D o)
	{
		if (x != o.x)
		{
			return (int)Math.signum(x - o.x);
		}
		if (y != o.y)
		{
			return (int)Math.signum(y - o.y);
		}
		return name.compareTo(o.name);
	}
	
}
