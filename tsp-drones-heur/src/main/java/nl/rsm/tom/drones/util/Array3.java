package nl.rsm.tom.drones.util;

/**
 * Since Java three dimensional arrays are apparently very slow,
 * this class mimics a 3d-array with just a single array
 * @author Paul Bouman
 *
 */

public class Array3
{
	private final double [] _data;
	private final int _xSize, _ySize, _zSize;
	private final boolean _isSafe;
	
	/**
	 * Initializes a 3D array of dimensions x by y by z
	 * @param x The x dimension of the array
	 * @param y The y dimension of the array
	 * @param z The z dimension of the array
	 */
	public Array3(int x, int y, int z)
	{
		this(x,y,z,true);
	}
	
	/**
	 * Initializes a 3D array of dimensions x by y by z
	 * can also specify whether indices should be properly checked
	 * @param x The x dimension of the array
	 * @param y The y dimension of the array
	 * @param z The z dimension of the array
	 * @param safe Whether indices should always be checked
	 */
	public Array3(int x, int y, int z, boolean safe)
	{
		if (x < 1)
		{
			throw new IllegalArgumentException("X-dimension must have size at least 1.");
		}
		if (y < 1)
		{
			throw new IllegalArgumentException("Y-dimension must have size at least 1.");
		}
		if (z < 1)
		{
			throw new IllegalArgumentException("Z-dimension must have size at least 1.");
		}
		_xSize = x;
		_ySize = y;
		_zSize = z;
		_data = new double[x*y*z];
		_isSafe = safe;
	}
	
	
	/**
	 * Sets the value of an entry in the array
	 * @param x The x index
	 * @param y The y index
	 * @param z The z index
	 * @param val The value to be set
	 */
	public void set(int x, int y, int z, double val)
	{
		if (_isSafe)
		{
			if (x < 0 || x >= _xSize)
			{
				throw new IndexOutOfBoundsException();
			}
			if (y < 0 || y >= _ySize)
			{
				throw new IndexOutOfBoundsException();
			}
			if (z < 0 || z >= _zSize)
			{
				throw new IndexOutOfBoundsException();
			}
		}
		_data[getIndex(x,y,z)] = val;
	}
	
	private int getIndex(int x, int y, int z)
	{
		return x * (_ySize * _zSize) + y * (_zSize) + z;
	}
	
	/**
	 * Gets the value stored in the array
	 * @param x The x index
	 * @param y The y index
	 * @param z The z index
	 * @return The value stored in the array
	 */
	public double get(int x, int y, int z)
	{
		if (_isSafe)
		{
			if (x < 0 || x >= _xSize)
			{
				throw new IndexOutOfBoundsException();
			}
			if (y < 0 || y >= _ySize)
			{
				throw new IndexOutOfBoundsException();
			}
			if (z < 0 || z >= _zSize)
			{
				throw new IndexOutOfBoundsException();
			}
		}
		return _data[getIndex(x,y,z)];
	}
	
	/**
	 * The x-dimension of this array
	 * @return The x-dimension of this array
	 */
	public int xDim()
	{
		return _xSize;
	}
	
	/**
	 * The y-dimension of this array
	 * @return The y-dimension of this array
	 */
	public int yDim()
	{
		return _ySize;
	}
	
	/**
	 * The z-dimension of this array
	 * @return The z-dimension of this array
	 */
	public int zDim()
	{
		return _zSize;
	}
	
	@Override
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		
		for (int x=0; x < _xSize; x++)
		{
			for (int y=0; y < _ySize; y++)
			{
				for (int z=0; z < _zSize; z++)
				{
					sb.append(x);
					sb.append(" ");
					sb.append(y);
					sb.append(" ");
					sb.append(z);
					sb.append("\t");
					sb.append(get(x,y,z));
					sb.append("\n");
				}
			}
		}
		return sb.toString();
	}
}
