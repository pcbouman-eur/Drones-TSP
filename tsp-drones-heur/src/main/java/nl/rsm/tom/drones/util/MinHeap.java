package nl.rsm.tom.drones.util;

/**
 * Heap that puts the minimal element in the front
 * Delegates all the work to a MaxHeap and inverts
 * the keys
 * @author Paul Bouman
 *
 * @param <E> the type of data to store in the heap
 */
public class MinHeap<E> extends MaxHeap<E>
{

	@Override
	public double getValue(int index)
	{
		return -super.getValue(index);
	}
	
	@Override
	public int add(E key, double value)
	{
		return super.add(key, -value);
	}
	
	@Override
	public int update(int index, double value)
	{
		return super.update(index, -value);
	}
}
