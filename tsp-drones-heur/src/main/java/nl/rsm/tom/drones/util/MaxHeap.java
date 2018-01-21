package nl.rsm.tom.drones.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Heap datastructure which keeps the maximal element in front
 * @author Paul
 *
 * @param <E>
 */

public class MaxHeap<E> implements Iterable<E>
{
	private static final boolean CHECKING_MODE = false;
	private static final double GROW_FACTOR = 2;
	
	private int _size;
	private int _maxSize;
	private double [] _values;
	private Object [] _keys;
	
	private String _prevState;
	
	/**
	 * Initiliazes a heap with an initial size of 1024
	 */
	public MaxHeap()
	{
		this(1024);
	}
	
	/**
	 * Initializes a heap with a provided maximum size
	 * @param maxSize
	 */
	public MaxHeap(int maxSize)
	{
		_maxSize = maxSize;
		_values = new double[maxSize];
		_keys = new Object[maxSize];
	}
	
	/**
	 * Returns an object with a certain index in the heap.
	 * The object at index zero will have a maximal value
	 * @param index
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public E getKey(int index)
	{
		return (E) _keys[index];
	}
	
	/**
	 * Gets the value of the current object at a certain index
	 * @param index
	 * @return
	 */
	public double getValue(int index)
	{
		return _values[index];
	}
	
	/**
	 * Gives the index of the left child for a certain index
	 * @param index The index for which the left child will be given
	 * @return The index of the left child
	 */
	public int leftChild(int index)
	{
		return 2*index+1;
	}
	
	/**
	 * Gives the index of the right child for a certain index
	 * @param index The index for which the right child will be given
	 * @return The index of the right child
	 */
	public int rightChild(int index)
	{
		return 2*index + 2;
	}
	
	/**
	 * Gives the index of the child with the highest value, or
	 * the index of the left child if not children are available
	 * @param index the index of which we want the child with highest value
	 * @return the index
	 */
	public int bestChild(int index)
	{
		int lc = leftChild(index);
		int rc = rightChild(index);
		if (lc >= _size || rc >= _size || _values[lc] >= _values[rc])
		{
			return lc;
		}
		return rc;
	}
	
	/**
	 * Gives the index of the parent of this index. The parent of the root,
	 * is the root itself
	 * @param index The index for which we will compute the parent index
	 * @return The index of the parent
	 */
	public int parent(int index)
	{
		return (index-1) / 2;
	}
	
	/**
	 * Adds an object to this heap
	 * @param key The object to be added
	 * @param value The value it will represent
	 * @return The index of the object in the heap
	 */
	public int add(E key, double value)
	{
		if (CHECKING_MODE)
		{
			_prevState = toString();
		}
		
		if (key == null)
		{
			throw new IllegalArgumentException("Adding null key...");
		}
		//System.out.println("Adding "+key+" with value "+value);
		if (_size >= _maxSize)
		{
			int newSize = (int)Math.ceil(_maxSize * GROW_FACTOR);
			expand(newSize);
			//throw new IllegalStateException("Heap is currently full!");
		}
		int index = _size;
		_keys[index] = key;
		_values[index] = value;
		_size++;
		index = bubbleUp(index);
		if (key instanceof HeapIndexChangedListener)
		{
			HeapIndexChangedListener l = (HeapIndexChangedListener) key;
			l.notifyHeapIndex(index);
		}
		//System.out.println(this);
		if (CHECKING_MODE)
		{
			checkIntegrity("While adding "+key+" with value "+value);
		}
		return index;
	}
	
	/**
	 * Removes an object from the heap
	 * @param index The index of the object to be removed
	 */
	public void remove(int index)
	{
		if (CHECKING_MODE)
		{
			_prevState = toString();
		}
		
		//System.out.println("PRIOR: "+this);
		//System.out.println("Removing "+_keys[index]+" with value "+_values[index]);
		Object obj = _keys[index];
		if (index < _size-1)
		{
			swap(_size-1, index);
			_size--;
			_keys[_size] = null;
			_values[_size] = 0;
			index = bubbleDown(index);
			index = bubbleUp(index);
			//System.out.println(this);
		}
		else
		{
			_size--;
		}
		
		if (obj instanceof HeapIndexChangedListener)
		{
			HeapIndexChangedListener l = (HeapIndexChangedListener) obj;
			l.notifyHeapIndex(-1);
		}
		
		if (CHECKING_MODE)
		{
			checkIntegrity("After removing "+index);
		}
		
		//System.out.println("POSTERIOR: "+this);
	}
	
	/**
	 * Returns the current size of the heap
	 * @return The current size of the heap
	 */
	public int size()
	{
		return _size;
	}
	
	/**
	 * Updates the value an object represents in the heap
	 * @param index The index of the object which value will be updated
	 * @param value The new value of the object
	 * @return The new index of the object in the heap, after performing the update
	 */
	public int update(int index, double value)
	{
		if (CHECKING_MODE)
		{
			_prevState = toString();
		}
		
		if (_keys[index] == null)
		{
			throw new IllegalStateException();
		}
		//System.out.println("Updating "+_keys[index]+" from "+_values[index]+" to "+value);
		_values[index] = value;
		index = bubbleUp(index);
		index = bubbleDown(index);
		//System.out.println(this);
		
		if (CHECKING_MODE)
		{
			checkIntegrity("While updating position "+index+" to value "+value);
		}
		
		return index;
	}
	
	private int bubbleUp(int index)
	{
		if (index < 1)
		{
			return index;
		}
		while (index > 0 && _values[parent(index)] < _values[index])
		{
			swap(index,parent(index));
			index = parent(index);
		}
		return index;
	}
	
	private int bubbleDown(int index)
	{
		int bestChild;
		while ((bestChild = bestChild(index)) < _size)
		{
			if (_values[bestChild] > _values[index])
			{
				swap(bestChild, index);
				index = bestChild;
			}
			else
			{
				return index;
			}
		}
		return index;
	}
	
	private void swap(int index, int otherIndex)
	{
		//System.out.println("Before swap "+this);
		//System.out.println("Swapping "+index+" with "+otherIndex);
		if (index < 0 || otherIndex < 0 || index >= _size || otherIndex >= _size)
		{
			throw new IndexOutOfBoundsException();
		}
		Object tmp = _keys[otherIndex];
		double tmpVal = _values[otherIndex];
		_keys[otherIndex] = _keys[index];
		_values[otherIndex] = _values[index];
		_keys[index] = tmp;
		_values[index] = tmpVal;
		if (_keys[index] instanceof HeapIndexChangedListener)
		{
			HeapIndexChangedListener l = (HeapIndexChangedListener) _keys[index];
			l.notifyHeapIndex(index);
		}
		if (_keys[otherIndex] instanceof HeapIndexChangedListener)
		{
			HeapIndexChangedListener l = (HeapIndexChangedListener) _keys[otherIndex];
			l.notifyHeapIndex(otherIndex);			
		}	
		//System.out.println("After swap: "+this);
	}
	
	private void expand(int newSize)
	{
		if (newSize < _maxSize)
		{
			return;
		}
		double [] newValue = new double[newSize];
		Object [] newKeys = new Object[newSize];
		for (int t=0; t < _size; t++)
		{
			newValue[t] = _values[t];
			newKeys[t] = _keys[t];
		}
		_values = newValue;
		_keys = newKeys;
		_maxSize = newSize;
	}

	@Override
	public String toString()
	{
		StringBuilder sb = new StringBuilder();
		sb.append("[");
		for (int t=0; t < _size; t++)
		{
			sb.append("(");
			sb.append(_keys[t]);
			sb.append(" => ");
			sb.append(_values[t]);
			sb.append(")");
		}
		sb.append("]");
		return sb.toString();
	}

	@SuppressWarnings("unchecked")
	@Override
	public Iterator<E> iterator()
	{
		List<E> res = new ArrayList<>(_size);
		for (int t=0; t < _size; t++)
		{
			res.add((E) _keys[t]);
		}
		return res.iterator();
	}
	
	private void checkIntegrity(String msg)
	{
		for (int t=0; t < _size; t++)
		{
			double val = _values[t];
			int lc = leftChild(t);
			int rc = rightChild(t);
			if (lc < _size && _values[lc] > val)
			{
				System.out.println("State before problem: "+_prevState);
				System.out.println("State after problem: "+toString());
				throw new IllegalArgumentException(msg+": Left Child of "+t+" at position "+lc+" has value "+_values[lc]+" while "+t+" has "+val);
			}
			if (rc < _size && _values[rc] > val)
			{
				System.out.println("State before problem: "+_prevState);
				System.out.println("State after problem: "+toString());
				throw new IllegalArgumentException(msg+": Right Child of "+t+" at position "+rc+" has value "+_values[rc]+" while "+t+" has "+val);
			}
		}
	}
	
	/**
	 * Interface that allows objects to listen to changes of their index in a heap
	 * @author Paul Bouman
	 *
	 */
	public static interface HeapIndexChangedListener
	{
		/**
		 * This is called by a heap providing an update on the current index of the object
		 * @param index The new index of this object in the heap
		 */
		public void notifyHeapIndex(int index);
	}

	
}
