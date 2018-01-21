

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.TreeSet;

import org.junit.Test;

import nl.rsm.tom.drones.util.MaxHeap;
import nl.rsm.tom.drones.util.MaxHeap.HeapIndexChangedListener;
import nl.rsm.tom.drones.util.MinHeap;

/**
 * Unit tests for the heap data structures used in greedy algorithms
 * @author Paul Bouman
 *
 */

public class HeapTest
{
	@Test
	public void test()
	{
		Random ran = new Random();
		for (int i=3; i < 25; i++)
		{
			testAdds(i, ran, 1000);
			testUpdates(i, ran, 1000);
			testAddsMin(i, ran, 1000);
			testUpdatesMin(i, ran, 1000);
		}
	}

	public void testAdds(int k, Random ran, int n)
	{
		for (int t=0; t < n; t++)
		{
			MaxHeap<StringNode> testHeap = new MaxHeap<>();
			Map<String,Double> ranMap = randomMap(k,ran);
			for (String str : ranMap.keySet())
			{
				testHeap.add(new StringNode(str), ranMap.get(str));
			}
			List<String> heapList = new ArrayList<>();
			while (testHeap.size() > 0)
			{
				StringNode sn = testHeap.getKey(0);
				testHeap.remove(0);
				heapList.add(sn.string);
			}
			List<String> sortList = mappedSort(ranMap);
			Collections.reverse(sortList);
			assertEquals(heapList, sortList);
		}
	}

	public void testAddsMin(int k, Random ran, int n)
	{
		for (int t=0; t < n; t++)
		{
			MinHeap<StringNode> testHeap = new MinHeap<>();
			Map<String,Double> ranMap = randomMap(k,ran);
			for (String str : ranMap.keySet())
			{
				testHeap.add(new StringNode(str), ranMap.get(str));
			}
			List<String> heapList = new ArrayList<>();
			while (testHeap.size() > 0)
			{
				StringNode sn = testHeap.getKey(0);
				testHeap.remove(0);
				heapList.add(sn.string);
			}
			List<String> sortList = mappedSort(ranMap);
			assertEquals(heapList, sortList);
		}
	}
	
	public void testUpdates(int k, Random ran, int n)
	{
		for (int t=0; t < n; t++)
		{
			MaxHeap<StringNode> testHeap = new MaxHeap<>();
			Map<String,Double> ranMap = randomMap(k,ran);
			for (String str : ranMap.keySet())
			{
				testHeap.add(new StringNode(str), ran.nextDouble());
			}
			for (StringNode sn : testHeap)
			{
				if (ran.nextBoolean())
				{
					testHeap.update(sn.index, ran.nextDouble());
				}
			}
			for (StringNode sn : testHeap)
			{
				testHeap.update(sn.index, ranMap.get(sn.string));
			}
			List<String> heapList = new ArrayList<>();
			while (testHeap.size() > 0)
			{
				StringNode sn = testHeap.getKey(0);
				testHeap.remove(0);
				heapList.add(sn.string);
			}
			List<String> sortList = mappedSort(ranMap);
			Collections.reverse(sortList);
			assertEquals(heapList, sortList);
		}
	}
	
	public void testUpdatesMin(int k, Random ran, int n)
	{
		for (int t=0; t < n; t++)
		{
			MinHeap<StringNode> testHeap = new MinHeap<>();
			Map<String,Double> ranMap = randomMap(k,ran);
			for (String str : ranMap.keySet())
			{
				testHeap.add(new StringNode(str), ran.nextDouble());
			}
			for (StringNode sn : testHeap)
			{
				if (ran.nextBoolean())
				{
					testHeap.update(sn.index, ran.nextDouble());
				}
			}
			for (StringNode sn : testHeap)
			{
				testHeap.update(sn.index, ranMap.get(sn.string));
			}
			List<String> heapList = new ArrayList<>();
			while (testHeap.size() > 0)
			{
				StringNode sn = testHeap.getKey(0);
				testHeap.remove(0);
				heapList.add(sn.string);
			}
			List<String> sortList = mappedSort(ranMap);
			assertEquals(heapList, sortList);
		}
	}
	
	public List<Double> randomValues(int k, Random ran)
	{
		TreeSet<Double> doubles = new TreeSet<>();
		while (doubles.size() < k)
		{
			doubles.add(ran.nextDouble());
		}
		List<Double> result = new ArrayList<>(doubles);
		Collections.shuffle(result, ran);
		return result;
	}
	
	public List<String> randomStrings(int k, Random ran)
	{
		TreeSet<String> strings = new TreeSet<>();
		while (strings.size() < k)
		{
			strings.add(randomString(ran));
		}
		List<String> result = new ArrayList<>(strings);
		Collections.shuffle(result, ran);
		return result;
	}
	
	public String randomString(Random ran)
	{
		String constant = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		char [] chars = new char[10];
		for (int t=0; t < chars.length; t++)
		{
			char c = constant.charAt(ran.nextInt(constant.length()));
			if (ran.nextBoolean())
			{
				c = Character.toLowerCase(c);
			}
			chars[t] = c;
		}
		return String.valueOf(chars);
	}
	
	public Map<String,Double> randomMap(int k, Random ran)
	{
		List<String> randomStrings = randomStrings(k, ran);
		List<Double> randomDoubles = randomValues(k, ran);
		Map<String,Double> resMap = new HashMap<>();
		for (int t=0; t < k; t++)
		{
			resMap.put(randomStrings.get(t), randomDoubles.get(t));
		}
		return resMap;
	}
	
	public List<String> mappedSort(Map<String,Double> map)
	{
		List<String> list = new ArrayList<>(map.keySet());
		list.sort((String s1, String s2) -> map.get(s1).compareTo(map.get(s2)));
		return list;
	}
	
	public static class StringNode implements HeapIndexChangedListener
	{

		public int index;
		public final String string;
		
		public StringNode(String str)
		{
			string = str;
		}
		
		@Override
		public void notifyHeapIndex(int index)
		{
			this.index = index;
		}
		
		@Override
		public String toString()
		{
			return "["+index+"] : "+string;
		}
		
	}
}
