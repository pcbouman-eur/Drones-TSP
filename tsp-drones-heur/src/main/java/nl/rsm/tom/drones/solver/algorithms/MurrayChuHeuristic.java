package nl.rsm.tom.drones.solver.algorithms;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;

/**
 * Implementation of the Murray and Chu heuristic, proposed in the paper
 * 
 * Chase C. Murray, Amanda G. Chu, The flying sidekick traveling salesman problem:
 * Optimization of drone-assisted parcel delivery,
 * Transportation Research Part C: Emerging Technologies, Volume 54, 2015, Pages 86-109,
 * ISSN 0968-090X, https://doi.org/10.1016/j.trc.2015.03.005.
 * 
 * This implementation reinterprets their heuristic as a greedy neighbourhood search
 * that explores the full neighbourhood in every step of the algorithm. Note that the
 * algorithm assumes an initial order of the nodes will be provided, but that reordering
 * is one of the steps that can be performed in the neighbourhood search.
 * 
 * @author Paul Bouman
 *
 * @param <E> the type of the locations in the instance
 */
public class MurrayChuHeuristic<E>
{
	private Instance<E> instance;
	
	private LSNode<E> begin;
	
	/**
	 * Constructor that sets up a structure on which the neighbourhood search can be
	 * performed.
	 * @param i the instance that has to be solved
	 * @param sol the initial solution on which the heuristic tries to improve.
	 */
	public MurrayChuHeuristic(Instance<E> i, Solution<E> sol)
	{
		instance = i;
		
		LSNode<E> prev = null;
		for (E loc : sol.getDriveOrderList())
		{
			LSNode<E> cur = new LSNode<E>(loc);
			if (begin == null)
			{
				begin = cur;
			}
			cur.prev = prev;
			if (prev != null)
			{
				prev.next = cur;
			}
			prev = cur;
		}
	}
	
	private double getCost()
	{
		return getSolution().getTotalCost();
	}
	
	/**
	 * Explores the full neighbourhood and picks the best one.
	 * Indicates if an improvement was found
	 * @return true if an improvement was found
	 */
	public boolean step()
	{
		double curValue = getCost();
		Action best = null;
		double bestSavings = 0;
		for (Action a : getNeighbourhood())
		{
			a.doAction();
			double newCost = getCost();
			Solution<E> sol = getSolution();
			newCost = sol.getTotalCost();
			double savings = curValue - newCost;
			if (savings > bestSavings)
			{
				best = a;
				bestSavings = savings;
			}
			a.undoAction();
		}
		if (best != null)
		{
			best.doAction();
			return true;
		}
		return false;
	}
	
	/**
	 * Runs the heuristic as long as improvements are found
	 */
	public void runHeuristic()
	{
		while (step()) {}
	}
	
	/**
	 * Obtain the current solution found by the heuristic
	 * @return the current solution
	 */
	public Solution<E> getSolution()
	{
		List<Operation<E>> res = new ArrayList<>();
		LSNode<E> cur = begin;
		while (cur.next != null)
		{
			Operation<E> op = getOp(cur);
			res.add(op);
			if (cur.nextFly != null)
			{
				cur = cur.nextFly.nextFly;
			}
			else
			{
				cur = cur.next;
			}
		}
		Solution<E> result = new Solution<E>(instance,res);
		if (!result.isFeasible())
		{
			throw new IllegalStateException("The current solution is infeasible!");
		}
		return result;
	}
	
	private Operation<E> getOp(LSNode<E> node)
	{
		if (node.next == null)
		{
			throw new IllegalArgumentException("Cannot build an operation out of the last local search node");
		}
		
		if (node.nextFly == null)
		{
			E from = node.location;
			E to = node.next.location;
			return new Operation<E>(from,to);
		}
		
		E start = node.location;
		E fly = node.nextFly.location;
		
		List<E> path = new ArrayList<>();
		LSNode<E> cur = node.nextFly.nextFly;
		while (cur != node)
		{
			path.add(cur.location);
			cur = cur.prev;
		}
		path.add(start);
		Collections.reverse(path);
		return new Operation<E>(path,fly);
	}
	
	
	private List<Action> getNeighbourhood()
	{
		ArrayList<Action> res = new ArrayList<>();
		res.addAll(driveNeighbourhood());
		res.addAll(flyNeighbourhood());
		return res;
	}
	
	private List<TruckAction<E>> driveNeighbourhood()
	{
		ArrayList<TruckAction<E>> result = new ArrayList<>();
		for (LSNode<E> source : begin)
		{
			if (!instance.isDepot(source.location))
			{
				for (LSNode<E> target : begin)
				{
					if (target.next != null && target.next != source && target != source)
					{
						TruckAction<E> action = new TruckAction<E>(source, target);
						if (action.checkLegal())
						{
							result.add(action);
						}
					}
				}
			}
		}
		return result;
	}
	
	private List<DroneAction<E>> flyNeighbourhood()
	{
		ArrayList<DroneAction<E>> result = new ArrayList<>();
		for (LSNode<E> source : begin)
		{
			if (!instance.isDepot(source.location))
			{
				for (LSNode<E> targetFrom : begin)
				{
					if (targetFrom.next != null && targetFrom.next != source && targetFrom != source)
					{
						for (LSNode<E> targetTo : targetFrom.next)
						{
							if (targetTo != null && targetTo != source)
							{
								DroneAction<E> da = new DroneAction<E>(source,targetFrom,targetTo);
								if (da.checkLegal())
								{
									result.add(da);
								}
							}
						}
					}
				}
			}
		}
		return result;
	}
		
	
	private static class LSNode<E> implements Iterable<LSNode<E>>
	{
		private final E location;
		public LSNode<E> prev;
		public LSNode<E> next;
		
		public LSNode<E> nextFly;
		public LSNode<E> prevFly;
			
		public LSNode(E loc)
		{
			location = loc;
		}

		
		@Override
		public Iterator<LSNode<E>> iterator()
		{
			final LSNode<E> itStart = this;
			return new Iterator<LSNode<E>>()
			{
				LSNode<E> cur = itStart;

				@Override
				public boolean hasNext()
				{
					return cur != null;
				}

				@Override
				public LSNode<E> next()
				{
					LSNode<E> result = cur;
					cur = cur.next;
					return result;
				}
			};
		}
	}
	
	private static interface Action
	{
		public void doAction();
		public void undoAction();
		public boolean checkLegal();
	}
	
	private static class TruckAction<E> implements Action
	{
		private final LSNode<E> subject;
		
		private LSNode<E> source;
		private final LSNode<E> target;
		
		public TruckAction(LSNode<E> subject, LSNode<E> target)
		{
			this.subject = subject;
			this.target = target;
			this.source = subject.prev;
		}
		
		@Override
		public boolean checkLegal()
		{
			return subject.nextFly == null && subject.prevFly == null;
		}
		
		@Override
		public void doAction()
		{
			if (target.next == subject)
			{
				throw new IllegalStateException("This action has already been performed!");
			}
			if (subject.nextFly != null || subject.prevFly != null)
			{
				throw new IllegalStateException("This action is invalid because it involved a Drone subject.");
			}
			
			LSNode<E> tLeft = target;
			LSNode<E> tRight = target.next;
			LSNode<E> sRight = subject.next;
			
			tLeft.next = subject;
			subject.prev = tLeft;
			subject.next = tRight;
			tRight.prev = subject;
			
			source.next = sRight;
			sRight.prev = source;
			
			
		}
		
		@Override
		public void undoAction()
		{
			if (source.next == subject)
			{
				throw new IllegalStateException("This action has not been performed or is already undone!");
			}
			if (subject.nextFly != null || subject.prevFly != null)
			{
				throw new IllegalStateException("This action is invalid becouse it involved a Drone subject.");
			}
			
			LSNode<E> sRight = source.next;
			LSNode<E> oLeft = subject.prev;
			LSNode<E> oRight = subject.next;
			
			source.next = subject;
			subject.prev = source;
			subject.next = sRight;
			sRight.prev = subject;
			
			oLeft.next = oRight;
			oRight.prev = oLeft;
		}
		
		@Override
		public String toString()
		{
			return "Move "+source.location+" to "+target.location+" as truck.";
		}
	}
	
	private static class DroneAction<E> implements Action
	{
		private final LSNode<E> subject;
		private final LSNode<E> source;
		private final LSNode<E> targetFrom;
		private final LSNode<E> targetTo;
		
		public DroneAction(LSNode<E> subject, LSNode<E> targetFrom, LSNode<E> targetTo)
		{
			this.subject = subject;
			this.targetFrom = targetFrom;
			this.targetTo = targetTo;
			this.source = subject.prev;
		}

		@Override
		public boolean checkLegal()
		{
			if (subject.nextFly != null || subject.prevFly != null)
			{
				return false;
			}
			LSNode<E> cur = targetFrom;

			// Case 2 (Drone flies already flies to another node)
			if (cur.nextFly != null)
			{
				return false;
			}
			
			while (cur != null && cur != targetTo)
			{
				// Either Case 1 (Drone attaches To Truck before target is reached)
				//     or Case 3 (Drone flies away before target is reached
				if (cur.prevFly != null || cur.nextFly != null)
				{
					return false;
				}
				cur = cur.next;
			}
			if (cur != targetTo)
			{
				return false;
			}
			// We know cur = targetTo
			while (cur != null)
			{
				// Case 5 (Drone returns and must have been away before this operation started)
				if (cur.prevFly != null)
				{
					return false;
				}
				// Case 4 (Drone flies away - it must have been attached to the truck all along)
				if (cur.nextFly != null)
				{
					return true;
				}
				cur = cur.next;
			}
			// We have reached the end; apperently there are no drone conflicts so this action is safe.
			return true;
		}
		
		@Override
		public void doAction()
		{
			if (targetFrom.nextFly == subject)
			{
				throw new IllegalStateException("This action has already been performed!");
			}
			if (subject.nextFly != null || subject.prevFly != null)
			{
				throw new IllegalStateException("This action is invalid becouse it involved a Drone subject.");
			}

			LSNode<E> oRight = subject.next;
			
			source.next = oRight;
			oRight.prev = source;

			targetFrom.nextFly = subject;
			subject.prevFly = targetFrom;
			subject.prev = null;
			subject.next = null;
			subject.nextFly = targetTo;
			targetTo.prevFly = subject;
		}
		
		@Override
		public void undoAction()
		{
			if (source.next == subject)
			{
				throw new IllegalStateException("This action has not been performed or is already undone!");
			}
			if (subject.nextFly == null || subject.prevFly == null)
			{
				throw new IllegalStateException("This action is invalid because it is a drone action without fly locations.");
			}

			LSNode<E> oRight = source.next;
			
			targetFrom.nextFly = null;
			targetTo.prevFly = null;
			subject.nextFly = null;
			subject.prevFly = null;
			
			source.next = subject;
			subject.prev = source;
			subject.next = oRight;
			oRight.prev = subject;
		}
		
		@Override
		public String toString()
		{
			return "Fly from "+targetFrom.location+" via "+subject.location+" to "+targetTo.location+".";
		}
	}

}
