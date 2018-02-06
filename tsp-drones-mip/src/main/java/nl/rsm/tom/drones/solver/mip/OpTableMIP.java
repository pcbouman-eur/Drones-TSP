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
package nl.rsm.tom.drones.solver.mip;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.paukov.combinatorics.CombinatoricsVector;
import org.paukov.combinatorics.ICombinatoricsVector;
import org.paukov.combinatorics.subsets.SubSetGenerator;

import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;
import nl.rsm.tom.drones.data.Operation;
import nl.rsm.tom.drones.data.Solution;
import nl.rsm.tom.drones.data.instance.Instance;
import nl.rsm.tom.drones.optable.AbstractOpTable;
import nl.rsm.tom.drones.util.EulerTourTools;

/**
 * Mixed-Integer Programming Model for the Travelling Salesman Problem with Drones.
 * This model assumes a fixed table of operations is provided, which are then included
 * as decision variables. The MIP searches for a Eulerian subgraph of operations that
 * cover all locations in the TSP-D. This Eulerian subgraph can be converted into a
 * Eulerian walk of operations that is the final solution.
 * @author Paul Bouman
 *
 * @param <E> The type of locations in the instance
 * @param <S> The encoding of sets of locations in the operations table
 */
public class OpTableMIP<E,S>
{
	private final static double THRESHOLD = 10e-8;
	
	private AbstractOpTable<E,S> table;
	private IloCplex model;
	private Instance<E> instance;
	
	private Map<IloNumVar,Operation<E>> xMap;
	private Map<Operation<E>,IloNumVar> opMap;
	
	private Map<IloNumVar,E> zMap;
	private Map<E,IloNumVar> locMap;
	
	/**
	 * A constructor that takes a table of operations and initializes a MIP
	 * model based on this table of operations.
	 * @param tab A table of operations
	 * @throws IloException if CPLEX runs into trouble
	 */
	public OpTableMIP(AbstractOpTable<E,S> tab) throws IloException
	{
		table = tab;
		instance = table.getInstance();
		model = new IloCplex();
		
		xMap = new HashMap<>();
		opMap = new HashMap<>();
		zMap = new HashMap<>();
		locMap = new HashMap<>();
		
		initVars();
		addCoverConstraints();
		addCountConstraints();
		addCoverDepot();
		addSubtourConstraints(true);
		initObj();
		model.setOut(null);
	}
	
	/**
	 * Mostly a constructor that is useful for testing: it builds a
	 * MIP model using only the operations in the provided solution.
	 * Typically, the MIP should be able to find this solution.
	 * @param sol the solution to be reconstructed by the MIP
	 * @throws IloException if CPLEX runs into trouble
	 */
	public OpTableMIP(Solution<E> sol) throws IloException
	{
		instance = sol.getInstance();
		model = new IloCplex();
		
		xMap = new HashMap<>();
		opMap = new HashMap<>();
		zMap = new HashMap<>();
		locMap = new HashMap<>();
		
		initVars(sol);
		addCoverConstraints();
		addCountConstraints();
		addCoverDepot();
		addSubtourConstraints(false);
		initObj();
		model.setOut(null);
	}
	
	/**
	 * Solves the MIP model
	 * @throws IloException if CPLEX runs into trouble
	 */
	public void solve() throws IloException
	{
		model.solve();
	}

	/**
	 * 
	 * @return
	 * @throws IloException
	 */
	public Solution<E> getSolution() throws IloException
	{
		List<Operation<E>> ops = new ArrayList<>();
		for (IloNumVar var : xMap.keySet())
		{
			double val = model.getValue(var);
			if (val >= 1 - THRESHOLD)
			{
				Operation<E> op = xMap.get(var);
				ops.add(op);
			}
		}
		return EulerTourTools.buildSolution(ops, instance);
	}

	private void initVars() throws IloException
	{
		for (E from : instance)
		{
			for (E to : instance)
			{
				for (Operation<E> op : table.getOperations(from, to))
				{
					IloIntVar x = model.boolVar();
					xMap.put(x, op);
					opMap.put(op, x);
				}
			}
			IloIntVar z = model.boolVar();
			if (instance.isDepot(from))
			{
				z.setLB(1);
			}
			zMap.put(z, from);
			locMap.put(from, z);
		}
	}

	private void initVars(Solution<E> sol) throws IloException
	{
		for (Operation<E> op : sol)
		{
			String name = "x_"+op.getStart()+"_"+op.getEnd();
			
			IloIntVar x = model.boolVar(name);
			xMap.put(x, op);
			opMap.put(op, x);
		}
		for (E loc : instance)
		{
			IloIntVar z = model.boolVar("z_"+loc);
			if (instance.isDepot(loc))
			{
				z.setLB(1);
			}
			zMap.put(z, loc);
			locMap.put(loc, z);
		}
	}
	
	private void initObj() throws IloException
	{
		IloNumExpr expr = model.constant(0);
		for (Operation<E> op : opMap.keySet())
		{
			IloNumVar var = opMap.get(op);
			IloNumExpr term = model.prod(op.getCost(instance), var);
			expr = model.sum(expr, term);
		}
		model.addMinimize(expr);
	}
	
	private void addCoverConstraints() throws IloException
	{
		for (E loc : instance)
		{
			IloNumExpr expr = model.constant(0);
			
			for (Operation<E> op : opMap.keySet())
			{
				if (		op.getInternalNodes(true).contains(loc)
						||	op.getStart().equals(loc)
						||  op.getEnd().equals(loc))
				{
					IloNumVar var = opMap.get(op);
					expr = model.sum(expr, var);
				}
			}
					
			model.addGe(expr, 1, "cover_"+loc);
		}
	}
	
	private void addCoverDepot() throws IloException
	{
		IloNumExpr expr = model.constant(0);
		
		for (Operation<E> op : opMap.keySet())
		{
			if (op.getEnd().equals(instance.getDepot()))
			{
				expr = model.sum(expr, opMap.get(op));
			}
		}
		
		model.addGe(expr, 1, "cover_depot");
	}
	
	private void addCountConstraints() throws IloException
	{
		for (E loc : instance)
		{
			
			IloNumExpr expr1 = model.constant(0);
			IloNumExpr expr2 = model.constant(0);
			for (Operation<E> op : opMap.keySet())
			{
				if (op.getStart().equals(loc))
				{
					IloNumVar var = opMap.get(op);
					expr1 = model.sum(expr1,var);
				}
				if (op.getEnd().equals(loc))
				{
					IloNumVar var = opMap.get(op);
					expr2 = model.sum(expr2,var);
				}
			}
			model.addEq(expr1, expr2, "inout_"+loc);
			
			IloNumVar z = locMap.get(loc);
			IloNumExpr rhs = model.prod(z, instance.getNodeCount());
			model.addLe(expr2, rhs, "count_"+loc);
		}
	}
	
	private void addSubtourConstraints(boolean addLazy) throws IloException
	{
		CombinatoricsVector<E> cv = new CombinatoricsVector<>();
		for (E loc : instance)
		{
			cv.addValue(loc);
		}
		SubSetGenerator<E> ssg = new SubSetGenerator<>(cv);
		
		for (ICombinatoricsVector<E> vec : ssg)
		{
			if (vec.contains(instance.getDepot()))
			{
				continue;
			}

			if (vec.getSize() > 0 && vec.getSize() < cv.getSize())
			{
				IloNumExpr lhs = model.constant(0);
				for (Operation<E> op : opMap.keySet())
				{
					if (!vec.contains(op.getStart()) && vec.contains(op.getEnd()))
					{
						IloNumVar var = opMap.get(op);
						lhs = model.sum(lhs, var);
					}
				}
				
				for (E loc : vec)
				{	

					String name = "st";
					for (E l : vec)
					{
						name += "_"+l;
					}
					name += "_z"+loc;
					
					IloNumVar var = locMap.get(loc);
					if (addLazy)
					{
						
						IloRange constraint = model.ge(model.diff(lhs, var), 0, name);
						model.addLazyConstraint(constraint);
					}
					else
					{
						model.addGe(lhs, var, name);
					}
				}
			}
		}
		
	}
	
	/**
	 * This clears the model from memory.
	 * @throws IloException if something goes wrong with CPLEX
	 */
	public void clear() throws IloException
	{
		model.clearModel();
		model.endModel();
		model.end();
		model = null;
	}
	
	@Override
	public void finalize()
	{
		if (model != null) {
			try
			{
				clear();
			}
			catch (IloException ie) {}
		}
	}

	
}
