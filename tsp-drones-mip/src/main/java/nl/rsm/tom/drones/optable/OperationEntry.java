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
package nl.rsm.tom.drones.optable;

import nl.rsm.tom.drones.data.Operation;

/**
 * Dynamic programming based operation entries that can be stored in a table
 * @author Paul Bouman
 *
 * @param <E> the type of locations in the networks
 * @param <S> the data type that is used to represent a set of locations
 */
public interface OperationEntry<E,S>
{
	/**
	 * Derives an operation associated with this operation entry
	 * @return a operation computed based on this entry
	 */
	public Operation<E> getOperation();
	
	/**
	 * Retrieve the set of locations covered by the operation represented
	 * by this entry
	 * @return the set of covered locations
	 */
	public S getSet();
	
	/**
	 * Retrieve the origin location of the operation modeled by this entry
	 * @return a origin location
	 */
	public E getFrom();
	
	/**
	 * Retrieve the destination location of the operation modeled by this entry
	 * @return the destination location
	 */
	public E getTo();
	
	/**
	 * Retrieve the location the drone visits in the operation modelen by this entry
	 * @return the location the drone visits
	 */
	public E getFly();
	
	/**
	 * Retrieve the costs of driving the truck
	 * @return the costs for the truck
	 */
	public double getDriveCost();
	
	/**
	 * Retrieve the costs of flying the drone
	 * @return the costs of the drone
	 */
	public double getFlyCost();
	
	/**
	 * Retrieve the costs of the operation associated with this entry. This is the
	 * maximum of the costs for the truck and the drone 
	 * @return the costs of the operation
	 */
	public double getCost();
	
	/**
	 * Get the number of locations in covered by the operation represented by this entry.
	 * @return the number of locations in this operation
	 */
	public int nodeCount();
	
	/**
	 * Get the number of locations that are only visited by the truck.
	 * @return the number of locations only visited by the truck
	 */
	public int getTruckOnlyCount();
	
	/**
	 * Get the previous entry in the dynamic programming recurrence.
	 * @return the previous operation entry in the dynammic programming recurrence.
	 */
	public OperationEntry<E,S> getPrev();
}
