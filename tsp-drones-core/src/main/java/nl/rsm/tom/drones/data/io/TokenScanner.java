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

package nl.rsm.tom.drones.data.io;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.Locale;
import java.util.Scanner;
import java.util.regex.Pattern;

public class TokenScanner
{
	private static Pattern COMMENT_PATTERN = Pattern.compile("/\\*.*?\\*/", Pattern.MULTILINE + Pattern.DOTALL);
	private static Pattern SPACE_PATTERN = Pattern.compile("[\\n\\r\\s]", Pattern.MULTILINE);
	private static Pattern MULTIPLE_SPACE_PATTERN = Pattern.compile("\\s\\s+", Pattern.MULTILINE);
	private static String IDENTIFIER_STRING = "A-Za-z0-9_=\\-\\[\\],\\.";
	private static Pattern IDENTIFIER = Pattern.compile("["+IDENTIFIER_STRING+"]+");
	
	/**
	 * Method that replaces all non-identifier symbols in a string with underscores
	 * @param in the input string
	 * @return a valid identifier string.
	 */
	public static String transform(String in)
	{
		return in.replaceAll("[^"+IDENTIFIER_STRING+"]", "_");
	}
	
	/**
	 * Filters out comments
	 * @param c The input string
	 * @return The string without comments
	 */
	
	public static String removeComments(CharSequence c)
	{
		String stripped = COMMENT_PATTERN.matcher(c).replaceAll("");
		String reduced = MULTIPLE_SPACE_PATTERN.matcher(stripped).replaceAll(" ");
		reduced = SPACE_PATTERN.matcher(reduced).replaceAll(" ");
		return reduced.trim();
	}
	
	private final Scanner _scanner;
	
	/**
	 * Creates a TokenScanner on the input string with the comments removed.
	 * @param input the input string
	 */
	public TokenScanner(String input)
	{
		this(input,true);
	}
	
	/**
	 * Creates a TokenScanner on the input string with a boolean indicating
	 * whether comments should be removed
	 * @param input the string that should be scanned
	 * @param removeComments whether the string should be filtered for comments or not
	 */
	
	public TokenScanner(String input, boolean removeComments)
	{
		if (removeComments)
		{
			_scanner = new Scanner(removeComments(input));
		}
		else
		{
			_scanner = new Scanner(input);
		}
		_scanner.useLocale(Locale.US);
	}

	/**
	 * Closes the scanner.
	 */
	public void close() {
		_scanner.close();
	}

	/**
	 * Whether there is a token left
	 * @return whether the is another token
	 */
	public boolean hasNext() {
		return _scanner.hasNext();
	}

	/**
	 * Whether the next token has a certain pattern
	 * @param arg0 the pattern 
	 * @return whether there is another token
	 */
	public boolean hasNext(Pattern arg0) {
		return _scanner.hasNext(arg0);
	}
	
	/**
	 * @return Whether the next token is an identifier
	 */
	public boolean hasNextIdentifier() {
		return _scanner.hasNext(IDENTIFIER);
	}

	/**
	 * @return whether the next token is as Big Decimal
 	 */
	public boolean hasNextBigDecimal() {
		return _scanner.hasNextBigDecimal();
	}

	/**
	 * @return whether the next token is a Big Integer
	 */
	public boolean hasNextBigInteger() {
		return _scanner.hasNextBigInteger();
	}

	/**
	 * @return whether the next token is a boolean
	 */
	public boolean hasNextBoolean() {
		return _scanner.hasNextBoolean();
	}

	/**
	 * @return whether the next token is a byte
	 */
	public boolean hasNextByte() {
		return _scanner.hasNextByte();
	}

	/**
	 * @return whether the next token is a double
	 */
	public boolean hasNextDouble() {
		return _scanner.hasNextDouble();
	}

	/**
	 * @return whether the next token is a float
	 */
	public boolean hasNextFloat() {
		return _scanner.hasNextFloat();
	}

	/**
	 * @return whether the next token is an int
	 */
	public boolean hasNextInt() {
		return _scanner.hasNextInt();
	}

	/**
	 * @return whether the next token is a long
	 */
	public boolean hasNextLong() {
		return _scanner.hasNextLong();
	}

	/**
	 * @return whether the next token is a short
	 */
	public boolean hasNextShort() {
		return _scanner.hasNextShort();
	}

	/**
	 * @return returns the string of the next token
	 */
	public String next() {
		return _scanner.next();
	}

	/**
	 * @param arg0 the pattern to be matched
	 * @return the pattern 
	 */
	public String next(Pattern arg0) {
		return _scanner.next(arg0);
	}

	/**
	 * @return the next token if it is an identifier
	 */
	public String nextIdentifier() {
		return _scanner.next(IDENTIFIER);
	}
	
	/**
	 * @return the next token if it is a BigDecimal
	 */
	
	public BigDecimal nextBigDecimal() {
		return _scanner.nextBigDecimal();
	}

	/**
	 * @return the next token if it is a BigInteger
	 */
	public BigInteger nextBigInteger() {
		return _scanner.nextBigInteger();
	}

	/**
	 * @return the next token if it is a Boolean
	 */
	public boolean nextBoolean() {
		return _scanner.nextBoolean();
	}

	/**
	 * @return the next token as a byte
	 */
	public byte nextByte()
	{
		return _scanner.nextByte();
	}

	/**
	 * @return the next token if it is a double
	 */
	public double nextDouble() {
		return _scanner.nextDouble();
	}

	/**
	 * @return the next token if it is a float
	 */
	public float nextFloat() {
		return _scanner.nextFloat();
	}

	/**
	 * @return the next token if it is an int
	 */
	public int nextInt() {
		return _scanner.nextInt();
	}

	/**
	 * @return the next token if it a long
	 */
	public long nextLong() {
		return _scanner.nextLong();
	}

	/**
	 * @return the next token if it is a short
	 */
	public short nextShort() {
		return _scanner.nextShort();
	}

	/**
	 * @return a scanner from the beginning of the sequence
	 */
	public Scanner reset() {
		return _scanner.reset();
	}

	/**
	 * Derives a scanner that skips a certain pattern
	 * @param arg0 the pattern that will skipped 
	 * @return the pattern which will be skipped
	 */
	public Scanner skip(Pattern arg0){
		return _scanner.skip(arg0);
	}

	public String toString()
	{
		return _scanner.toString();
	}	
}
