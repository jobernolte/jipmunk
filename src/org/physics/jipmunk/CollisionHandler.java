/*
 * Copyright (c) 2007 Scott Lembcke, (c) 2011 Jürgen Obernolte
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.physics.jipmunk;

/** @author jobernolte */
public class CollisionHandler {

	final CollisionType typeA;
	final CollisionType typeB;
	CollisionBeginFunc beginFunc;
	CollisionPreSolveFunc preSolveFunc;
	CollisionPostSolveFunc postSolveFunc;
	CollisionSeparateFunc separateFunc;

	public CollisionHandler(CollisionType typeA, CollisionType typeB) {
		this.typeA = typeA;
		this.typeB = typeB;
	}

	public CollisionHandler(CollisionType typeA, CollisionType typeB, CollisionBeginFunc beginFunc,
			CollisionPreSolveFunc preSolveFunc, CollisionPostSolveFunc postSolveFunc,
			CollisionSeparateFunc separateFunc) {
		this.typeA = typeA;
		this.typeB = typeB;
		this.beginFunc = beginFunc;
		this.preSolveFunc = preSolveFunc;
		this.postSolveFunc = postSolveFunc;
		this.separateFunc = separateFunc;
	}

	public CollisionType getTypeA() {
		return typeA;
	}

	public CollisionType getTypeB() {
		return typeB;
	}

	public CollisionBeginFunc getBeginFunc() {
		return beginFunc;
	}

	public void setBeginFunc(CollisionBeginFunc beginFunc) {
		this.beginFunc = beginFunc;
	}

	public CollisionPreSolveFunc getPreSolveFunc() {
		return preSolveFunc;
	}

	public void setPreSolveFunc(CollisionPreSolveFunc preSolveFunc) {
		this.preSolveFunc = preSolveFunc;
	}

	public CollisionPostSolveFunc getPostSolveFunc() {
		return postSolveFunc;
	}

	public void setPostSolveFunc(CollisionPostSolveFunc postSolveFunc) {
		this.postSolveFunc = postSolveFunc;
	}

	public CollisionSeparateFunc getSeparateFunc() {
		return separateFunc;
	}

	public void setSeparateFunc(CollisionSeparateFunc separateFunc) {
		this.separateFunc = separateFunc;
	}

	public boolean begin(Arbiter arb, Space space) {
		return beginFunc.apply(arb, space);
	}

	public boolean preSolve(Arbiter arb, Space space) {
		return preSolveFunc.apply(arb, space);
	}

	public void postSolve(Arbiter arb, Space space) {
		postSolveFunc.apply(arb, space);
	}

	public void separate(Arbiter arb, Space space) {
		separateFunc.apply(arb, space);
	}

	public static boolean defaultBegin(Arbiter arb, Space space) {
		boolean retA = arb.callWildcardBeginA(space);
		boolean retB = arb.callWildcardBeginB(space);
		return retA && retB;
	}

	public static boolean defaultPreSolve(Arbiter arb, Space space) {
		boolean retA = arb.callWildcardPreSolveA(space);
		boolean retB = arb.callWildcardPreSolveB(space);
		return retA && retB;
	}

	public static void defaultPostSolve(Arbiter arb, Space space) {
		arb.callWildcardPostSolveA(space);
		arb.callWildcardPostSolveB(space);
	}

	public static void defaultSeparate(Arbiter arb, Space space) {
		arb.callWildcardSeparateA(space);
		arb.callWildcardSeparateB(space);
	}

	public static boolean alwaysCollide(Arbiter arb, Space space) {
		return true;
	}

	public static void doNothing(Arbiter arb, Space space) {
	}

	public static CollisionHandler createDefaultHandler() {
		return new CollisionHandler(CollisionType.WILDCARD, CollisionType.WILDCARD, CollisionHandler::defaultBegin,
									CollisionHandler::defaultPreSolve, CollisionHandler::defaultPostSolve,
									CollisionHandler::defaultSeparate);
	}

	public static CollisionHandler createDoNothingHandler() {
		return new CollisionHandler(CollisionType.WILDCARD, CollisionType.WILDCARD, CollisionHandler::alwaysCollide,
									CollisionHandler::alwaysCollide, CollisionHandler::doNothing,
									CollisionHandler::doNothing);
	}
}
