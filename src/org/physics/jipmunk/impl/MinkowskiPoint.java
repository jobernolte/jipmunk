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

package org.physics.jipmunk.impl;

import org.physics.jipmunk.CollisionID;
import org.physics.jipmunk.Vector2f;

import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvsub;

/**
 * A point on the surface of two shape's minkowski difference.
 *
 * @author jobernolte
 */
public class MinkowskiPoint {
	/** Cache the two original support points. */
	private final Vector2f a, b;
	/** b - a */
	private final Vector2f ab;
	/** Concatenate the two support point indexes. */
	private final CollisionID id;

	public Vector2f getA() {
		return a;
	}

	public Vector2f getB() {
		return b;
	}

	public Vector2f getAb() {
		return ab;
	}

	public CollisionID getId() {
		return id;
	}

	public static MinkowskiPoint create(SupportPoint a, SupportPoint b) {
		return new MinkowskiPoint(a.getP(), b.getP(), cpvsub(b.getP(), a.getP()), new CollisionID(
				(a.getIndex().getValue() & 0xFF) << 8 | (b.getIndex().getValue() & 0xFF)));
	}

	/**
	 * Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
	 *
	 * @param ctx the {@link org.physics.jipmunk.impl.SupportContext} to use.
	 * @param n   the normal vector.
	 * @return a new {@link org.physics.jipmunk.impl.MinkowskiPoint} instance.
	 */
	public static MinkowskiPoint support(SupportContext ctx, Vector2f n) {
		SupportPoint a = ctx.func1.apply(ctx.shape1, cpvneg(n));
		SupportPoint b = ctx.func2.apply(ctx.shape2, n);
		return create(a, b);
	}

	public MinkowskiPoint(Vector2f a, Vector2f b, Vector2f ab, CollisionID id) {
		this.a = new Vector2f(a);
		this.b = new Vector2f(b);
		this.ab = new Vector2f(ab);
		this.id = id;
	}

	@Override
	public String toString() {
		return "MinkowskiPoint{" +
				"a=" + a +
				", b=" + b +
				", ab=" + ab +
				", id=" + id +
				'}';
	}
}
