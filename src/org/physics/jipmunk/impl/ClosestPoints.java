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

import static org.physics.jipmunk.Assert.cpAssertWarn;
import static org.physics.jipmunk.Util.*;

/**
 * @author jobernolte
 */
public class ClosestPoints {
	private static boolean ENABLE_CACHING = true;
	private final static int MAX_GJK_ITERATIONS = 30;
	private final static int WARN_GJK_ITERATIONS = 20;
	private final static int MAX_EPA_ITERATIONS = 30;
	private final static int WARN_EPA_ITERATIONS = 20;
	public final Vector2f a, b;
	public final Vector2f n;
	public final float d;
	public final CollisionID id;

	public ClosestPoints(Vector2f a, Vector2f b, Vector2f n, float d, CollisionID id) {
		this.a = a;
		this.b = b;
		this.n = n;
		this.d = d;
		this.id = id;
	}

	static float closestT(Vector2f a, Vector2f b) {
		Vector2f delta = cpvsub(b, a);
		return -cpfclamp(cpvdot(delta, cpvadd(a, b)) / cpvlengthsq(delta), -1.0f, 1.0f);
	}

	static Vector2f lerpT(Vector2f a, Vector2f b, float t) {
		float ht = 0.5f * t;
		return cpvadd(cpvmult(a, 0.5f - ht), cpvmult(b, 0.5f + ht));
	}

	static ClosestPoints create(MinkowskiPoint v0, MinkowskiPoint v1) {
		float t = closestT(v0.getAb(), v1.getAb());
		Vector2f p = lerpT(v0.getAb(), v1.getAb(), t);

		Vector2f pa = lerpT(v0.getA(), v1.getA(), t);
		Vector2f pb = lerpT(v0.getB(), v1.getB(), t);
		CollisionID id = new CollisionID((v0.getId().getValue() & 0xFFFF) << 16 | (v1.getId().getValue() & 0xFFFF));

		Vector2f delta = cpvsub(v1.getAb(), v0.getAb());
		Vector2f n = cpvnormalize(cpvperp(delta));
		float d = -cpvdot(n, p);

		if (d <= 0.0f || (0.0f < t && t < 1.0f)) {
			return new ClosestPoints(pa, pb, cpvneg(n), d, id);
		} else {
			float d2 = cpvlength(p);
			Vector2f n2 = cpvnormalize(cpvmult(p, 1.0f / (d2 + Float.MIN_VALUE)));

			return new ClosestPoints(pa, pb, n2, d2, id);
		}
	}

	public static ClosestPoints GJK(SupportContext ctx, CollisionID id) {

		MinkowskiPoint v0, v1;
		if (id.getValue() != 0 && ENABLE_CACHING) {
			v0 = MinkowskiPoint.create(SupportPoint.shapePoint(ctx.shape1, (id.getValue() >> 24) & 0xFF),
									   SupportPoint.shapePoint(ctx.shape2, (id.getValue() >> 16) & 0xFF));
			v1 = MinkowskiPoint.create(SupportPoint.shapePoint(ctx.shape1, (id.getValue() >> 8) & 0xFF),
									   SupportPoint.shapePoint(ctx.shape2, (id.getValue()) & 0xFF));
		} else {
			Vector2f axis = cpvperp(cpvsub(ctx.shape1.getBB().getCenter(), ctx.shape2.getBB().getCenter()));
			v0 = MinkowskiPoint.support(ctx, axis);
			v1 = MinkowskiPoint.support(ctx, cpvneg(axis));
		}

		ClosestPoints points = GJKRecurse(ctx, v0, v1, 1);
		id.setValue(points.id.getValue());
		return points;
	}

	static ClosestPoints GJKRecurse(final SupportContext ctx, final MinkowskiPoint v0, final MinkowskiPoint v1,
			int iteration) {
		if (iteration > MAX_GJK_ITERATIONS) {
			cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: " + iteration);
			return create(v0, v1);
		}

		Vector2f delta = cpvsub(v1.getAb(), v0.getAb());
		if (cpvcross(delta, cpvadd(v0.getAb(), v1.getAb())) > 0.0f) {
			// Origin is behind axis. Flip and try again.
			return GJKRecurse(ctx, v1, v0, iteration + 1);
		} else {
			float t = closestT(v0.getAb(), v1.getAb());
			Vector2f n = (-1.0f < t && t < 1.0f ? cpvperp(delta) : cpvneg(lerpT(v0.getAb(), v1.getAb(), t)));
			MinkowskiPoint p = MinkowskiPoint.support(ctx, n);

			if (cpvcross(cpvsub(v1.getAb(), p.getAb()), cpvadd(v1.getAb(), p.getAb())) > 0.0f
					&& cpvcross(cpvsub(v0.getAb(), p.getAb()), cpvadd(v0.getAb(), p.getAb())) < 0.0f) {
				cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK->EPA iterations: " + iteration);
				// The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
				return EPA(ctx, v0, p, v1);
			} else {
				// The new point must be farther along the normal than the existing points.
				if (cpvdot(p.getAb(), n) <= cpfmax(cpvdot(v0.getAb(), n), cpvdot(v1.getAb(), n))) {
					cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: " + iteration);
					return create(v0, v1);
				} else {
					if (ClosestDist(v0.getAb(), p.getAb()) < ClosestDist(p.getAb(), v1.getAb())) {
						return GJKRecurse(ctx, v0, p, iteration + 1);
					} else {
						return GJKRecurse(ctx, p, v1, iteration + 1);
					}
				}
			}
		}
	}

	static float ClosestDist(final Vector2f v0, final Vector2f v1) {
		return cpvlengthsq(lerpT(v0, v1, closestT(v0, v1)));
	}

	static ClosestPoints EPARecurse(final SupportContext ctx, final int count, final MinkowskiPoint[] hull,
			final int iteration) {
		int mini = 0;
		float minDist = Float.POSITIVE_INFINITY;

		// TODO: precalculate this when building the hull and save a step.
		for (int j = 0, i = count - 1; j < count; i = j, j++) {
			float d = ClosestDist(hull[i].getAb(), hull[j].getAb());
			if (d < minDist) {
				minDist = d;
				mini = i;
			}
		}

		MinkowskiPoint v0 = hull[mini];
		MinkowskiPoint v1 = hull[(mini + 1) % count];
		if (cpveql(v0.getAb(), v1.getAb())) {
			throw new IllegalStateException(
					String.format("Internal Error: EPA vertexes are the same (%d and %d)", mini, (mini + 1) % count));
		}

		MinkowskiPoint p = MinkowskiPoint.support(ctx, cpvperp(cpvsub(v1.getAb(), v0.getAb())));

		float area2x = cpvcross(cpvsub(v1.getAb(), v0.getAb()),
								cpvadd(cpvsub(p.getAb(), v0.getAb()), cpvsub(p.getAb(), v1.getAb())));
		if (area2x > 0.0f && iteration < MAX_EPA_ITERATIONS) {
			int count2 = 1;
			MinkowskiPoint[] hull2 = new MinkowskiPoint[count + 1];
			hull2[0] = p;

			for (int i = 0; i < count; i++) {
				int index = (mini + 1 + i) % count;

				Vector2f h0 = hull2[count2 - 1].getAb();
				Vector2f h1 = hull[index].getAb();
				Vector2f h2 = (i + 1 < count ? hull[(index + 1) % count] : p).getAb();

				// TODO: Should this be changed to an area2x check?
				if (cpvcross(cpvsub(h2, h0), cpvsub(h1, h0)) > 0.0f) {
					hull2[count2] = hull[index];
					count2++;
				}
			}

			return EPARecurse(ctx, count2, hull2, iteration + 1);
		} else {
			if (!(iteration < WARN_EPA_ITERATIONS)) {
				throw new IllegalStateException(String.format("High EPA iterations: %d", iteration));
			}
			return create(v0, v1);
		}
	}

	static ClosestPoints EPA(final SupportContext ctx, final MinkowskiPoint v0, final MinkowskiPoint v1,
			final MinkowskiPoint v2) {
		// TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse
		MinkowskiPoint[] hull = { v0, v1, v2 };
		return EPARecurse(ctx, 3, hull, 1);
	}
}

