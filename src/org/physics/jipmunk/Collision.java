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

import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.Contact.cpContactInit;
import static org.physics.jipmunk.PolyShape.cpPolyShapeContainsVert;
import static org.physics.jipmunk.PolyShape.cpPolyShapeContainsVertPartial;
import static org.physics.jipmunk.PolyShape.cpPolyShapeValueOnAxis;
import static org.physics.jipmunk.Util.CP_HASH_PAIR;
import static org.physics.jipmunk.Util.cpfabs;
import static org.physics.jipmunk.Util.cpfmin;
import static org.physics.jipmunk.Util.cpfsqrt;
import static org.physics.jipmunk.Util.cpvadd;
import static org.physics.jipmunk.Util.cpvcross;
import static org.physics.jipmunk.Util.cpvdot;
import static org.physics.jipmunk.Util.cpvlengthsq;
import static org.physics.jipmunk.Util.cpvmult;
import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvproject;
import static org.physics.jipmunk.Util.cpvsub;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
class Collision {

	private final static DefaultVector2f ZERO_ONE = new DefaultVector2f(1, 0);

	// Add contact points for circle to circle collisions.
	// Used by several collision tests.
	static int circle2circleQuery(final Vector2f p1, final Vector2f p2, final float r1, final float r2,
			ContactList arr) {
		float mindist = r1 + r2;
		DefaultVector2f delta = new DefaultVector2f(p2).sub(p1);
		float distsq = cpvlengthsq(delta);
		if (distsq >= mindist * mindist) return 0;

		float dist = cpfsqrt(distsq);

		Contact con = nextContactPoint(arr);
		// Allocate and initialize the contact.
		final DefaultVector2f cpvmult = new DefaultVector2f(delta).mult(
				0.5f + (r1 - 0.5f * mindist) / (dist != 0 ? dist : Float.POSITIVE_INFINITY));
		final DefaultVector2f p = new DefaultVector2f(p1).add(cpvmult);
		final DefaultVector2f n = dist != 0 ? new DefaultVector2f(delta).mult(1.0f / dist) : ZERO_ONE;
		con.init(p, n, dist - mindist, 0);

		return 1;
	}

	// Collide circle shapes.
	static int circle2circle(final Shape shape1, final Shape shape2, ContactList arr) {
		CircleShape circ1 = (CircleShape) shape1; //TODO
		CircleShape circ2 = (CircleShape) shape2;

		return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r, arr);
	}

	static int segmentEncapQuery(Vector2f p1, Vector2f p2, float r1, float r2, ContactList arr, Vector2f tangent) {
		int count = circle2circleQuery(p1, p2, r1, r2, arr);
		//	printf("dot %5.2f\n", cpvdot(con[0].n, tangent));
		return (count > 0 && cpvdot(arr.get(0).n, tangent) >= 0.0 ? count : 0);
	}

	// Collide circles to segment shapes.
	static int circle2segment(final Shape circleShape, final Shape segmentShape, ContactList arr) {
		CircleShape circ = (CircleShape) circleShape;
		SegmentShape seg = (SegmentShape) segmentShape;

		// Radius sum
		float rsum = circ.r + seg.r;

		// Calculate normal distance from segment.
		float dn = cpvdot(seg.tn, circ.tc) - cpvdot(seg.ta, seg.tn);
		float dist = cpfabs(dn) - rsum;
		if (dist > 0.0f) return 0;

		// Calculate tangential distance along segment.
		float dt = -cpvcross(seg.tn, circ.tc);
		float dtMin = -cpvcross(seg.tn, seg.ta);
		float dtMax = -cpvcross(seg.tn, seg.tb);

		// Decision tree to decide which feature of the segment to collide with.
		if (dt < dtMin) {
			if (dt < (dtMin - rsum)) {
				return 0;
			} else {
				return segmentEncapQuery(circ.tc, seg.ta, circ.r, seg.r, arr, seg.a_tangent);
			}
		} else {
			if (dt < dtMax) {
				Vector2f n = (dn < 0.0f) ? seg.tn : cpvneg(seg.tn);
				Contact con = nextContactPoint(arr);
				con.init(
						cpvadd(circ.tc, cpvmult(n, circ.r + dist * 0.5f)),
						n,
						dist,
						0
				);
				return 1;
			} else {
				if (dt < (dtMax + rsum)) {
					return segmentEncapQuery(circ.tc, seg.tb, circ.r, seg.r, arr, seg.b_tangent);
				} else {
					return 0;
				}
			}
		}
	}

	// Helper function for working with contact buffers
	// This used to malloc/realloc memory on the fly but was repurposed.
	static Contact nextContactPoint(ContactList arr) {
		/*Contact con = new Contact();
				arr.add(con);
				return con;*/
		return arr.nextContactPoint();
	}

	static class FloatRef {
		float value;
	}

	// Find the minimum separating axis for the give poly and axis list.
	static int findMSA(final PolyShape poly, final SplittingPlane[] axes, final int num, FloatRef min_out) {
		int min_index = 0;
		float min = cpPolyShapeValueOnAxis(poly, axes[0].n, axes[0].d);
		if (min > 0.0f) return -1;

		for (int i = 1; i < num; i++) {
			float dist = cpPolyShapeValueOnAxis(poly, axes[i].n, axes[i].d);
			if (dist > 0.0f) {
				return -1;
			} else if (dist > min) {
				min = dist;
				min_index = i;
			}
		}

		min_out.value = min;
		return min_index;
	}

	// Add contacts for probably penetrating vertexes.
	// This handles the degenerate case where an overlap was detected, but no vertexes fall inside
	// the opposing polygon. (like a star of david)
	static int findVertsFallback(ContactList arr, final PolyShape poly1, final PolyShape poly2, final Vector2f n,
			final float dist) {
		int num = arr.size();

		for (int i = 0; i < poly1.verts.length; i++) {
			Vector2f v = poly1.tVerts[i];
			if (cpPolyShapeContainsVertPartial(poly2, v, cpvneg(n))) {
				cpContactInit(nextContactPoint(arr), v, n, dist, CP_HASH_PAIR(poly1.hashid, i));
			}
		}

		for (int i = 0; i < poly2.verts.length; i++) {
			Vector2f v = poly2.tVerts[i];
			if (cpPolyShapeContainsVertPartial(poly1, v, n)) {
				cpContactInit(nextContactPoint(arr), v, n, dist, CP_HASH_PAIR(poly2.hashid, i));
			}
		}

		return arr.size() - num;
	}

	// Add contacts for penetrating vertexes.
	static int findVerts(ContactList arr, final PolyShape poly1, final PolyShape poly2, final Vector2f n,
			final float dist) {
		int num = arr.size();

		for (int i = 0; i < poly1.verts.length; i++) {
			Vector2f v = poly1.tVerts[i];
			if (cpPolyShapeContainsVert(poly2, v)) {
				cpContactInit(nextContactPoint(arr), v, n, dist, CP_HASH_PAIR(poly1.hashid, i));
			}
		}

		for (int i = 0; i < poly2.verts.length; i++) {
			Vector2f v = poly2.tVerts[i];
			if (cpPolyShapeContainsVert(poly1, v)) {
				cpContactInit(nextContactPoint(arr), v, n, dist, CP_HASH_PAIR(poly2.hashid, i));
			}
		}

		return (num != arr.size() ? (arr.size() - num) : findVertsFallback(arr, poly1, poly2, n, dist));
	}

	// Collide poly shapes together.
	static int poly2poly(final Shape shape1, final Shape shape2, ContactList arr) {
		PolyShape poly1 = (PolyShape) shape1;
		PolyShape poly2 = (PolyShape) shape2;

		FloatRef min1 = new FloatRef();
		int mini1 = findMSA(poly2, poly1.tPlanes, poly1.verts.length, min1);
		if (mini1 == -1) return 0;

		FloatRef min2 = new FloatRef();
		int mini2 = findMSA(poly1, poly2.tPlanes, poly2.verts.length, min2);
		if (mini2 == -1) return 0;

		// There is overlap, find the penetrating verts
		if (min1.value > min2.value)
			return findVerts(arr, poly1, poly2, poly1.tPlanes[mini1].n, min1.value);
		else
			return findVerts(arr, poly1, poly2, cpvneg(poly2.tPlanes[mini2].n), min2.value);
	}

	// Like cpPolyValueOnAxis(), but for segments.
	static float segValueOnAxis(final SegmentShape seg, final Vector2f n, final float d) {
		float a = cpvdot(n, seg.ta) - seg.r;
		float b = cpvdot(n, seg.tb) - seg.r;
		return cpfmin(a, b) - d;
	}

	// Identify vertexes that have penetrated the segment.
	static void findPointsBehindSeg(ContactList arr, final SegmentShape seg, final PolyShape poly, final float pDist,
			final float coef) {
		float dta = cpvcross(seg.tn, seg.ta);
		float dtb = cpvcross(seg.tn, seg.tb);
		Vector2f n = cpvmult(seg.tn, coef);

		for (int i = 0; i < poly.verts.length; i++) {
			Vector2f v = poly.tVerts[i];
			if (cpvdot(v, n) < cpvdot(seg.tn, seg.ta) * coef + seg.r) {
				float dt = cpvcross(seg.tn, v);
				if (dta >= dt && dt >= dtb) {
					cpContactInit(nextContactPoint(arr), v, n, pDist, CP_HASH_PAIR(poly.hashid, i));
				}
			}
		}
	}

	// This one is complicated and gross. Just don't go there...
	// TODO: Comment me!
	static int seg2poly(final Shape shape1, final Shape shape2, ContactList arr) {
		SegmentShape seg = (SegmentShape) shape1;
		PolyShape poly = (PolyShape) shape2;
		SplittingPlane[] axes = poly.tPlanes;

		float segD = cpvdot(seg.tn, seg.ta);
		float minNorm = cpPolyShapeValueOnAxis(poly, seg.tn, segD) - seg.r;
		float minNeg = cpPolyShapeValueOnAxis(poly, cpvneg(seg.tn), -segD) - seg.r;
		if (minNeg > 0.0f || minNorm > 0.0f) return 0;

		int mini = 0;
		float poly_min = segValueOnAxis(seg, axes[0].n, axes[0].d);
		if (poly_min > 0.0f) return 0;
		for (int i = 0; i < poly.verts.length; i++) {
			float dist = segValueOnAxis(seg, axes[i].n, axes[i].d);
			if (dist > 0.0f) {
				return 0;
			} else if (dist > poly_min) {
				poly_min = dist;
				mini = i;
			}
		}

		int num = arr.size();

		Vector2f poly_n = cpvneg(axes[mini].n);

		Vector2f va = cpvadd(seg.ta, cpvmult(poly_n, seg.r));
		Vector2f vb = cpvadd(seg.tb, cpvmult(poly_n, seg.r));
		if (cpPolyShapeContainsVert(poly, va)) {
			cpContactInit(nextContactPoint(arr), va, poly_n, poly_min, CP_HASH_PAIR(seg.hashid, 0));
		}
		if (cpPolyShapeContainsVert(poly, vb)) {
			cpContactInit(nextContactPoint(arr), vb, poly_n, poly_min, CP_HASH_PAIR(seg.hashid, 1));
		}

		// Floating point precision problems here.
		// This will have to do for now.
		// poly_min -= cp_collision_slop; // TODO is this needed anymore?

		if (minNorm >= poly_min || minNeg >= poly_min) {
			if (minNorm > minNeg) {
				findPointsBehindSeg(arr, seg, poly, minNorm, 1.0f);
			} else {
				findPointsBehindSeg(arr, seg, poly, minNeg, -1.0f);
			}
		}

		// If no other collision points are found, try colliding endpoints.
		if (num == arr.size()) {
			Vector2f poly_a = poly.tVerts[mini];
			Vector2f poly_b = poly.tVerts[(mini + 1) % poly.verts.length];

			if (segmentEncapQuery(seg.ta, poly_a, seg.r, 0.0f, arr, cpvneg(seg.a_tangent)) != 0)
				return 1;

			if (segmentEncapQuery(seg.tb, poly_a, seg.r, 0.0f, arr, cpvneg(seg.b_tangent)) != 0)
				return 1;

			if (segmentEncapQuery(seg.ta, poly_b, seg.r, 0.0f, arr, cpvneg(seg.a_tangent)) != 0)
				return 1;

			if (segmentEncapQuery(seg.tb, poly_b, seg.r, 0.0f, arr, cpvneg(seg.b_tangent)) != 0)
				return 1;
		}

		return arr.size() - num;
	}

	// This one is less gross, but still gross.
	// TODO: Comment me!
	static int circle2poly(final Shape shape1, final Shape shape2, ContactList arr) {
		CircleShape circ = (CircleShape) shape1;
		PolyShape poly = (PolyShape) shape2;
		SplittingPlane[] axes = poly.tPlanes;

		int mini = 0;
		float min = cpvdot(axes[0].n, circ.tc) - axes[0].d - circ.r;
		for (int i = 0; i < poly.verts.length; i++) {
			float dist = cpvdot(axes[i].n, circ.tc) - axes[i].d - circ.r;
			if (dist > 0.0f) {
				return 0;
			} else if (dist > min) {
				min = dist;
				mini = i;
			}
		}

		Vector2f n = axes[mini].n;
		Vector2f a = poly.tVerts[mini];
		Vector2f b = poly.tVerts[(mini + 1) % poly.verts.length];
		float dta = cpvcross(n, a);
		float dtb = cpvcross(n, b);
		float dt = cpvcross(n, circ.tc);

		if (dt < dtb) {
			return circle2circleQuery(circ.tc, b, circ.r, 0.0f, arr);
		} else if (dt < dta) {
			Contact con = nextContactPoint(arr);
			cpContactInit(
					con,
					cpvsub(circ.tc, cpvmult(n, circ.r + min / 2.0f)),
					cpvneg(n),
					min,
					0
			);

			return 1;
		} else {
			return circle2circleQuery(circ.tc, a, circ.r, 0.0f, arr);
		}
	}

	// Submitted by LegoCyclon
	static int seg2seg(final Shape shape1, final Shape shape2, ContactList con) {
		SegmentShape seg1 = (SegmentShape) shape1;
		SegmentShape seg2 = (SegmentShape) shape2;

		Vector2f v1 = cpvsub(seg1.tb, seg1.ta);
		Vector2f v2 = cpvsub(seg2.tb, seg2.ta);
		float v1lsq = cpvlengthsq(v1);
		float v2lsq = cpvlengthsq(v2);
		// project seg2 onto seg1
		Vector2f p1a = cpvproject(cpvsub(seg2.ta, seg1.ta), v1);
		Vector2f p1b = cpvproject(cpvsub(seg2.tb, seg1.ta), v1);
		// project seg1 onto seg2
		Vector2f p2a = cpvproject(cpvsub(seg1.ta, seg2.ta), v2);
		Vector2f p2b = cpvproject(cpvsub(seg1.tb, seg2.ta), v2);

		// clamp projections to segment endcaps
		if (cpvdot(p1a, v1) < 0.0f)
			p1a = cpvzero();
		else if (cpvdot(p1a, v1) > 0.0f && cpvlengthsq(p1a) > v1lsq)
			p1a = v1;
		if (cpvdot(p1b, v1) < 0.0f)
			p1b = cpvzero();
		else if (cpvdot(p1b, v1) > 0.0f && cpvlengthsq(p1b) > v1lsq)
			p1b = v1;
		if (cpvdot(p2a, v2) < 0.0f)
			p2a = cpvzero();
		else if (cpvdot(p2a, v2) > 0.0f && cpvlengthsq(p2a) > v2lsq)
			p2a = v2;
		if (cpvdot(p2b, v2) < 0.0f)
			p2b = cpvzero();
		else if (cpvdot(p2b, v2) > 0.0f && cpvlengthsq(p2b) > v2lsq)
			p2b = v2;

		p1a = cpvadd(p1a, seg1.ta);
		p1b = cpvadd(p1b, seg1.ta);
		p2a = cpvadd(p2a, seg2.ta);
		p2b = cpvadd(p2b, seg2.ta);

		circle2circleQuery(p1a, p2a, seg1.r, seg2.r, con);
		circle2circleQuery(p1b, p2b, seg1.r, seg2.r, con);
		circle2circleQuery(p1a, p2b, seg1.r, seg2.r, con);
		circle2circleQuery(p1b, p2a, seg1.r, seg2.r, con);

		return con.size();
	}

	static int cpCollideShapes(final Shape a, final Shape b, ContactList arr) {
		// Their shape types must be in order.
		cpAssertSoft(a.getType().ordinal() <= b.getType().ordinal(),
				"Collision shapes passed to cpCollideShapes() are not sorted.");

		/*collisionFunc cfunc = colfuncs[a.klass.type + b.klass.type*CP_NUM_SHAPES];
			  return (cfunc) ? cfunc(a, b, arr) : 0;*/
		switch (a.getType()) {
			case CIRCLE_SHAPE: {
				switch (b.getType()) {
					case CIRCLE_SHAPE:
						return circle2circle(a, b, arr);
					case SEGMENT_SHAPE:
						return circle2segment(a, b, arr);
					case POLY_SHAPE:
						return circle2poly(a, b, arr);
				}
				break;
			}
			case SEGMENT_SHAPE: {
				switch (b.getType()) {
					case POLY_SHAPE:
						return seg2poly(a, b, arr);
					case SEGMENT_SHAPE:
						return seg2seg(a, b, arr);
				}
				break;
			}
			case POLY_SHAPE: {
				switch (b.getType()) {
					case POLY_SHAPE:
						return poly2poly(a, b, arr);
				}
				break;
			}
			default:
				return 0;
		}
		return 0;
	}
}
