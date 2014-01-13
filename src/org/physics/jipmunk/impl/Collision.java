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

import org.physics.jipmunk.*;

import java.util.EnumMap;

import static org.physics.jipmunk.Util.*;

/** @author jobernolte */
public class Collision {

	@FunctionalInterface
	private static interface CollisionFunc {
		void apply(final Shape shape1, final Shape shape2, CollisionInfo info);
	}

	/**
	 * Given two support edges, find contact point pairs on their surfaces.
	 *
	 * @param e1     the first edge.
	 * @param e2     the second edge.
	 * @param points the closest points.
	 * @param info   the collision info.
	 */
	static void contactPoints(final Edge e1, final Edge e2, final ClosestPoints points, CollisionInfo info) {
		float mindist = e1.getR() + e2.getR();
		if (points.d <= mindist) {
			Vector2f n = points.n;
			info.setN(n);

			final EdgePoint edgePoint1A = e1.getA();
			final EdgePoint edgePoint1B = e1.getB();
			final EdgePoint edgePoint2A = e2.getA();
			final EdgePoint edgePoint2B = e2.getB();

			// Distances along the axis parallel to n
			float d_e1_a = cpvcross(edgePoint1A.getP(), n);
			float d_e1_b = cpvcross(edgePoint1B.getP(), n);
			float d_e2_a = cpvcross(edgePoint2A.getP(), n);
			float d_e2_b = cpvcross(edgePoint2B.getP(), n);

			float e1_denom = 1.0f / (d_e1_b - d_e1_a);
			float e2_denom = 1.0f / (d_e2_b - d_e2_a);

			// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
			// Compare the projected points to the collision normal to see if the shapes overlap there.
			{
				Vector2f p1 = cpvadd(cpvmult(n, e1.getR()), cpvlerp(edgePoint1A.getP(), edgePoint1B.getP(),
																	cpfclamp01((d_e2_b - d_e1_a) * e1_denom)));
				Vector2f p2 = cpvadd(cpvmult(n, -e2.getR()), cpvlerp(edgePoint2A.getP(), edgePoint2B.getP(),
																	 cpfclamp01((d_e1_a - d_e2_a) * e2_denom)));
				float dist = cpvdot(cpvsub(p2, p1), n);
				if (dist <= 0.0f) {
					int hash_1a2b = HashValue.hashPair(edgePoint1A.getHash(), edgePoint2B.getHash());
					info.addContact(p1, p2, hash_1a2b);
					if (info.getA().getType() == ShapeType.POLY_SHAPE
							&& info.getB().getType() == ShapeType.POLY_SHAPE) {
						// TODO remove me
						info.getA();
					}
				}
			}
			{
				Vector2f p1 = cpvadd(cpvmult(n, e1.getR()), cpvlerp(edgePoint1A.getP(), edgePoint1B.getP(),
																	cpfclamp01((d_e2_a - d_e1_a) * e1_denom)));
				Vector2f p2 = cpvadd(cpvmult(n, -e2.getR()), cpvlerp(edgePoint2A.getP(), edgePoint2B.getP(),
																	 cpfclamp01((d_e1_b - d_e2_a) * e2_denom)));
				float dist = cpvdot(cpvsub(p2, p1), n);
				if (dist <= 0.0f) {
					int hash_1b2a = HashValue.hashPair(edgePoint1B.getHash(), edgePoint2A.getHash());
					info.addContact(p1, p2, hash_1b2a);
					if (info.getA().getType() == ShapeType.POLY_SHAPE
							&& info.getB().getType() == ShapeType.POLY_SHAPE) {
						// TODO remove me
						info.getA();
					}
				}
			}
		}
	}

	// Collide circle shapes.
	static void circleToCircle(final Shape shape1, final Shape shape2, CollisionInfo info) {
		CircleShape c1 = (CircleShape) shape1;
		CircleShape c2 = (CircleShape) shape2;
		final float radius1 = c1.getRadius();
		final float radius2 = c2.getRadius();
		float mindist = radius1 + radius2;
		final Vector2f tc1 = c1.getTransformedCenter();
		final Vector2f tc2 = c2.getTransformedCenter();
		Vector2f delta = cpvsub(tc2, tc1);
		float distsq = cpvlengthsq(delta);

		if (distsq < mindist * mindist) {
			float dist = cpfsqrt(distsq);
			Vector2f n = (dist != 0.0f ? cpvmult(delta, 1.0f / dist) : cpv(1.0f, 0.0f));
			info.setN(n);
			info.addContact(cpvadd(tc1, cpvmult(n, radius1)), cpvadd(tc2, cpvmult(n, -radius2)), 0);
		}
	}

	// Collide circles to segment shapes.
	static void circleToSegment(Shape a, Shape b, CollisionInfo info) {
		CircleShape circle = (CircleShape) a;
		SegmentShape segment = (SegmentShape) b;

		Vector2f seg_a = segment.getTa();
		Vector2f seg_b = segment.getTb();
		Vector2f center = circle.getTransformedCenter();

		// Find the closest point on the segment to the circle.
		Vector2f seg_delta = cpvsub(seg_b, seg_a);
		float closest_t = cpfclamp01(cpvdot(seg_delta, cpvsub(center, seg_a)) / cpvlengthsq(seg_delta));
		Vector2f closest = cpvadd(seg_a, cpvmult(seg_delta, closest_t));

		final float circleRadius = circle.getRadius();
		final float segmentRadius = segment.getRadius();
		// Compare the radii of the two shapes to see if they are colliding.
		float mindist = circleRadius + segmentRadius;
		Vector2f delta = cpvsub(closest, center);
		float distsq = cpvlengthsq(delta);
		if (distsq < mindist * mindist) {
			float dist = cpfsqrt(distsq);
			// Handle coincident shapes as gracefully as possible.
			Vector2f n = (dist != 0.0f ? cpvmult(delta, 1.0f / dist) : segment.getTn());
			info.setN(n);

			// Reject endcap collisions if tangents are provided.
			Vector2f rot = segment.getBody().getRotation();
			if ((closest_t != 0.0f || cpvdot(n, cpvrotate(segment.getATangent(), rot)) >= 0.0f) && (closest_t != 1.0f
					|| cpvdot(n, cpvrotate(segment.getBTangent(), rot)) >= 0.0f)) {
				info.addContact(cpvadd(center, cpvmult(n, circleRadius)), cpvadd(closest, cpvmult(n, -segmentRadius)),
								0);
			}
		}
	}

	static void segmentToSegment(Shape a, Shape b, CollisionInfo info) {
		SegmentShape seg1 = (SegmentShape) a;
		SegmentShape seg2 = (SegmentShape) b;

		SupportContext context =
				new SupportContext(seg1, seg2, SupportPoint::segmentSupportPoint, SupportPoint::segmentSupportPoint);
		ClosestPoints points = ClosestPoints.GJK(context, info.getId());

		Vector2f n = points.n;
		Vector2f rot1 = seg1.getBody().getRotation();
		Vector2f rot2 = seg2.getBody().getRotation();

		// If the closest points are nearer than the sum of the radii...
		if (points.d <= (seg1.getRadius() + seg2.getRadius()) && (
				// Reject endcap collisions if tangents are provided.
				(!cpveql(points.a, seg1.getTa()) || cpvdot(n, cpvrotate(seg1.getATangent(), rot1)) <= 0.0f) &&
						(!cpveql(points.a, seg1.getTb()) || cpvdot(n, cpvrotate(seg1.getBTangent(), rot1)) <= 0.0f) &&
						(!cpveql(points.b, seg2.getTa()) || cpvdot(n, cpvrotate(seg2.getATangent(), rot2)) >= 0.0f) &&
						(!cpveql(points.b, seg2.getTb()) || cpvdot(n, cpvrotate(seg2.getBTangent(), rot2)) >= 0.0f))) {
			contactPoints(Edge.edgeForSegment(seg1, n), Edge.edgeForSegment(seg2, cpvneg(n)), points, info);
		}
	}

	static void circleToPoly(Shape a, Shape b, CollisionInfo info) {
		CircleShape circle = (CircleShape) a;
		PolyShape poly = (PolyShape) b;
		SupportContext context =
				new SupportContext(circle, poly, SupportPoint::circleSupportPoint, SupportPoint::polySupportPoint);
		ClosestPoints points = ClosestPoints.GJK(context, info.getId());

		// If the closest points are nearer than the sum of the radii...
		if (points.d <= circle.getRadius() + poly.getRadius()) {
			Vector2f n = points.n;
			info.setN(n);
			info.addContact(cpvadd(points.a, cpvmult(n, circle.getRadius())),
							cpvadd(points.b, cpvmult(n, poly.getRadius())), 0);
		}
	}

	static void segmentToPoly(Shape a, Shape b, CollisionInfo info) {
		SegmentShape seg = (SegmentShape) a;
		PolyShape poly = (PolyShape) b;
		SupportContext context =
				new SupportContext(seg, poly, SupportPoint::segmentSupportPoint, SupportPoint::polySupportPoint);
		ClosestPoints points = ClosestPoints.GJK(context, info.getId());

		// Reject endcap collisions if tangents are provided.
		Vector2f n = points.n;
		Vector2f rot = seg.getBody().getRotation();
		if (points.d - seg.getRadius() - poly.getRadius() <= 0.0f && (
				(!cpveql(points.a, seg.getTa()) || cpvdot(n, cpvrotate(seg.getATangent(), rot)) <= 0.0f) && (
						!cpveql(points.a, seg.getTb()) || cpvdot(n, cpvrotate(seg.getBTangent(), rot)) <= 0.0f))) {
			contactPoints(Edge.edgeForSegment(seg, n), Edge.edgeForPoly(poly, cpvneg(n)), points, info);
		}
	}

	static void polyToPoly(Shape a, Shape b, CollisionInfo info) {
		PolyShape poly1 = (PolyShape) a;
		PolyShape poly2 = (PolyShape) b;
		SupportContext context =
				new SupportContext(poly1, poly2, SupportPoint::polySupportPoint, SupportPoint::polySupportPoint);
		ClosestPoints points = ClosestPoints.GJK(context, info.getId());

		// If the closest points are nearer than the sum of the radii...
		if (points.d - poly1.getRadius() - poly2.getRadius() <= 0.0f) {
			contactPoints(Edge.edgeForPoly(poly1, points.n), Edge.edgeForPoly(poly2, cpvneg(points.n)), points, info);
		}
	}

	final static EnumMap<ShapeType, EnumMap<ShapeType, CollisionFunc>> SHAPE_TYPE_COLLISION_FUNC_MAP =
			new EnumMap<>(ShapeType.class);

	static {
		// circle functions
		EnumMap<ShapeType, CollisionFunc> circleFuncMap = new EnumMap<>(ShapeType.class);
		circleFuncMap.put(ShapeType.CIRCLE_SHAPE, Collision::circleToCircle);
		circleFuncMap.put(ShapeType.SEGMENT_SHAPE, Collision::circleToSegment);
		circleFuncMap.put(ShapeType.POLY_SHAPE, Collision::circleToPoly);
		SHAPE_TYPE_COLLISION_FUNC_MAP.put(ShapeType.CIRCLE_SHAPE, circleFuncMap);
		// segment functions
		EnumMap<ShapeType, CollisionFunc> segmentFuncMap = new EnumMap<>(ShapeType.class);
		segmentFuncMap.put(ShapeType.SEGMENT_SHAPE, Collision::segmentToSegment);
		segmentFuncMap.put(ShapeType.POLY_SHAPE, Collision::segmentToPoly);
		SHAPE_TYPE_COLLISION_FUNC_MAP.put(ShapeType.SEGMENT_SHAPE, segmentFuncMap);
		// poly functions
		EnumMap<ShapeType, CollisionFunc> polyFuncMap = new EnumMap<>(ShapeType.class);
		polyFuncMap.put(ShapeType.POLY_SHAPE, Collision::polyToPoly);
		SHAPE_TYPE_COLLISION_FUNC_MAP.put(ShapeType.POLY_SHAPE, polyFuncMap);
	}

	public static CollisionInfo collide(final Shape a, final Shape b, CollisionID id) {
		CollisionInfo info = new CollisionInfo(a, b, id, cpvzero());

		// Make sure the shape types are in order.
		if (a.getType().ordinal() > b.getType().ordinal()) {
			info.setA(b);
			info.setB(a);
		}

		EnumMap<ShapeType, CollisionFunc> collisionFuncMap = SHAPE_TYPE_COLLISION_FUNC_MAP.get(info.getA().getType());
		if (collisionFuncMap == null) {
			throw new IllegalArgumentException("Internal Error: Shape types are not sorted.");
		}
		CollisionFunc func = collisionFuncMap.get(info.getB().getType());
		if (func == null) {
			throw new IllegalArgumentException("Internal Error: Shape types are not sorted.");
		}
		func.apply(info.getA(), info.getB(), info);
		return info;
	}
}
