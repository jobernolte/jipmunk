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

import static org.physics.jipmunk.Space.cpSpaceLock;
import static org.physics.jipmunk.Space.cpSpaceUnlock;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexQuery;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
class SpaceQuery {

	static class SegmentQueryContext {
		Vector2f start, end;
		float radius;
		ShapeFilter filter;
		SpaceSegmentQueryFunc func;

		void init(Vector2f start, Vector2f end, float radius, ShapeFilter filter, SpaceSegmentQueryFunc func) {
			this.start = start;
			this.end = end;
			this.radius = radius;
			this.filter = filter;
			this.func = func;
		}

		public SegmentQueryContext(Vector2f start, Vector2f end, float radius, ShapeFilter filter,
				SpaceSegmentQueryFunc func) {
			init(start, end, radius, filter, func);
		}
	}

	static float SegmentQuery(SegmentQueryContext context, Shape shape) {
		SegmentQueryInfo info = new SegmentQueryInfo();

		if (!shape.filter.reject(context.filter) && shape
				.segmentQuery(context.start, context.end, context.radius, info)) {
			context.func.apply(shape, info.point, info.normal, info.alpha);
		}

		return 1.0f;
	}

	static void cpSpaceSegmentQuery(Space space, Vector2f start, Vector2f end, float radius, ShapeFilter filter,
			SpaceSegmentQueryFunc func) {
		SegmentQueryContext context = new SegmentQueryContext(start, end, radius, filter, func);

		cpSpaceLock(space);
		{
			space.staticShapes.segmentQuery(null, start, end, 1.0f, (shape1, shape2) -> SegmentQuery(context, shape2));
			space.dynamicShapes.segmentQuery(null, start, end, 1.0f, (shape1, shape2) -> SegmentQuery(context, shape2));
		}
		cpSpaceUnlock(space, true);
	}

	static float SegmentQueryFirst(SegmentQueryContext context, Shape shape, SegmentQueryInfo out) {
		SegmentQueryInfo info = new SegmentQueryInfo();

		if (!shape.filter.reject(context.filter) && !shape.isSensor() &&
				shape.segmentQuery(context.start, context.end, context.radius, info) &&
				info.alpha < out.alpha) {
			out.set(info);
		}

		return out.alpha;
	}

	static SegmentQueryInfo cpSpaceSegmentQueryFirst(Space space, Vector2f start, Vector2f end, float radius,
			ShapeFilter filter, final SegmentQueryInfo out) {
		out.set(null, end, cpvzero(), 1.0f);
		SegmentQueryContext context = new SegmentQueryContext(start, end, radius, filter, null);

		space.staticShapes
				.segmentQuery(null, start, end, 1.0f, (shape1, shape2) -> SegmentQueryFirst(context, shape2, out));
		space.dynamicShapes
				.segmentQuery(null, start, end, out.alpha, (shape1, shape2) -> SegmentQueryFirst(context, shape2, out));

		return out;
	}

	static class BBQueryContext {
		BB bb;
		ShapeFilter filter;
		SpaceBBQueryFunc func;

		BBQueryContext(BB bb, ShapeFilter filter, SpaceBBQueryFunc func) {
			this.bb = bb;
			this.filter = filter;
			this.func = func;
		}
	}

	static CollisionID BBQuery(BBQueryContext context, Shape shape, CollisionID id) {
		if (!shape.filter.reject(context.filter) && context.bb.intersects(shape.bb)) {
			context.func.apply(shape);
		}

		return id;
	}

	static void cpSpaceBBQuery(Space space, BB bb, ShapeFilter filter, SpaceBBQueryFunc func) {
		BBQueryContext context = new BBQueryContext(bb, filter, func);

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.dynamicShapes, null, bb, (shape1, shape2, id) -> BBQuery(context, shape2, id));
			cpSpatialIndexQuery(space.staticShapes, null, bb, (shape1, shape2, id) -> BBQuery(context, shape2, id));
		}
		cpSpaceUnlock(space, true);
	}

	private static class PointQueryContext {
		Vector2f point;
		float maxDistance;
		ShapeFilter filter;
		SpacePointQueryFunc func;

		private PointQueryContext(Vector2f point, float maxDistance, ShapeFilter filter, SpacePointQueryFunc func) {
			this.point = point;
			this.maxDistance = maxDistance;
			this.filter = filter;
			this.func = func;
		}
	}

	static CollisionID nearestPointQuery(PointQueryContext context, Shape shape, CollisionID id) {
		if (!shape.filter.reject(context.filter)) {
			PointQueryInfo info;
			info = shape.pointQuery(context.point, null);

			if (info.shape != null && info.distance < context.maxDistance) {
				context.func.apply(shape, info.point, info.distance, info.gradient);
			}
		}
		return id;
	}

	static void cpSpacePointQuery(Space space, Vector2f point, float maxDistance, ShapeFilter filter,
			SpacePointQueryFunc func) {
		final PointQueryContext context = new PointQueryContext(point, maxDistance, filter, func);
		BB bb = BB.forCircle(point, Util.cpfmax(maxDistance, 0.0f));

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.dynamicShapes, null, bb,
								(shape1, shape2, id) -> nearestPointQuery(context, shape2, id));
			cpSpatialIndexQuery(space.staticShapes, null, bb,
								(shape1, shape2, id) -> nearestPointQuery(context, shape2, id));
		}
		cpSpaceUnlock(space, true);
	}

	static CollisionID nearestPointQueryNearest(PointQueryContext context, Shape shape, CollisionID id,
			PointQueryInfo out) {
		if (!shape.filter.reject(context.filter) && !shape.isSensor()) {
			PointQueryInfo info = shape.pointQuery(context.point, null);

			if (info.distance < out.distance) {
				out.set(info);
			}
		}
		return id;
	}

	static PointQueryInfo cpSpacePointQueryNearest(Space space, Vector2f point, float maxDistance, ShapeFilter filter,
			PointQueryInfo out) {
		final PointQueryInfo info = out == null ? new PointQueryInfo(null, Util.cpvzero(), maxDistance) : out;

		final PointQueryContext context = new PointQueryContext(point, maxDistance, filter, null);

		BB bb = BB.forCircle(point, Util.cpfmax(maxDistance, 0.0f));
		cpSpatialIndexQuery(space.dynamicShapes, null, bb,
							(shape1, shape2, id) -> nearestPointQueryNearest(context, shape2, id, info));
		cpSpatialIndexQuery(space.staticShapes, null, bb,
							(shape1, shape2, id) -> nearestPointQueryNearest(context, shape2, id, info));

		return info;
	}

	static class ShapeQueryContext {
		SpaceShapeQueryFunc func;
		boolean anyCollision;

		ShapeQueryContext(SpaceShapeQueryFunc func, boolean anyCollision) {
			this.func = func;
			this.anyCollision = anyCollision;
		}
	}

	// Callback from the spatial hash.
	static CollisionID ShapeQuery(Shape a, Shape b, CollisionID id, ShapeQueryContext context) {
		if (a.filter.reject(b.filter) || a == b) {
			return id;
		}

		ContactPointSet set = Shape.shapesCollide(a, b);
		if (set.getCount() > 0) {
			if (context.func != null) {
				context.func.apply(b, set);
			}
			context.anyCollision = !(a.isSensor() || b.isSensor());
		}

		return id;
	}

	static boolean cpSpaceShapeQuery(Space space, Shape shape, SpaceShapeQueryFunc func) {
		Body body = shape.body;
		BB bb = (body != null ? shape.update(body.transform) : shape.getBB());
		ShapeQueryContext context = new ShapeQueryContext(func, false);

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.dynamicShapes, shape, bb,
								(shape1, shape2, id) -> ShapeQuery(shape1, shape2, id, context));
			cpSpatialIndexQuery(space.staticShapes, shape, bb,
								(shape1, shape2, id) -> ShapeQuery(shape1, shape2, id, context));
		}
		cpSpaceUnlock(space, true);

		return context.anyCollision;
	}
}
