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

import org.physics.jipmunk.constraints.NearestPointQueryInfo;

import static org.physics.jipmunk.Collision.cpCollideShapes;
import static org.physics.jipmunk.Shape.cpShapePointQuery;
import static org.physics.jipmunk.Shape.cpShapeSegmentQuery;
import static org.physics.jipmunk.Shape.cpShapeUpdate;
import static org.physics.jipmunk.Space.cpSpaceLock;
import static org.physics.jipmunk.Space.cpSpaceUnlock;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexQuery;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexSegmentQuery;
import static org.physics.jipmunk.Util.cpvneg;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
class SpaceQuery {
	static class PointQueryHelper implements SpatialIndexQueryFunc<Shape> {
		Vector2f point;
		int layers;
		int group;
		SpacePointQueryFunc func;

		void init(Vector2f point, int layers, int group, SpacePointQueryFunc func) {
			this.point = point;
			this.layers = layers;
			this.group = group;
			this.func = func;
		}

		void reset() {
			this.func = null;
		}

		@Override
		public void apply(Shape shape) {
			if (!(shape.group != 0 && group == shape.group) && (layers & shape.layers) != 0 && cpShapePointQuery
					(shape, point)) {
				func.apply(shape);
			}
		}
	}

	static PointQueryHelper pointQueryHelper = new PointQueryHelper();

	static void cpSpacePointQuery(Space space, Vector2f point, int layers, int group, SpacePointQueryFunc func) {
		pointQueryHelper.init(point, layers, group, func);
		BB bb = BB.forCircle(point, 0.0f);

		cpSpaceLock(space);
		{
			pointQueryHelper.point = point;
			cpSpatialIndexQuery(space.activeShapes, bb, pointQueryHelper);
			cpSpatialIndexQuery(space.staticShapes, bb, pointQueryHelper);
		}
		cpSpaceUnlock(space, true);

		pointQueryHelper.reset();
	}

	static class RememberLastPointQuery implements SpacePointQueryFunc {

		Shape outShape;

		@Override
		public void apply(Shape shape) {
			if (!shape.sensor) outShape = shape;
		}
	}

	static RememberLastPointQuery rememberLastPointQuery = new RememberLastPointQuery();

	static Shape cpSpacePointQueryFirst(Space space, Vector2f point, int layers, int group) {
		cpSpacePointQuery(space, point, layers, group, rememberLastPointQuery);
		Shape shape = rememberLastPointQuery.outShape;
		rememberLastPointQuery.outShape = null;

		return shape;
	}

	static class SegQueryFunc implements SpatialIndexSegmentQueryFunc<Shape> {
		Vector2f start, end;
		int layers;
		int group;
		SpaceSegmentQueryFunc func;

		void init(Vector2f start, Vector2f end, int layers, int group, SpaceSegmentQueryFunc func) {
			this.start = start;
			this.end = end;
			this.layers = layers;
			this.group = group;
			this.func = func;
		}

		void reset() {
			func = null;
		}

		@Override
		public float apply(Shape shape) {
			SegmentQueryInfo info = new SegmentQueryInfo();

			if (!(shape.group != 0 && group == shape.group) && (layers & shape.layers) != 0 &&
					cpShapeSegmentQuery(shape, start, end, info)) {
				func.apply(shape, info.t, info.n);
			}

			return 1.0f;
		}
	}

	static SegQueryFunc segQueryFunc = new SegQueryFunc();

	static void cpSpaceSegmentQuery(Space space, Vector2f start, Vector2f end, int layers, int group,
			SpaceSegmentQueryFunc func) {
		segQueryFunc.init(start, end, layers, group, func);

		cpSpaceLock(space);
		{
			cpSpatialIndexSegmentQuery(space.staticShapes, start, end, 1.0f, segQueryFunc);
			cpSpatialIndexSegmentQuery(space.activeShapes, start, end, 1.0f, segQueryFunc);
		}
		cpSpaceUnlock(space, true);

		segQueryFunc.reset();
	}

	static class SegQueryFirst implements SpatialIndexSegmentQueryFunc<Shape> {
		Vector2f start, end;
		int layers;
		int group;
		SegmentQueryInfo info = new SegmentQueryInfo();
		SegmentQueryInfo out;

		void init(Vector2f start, Vector2f end, int layers, int group, SegmentQueryInfo out) {
			this.start = start;
			this.end = end;
			this.layers = layers;
			this.group = group;
			this.out = out;
		}

		void reset() {
			this.out = null;
			this.info.reset();
		}

		@Override
		public float apply(Shape shape) {
			info.reset();
			if (!(shape.group != 0 && group == shape.group) && (layers & shape.layers) != 0 &&
					!shape.sensor && shape.segmentQuery(start, end, info) && info.t < out.t) {
				out.set(info.shape, info.t, info.n);
			}

			return out.t;
		}
	}

	static SegQueryFirst segQueryFirst = new SegQueryFirst();
	static final SegmentQueryInfo dummyOut = new SegmentQueryInfo();

	static Shape cpSpaceSegmentQueryFirst(Space space, Vector2f start, Vector2f end, int layers, int group,
			SegmentQueryInfo out) {
		/*SegmentQueryInfo info = new SegmentQueryInfo(null, 1.0f, cpvzero());
				if(out != null){
					(*out) = info;
			  } else {
					out = &info;
				} */
		if (out == null) {
			out = dummyOut;
		}
		out.set(null, 1.0f, cpvzero());

		segQueryFirst.init(start, end, layers, group, out);

		cpSpatialIndexSegmentQuery(space.staticShapes, start, end, 1.0f, segQueryFirst);
		cpSpatialIndexSegmentQuery(space.activeShapes, start, end, out.t, segQueryFirst);

		segQueryFirst.reset();

		return out.shape;
	}

	static class BBQueryHelper implements SpatialIndexQueryFunc<Shape> {

		BB bb;
		int layers;
		int group;
		SpaceBBQueryFunc func;

		void init(BB bb, int layers, int group, SpaceBBQueryFunc func) {
			this.bb = bb;
			this.layers = layers;
			this.group = group;
			this.func = func;
		}

		void reset() {
			func = null;
		}

		@Override
		public void apply(Shape shape) {
			if (!(shape.group != 0 && group == shape.group) && (layers & shape.layers) != 0
					&& bb.intersects(shape.bb)) {
				func.apply(shape);
			}
		}
	}

	static BBQueryHelper bbQueryHelper = new BBQueryHelper();

	void cpSpaceBBQuery(Space space, BB bb, int layers, int group, SpaceBBQueryFunc func) {
		bbQueryHelper.init(bb, layers, group, func);

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.activeShapes, bb, bbQueryHelper);
			cpSpatialIndexQuery(space.staticShapes, bb, bbQueryHelper);
		}
		cpSpaceUnlock(space, true);

		bbQueryHelper.reset();
	}

	static ContactList contacts = new ContactList();

	static class ShapeQueryHelper implements SpatialIndexQueryFunc<Shape> {

		Shape a;
		SpaceShapeQueryFunc func;
		boolean anyCollision;

		void init(Shape a, SpaceShapeQueryFunc func, boolean anyCollision) {
			this.a = a;
			this.func = func;
			this.anyCollision = anyCollision;
		}

		boolean reset() {
			a = null;
			func = null;
			return anyCollision;
		}

		@Override
		public void apply(Shape b) {
			// Reject any of the simple cases
			if ((a.group != 0 && a.group == b.group) || (a.layers & b.layers) == 0 || a == b) return;

			//cpContact contacts[CP_MAX_CONTACTS_PER_ARBITER];
			int numContacts;

			// Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
			if (a.getType().ordinal() <= b.getType().ordinal()) {
				numContacts = cpCollideShapes(a, b, contacts);
			} else {
				numContacts = cpCollideShapes(b, a, contacts);
				for (int i = 0; i < numContacts; i++) contacts.get(i).n = cpvneg(contacts.get(i).n);
			}

			if (numContacts > 0) {
				anyCollision = !(a.sensor || b.sensor);

				if (func != null) {
					ContactPointSet set = new ContactPointSet(numContacts);
					for (int i = 0; i < numContacts; i++) {
						Contact contact = contacts.get(i);
						set.add(contact.p, contact.n, contact.dist);
					}

					func.apply(b, set);
				}
				contacts.clear();
			}
		}
	}

	static ShapeQueryHelper shapeQueryHelper = new ShapeQueryHelper();

	static boolean cpSpaceShapeQuery(Space space, Shape shape, SpaceShapeQueryFunc func) {
		Body body = shape.body;
		BB bb = (body != null ? cpShapeUpdate(shape, body.p, body.rot) : shape.bb);
		shapeQueryHelper.init(shape, func, false);

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.activeShapes, bb, shapeQueryHelper);
			cpSpatialIndexQuery(space.staticShapes, bb, shapeQueryHelper);
		}
		cpSpaceUnlock(space, true);

		return shapeQueryHelper.reset();
	}

	private static class NearestPointQueryContext {
		Vector2f point;
		float maxDistance;
		int layers;
		int group;
		SpaceNearestPointQueryFunc func;

		private NearestPointQueryContext(Vector2f point, float maxDistance, int layers, int group,
				SpaceNearestPointQueryFunc func) {
			this.point = point;
			this.maxDistance = maxDistance;
			this.layers = layers;
			this.group = group;
			this.func = func;
		}
	}

	static void nearestPointQuery(NearestPointQueryContext context, Shape shape) {
		if (!(shape.group != 0 && context.group == shape.group) && (context.layers & shape.layers) != 0) {
			NearestPointQueryInfo info = shape.nearestPointQuery(context.point, null);

			if (info.shape != null && info.d < context.maxDistance) {
				context.func.apply(shape, info.d, info.p);
			}
		}
	}

	static void cpSpaceNearestPointQuery(Space space, Vector2f point, float maxDistance, int layers, int group,
			SpaceNearestPointQueryFunc func) {
		final NearestPointQueryContext context = new NearestPointQueryContext(point, maxDistance, layers, group, func);
		BB bb = BB.forCircle(point, Util.cpfmax(maxDistance, 0.0f));

		cpSpaceLock(space);
		{
			cpSpatialIndexQuery(space.activeShapes, bb, new SpatialIndexQueryFunc<Shape>() {
				@Override
				public void apply(Shape shape) {
					nearestPointQuery(context, shape);
				}
			});
			cpSpatialIndexQuery(space.staticShapes, bb, new SpatialIndexQueryFunc<Shape>() {
				@Override
				public void apply(Shape shape) {
					nearestPointQuery(context, shape);
				}
			});
		}
		cpSpaceUnlock(space, true);
	}

	static void nearestPointQueryNearest(NearestPointQueryContext context, Shape shape, NearestPointQueryInfo out) {
		if (!(shape.group != 0 && context.group == shape.group) && (context.layers & shape.layers) != 0
				&& !shape.sensor) {
			NearestPointQueryInfo info = shape.nearestPointQuery(context.point, null);

			if (info.d < out.d) {
				out.set(info);
			}
		}
	}

	static NearestPointQueryInfo cpSpaceNearestPointQueryNearest(Space space, Vector2f point, float maxDistance,
			int layers, int group, NearestPointQueryInfo out) {
		final NearestPointQueryInfo info = out == null ? new NearestPointQueryInfo(null, Util.cpvzero(), maxDistance)
				: out;

		final NearestPointQueryContext context = new NearestPointQueryContext(
				point, maxDistance,
				layers, group,
				null
		);

		BB bb = BB.forCircle(point, Util.cpfmax(maxDistance, 0.0f));
		cpSpatialIndexQuery(space.activeShapes, bb, new SpatialIndexQueryFunc<Shape>() {
			@Override
			public void apply(Shape shape) {
				nearestPointQueryNearest(context, shape, info);
			}
		});
		cpSpatialIndexQuery(space.staticShapes, bb, new SpatialIndexQueryFunc<Shape>() {
			@Override
			public void apply(Shape shape) {
				nearestPointQueryNearest(context, shape, info);
			}
		});

		return info;
	}
}
