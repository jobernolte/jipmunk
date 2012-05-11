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
abstract class SpatialIndex<T> {
	SpatialIndexBBFunc<T> bbfunc;
	SpatialIndex<T> staticIndex;
	SpatialIndex<T> dynamicIndex;

	public SpatialIndex(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
		this.bbfunc = bbfunc;
		this.staticIndex = staticIndex;
		if (staticIndex != null) {
			assert staticIndex.dynamicIndex == null;
			staticIndex.dynamicIndex = this;
		}
	}

	public abstract int count();

	public abstract void each(SpatialIndexIteratorFunc<T> iterator);

	public abstract boolean contains(T obj, int hashValue);

	public abstract void insert(T obj, int hashValue);

	public abstract void remove(T obj, int hashValue);

	public abstract void reindex();

	public abstract void reindexObject(T obj, int hashValue);

	public abstract void reindexQuery(SpatialReIndexQueryFunc<T> func);

	public abstract void pointQuery(final Vector2f point, SpatialIndexQueryFunc<T> func);

	public abstract void segmentQuery(final Vector2f a, final Vector2f b, float exit,
			SpatialIndexSegmentQueryFunc<T> func);

	public abstract void query(final BB bb, SpatialIndexQueryFunc<T> func);

	static class DynamicToStaticContext<T> {
		SpatialIndexBBFunc<T> bbfunc;
		SpatialIndex<T> staticIndex;
		SpatialReIndexQueryFunc<T> queryFunc;

		DynamicToStaticContext(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex,
				SpatialReIndexQueryFunc<T> queryFunc) {
			this.bbfunc = bbfunc;
			this.staticIndex = staticIndex;
			this.queryFunc = queryFunc;
		}
	}

	static <T> void collideStatic(SpatialIndex<T> dynamicIndex, SpatialIndex<T> staticIndex,
			SpatialReIndexQueryFunc<T> func) {
		if (staticIndex.count() > 0) {
			final DynamicToStaticContext<T> context = new DynamicToStaticContext<T>(dynamicIndex.bbfunc, staticIndex,
					func);
			dynamicIndex.each(new SpatialIndexIteratorFunc<T>() {
				@Override
				public void visit(final T obj1) {
					context.staticIndex.query(context.bbfunc.apply(obj1), new SpatialIndexQueryFunc<T>() {
						@Override
						public void apply(T obj2) {
							context.queryFunc.apply(obj1, obj2);
						}
					});
				}
			});
		}
	}

	static <T> void cpSpatialIndexInsert(SpatialIndex<T> spatialIndex, T object, int hashid) {
		spatialIndex.insert(object, hashid);
	}

	static <T> void cpSpatialIndexRemove(SpatialIndex<T> spatialIndex, T object, int hashid) {
		spatialIndex.remove(object, hashid);
	}

	static <T> void cpSpatialIndexEach(SpatialIndex<T> index, SpatialIndexIteratorFunc<T> func) {
		index.each(func);
	}

	/**
	 * Simultaneously reindex and find all colliding objects.
	 * <p/>
	 * The func will be called once for each potentially overlapping pair of objects found. + If the spatial index was
	 * initialized with a static index, it will collide it's objects against that as well.
	 *
	 * @param index the index to reindex
	 * @param func  the func to use
	 */
	static <T> void cpSpatialIndexReindexQuery(SpatialIndex<T> index, SpatialReIndexQueryFunc<T> func) {
		index.reindexQuery(func);
	}

	/// Perform a point query against the spatial index, calling @c func for each potential match.
/// A pointer to the point will be passed as @c obj1 of @c func.
	static <T> void cpSpatialIndexPointQuery(SpatialIndex<T> index, final Vector2f point,
			SpatialIndexQueryFunc<T> func) {
		index.pointQuery(point, func);
	}

	/// Perform a segment query against the spatial index, calling @c func for each potential match.
	static <T> void cpSpatialIndexSegmentQuery(SpatialIndex<T> index, Vector2f a, Vector2f b, float t_exit,
			SpatialIndexSegmentQueryFunc<T> func) {
		index.segmentQuery(a, b, t_exit, func);
	}

	static <T> void cpSpatialIndexQuery(SpatialIndex<T> index, final BB bb, SpatialIndexQueryFunc<T> func) {
		index.query(bb, func);
	}

	static <T> void cpSpatialIndexCollideStatic(SpatialIndex<T> dynamicIndex, SpatialIndex<T> staticIndex,
			SpatialReIndexQueryFunc<T> func) {
		collideStatic(dynamicIndex, staticIndex, func);
	}
}
