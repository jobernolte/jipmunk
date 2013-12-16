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

	/**
	 * Simultaneously reindex and find all colliding objects.
	 * <point/>
	 * The func will be called once for each potentially overlapping pair of objects found. + If the spatial index was
	 * initialized with a static index, it will collide it's objects against that as well.
	 *
	 * @param func the func to use
	 */
	public abstract void reindexQuery(SpatialIndexQueryFunc<T> func);

	public abstract void query(T obj, final BB bb, SpatialIndexQueryFunc<T> func);

	/**
	 * Perform a segment query against the spatial index, calling <code>func</code> for each potential match.
	 *
	 * @param obj  the obj to query.
	 * @param a    the start of the query.
	 * @param b    the end of the query.
	 * @param exit the exit value.
	 * @param func the func to call.
	 */
	public abstract void segmentQuery(T obj, final Vector2f a, final Vector2f b, float exit,
			SpatialIndexSegmentQueryFunc<T> func);

	static class DynamicToStaticContext<T> {
		SpatialIndexBBFunc<T> bbfunc;
		SpatialIndex<T> staticIndex;
		SpatialIndexQueryFunc<T> queryFunc;

		DynamicToStaticContext(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex,
				SpatialIndexQueryFunc<T> queryFunc) {
			this.bbfunc = bbfunc;
			this.staticIndex = staticIndex;
			this.queryFunc = queryFunc;
		}
	}

	static <T> void collideStatic(SpatialIndex<T> dynamicIndex, SpatialIndex<T> staticIndex,
			SpatialIndexQueryFunc<T> func) {
		if (staticIndex.count() > 0) {
			final DynamicToStaticContext<T> context =
					new DynamicToStaticContext<>(dynamicIndex.bbfunc, staticIndex, func);
			dynamicIndex.each(obj1 -> context.staticIndex.query(obj1, context.bbfunc.apply(obj1), context.queryFunc));
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

	static <T> void cpSpatialIndexQuery(SpatialIndex<T> index, T obj, final BB bb, SpatialIndexQueryFunc<T> func) {
		index.query(obj, bb, func);
	}

	static <T> void cpSpatialIndexCollideStatic(SpatialIndex<T> dynamicIndex, SpatialIndex<T> staticIndex,
			SpatialIndexQueryFunc<T> func) {
		collideStatic(dynamicIndex, staticIndex, func);
	}
}
