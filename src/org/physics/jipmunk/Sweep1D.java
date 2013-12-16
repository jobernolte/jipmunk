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

import java.util.Arrays;
import java.util.Comparator;

/** @author jobernolte */
class Sweep1D<T> extends SpatialIndex<T> {

	static class Bounds {
		float min, max;

		Bounds(BB bb) {
			this.min = bb.l;
			this.max = bb.r;
		}

		public static boolean overlap(Bounds a, Bounds b) {
			return (a.min <= b.max && b.min <= a.max);
		}
	}

	static class TableCell<T> {
		T obj;
		Bounds bounds;

		TableCell(T obj, SpatialIndex<T> spatialIndex) {
			this.obj = obj;
			this.bounds = new Bounds(spatialIndex.bbfunc.apply(obj));
		}
	}

	private int num;
	private int max;
	private TableCell<T>[] table;

	public Sweep1D(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
		super(bbfunc, staticIndex);
		resize(32);
	}

	public void resize(int size) {
		this.max = size;
		this.table = Arrays.copyOf(table, size);
	}

	@Override
	public int count() {
		return num;
	}

	@Override
	public void each(SpatialIndexIteratorFunc<T> func) {
		for (int i = 0; i < num; i++) {
			func.visit(table[i].obj);
		}
	}

	@Override
	public boolean contains(T obj, int hashValue) {
		for (int i = 0; i < num; i++) {
			if (table[i].obj == obj) {
				return true;
			}
		}
		return false;
	}

	@Override
	public void insert(T obj, int hashValue) {
		if (num == max) {
			resize(max * 2);
		}
		table[num++] = new TableCell<T>(obj, this);
	}

	@Override
	public void remove(T obj, int hashid) {
		for (int i = 0; i < num; i++) {
			if (table[i].obj == obj) {
				num--;

				table[i] = table[num];
				table[num] = null;

				return;
			}
		}
	}

	@Override
	public void reindex() {
		// Nothing to do here
		// Could perform a sort, but queries are not accelerated anyway.
	}

	@Override
	public void reindexObject(T obj, int hashValue) {
		// Nothing to do here
	}

	@Override
	public void query(T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		// Implementing binary search here would allow you to find an upper limit
		// but not a lower limit. Probably not worth the hassle.

		Bounds bounds = new Bounds(bb);

		for (int i = 0; i < num; i++) {
			TableCell<T> cell = table[i];
			if (Bounds.overlap(bounds, cell.bounds) && obj != cell.obj) {
				func.apply(obj, cell.obj, new CollisionID(0));
			}
		}
	}

	@Override
	public void segmentQuery(T obj, Vector2f a, Vector2f b, float exit, SpatialIndexSegmentQueryFunc<T> func) {
		BB bb = BB.expand(new BB(a.getX(), a.getY(), a.getX(), a.getY()), b);
		Bounds bounds = new Bounds(bb);

		for (int i = 0; i < num; i++) {
			TableCell<T> cell = table[i];
			if (Bounds.overlap(bounds, cell.bounds)) {
				func.apply(obj, cell.obj);
			}
		}
	}

	@Override
	public void reindexQuery(SpatialIndexQueryFunc<T> func) {
		// Update bounds and sort
		for (int i = 0; i < num; i++) {
			table[i] = new TableCell<T>(table[i].obj, this);
		}
		Arrays.sort(table, new Comparator<TableCell<T>>() {
			@Override
			public int compare(TableCell<T> a, TableCell<T> b) {
				return (a.bounds.min < b.bounds.min ? -1 : (a.bounds.min > b.bounds.min ? 1 : 0));
			}
		});
		for (int i = 0; i < num; i++) {
			TableCell<T> cell = table[i];
			float max = cell.bounds.max;

			for (int j = i + 1; table[j].bounds.min < max && j < num; j++) {
				func.apply(cell.obj, table[j].obj, new CollisionID(0));
			}
		}
		// Reindex query is also responsible for colliding against the static index.
		// Fortunately there is a helper function for that.
		collideStatic(this, staticIndex, func);
	}
}
