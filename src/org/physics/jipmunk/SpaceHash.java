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

import java.util.LinkedList;
import java.util.List;

/** @author jobernolte */
public class SpaceHash<T> extends SpatialIndex<T> {

	private static class HashBin<T> {
		Handle<T> handle;
		HashBin<T> next;

		private HashBin() {
		}

		private HashBin(Handle<T> handle, HashBin<T> next) {
			this.handle = handle;
			this.next = next;
		}

		@Override
		public String toString() {
			return "HashBin{" +
					"handle=" + handle +
					", next=" + (next != null ? next.hashCode() : null) +
					'}';
		}
	}

	private static class Handle<T> {
		T obj;
		int retain;
		long stamp;

		void init(T obj) {
			this.obj = obj;
			this.retain = 0;
			this.stamp = 0;
		}

		@Override
		public String toString() {
			return "Handle{" +
					"obj=" + obj +
					", retain=" + retain +
					", stamp=" + stamp +
					'}';
		}
	}

	private int numcells;
	private float celldim;
	//private List<HashBin<T>> table;
	private HashBin<T>[] table;
	private IntHashMap<Handle<T>> handleSet;
	// private HashBin<T> pooledBins;
	private LinkedList<Handle<T>> pooledHandles = new LinkedList<>();
	private long stamp;
	private int numBins;

	private void handleRetain(Handle<T> hand) {
		hand.retain++;
	}

	private void handleRelease(Handle<T> hand, List<Handle<T>> pooledHandles) {
		hand.retain--;
		if (hand.retain == 0) {
			hand.obj = null;
			hand.stamp = 0;
			pooledHandles.add(hand);
		}
	}

	private void recycleBin(HashBin<T> bin) {
		/*
		bin.handle = null;
		bin.next = this.pooledBins;
		this.pooledBins = bin;
		this.numBins--;
		*/
		bin.handle = null;
		bin.next = null;
	}

	private void clearTableCell(int idx) {
		HashBin<T> bin = this.table[idx]; // this.table.get(idx);
		while (bin != null) {
			HashBin<T> next = bin.next;

			handleRelease(bin.handle, pooledHandles);
			recycleBin(bin);

			bin = next;
		}

		//table.set(idx, null);
		table[idx] = null;
	}

	private void clearTable() {
		for (int i = 0; i < numcells; i++) {
			// clearTableCell(i);
			this.table[i] = null;
		}
	}

	// Frees the old table, and allocate a new one.
	private void allocTable(int numcells) {
		this.numcells = numcells;
		this.table = Array.newInstance(HashBin.class, numcells);
	}

	// Get a recycled or new bin.
	/*
	private HashBin<T> getEmptyBin() {
		HashBin<T> bin = pooledBins;

		this.numBins++;
		if (bin != null) {
			pooledBins = bin.next;
			return bin;
		} else {
			return new HashBin<>();
		}
	}
	*/

	public SpaceHash(float celldim, int numcells, SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
		super(bbfunc, staticIndex);

		allocTable(Prime.nextPrime(numcells));
		this.celldim = celldim;

		this.handleSet = new IntHashMap<>(); // cpHashSetNew(0, (cpHashSetEqlFunc) handleSetEql);
		// this.pooledBins = null;
		this.stamp = 1;
	}

	private boolean containsHandle(HashBin<T> bin, Handle<T> hand) {
		while (bin != null) {
			if (bin.handle == hand) {
				return true;
			}
			bin = bin.next;
		}
		return false;
	}

	// The hash function itself.
	private int hash_func(int x, int y, int n) {
		final int hash = (int) (x * 1640531513L ^ y * 2654435789L) % n;
		return (hash < 0) ? -hash : hash;
	}

	// Much faster than (int)floor(f)
	// Profiling showed floor() to be a sizable performance hog
	private static int floor_int(float f) {
		int i = (int) f;
		return (f < 0.0f && f != i ? i - 1 : i);
	}

	private void hashHandle(Handle<T> hand, BB bb) {
		// Find the dimensions in cell coordinates.
		float dim = this.celldim;
		int l = floor_int(bb.l / dim); // Fix by ShiftZ
		int r = floor_int(bb.r / dim);
		int b = floor_int(bb.b / dim);
		int t = floor_int(bb.t / dim);

		int n = this.numcells;
		for (int i = l; i <= r; i++) {
			for (int j = b; j <= t; j++) {
				int idx = hash_func(i, j, n);
				HashBin<T> bin = this.table[idx]; // this.table.get(idx);

				// Don't add an object twice to the same cell.
				if (containsHandle(bin, hand)) {
					continue;
				}

				handleRetain(hand);
				// Insert a new bin for the handle in this cell.
				HashBin<T> newBin = new HashBin<>(hand, bin);
				/*newBin.handle = hand;
				newBin.next = bin;*/
				this.table[idx] = newBin; // this.table.set(idx, newBin);
			}
		}
	}

	@Override
	public int count() {
		return handleSet.size();
	}

	@Override
	public void each(SpatialIndexIteratorFunc<T> iterator) {
		for (Handle<T> handle : handleSet.values()) {
			iterator.visit(handle.obj);
		}
	}

	@Override
	public boolean contains(T obj, int hashValue) {
		return handleSet.get(hashValue) == obj;
	}

	private Handle<T> createHandle(T obj) {
		Handle<T> hand;
		if (this.pooledHandles.isEmpty()) {
			hand = new Handle<>();
		} else {
			hand = this.pooledHandles.removeFirst();
		}

		hand.init(obj);
		handleRetain(hand);

		return hand;
	}

	@Override
	public void insert(T obj, int hashValue) {
		Handle<T> hand = createHandle(obj);
		this.handleSet.put(hashValue, hand);
		hashHandle(hand, this.bbfunc.apply(obj));
	}

	@Override
	public void remove(T obj, int hashValue) {
		Handle<T> hand = handleSet.remove(hashValue);

		if (hand != null) {
			hand.obj = null;
			handleRelease(hand, pooledHandles);
		}
	}

	private void rehash_helper(Handle<T> hand) {
		hashHandle(hand, bbfunc.apply(hand.obj));
	}

	@Override
	public void reindex() {
		clearTable();
		//cpHashSetEach(this.handleSet, (cpHashSetIteratorFunc)rehash_helper, hash);
		for (Handle<T> hand : handleSet.values()) {
			rehash_helper(hand);
		}
	}

	@Override
	public void reindexObject(T obj, int hashValue) {
		Handle<T> hand = handleSet.remove(hashValue);

		if (hand != null) {
			hand.obj = null;
			handleRelease(hand, pooledHandles);
			insert(obj, hashValue);
		}
	}

	private HashBin<T> remove_orphaned_handles(int idx, HashBin<T> bin_ptr) {
		HashBin<T> bin = bin_ptr;
		while (bin != null) {
			Handle<T> hand = bin.handle;
			HashBin<T> next = bin.next;

			if (hand.obj == null) {
				// orphaned handle, unlink and recycle the bin
				// ( * bin_ptr)=bin - > next;
				if (this.table[idx] == bin_ptr) {
					bin_ptr = this.table[idx] = next;
				} else {
					bin_ptr.next = next;
				}
				recycleBin(bin);

				handleRelease(hand, pooledHandles);
			} else {
				// bin_ptr =&bin - > next;
				bin_ptr = bin;
			}

			bin = next;
		}
		return bin_ptr;
	}

	private void query_helper(int idx, T obj, SpatialIndexQueryFunc<T> func) {
		HashBin<T> bin = this.table[idx], prev = bin;
		while (bin != null) {
			Handle<T> hand = bin.handle;
			T other = hand.obj;

			if (hand.stamp == this.stamp || obj == other) {
				prev = bin;
				bin = bin.next;
			} else if (other != null) {
				func.apply(obj, other, new CollisionID(0));
				hand.stamp = this.stamp;
				prev = bin;
				bin = bin.next;
			} else {
				// The object for this handle has been removed
				// cleanup this cell and restart the query
				prev = bin = remove_orphaned_handles(idx, prev);
			}
		}
	}

	// Hashset iterator func used with cpSpaceHashQueryRehash().
	private void queryRehash_helper(Handle<T> hand, SpatialIndexQueryFunc<T> func) {
		float dim = this.celldim;
		int n = this.numcells;

		T obj = hand.obj;
		BB bb = this.bbfunc.apply(obj);

		int l = floor_int(bb.l / dim);
		int r = floor_int(bb.r / dim);
		int b = floor_int(bb.b / dim);
		int t = floor_int(bb.t / dim);

		//HashBin **table = this.table;

		for (int i = l; i <= r; i++) {
			for (int j = b; j <= t; j++) {
				int idx = hash_func(i, j, n);
				HashBin<T> bin = this.table[idx]; // table.get(idx);

				if (containsHandle(bin, hand)) {
					continue;
				}

				handleRetain(hand); // this MUST be done first in case the object is removed in func()
				query_helper(idx, obj, func);

				HashBin<T> newBin = new HashBin<>(hand, bin); // getEmptyBin();
				/*newBin.handle = hand;
				newBin.next = bin;*/
				this.table[idx] = newBin; // table.set(idx, newBin);
			}
		}

		// Increment the stamp for each object hashed.
		this.stamp++;
	}

	@Override
	public void reindexQuery(final SpatialIndexQueryFunc<T> func) {
		clearTable();

		//queryRehashContext context = {hash, func, data};
		//cpHashSetEach(this.handleSet, (cpHashSetIteratorFunc)queryRehash_helper, &context);
		for (Handle<T> hand : handleSet.values()) {
			queryRehash_helper(hand, func);
		}

		cpSpatialIndexCollideStatic(this, this.staticIndex, func);
	}

	private float segmentQuery_helper(int idx, T obj, SpatialIndexSegmentQueryFunc<T> func) {
		HashBin<T> bin_ptr = this.table[idx]; // table.get(idx);
		HashBin<T> bin = bin_ptr;
		float t = 1.0f;

		while (bin != null) {
			Handle<T> hand = bin.handle;
			T other = hand.obj;

			// Skip over certain conditions
			if (hand.stamp == this.stamp) {
				bin = bin.next;
			} else if (other != null) {
				t = Util.cpfmin(t, func.apply(obj, other));
				hand.stamp = this.stamp;
				bin = bin.next;
			} else {
				// The object for this handle has been removed
				// cleanup this cell and restart the query
				bin = bin_ptr = remove_orphaned_handles(idx, bin_ptr);
			}
		}

		return t;
	}

	@Override
	public void segmentQuery(T obj, Vector2f a, Vector2f b, float t_exit, SpatialIndexSegmentQueryFunc<T> func) {
		a = Util.cpvmult(a, 1.0f / this.celldim);
		b = Util.cpvmult(b, 1.0f / this.celldim);

		int cell_x = floor_int(a.getX()), cell_y = floor_int(a.getY());

		float t = 0;

		int x_inc, y_inc;
		float temp_v, temp_h;

		if (b.getX() > a.getX()) {
			x_inc = 1;
			temp_h = (Util.cpffloor(a.getX() + 1.0f) - a.getX());
		} else {
			x_inc = -1;
			temp_h = (a.getX() - Util.cpffloor(a.getX()));
		}

		if (b.getY() > a.getY()) {
			y_inc = 1;
			temp_v = (Util.cpffloor(a.getY() + 1.0f) - a.getY());
		} else {
			y_inc = -1;
			temp_v = (a.getY() - Util.cpffloor(a.getY()));
		}

		// Division by zero is *very* slow on ARM
		float dx = Util.cpfabs(b.getX() - a.getX()), dy = Util.cpfabs(b.getY() - a.getY());
		float dt_dx = (dx != 0 ? 1.0f / dx : Float.POSITIVE_INFINITY), dt_dy =
				(dy != 0 ? 1.0f / dy : Float.POSITIVE_INFINITY);

		// fix NANs in horizontal directions
		float next_h = (temp_h != 0 ? temp_h * dt_dx : dt_dx);
		float next_v = (temp_v != 0 ? temp_v * dt_dy : dt_dy);

		int n = this.numcells;
		//HashBin<T> **table = this.table;

		while (t < t_exit) {
			int idx = hash_func(cell_x, cell_y, n);
			t_exit = Util.cpfmin(t_exit, segmentQuery_helper(idx, obj, func));

			if (next_v < next_h) {
				cell_y += y_inc;
				t = next_v;
				next_v += dt_dy;
			} else {
				cell_x += x_inc;
				t = next_h;
				next_h += dt_dx;
			}
		}

		this.stamp++;
	}

	@Override
	public void query(T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		// Get the dimensions in cell coordinates.
		float dim = this.celldim;
		int l = floor_int(bb.l / dim);  // Fix by ShiftZ
		int r = floor_int(bb.r / dim);
		int b = floor_int(bb.b / dim);
		int t = floor_int(bb.t / dim);

		int n = this.numcells;
		//HashBin<T> **table = hash - > table;

		// Iterate over the cells and query them.
		for (int i = l; i <= r; i++) {
			for (int j = b; j <= t; j++) {
				query_helper(hash_func(i, j, n), obj, func);
			}
		}
		this.stamp++;
	}
}
