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
	}

	private int numcells;
	private float celldim;

	//private List<HashBin<T>> table;
	private HashBin<T>[] table;
	private IntHashMap<Handle<T>> handleSet;

	private HashBin<T> pooledBins;
	private LinkedList<Handle<T>> pooledHandles = new LinkedList<>();

	private long stamp;

	private void handleRetain(Handle<T> hand) {
		hand.retain++;
	}

	private void handleRelease(Handle<T> hand, List<Handle<T>> pooledHandles) {
		hand.retain--;
		if (hand.retain == 0) {
			pooledHandles.add(hand);
		}
	}

	private void recycleBin(HashBin<T> bin) {
		bin.next = this.pooledBins;
		this.pooledBins = bin;
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
			clearTableCell(i);
		}
	}

	// Frees the old table, and allocate a new one.
	private void allocTable(int numcells) {
		this.numcells = numcells;
		/*this.table = new ArrayList<>(numcells);
		for (int i = 0; i < numcells; i++) {
			table.add(null);
		}*/
		this.table = Array.newInstance(HashBin.class, numcells);
	}

	// Get a recycled or new bin.
	private HashBin<T> getEmptyBin() {
		HashBin<T> bin = pooledBins;

		if (bin != null) {
			pooledBins = bin.next;
			return bin;
		} else {
			/*// Pool is exhausted, make more
			int count = CP_BUFFER_BYTES / sizeof(cpSpaceHashBin);
			cpAssertHard(count, "Internal Error: Buffer size is too small.");

			cpSpaceHashBin * buffer = (cpSpaceHashBin *) cpcalloc(1, CP_BUFFER_BYTES);
			cpArrayPush(hash - > allocatedBuffers, buffer);

			// push all but the first one, return the first instead
			for (int i = 1; i < count; i++) recycleBin(hash, buffer + i);
			return buffer;*/
			return new HashBin<T>();
		}
	}

	public SpaceHash(float celldim, int numcells, SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
		super(bbfunc, staticIndex);

		allocTable(numcells);
		this.celldim = celldim;

		this.handleSet = new IntHashMap<Handle<T>>(); // cpHashSetNew(0, (cpHashSetEqlFunc) handleSetEql);
		this.pooledBins = null;
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
		return (int) Math.abs((x * 1640531513L ^ y * 2654435789L) % n);
	}

	// Much faster than (int)floor(f)
	// Profiling showed floor() to be a sizable performance hog
	private int floor_int(float f) {
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
				HashBin<T> newBin = getEmptyBin();
				newBin.handle = hand;
				newBin.next = bin;
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
			hand = new Handle<T>();
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
		for (Handle<T> hand : handleSet.values) {
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
				if (this.table[idx] == bin_ptr /*table.get(idx) == bin_ptr*/) {
					this.table[idx] = bin.next; // table.set(idx, bin.next);
				} else {
					bin_ptr.next = bin.next;
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

	private void query_helper(int idx, SpatialIndexQueryFunc<T> func) {
		HashBin<T> bin_ptr = this.table[idx]; // table.get(idx);
		HashBin<T> bin = bin_ptr;
		//for (HashBin<T> bin = bin_ptr; bin != null; bin = bin.next) {
		while (bin != null) {
			Handle<T> hand = bin.handle;
			T other = hand.obj;

			if (hand.stamp == this.stamp) {
				bin = bin.next;
			} else if (other != null) {
				func.apply(other);
				hand.stamp = this.stamp;
				bin = bin.next;
			} else {
				// The object for this handle has been removed
				// cleanup this cell and restart the query
				bin = bin_ptr = remove_orphaned_handles(idx, bin_ptr);
			}
		}
	}

	private void query_helper(int idx, SpatialReIndexQueryFunc<T> func, T obj) {
		HashBin<T> bin_ptr = this.table[idx]; // table.get(idx);
		HashBin<T> bin = bin_ptr;
		//for (HashBin<T> bin = bin_ptr; bin != null; bin = bin.next) {
		while (bin != null) {
			Handle<T> hand = bin.handle;
			T other = hand.obj;

			if (hand.stamp == this.stamp) {
				bin = bin.next;
			} else if (other != null) {
				func.apply(obj, other);
				hand.stamp = this.stamp;
				bin = bin.next;
			} else {
				// The object for this handle has been removed
				// cleanup this cell and restart the query
				bin = bin_ptr = remove_orphaned_handles(idx, bin_ptr);
			}
		}
	}

	// Hashset iterator func used with cpSpaceHashQueryRehash().
	private void queryRehash_helper(Handle<T> hand, SpatialReIndexQueryFunc<T> func) {
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
				query_helper(idx, func, obj);

				HashBin<T> newBin = getEmptyBin();
				newBin.handle = hand;
				newBin.next = bin;
				this.table[idx] = newBin; // table.set(idx, newBin);
			}
		}

		// Increment the stamp for each object hashed.
		this.stamp++;
	}

	@Override
	public void reindexQuery(final SpatialReIndexQueryFunc<T> tSpatialReIndexQueryFunc) {
		clearTable();

		//queryRehashContext context = {hash, func, data};
		//cpHashSetEach(this.handleSet, (cpHashSetIteratorFunc)queryRehash_helper, &context);
		for (Handle<T> hand : handleSet.values()) {
			queryRehash_helper(hand, tSpatialReIndexQueryFunc);
		}

		cpSpatialIndexCollideStatic(this, this.staticIndex, tSpatialReIndexQueryFunc);
	}

	private float segmentQuery_helper(int idx, SpatialIndexSegmentQueryFunc<T> func) {
		HashBin<T> bin_ptr = this.table[idx]; // table.get(idx);
		float t = 1.0f;

		for (HashBin<T> bin = bin_ptr; bin != null; bin = bin.next) {
			Handle<T> hand = bin.handle;
			T other = hand.obj;

			// Skip over certain conditions
			if (hand.stamp == this.stamp) {
				continue;
			} else if (other != null) {
				t = Util.cpfmin(t, func.apply(other));
				hand.stamp = this.stamp;
			} else {
				// The object for this handle has been removed
				// cleanup this cell and restart the query
				remove_orphaned_handles(idx, bin_ptr);
			}
		}

		return t;
	}

	@Override
	public void segmentQuery(Vector2f a, Vector2f b, float t_exit,
			SpatialIndexSegmentQueryFunc<T> tSpatialIndexSegmentQueryFunc) {
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
		float dt_dx = (dx != 0 ? 1.0f / dx : Float.POSITIVE_INFINITY), dt_dy = (dy != 0 ? 1.0f / dy
				: Float.POSITIVE_INFINITY);

		// fix NANs in horizontal directions
		float next_h = (temp_h != 0 ? temp_h * dt_dx : dt_dx);
		float next_v = (temp_v != 0 ? temp_v * dt_dy : dt_dy);

		int n = this.numcells;
		//HashBin<T> **table = this.table;

		while (t < t_exit) {
			int idx = hash_func(cell_x, cell_y, n);
			t_exit = Util.cpfmin(t_exit, segmentQuery_helper(idx, tSpatialIndexSegmentQueryFunc));

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
	public void query(BB bb, SpatialIndexQueryFunc<T> tSpatialIndexQueryFunc) {
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
				query_helper(hash_func(i, j, n), tSpatialIndexQueryFunc);
			}
		}
		this.stamp++;
	}
}
