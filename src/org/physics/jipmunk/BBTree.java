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

import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.Util.*;

/** @author jobernolte */
class BBTree<T> extends SpatialIndex<T> {

	static class Node<T> {
		T obj;
		BB bb;
		Node<T> parent;
		// Internal nodes
		Node<T> a, b;
		int stamp;
		Pair<T> pairs;

		void reset() {
			obj = null;
			bb = null;
			parent = null;
			a = b = null;
			pairs = null;
		}
	}

	static class Thread<T> {
		Pair<T> prev;
		Node<T> leaf;
		Pair<T> next;

		Thread(Pair<T> prev, Node<T> leaf, Pair<T> next) {
			this.prev = prev;
			this.leaf = leaf;
			this.next = next;
		}
	}

	static class Pair<T> {
		Thread<T> a, b;
		CollisionID id;

		void init(Thread<T> a, Thread<T> b, CollisionID id) {
			this.a = a;
			this.b = b;
			this.id = id;
		}

		void reset() {
			this.a = this.b = null;
			this.id = null;
		}
	}

	BBTreeVelocityFunc<T> velocityFunc;
	//cpHashSet *leaves;
	IntHashMap<Node<T>> leaves = new IntHashMap<>();
	Node<T> root;
	Pool<Node<T>> pooledNodes = new Pool<Node<T>>(10000) {
		@Override
		protected Node<T> create() {
			return new Node<>();
		}
	};
	Pool<Pair<T>> pooledPairs = new Pool<Pair<T>>(10000) {
		@Override
		protected Pair<T> create() {
			return new Pair<>();
		}
	};
	//cpArray*allocatedBuffers;
	// List<Pair<T>> allocatedBuffers = new ArrayList<Pair<T>>();
	int stamp;

	static <T> BB GetBB(BBTree<T> tree, T obj) {
		BB bb = tree.bbfunc.apply(obj);

		BBTreeVelocityFunc<T> velocityFunc = tree.velocityFunc;
		if (velocityFunc != null) {
			float coef = 0.1f;
			float x = (bb.r - bb.l) * coef;
			float y = (bb.t - bb.b) * coef;

			Vector2f v = cpvmult(velocityFunc.apply(obj), 0.1f);
			return new BB(bb.l + cpfmin(-x, v.getX()), bb.b + cpfmin(-y, v.getY()), bb.r + cpfmax(x, v.getX()),
						  bb.t + cpfmax(y, v.getY()));
		} else {
			return bb;
		}
	}

	static <T> int GetStamp(BBTree<T> tree) {
		BBTree<T> dynamicTree = GetTree(tree.dynamicIndex);
		return (dynamicTree != null ? dynamicTree.stamp : tree.stamp);
	}

	private static <T> BBTree<T> GetTree(SpatialIndex<T> index) {
		return index instanceof BBTree ? (BBTree<T>) index : null;
	}

	static <T> void IncrementStamp(BBTree<T> tree) {
		BBTree<T> dynamicTree = GetTree(tree.dynamicIndex);
		if (dynamicTree != null) {
			dynamicTree.stamp++;
		} else {
			tree.stamp++;
		}
	}

	static <T> void PairRecycle(BBTree<T> tree, Pair<T> pair) {
		/*pair.a.next = tree.pooledPairs;
				tree.pooledPairs = pair;*/
		pair.reset();
		tree.pooledPairs.free(pair);
	}

	static <T> Pair<T> PairFromPool(BBTree<T> tree) {
		/*Pair pair = tree.pooledPairs;
		
			  if(pair != null){
				  tree.pooledPairs = pair.a.next;
				  return pair;
			  } else {
				  // Pool is exhausted, make more
				  int count = CP_BUFFER_BYTES/sizeof(Pair);
				  cpAssertSoft(count, "Buffer size is too small.");
		
				  Pair buffer = (Pair )cpcalloc(1, CP_BUFFER_BYTES);
				  cpArrayPush(tree.allocatedBuffers, buffer);
		
				  // push all but the first one, return the first instead
				  for(int i=1; i<count; i++) PairRecycle(tree, buffer + i);
				  return buffer;
			  }*/
		return tree.pooledPairs.alloc();
	}

	static <T> void ThreadUnlink(Thread<T> thread) {
		Pair<T> next = thread.next;
		Pair<T> prev = thread.prev;

		if (next != null) {
			if (next.a.leaf == thread.leaf)
				next.a.prev = prev;
			else
				next.b.prev = prev;
		}

		if (prev != null) {
			if (prev.a.leaf == thread.leaf)
				prev.a.next = next;
			else
				prev.b.next = next;
		} else {
			thread.leaf.pairs = next;
		}
	}

	static <T> void PairsClear(Node<T> leaf, BBTree<T> tree) {
		Pair<T> pair = leaf.pairs;
		leaf.pairs = null;

		while (pair != null) {
			if (pair.a.leaf == leaf) {
				Pair<T> next = pair.a.next;
				ThreadUnlink(pair.b);
				PairRecycle(tree, pair);
				pair = next;
			} else {
				Pair<T> next = pair.b.next;
				ThreadUnlink(pair.a);
				PairRecycle(tree, pair);
				pair = next;
			}
		}
	}

	static <T> void PairInsert(Node<T> a, Node<T> b, BBTree<T> tree) {
		Pair<T> nextA = a.pairs, nextB = b.pairs;
		Pair<T> pair = PairFromPool(tree);
		pair.init(new Thread<>(null, a, nextA), new Thread<>(null, b, nextB), new CollisionID(0));

		a.pairs = b.pairs = pair;
		//*pair = temp;

		if (nextA != null) {
			if (nextA.a.leaf == a)
				nextA.a.prev = pair;
			else
				nextA.b.prev = pair;
		}

		if (nextB != null) {
			if (nextB.a.leaf == b)
				nextB.a.prev = pair;
			else
				nextB.b.prev = pair;
		}
	}

	static <T> void NodeRecycle(BBTree<T> tree, Node<T> node) {
		/*sleeping.parent = tree.pooledNodes;
			  tree.pooledNodes = sleeping;*/
		node.reset();
		tree.pooledNodes.free(node);
	}

	static <T> Node<T> NodeFromPool(BBTree<T> tree) {
		/*Node sleeping = tree.pooledNodes;
		
			  if(sleeping){
				  tree.pooledNodes = sleeping.parent;
				  return sleeping;
			  } else {
				  // Pool is exhausted, make more
				  int count = CP_BUFFER_BYTES/sizeof(Node);
				  cpAssertSoft(count, "Buffer size is too small.");
		
				  Node buffer = (Node )cpcalloc(1, CP_BUFFER_BYTES);
				  cpArrayPush(tree.allocatedBuffers, buffer);
		
				  // push all but the first one, return the first instead
				  for(int i=1; i<count; i++) NodeRecycle(tree, buffer + i);
				  return buffer;
			  }*/
		return tree.pooledNodes.alloc();
	}

	static <T> void NodeSetA(Node<T> node, Node<T> value) {
		node.a = value;
		value.parent = node;
	}

	static <T> void NodeSetB(Node<T> node, Node<T> value) {
		node.b = value;
		value.parent = node;
	}

	static <T> Node<T> NodeNew(BBTree<T> tree, Node<T> a, Node<T> b) {
		Node<T> node = NodeFromPool(tree);

		node.obj = null;
		node.bb = a.bb.merge(b.bb);
		node.parent = null;

		NodeSetA(node, a);
		NodeSetB(node, b);

		return node;
	}

	static <T> boolean NodeIsLeaf(Node<T> node) {
		return (node.obj != null);
	}

	static <T> Node<T> NodeOther(Node<T> node, Node<T> child) {
		return (node.a == child ? node.b : node.a);
	}

	static <T> void NodeReplaceChild(Node<T> parent, Node<T> child, Node<T> value, BBTree<T> tree) {
		cpAssertSoft(!NodeIsLeaf(parent), "Cannot replace child of a leaf.");
		cpAssertSoft(child == parent.a || child == parent.b, "Node is not a child of parent.");

		if (parent.a == child) {
			NodeRecycle(tree, parent.a);
			NodeSetA(parent, value);
		} else {
			NodeRecycle(tree, parent.b);
			NodeSetB(parent, value);
		}

		for (Node<T> node = parent; node != null; node = node.parent) {
			node.bb = node.a.bb.merge(node.b.bb);
		}
	}

	static float cpBBProximity(BB a, BB b) {
		return cpfabs(a.l + a.r - b.l - b.r) + cpfabs(a.b + a.t - b.b - b.t);
	}

	Node<T> SubtreeInsert(Node<T> subtree, Node<T> leaf, BBTree<T> tree) {
		if (subtree == null) {
			return leaf;
		} else if (NodeIsLeaf(subtree)) {
			return NodeNew(tree, leaf, subtree);
		} else {
			float cost_a = cpBBArea(subtree.b.bb) + cpBBMergedArea(subtree.a.bb, leaf.bb);
			float cost_b = cpBBArea(subtree.a.bb) + cpBBMergedArea(subtree.b.bb, leaf.bb);

			if (cost_a == cost_b) {
				cost_a = cpBBProximity(subtree.a.bb, leaf.bb);
				cost_b = cpBBProximity(subtree.b.bb, leaf.bb);
			}

			if (cost_b < cost_a) {
				NodeSetB(subtree, SubtreeInsert(subtree.b, leaf, tree));
			} else {
				NodeSetA(subtree, SubtreeInsert(subtree.a, leaf, tree));
			}

			subtree.bb = subtree.bb.merge(leaf.bb);
			return subtree;
		}
	}

	void SubtreeQuery(Node<T> subtree, T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		if (subtree.bb.intersects(bb)) {
			if (NodeIsLeaf(subtree)) {
				func.apply(obj, subtree.obj, new CollisionID(0));
			} else {
				SubtreeQuery(subtree.a, obj, bb, func);
				SubtreeQuery(subtree.b, obj, bb, func);
			}
		}
	}

	// TODO Needs early exit optimization for ray queries
	void SubtreeSegmentQuery(Node<T> subtree, T obj, Vector2f a, Vector2f b, SpatialIndexSegmentQueryFunc<T> func) {
		if (cpBBIntersectsSegment(subtree.bb, a, b)) {
			if (NodeIsLeaf(subtree)) {
				func.apply(obj, subtree.obj);
			} else {
				SubtreeSegmentQuery(subtree.a, obj, a, b, func);
				SubtreeSegmentQuery(subtree.b, obj, a, b, func);
			}
		}
	}

	static <T> void SubtreeRecycle(BBTree<T> tree, Node<T> node) {
		if (!NodeIsLeaf(node)) {
			SubtreeRecycle(tree, node.a);
			SubtreeRecycle(tree, node.b);
			NodeRecycle(tree, node);
		}
	}

	static <T> Node<T> SubtreeRemove(Node<T> subtree, Node<T> leaf, BBTree<T> tree) {
		if (leaf == subtree) {
			return null;
		} else {
			Node<T> parent = leaf.parent;
			if (parent == subtree) {
				Node<T> other = NodeOther(subtree, leaf);
				other.parent = subtree.parent;
				NodeRecycle(tree, subtree);
				return other;
			} else {
				NodeReplaceChild(parent.parent, parent, NodeOther(parent, leaf), tree);
				return subtree;
			}
		}
	}

	static class MarkContext<V> {
		BBTree<V> tree;
		Node<V> staticRoot;
		SpatialIndexQueryFunc<V> func;

		MarkContext(BBTree<V> tree, Node<V> staticRoot, SpatialIndexQueryFunc<V> func) {
			this.tree = tree;
			this.staticRoot = staticRoot;
			this.func = func;
		}
	}

	static <T> void MarkLeafQuery(Node<T> subtree, Node<T> leaf, boolean left, MarkContext<T> context) {
		if (leaf.bb.intersects(subtree.bb)) {
			if (NodeIsLeaf(subtree)) {
				if (left) {
					PairInsert(leaf, subtree, context.tree);
				} else {
					if (subtree.stamp < leaf.stamp) {
						PairInsert(subtree, leaf, context.tree);
					}
					context.func.apply(leaf.obj, subtree.obj, new CollisionID(0));
				}
			} else {
				MarkLeafQuery(subtree.a, leaf, left, context);
				MarkLeafQuery(subtree.b, leaf, left, context);
			}
		}
	}

	static <T> void MarkLeaf(Node<T> leaf, MarkContext<T> context) {
		BBTree<T> tree = context.tree;
		if (leaf.stamp == GetStamp(tree)) {
			Node<T> staticRoot = context.staticRoot;
			if (staticRoot != null) {
				MarkLeafQuery(staticRoot, leaf, false, context);
			}

			for (Node<T> node = leaf; node.parent != null; node = node.parent) {
				if (node == node.parent.a) {
					MarkLeafQuery(node.parent.b, leaf, true, context);
				} else {
					MarkLeafQuery(node.parent.a, leaf, false, context);
				}
			}
		} else {
			Pair<T> pair = leaf.pairs;
			while (pair != null) {
				if (leaf == pair.b.leaf) {
					pair.id = context.func.apply(pair.a.leaf.obj, leaf.obj, pair.id);
					pair = pair.b.next;
				} else {
					pair = pair.a.next;
				}
			}
		}
	}

	static <T> void MarkSubtree(Node<T> subtree, MarkContext<T> context) {
		if (NodeIsLeaf(subtree)) {
			MarkLeaf(subtree, context);
		} else {
			MarkSubtree(subtree.a, context);
			MarkSubtree(subtree.b, context);
		}
	}

	static <T> Node<T> LeafNew(BBTree<T> tree, T obj) {
		Node<T> node = NodeFromPool(tree);
		node.obj = obj;
		node.bb = GetBB(tree, obj);

		node.parent = null;
		node.stamp = 0;
		node.pairs = null;

		return node;
	}

	boolean LeafUpdate(Node<T> leaf, BBTree<T> tree) {
		Node<T> root = tree.root;
		BB bb = tree.bbfunc.apply(leaf.obj);

		if (!leaf.bb.contains(bb)) {
			leaf.bb = GetBB(tree, leaf.obj);

			root = SubtreeRemove(root, leaf, tree);
			tree.root = SubtreeInsert(root, leaf, tree);

			PairsClear(leaf, tree);
			leaf.stamp = GetStamp(tree);

			return true;
		} else {
			return false;
		}
	}

	/*static SpatialIndexQueryFunc<T> voidQueryFunc = new SpatialIndexQueryFunc<T>() {
			@Override
			public void apply(T obj2) {
			}
		};*/

	void LeafAddPairs(Node<T> leaf, BBTree<T> tree) {
		SpatialIndex<T> dynamicIndex = tree.dynamicIndex;
		if (dynamicIndex != null) {
			Node<T> dynamicRoot = GetRootIfTree(dynamicIndex);
			if (dynamicRoot != null) {
				BBTree<T> dynamicTree = GetTree(dynamicIndex);
				MarkContext<T> context = new MarkContext<>(dynamicTree, null, null);
				MarkLeafQuery(dynamicRoot, leaf, true, context);
			}
		} else {
			Node<T> staticRoot = GetRootIfTree(tree.staticIndex);
			MarkContext<T> context = new MarkContext<>(tree, staticRoot, (obj1, obj2, id) -> id);
			MarkLeaf(leaf, context);
		}
	}

	private static <T> Node<T> GetRootIfTree(SpatialIndex<T> index) {
		return index instanceof BBTree ? ((BBTree<T>) index).root : null;
	}

	/*
 BBTree
 cpBBTreeAlloc(void)
 {
	 return (BBTree )cpcalloc(1, sizeof(cpBBTree));
 }
 
 static int
 leafSetEql(Object obj, Node sleeping)
 {
	 return (obj == sleeping.obj);
 }
 
 static Object 
 leafSetTrans(Object obj, BBTree tree)
 {
	 return LeafNew(tree, obj, tree.spatialIndex.bbfunc(obj));
 }
 
 SpatialIndex 
 cpBBTreeInit(BBTree tree, SpatialIndexBBFunc bbfunc, SpatialIndex staticIndex)
 {
	 cpSpatialIndexInit((SpatialIndex )tree, &klass, bbfunc, staticIndex);
	 
	 tree.velocityFunc = null;
	 
	 tree.leaves = cpHashSetNew(0, (cpHashSetEqlFunc)leafSetEql);
	 tree.root = null;
	 
	 tree.pooledNodes = null;
	 tree.allocatedBuffers = cpArrayNew(0);
	 
	 tree.stamp = 0;
	 
	 return (SpatialIndex )tree;
 }*/

	static <T> void cpBBTreeSetVelocityFunc(SpatialIndex<T> index, BBTreeVelocityFunc<T> func) {

		((BBTree<T>) index).velocityFunc = func;
	}

/*SpatialIndex
cpBBTreeNew(SpatialIndexBBFunc bbfunc, SpatialIndex staticIndex)
{
	return cpBBTreeInit(cpBBTreeAlloc(), bbfunc, staticIndex);
}

static void
cpBBTreeDestroy(BBTree tree)
{
	cpHashSetFree(tree.leaves);
	
	if(tree.allocatedBuffers) cpArrayFreeEach(tree.allocatedBuffers, cpfree);
	cpArrayFree(tree.allocatedBuffers);
} */

	void cpBBTreeInsert(BBTree<T> tree, T obj, int hashid) {
		//Node leaf = (Node) cpHashSetInsert(tree.leaves, hashid, obj, tree, (cpHashSetTransFunc) leafSetTrans);
		Node<T> leaf = tree.leaves.get(hashid);
		if (leaf == null) {
			leaf = LeafNew(tree, obj);
			tree.leaves.put(hashid, leaf);
		}

		Node<T> root = tree.root;
		tree.root = SubtreeInsert(root, leaf, tree);

		leaf.stamp = GetStamp(tree);
		LeafAddPairs(leaf, tree);
		IncrementStamp(tree);
	}

	static <T> void cpBBTreeRemove(BBTree<T> tree, T obj, int hashid) {
		//Node leaf = (Node )cpHashSetRemove(tree.leaves, hashid, obj);
		Node<T> leaf = tree.leaves.remove(hashid);

		tree.root = SubtreeRemove(tree.root, leaf, tree);
		PairsClear(leaf, tree);
		NodeRecycle(tree, leaf);
	}

	static <T> boolean cpBBTreeContains(BBTree<T> tree, T obj, int hashid) {
		//return (cpHashSetFind(tree.leaves, hashid, obj) != null);
		return tree.leaves.containsKey(hashid);
	}

	void cpBBTreeReindexQuery(BBTree<T> tree, SpatialIndexQueryFunc<T> func) {
		if (tree.root == null) {
			return;
		}

		// LeafUpdate() may modify tree.root. Don'alpha cache it.
		// cpHashSetEach(tree.leaves, (HashSetIteratorFunc) LeafUpdate, tree);
		for (Node<T> node : tree.leaves.values()) {
			LeafUpdate(node, tree);
		}

		SpatialIndex<T> staticIndex = tree.staticIndex;
		Node<T> staticRoot = GetRootIfTree(staticIndex);

		MarkContext<T> context = new MarkContext<>(tree, staticRoot, func);
		MarkSubtree(tree.root, context);
		if (staticIndex != null && staticRoot == null) {
			collideStatic(tree, staticIndex, func);
		}

		IncrementStamp(tree);
	}

	void cpBBTreeReindex(BBTree<T> tree) {
		cpBBTreeReindexQuery(tree, (obj1, obj2, id) -> id);
	}

	void cpBBTreeReindexObject(BBTree<T> tree, T obj, int hashid) {
		Node<T> leaf = tree.leaves.get(hashid);
		if (leaf != null) {
			if (LeafUpdate(leaf, tree)) {
				LeafAddPairs(leaf, tree);
			}
			IncrementStamp(tree);
		}
	}

	void cpBBTreeQuery(BBTree<T> tree, T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		if (tree.root != null) {
			SubtreeQuery(tree.root, obj, bb, func);
		}
	}

	int cpBBTreeCount(BBTree<T> tree) {
		//return cpHashSetCount(tree.leaves);
		return tree.leaves.size();
	}

	void cpBBTreeEach(BBTree<T> tree, final SpatialIndexIteratorFunc<T> func) {
		for (Node<T> leaf : tree.leaves.values()) {
			func.visit(leaf.obj);
		}
	}

	static int cpfcompare(float a, float b) {
		return (a < b ? -1 : (b < a ? 1 : 0));
	}

	/*static void fillNodeArray(Node sleeping, Node **cursor) {
			( **cursor)=sleeping;
			( * cursor)++;
		}*/

	Node<T> partitionNodes(BBTree<T> tree, Node<T>[] nodes, int index, int count) {
		if (count == 1) {
			return nodes[index + 0];
		} else if (count == 2) {
			return NodeNew(tree, nodes[index + 0], nodes[index + 1]);
		}

		// Find the AABB for these nodes
		BB bb = nodes[index + 0].bb;
		for (int i = 1; i < count; i++)
			bb = bb.merge(nodes[index + i].bb);

		// Split it on it's longest axis
		boolean splitWidth = (bb.r - bb.l > bb.t - bb.b);

		// Sort the bounds and use the median as the splitting point
		float[] bounds = new float[count * 2]; //  (float *)cpcalloc(count*2, sizeof(float));
		if (splitWidth) {
			for (int i = 0; i < count; i++) {
				bounds[2 * i + 0] = nodes[index + i].bb.l;
				bounds[2 * i + 1] = nodes[index + i].bb.r;
			}
		} else {
			for (int i = 0; i < count; i++) {
				bounds[2 * i + 0] = nodes[index + i].bb.b;
				bounds[2 * i + 1] = nodes[index + i].bb.t;
			}
		}

		//qsort(bounds, count * 2, sizeof(float),(int( *)(const Object,const Object))cpfcompare);
		Arrays.sort(bounds);

		float split = (bounds[count - 1] + bounds[count]) * 0.5f; // use the medain as the split
		//cpfree(bounds);

		// Generate the child BBs
		BB a = bb, b = bb;
		if (splitWidth)
			a.r = b.l = split;
		else
			a.t = b.b = split;

		// Partition the nodes
		int right = count;
		for (int left = 0; left < right; ) {
			Node<T> node = nodes[index + left];
			if (cpBBMergedArea(node.bb, b) < cpBBMergedArea(node.bb, a)) {
				//		if(cpBBProximity(sleeping.bb, b) < cpBBProximity(sleeping.bb, a)){
				right--;
				nodes[index + left] = nodes[index + right];
				nodes[index + right] = node;
			} else {
				left++;
			}
		}

		if (right == count) {
			Node<T> node = null;
			for (int i = 0; i < count; i++) {
				node = SubtreeInsert(node, nodes[i], tree);
			}
			return node;
		}

		// Recurse and build the sleeping!
		return NodeNew(tree, partitionNodes(tree, nodes, index, right),
					   partitionNodes(tree, nodes, index + right, count - right));
	}

	public static <T> T[] newArray(Class<T[]> type, int size) {
		return type.cast(java.lang.reflect.Array.newInstance(type.getComponentType(), size));
	}

	public void optimize() {
		Node<T> root = this.root;
		if (root == null) {
			return;
		}

		int count = cpBBTreeCount(this);
		/*Node * nodes = (Node *)
				cpcalloc(count, sizeof(Node));
				Node * cursor = nodes;
		
				cpHashSetEach(tree.leaves, (cpHashSetIteratorFunc) fillNodeArray, & cursor);*/
		//java.lang.reflect.Array.newInstance()
		@SuppressWarnings("unchecked") Node<T>[] nodes = (Node<T>[]) newArray(Node[].class, this.leaves.size());
		int i = 0;
		for (Node<T> leaf : this.leaves.values()) {
			nodes[i] = this.pooledNodes.alloc();
			nodes[i].bb = leaf.bb;
			nodes[i].obj = leaf.obj;
		}

		SubtreeRecycle(this, root);
		this.root = partitionNodes(this, nodes, 0, count);
		//cpfree(nodes);
	}

	public BBTree(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
		super(bbfunc, staticIndex);
	}

	@Override
	public int count() {
		return cpBBTreeCount(this);
	}

	@Override
	public void each(SpatialIndexIteratorFunc<T> iterator) {
		cpBBTreeEach(this, iterator);
	}

	@Override
	public boolean contains(T obj, int hashValue) {
		return cpBBTreeContains(this, obj, hashValue);
	}

	@Override
	public void insert(T obj, int hashValue) {
		cpBBTreeInsert(this, obj, hashValue);
	}

	@Override
	public void remove(T obj, int hashValue) {
		cpBBTreeRemove(this, obj, hashValue);
	}

	@Override
	public void reindex() {
		cpBBTreeReindex(this);
	}

	@Override
	public void reindexObject(T obj, int hashValue) {
		cpBBTreeReindexObject(this, obj, hashValue);
	}

	@Override
	public void reindexQuery(SpatialIndexQueryFunc<T> func) {
		cpBBTreeReindexQuery(this, func);
	}

	@Override
	public void segmentQuery(T obj, Vector2f a, Vector2f b, float exit, SpatialIndexSegmentQueryFunc<T> func) {
		Node<T> root = this.root;
		if (root != null) {
			SubtreeSegmentQuery(root, obj, a, b, func);
		}
	}

	@Override
	public void query(T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		cpBBTreeQuery(this, obj, bb, func);
	}
}
