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

import static org.physics.jipmunk.Util.*;

/**
 * @author jobernolte
 */
@SuppressWarnings("unchecked")
public class BBTree2<T> extends SpatialIndex<T> {
	BBTreeVelocityFunc<T> velocityFunc;
	IntHashMap<Leaf<T>> leaves = new IntHashMap<>();
	Node<T> root;
	int stamp;

	static abstract class Node<T> {
		BB bb;
		SubTree<T> parent;

		void reset() {
			this.bb = null;
			this.parent = null;
		}

		abstract boolean isLeaf();

		@Override
		public String toString() {
			return "Node{" +
					"bb=" + bb +
					", parent=" + parent +
					'}';
		}
	}

	static class SubTree<T> extends Node<T> {
		Node<T> a;
		Node<T> b;

		@Override
		boolean isLeaf() {
			return false;
		}
	}

	static class Leaf<T> extends Node<T> {
		T obj;
		int stamp;
		Pair<T> pairs;

		@Override
		boolean isLeaf() {
			return true;
		}

		@Override
		public String toString() {
			return "Leaf{" +
					"obj=" + obj +
					", stamp=" + stamp +
					", pairs=" + pairs +
					"} " + super.toString();
		}
	}

	static class Thread<T> {
		Pair<T> prev;
		Leaf<T> leaf;
		Pair<T> next;

		Thread(Pair<T> prev, Leaf<T> leaf, Pair<T> next) {
			this.prev = prev;
			this.leaf = leaf;
			this.next = next;
		}
	}

	static class Pair<T> {
		Thread<T> a, b;
		CollisionID id;

		Pair(Thread<T> a, Thread<T> b, CollisionID id) {
			this.a = a;
			this.b = b;
			this.id = id;
		}

		void reset() {
			this.a = this.b = null;
			this.id = null;
		}
	}

	//MARK: Misc Functions

	static <T> BB GetBB(BBTree2<T> tree, T obj) {
		BB bb = tree.bbfunc.apply(obj);

		BBTreeVelocityFunc<T> velocityFunc = tree.velocityFunc;
		if (velocityFunc != null) {
			float coef = 0.1f;
			float x = (bb.r - bb.l) * coef;
			float y = (bb.t - bb.b) * coef;

			Vector2f v = cpvmult(velocityFunc.apply(obj), 0.1f);
			return new BB(bb.l + cpfmin(-x, v.x), bb.b + cpfmin(-y, v.y), bb.r + cpfmax(x, v.x), bb.t + cpfmax(y, v.y));
		} else {
			return bb;
		}
	}

	static <T> BBTree2<T> GetTree(SpatialIndex<T> index) {
		return ((index != null && index instanceof BBTree) ? (BBTree2<T>) index : null);
	}

	static <T> Node<T> GetRootIfTree(SpatialIndex index) {
		return ((index != null && index instanceof BBTree) ? ((BBTree2<T>) index).root : null);
	}

	static <T> BBTree2<T> GetMasterTree(BBTree2<T> tree) {
		BBTree2<T> dynamicTree = GetTree(tree.dynamicIndex);
		return (dynamicTree != null ? dynamicTree : tree);
	}

	static <T> void IncrementStamp(BBTree2<T> tree) {
		BBTree2<T> dynamicTree = GetTree(tree.dynamicIndex);
		if (dynamicTree != null) {
			dynamicTree.stamp++;
		} else {
			tree.stamp++;
		}
	}

	//MARK: Pair/Thread Functions

	static <T> void PairRecycle(Pair<T> pair) {
		// Share the pool of the master tree.
		// TODO: would be lovely to move the pairs stuff into an external data structure.
		pair.reset();
		// tree = GetMasterTree(tree);
		// tree.pooledPairs.free(pair);
	}

	/*static <T> Pair<T> PairFromPool(BBTree2<T> tree) {
		// Share the pool of the master tree.
		// TODO: would be lovely to move the pairs stuff into an external data structure.
		tree = GetMasterTree(tree);
		return tree.pooledPairs.alloc();
	}*/

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

	@SuppressWarnings("unused")
	static <T> void PairsClear(Leaf<T> leaf, BBTree2<T> tree) {
		Pair<T> pair = leaf.pairs;
		leaf.pairs = null;

		while (pair != null) {
			if (pair.a.leaf == leaf) {
				Pair<T> next = pair.a.next;
				ThreadUnlink(pair.b);
				PairRecycle(pair);
				pair = next;
			} else {
				Pair<T> next = pair.b.next;
				ThreadUnlink(pair.a);
				PairRecycle(pair);
				pair = next;
			}
		}
	}

	static <T> void PairInsert(Leaf<T> a, Leaf<T> b) {
		Pair<T> nextA = a.pairs, nextB = b.pairs;
		Pair<T> pair = new Pair<>(new Thread<T>(null, a, nextA), new Thread<T>(null, b, nextB), new CollisionID(0));

		a.pairs = b.pairs = pair;

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

	//MARK: Node Functions

	static <T> void NodeRecycle(Node<T> node) {
		node.reset();
		// tree.pooledNodes.free(node);
	}

	/*static <T> Node<T> NodeFromPool(BBTree2<T> tree) {
		return tree.pooledNodes.alloc();
	}*/

	static <T> void NodeSetA(SubTree<T> node, Node<T> value) {
		node.a = value;
		value.parent = node;
	}

	static <T> void NodeSetB(SubTree<T> node, Node<T> value) {
		node.b = value;
		value.parent = node;
	}

	static <T> SubTree<T> NodeNew(Node<T> a, Node<T> b) {
		SubTree<T> node = new SubTree<>(); // NodeFromPool(tree);
		if (a.bb == null || b.bb == null) {
			throw new IllegalStateException();
		}
		node.bb = BB.merge(a.bb, b.bb);
		node.parent = null;

		NodeSetA(node, a);
		NodeSetB(node, b);

		return node;
	}

	static <T> boolean NodeIsLeaf(Node<T> node) {
		// return (node->obj != NULL);
		return node.isLeaf();
	}

	static <T> Node<T> NodeOther(SubTree<T> node, Node<T> child) {
		return (node.a == child ? node.b : node.a);
	}

	static <T> void NodeReplaceChild(SubTree<T> parent, Node<T> child, Node<T> value) {
		if (NodeIsLeaf(parent)) {
			throw new IllegalStateException("Internal Error: Cannot replace child of a leaf.");
		}
		if (!(child == parent.a || child == parent.b)) {
			throw new IllegalStateException("Internal Error: Node is not a child of parent.");
		}

		if (parent.a == child) {
			NodeRecycle(parent.a);
			NodeSetA(parent, value);
		} else {
			NodeRecycle(parent.b);
			NodeSetB(parent, value);
		}

		for (SubTree<T> node = parent; node != null; node = node.parent) {
			node.bb = BB.merge(node.a.bb, node.b.bb);
		}
	}

	//MARK: Subtree Functions

	static float cpBBProximity(BB a, BB b) {
		return cpfabs(a.l + a.r - b.l - b.r) + cpfabs(a.b + a.t - b.b - b.t);
	}

	static <T> Node<T> SubtreeInsert(Node<T> node, Leaf<T> leaf) {
		if (node == null) {
			return leaf;
		} else if (NodeIsLeaf(node)) {
			return NodeNew(leaf, node);
		} else {
			SubTree<T> subtree = (SubTree<T>) node;
			float cost_a = cpBBArea(subtree.b.bb) + cpBBMergedArea(subtree.a.bb, leaf.bb);
			float cost_b = cpBBArea(subtree.a.bb) + cpBBMergedArea(subtree.b.bb, leaf.bb);

			if (cost_a == cost_b) {
				cost_a = cpBBProximity(subtree.a.bb, leaf.bb);
				cost_b = cpBBProximity(subtree.b.bb, leaf.bb);
			}

			if (cost_b < cost_a) {
				final Node<T> tNode = SubtreeInsert(subtree.b, leaf);
				NodeSetB(subtree, tNode);
			} else {
				final Node<T> tNode = SubtreeInsert(subtree.a, leaf);
				NodeSetA(subtree, tNode);
			}

			subtree.bb = BB.merge(subtree.bb, leaf.bb);
			return subtree;
		}
	}

	static <T> void SubtreeQuery(Node<T> node, T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		if (node.bb.intersects(bb)) {
			if (NodeIsLeaf(node)) {
				func.apply(obj, ((Leaf<T>) node).obj, new CollisionID(0));
			} else {
				SubTree<T> subtree = (SubTree<T>) node;
				SubtreeQuery(subtree.a, obj, bb, func);
				SubtreeQuery(subtree.b, obj, bb, func);
			}
		}
	}

	static <T> float SubtreeSegmentQuery(Node<T> node, T obj, Vector2f a, Vector2f b, float t_exit,
			SpatialIndexSegmentQueryFunc<T> func) {
		if (NodeIsLeaf(node)) {
			return func.apply(obj, ((Leaf<T>) node).obj);
		} else {
			SubTree<T> subtree = (SubTree<T>) node;
			float t_a = BB.segmentQuery(subtree.a.bb, a, b);
			float t_b = BB.segmentQuery(subtree.b.bb, a, b);

			if (t_a < t_b) {
				if (t_a < t_exit)
					t_exit = cpfmin(t_exit, SubtreeSegmentQuery(subtree.a, obj, a, b, t_exit, func));
				if (t_b < t_exit)
					t_exit = cpfmin(t_exit, SubtreeSegmentQuery(subtree.b, obj, a, b, t_exit, func));
			} else {
				if (t_b < t_exit)
					t_exit = cpfmin(t_exit, SubtreeSegmentQuery(subtree.b, obj, a, b, t_exit, func));
				if (t_a < t_exit)
					t_exit = cpfmin(t_exit, SubtreeSegmentQuery(subtree.a, obj, a, b, t_exit, func));
			}

			return t_exit;
		}
	}

	static <T> void SubtreeRecycle(Node<T> node) {
		if (!NodeIsLeaf(node)) {
			SubTree<T> subtree = (SubTree<T>) node;
			SubtreeRecycle(subtree.a);
			SubtreeRecycle(subtree.b);
			NodeRecycle(node);
		}
	}

	static <T> Node<T> SubtreeRemove(Node<T> node, Leaf<T> leaf) {
		if (leaf == node) {
			return null;
		} else {
			SubTree<T> subtree = (SubTree<T>) node;
			Node<T> parent = leaf.parent;
			if (parent == subtree) {
				Node<T> other = NodeOther(subtree, leaf);
				other.parent = subtree.parent;
				NodeRecycle(subtree);
				return other;
			} else {
				NodeReplaceChild(parent.parent, parent, NodeOther((SubTree<T>) parent, leaf));
				return subtree;
			}
		}
	}

	//MARK: Marking Functions

	static class MarkContext<T> {
		BBTree2<T> tree;
		Node<T> staticRoot;
		SpatialIndexQueryFunc<T> func;

		MarkContext(BBTree2<T> tree, Node<T> staticRoot, SpatialIndexQueryFunc<T> func) {
			this.tree = tree;
			this.staticRoot = staticRoot;
			this.func = func;
		}
	}

	static <T> void MarkLeafQuery(Node<T> node, Leaf<T> leaf, boolean left, MarkContext<T> context) {
		if (leaf.bb.intersects(node.bb)) {
			if (NodeIsLeaf(node)) {
				if (left) {
					PairInsert(leaf, (Leaf<T>) node);
				} else {
					if (((Leaf<T>) node).stamp < leaf.stamp)
						PairInsert((Leaf<T>) node, leaf);
					context.func.apply(leaf.obj, ((Leaf<T>) node).obj, new CollisionID(0));
				}
			} else {
				SubTree<T> subtree = (SubTree<T>) node;
				MarkLeafQuery(subtree.a, leaf, left, context);
				MarkLeafQuery(subtree.b, leaf, left, context);
			}
		}
	}

	static <T> void MarkLeaf(Leaf<T> leaf, MarkContext<T> context) {
		BBTree2<T> tree = context.tree;
		if (leaf.stamp == GetMasterTree(tree).stamp) {
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

	static <T> void MarkSubtree(Node<T> node, MarkContext<T> context) {
		if (NodeIsLeaf(node)) {
			MarkLeaf((Leaf<T>) node, context);
		} else {
			SubTree<T> subtree = (SubTree<T>) node;
			MarkSubtree(subtree.a, context);
			MarkSubtree(subtree.b, context); // TODO: Force TCO here?
		}
	}

	//MARK: Leaf Functions

	static <T> Leaf<T> LeafNew(BBTree2<T> tree, T obj) {
		//Node<T> node = NodeFromPool(tree);
		Leaf<T> node = new Leaf<>();
		node.obj = obj;
		node.bb = GetBB(tree, obj);

		node.parent = null;
		node.stamp = 0;
		node.pairs = null;

		return node;
	}

	static <T> boolean LeafUpdate(Leaf<T> leaf, BBTree2<T> tree) {
		Node<T> root = tree.root;
		BB bb = tree.bbfunc.apply(leaf.obj);

		if (!leaf.bb.contains(bb)) {
			leaf.bb = GetBB(tree, leaf.obj);

			root = SubtreeRemove(root, leaf);
			tree.root = SubtreeInsert(root, leaf);

			PairsClear(leaf, tree);
			leaf.stamp = GetMasterTree(tree).stamp;

			return true;
		} else {
			return false;
		}
	}

	@SuppressWarnings("unused")
	static <T> CollisionID VoidQueryFunc(T obj1, T obj2, CollisionID id) {
		return id;
	}

	static <T> void LeafAddPairs(Leaf<T> leaf, BBTree2<T> tree) {
		SpatialIndex<T> dynamicIndex = tree.dynamicIndex;
		if (dynamicIndex != null) {
			Node<T> dynamicRoot = GetRootIfTree(dynamicIndex);
			if (dynamicRoot != null) {
				BBTree2<T> dynamicTree = GetTree(dynamicIndex);
				MarkContext<T> context = new MarkContext<>(dynamicTree, null, null);
				MarkLeafQuery(dynamicRoot, leaf, true, context);
			}
		} else {
			Node<T> staticRoot = GetRootIfTree(tree.staticIndex);
			MarkContext<T> context = new MarkContext<>(tree, staticRoot, BBTree2::VoidQueryFunc);
			MarkLeaf(leaf, context);
		}
	}

	//MARK: Memory Management Functions
	/*
	BBTree2<T> *
	cpBBTreeAlloc(void)
	{
		return (BBTree2<T> *)cpcalloc(1, sizeof(BBTree2<T>));
	}

	static int
	leafSetEql(T obj, Node<T> node)
	{
		return (obj == node->obj);
	}

	static void *
	leafSetTrans(T obj, BBTree2<T> tree)
	{
		return LeafNew(tree, obj, tree->spatialIndex.bbfunc(obj));
	}

	cpSpatialIndex *
	cpBBTreeInit(BBTree2<T> tree, cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
	{
		cpSpatialIndexInit((cpSpatialIndex *)tree, Klass(), bbfunc, staticIndex);

		tree->velocityFunc = NULL;

		tree->leaves = cpHashSetNew(0, (cpHashSetEqlFunc)leafSetEql);
		tree->root = NULL;

		tree->pooledNodes = NULL;
		tree->allocatedBuffers = cpArrayNew(0);

		tree.stamp = 0;

		return (cpSpatialIndex *)tree;
	}
	*/
	static <T> void cpBBTreeSetVelocityFunc(SpatialIndex<T> index, BBTreeVelocityFunc<T> func) {
		if (!(index instanceof BBTree2)) {
			throw new IllegalStateException("Ignoring cpBBTreeSetVelocityFunc() call to non-tree spatial index.");
		}

		((BBTree2<T>) index).velocityFunc = func;
	}

	/*cpSpatialIndex *
	cpBBTreeNew(cpSpatialIndexBBFunc bbfunc, cpSpatialIndex *staticIndex)
	{
		return cpBBTreeInit(cpBBTreeAlloc(), bbfunc, staticIndex);
	}

	static void
	cpBBTreeDestroy(BBTree2<T> tree)
	{
		cpHashSetFree(tree->leaves);

		if(tree->allocatedBuffers) cpArrayFreeEach(tree->allocatedBuffers, cpfree);
		cpArrayFree(tree->allocatedBuffers);
	} */

	//MARK: Insert/Remove

	static <T> void cpBBTreeInsert(BBTree2<T> tree, T obj, int hashid) {
		// Leaf<T> leaf = (Node<T> )cpHashSetInsert(tree->leaves, hashid, obj, (cpHashSetTransFunc)leafSetTrans, tree);
		Leaf<T> leaf = tree.leaves.get(hashid);
		if (leaf == null) {
			leaf = LeafNew(tree, obj);
			tree.leaves.put(hashid, leaf);
		} else {
			leaf.obj = obj;
		}

		Node<T> root = tree.root;
		tree.root = SubtreeInsert(root, leaf);

		leaf.stamp = GetMasterTree(tree).stamp;
		LeafAddPairs(leaf, tree);
		IncrementStamp(tree);
	}

	@SuppressWarnings("unused")
	static <T> void cpBBTreeRemove(BBTree2<T> tree, T obj, int hashid) {
		// Node<T> leaf = (Node<T> )cpHashSetRemove(tree->leaves, hashid, obj);
		Leaf<T> leaf = tree.leaves.remove(hashid);

		tree.root = SubtreeRemove(tree.root, leaf);
		PairsClear(leaf, tree);
		NodeRecycle(leaf);
	}

	@SuppressWarnings("unused")
	static <T> boolean cpBBTreeContains(BBTree2<T> tree, T obj, int hashid) {
		//return (cpHashSetFind(tree->leaves, hashid, obj) != NULL);
		return tree.leaves.containsKey(hashid);
	}

	//MARK: Reindex

	static <T> void LeafUpdateWrap(Leaf<T> leaf, BBTree2<T> tree) {
		LeafUpdate(leaf, tree);
	}

	static <T> void cpBBTreeReindexQuery(BBTree2<T> tree, SpatialIndexQueryFunc<T> func) {
		if (tree.root == null)
			return;

		// LeafUpdate() may modify tree->root. Don't cache it.
		// cpHashSetEach(tree->leaves, (cpHashSetIteratorFunc)LeafUpdateWrap, tree);
		for (Leaf<T> leaf : tree.leaves.values()) {
			LeafUpdateWrap(leaf, tree);
		}

		SpatialIndex<T> staticIndex = tree.staticIndex;
		Node<T> staticRoot =
				(staticIndex != null && (staticIndex instanceof BBTree2) ? ((BBTree2<T>) staticIndex).root : null);

		MarkContext<T> context = new MarkContext<>(tree, staticRoot, func);
		MarkSubtree(tree.root, context);
		if (staticIndex != null && staticRoot == null)
			cpSpatialIndexCollideStatic(tree, staticIndex, func);

		IncrementStamp(tree);
	}

	static <T> void cpBBTreeReindex(BBTree2<T> tree) {
		cpBBTreeReindexQuery(tree, BBTree2::VoidQueryFunc);
	}

	@SuppressWarnings("unused")
	static <T> void cpBBTreeReindexObject(BBTree2<T> tree, T obj, int hashid) {
		//Leaf<T> leaf = (Node<T> )cpHashSetFind(tree->leaves, hashid, obj);
		Leaf<T> leaf = tree.leaves.get(hashid);
		if (leaf != null) {
			if (LeafUpdate(leaf, tree))
				LeafAddPairs(leaf, tree);
			IncrementStamp(tree);
		}
	}

	//MARK: Query

	static <T> void cpBBTreeSegmentQuery(BBTree2<T> tree, T obj, Vector2f a, Vector2f b, float t_exit,
			SpatialIndexSegmentQueryFunc<T> func) {
		Node<T> root = tree.root;
		if (root != null)
			SubtreeSegmentQuery(root, obj, a, b, t_exit, func);
	}

	static <T> void cpBBTreeQuery(BBTree2<T> tree, T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		if (tree.root != null)
			SubtreeQuery(tree.root, obj, bb, func);
	}

	//MARK: Misc

	static <T> int cpBBTreeCount(BBTree2<T> tree) {
		// return cpHashSetCount(tree->leaves);
		return tree.leaves.size();
	}

	/*static class EachContext {
		SpatialIndexIteratorFunc<T> func;
	}

	static void each_helper(Node<T> node, eachContext *context){context->func(node->obj, context->data);}*/

	static <T> void cpBBTreeEach(BBTree2<T> tree, SpatialIndexIteratorFunc<T> func) {
		//eachContext context = {func, data};
		//cpHashSetEach(tree->leaves, (cpHashSetIteratorFunc)each_helper, &context);
		for (Leaf<T> leaf : tree.leaves.values()) {
			func.visit(leaf.obj);
		}
	}

	/*static cpSpatialIndexClass klass = {
			(cpSpatialIndexDestroyImpl)cpBBTreeDestroy,

			(cpSpatialIndexCountImpl)cpBBTreeCount,
			(cpSpatialIndexEachImpl)cpBBTreeEach,

			(cpSpatialIndexContainsImpl)cpBBTreeContains,
			(cpSpatialIndexInsertImpl)cpBBTreeInsert,
			(cpSpatialIndexRemoveImpl)cpBBTreeRemove,

			(cpSpatialIndexReindexImpl)cpBBTreeReindex,
			(cpSpatialIndexReindexObjectImpl)cpBBTreeReindexObject,
			(cpSpatialIndexReindexQueryImpl)cpBBTreeReindexQuery,

			(cpSpatialIndexQueryImpl)cpBBTreeQuery,
			(cpSpatialIndexSegmentQueryImpl)cpBBTreeSegmentQuery,
	};
	static inline cpSpatialIndexClass *Klass(){return &klass;}*/

	//MARK: Tree Optimization

	/*static int
	cpfcompare(const float *a, const float *b){
		return (*a < *b ? -1 : (*b < *a ? 1 : 0));
	}*/

	static <T> int fillNodeArray(Node<T> node, Node<T>[] nodes, int cursor) {
		/* (**cursor) = node;
		(*cursor)++; */
		nodes[cursor++] = node;
		return cursor;
	}

	@SuppressWarnings("unchecked")
	static <T> Node<T> partitionNodes(Node[] nodes, int index, int count) {
		if (count == 1) {
			return nodes[index];
		} else if (count == 2) {
			return NodeNew(nodes[index], nodes[index + 1]);
		}

		// Find the AABB for these nodes
		BB bb = nodes[index].bb;
		for (int i = 1; i < count; i++)
			bb = BB.merge(bb, nodes[index + i].bb);

		// Split it on it's longest axis
		boolean splitWidth = (bb.r - bb.l > bb.t - bb.b);

		// Sort the bounds and use the median as the splitting point
		float[] bounds = new float[count * 2]; // (float *)cpcalloc(count*2, sizeof(float));
		if (splitWidth) {
			for (int i = 0; i < count; i++) {
				bounds[(2 * i)] = nodes[index + i].bb.l;
				bounds[2 * i + 1] = nodes[index + i].bb.r;
			}
		} else {
			for (int i = 0; i < count; i++) {
				bounds[(2 * i)] = nodes[index + i].bb.b;
				bounds[2 * i + 1] = nodes[index + i].bb.t;
			}
		}

		// qsort(bounds, count*2, sizeof(float), (int (*)(const void *, const void *))cpfcompare);
		Arrays.sort(bounds);
		float split = (bounds[count - 1] + bounds[count]) * 0.5f; // use the medain as the split
		// cpfree(bounds);

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
				//		if(cpBBProximity(node.bb, b) < cpBBProximity(node.bb, a)){
				right--;
				nodes[index + left] = nodes[index + right];
				nodes[index + right] = node;
			} else {
				left++;
			}
		}

		if (right == count) {
			Node<T> node = null;
			for (int i = 0; i < count; i++)
				node = SubtreeInsert(node, (Leaf<T>) nodes[index + i]);
			return node;
		}

		// Recurse and build the node!
		final Node<T> nodeA = partitionNodes(nodes, index, right);
		final Node<T> nodeB = partitionNodes(nodes, index + right, count - right);
		return NodeNew(nodeA, nodeB);
	}

	//static void
	//cpBBTreeOptimizeIncremental(BBTree2<T> tree, int passes)
	//{
	//	for(int i=0; i<passes; i++){
	//		Node<T> root = tree->root;
	//		Node<T> node = root;
	//		int bit = 0;
	//		unsigned int path = tree->opath;
	//		
	//		while(!NodeIsLeaf(node)){
	//			node = (path&(1<<bit) ? node.a : node.b);
	//			bit = (bit + 1)&(sizeof(unsigned int)*8 - 1);
	//		}
	//		
	//		root = subtreeRemove(root, node, tree);
	//		tree->root = subtreeInsert(root, node, tree);
	//	}
	//}

	@SuppressWarnings("unused")
	<T> void cpBBTreeOptimize(SpatialIndex<T> index) {
		if (!(index instanceof BBTree2)) {
			throw new IllegalStateException("Ignoring cpBBTreeOptimize() call to non-tree spatial index.");
		}

		BBTree2<T> tree = (BBTree2<T>) index;
		Node<T> root = tree.root;
		if (root == null)
			return;

		int count = cpBBTreeCount(tree);
		Node<T>[] nodes = new Node[count]; // (Node<T> *)cpcalloc(count, sizeof(Node<T> ));
		//Node<T> cursor = nodes;
		int cursor = 0;

		// cpHashSetEach(tree->leaves, (cpHashSetIteratorFunc)fillNodeArray, &cursor);
		for (Leaf<T> leaf : tree.leaves.values()) {
			cursor = fillNodeArray(leaf, nodes, cursor);
		}

		SubtreeRecycle(root);
		tree.root = partitionNodes(nodes, 0, count);
		// cpfree(nodes);
	}

	public BBTree2(SpatialIndexBBFunc<T> bbfunc, SpatialIndex<T> staticIndex) {
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
	public void query(T obj, BB bb, SpatialIndexQueryFunc<T> func) {
		cpBBTreeQuery(this, obj, bb, func);
	}

	@Override
	public void segmentQuery(T obj, Vector2f a, Vector2f b, float exit, SpatialIndexSegmentQueryFunc<T> func) {
		cpBBTreeSegmentQuery(this, obj, a, b, exit, func);
	}
}
