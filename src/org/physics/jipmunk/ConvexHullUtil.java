package org.physics.jipmunk;

/** @author jobernolte */
public class ConvexHullUtil {

	static class StartEnd {
		int start;
		int end;
	}

	static void loopIndices(Vector2f[] verts, int count, StartEnd startEnd) {
		startEnd.start = startEnd.end = 0;
		Vector2f min = verts[0];
		Vector2f max = min;

		for (int i = 1; i < count; i++) {
			Vector2f v = verts[i];

			if (v.x < min.x || (v.x == min.x && v.y < min.y)) {
				min = v;
				startEnd.start = i;
			} else if (v.x > max.x || (v.x == max.x && v.y > max.y)) {
				max = v;
				startEnd.end = i;
			}
		}
	}

	static void swap(Vector2f[] verts, int a, int b) {
		Vector2f tmp = verts[a];
		verts[a] = verts[b];
		verts[b] = tmp;
	}

	static int QHullPartition(Vector2f[] verts, int offset, int count, Vector2f a, Vector2f b, float tol) {
		if (count == 0) {
			return 0;
		}

		float max = 0;
		int pivot = 0;

		Vector2f delta = Util.cpvsub(b, a);
		float valueTol = tol * Util.cpvlength(delta);

		int head = 0;
		for (int tail = count - 1; head <= tail; ) {
			float value = Util.cpvcross(Util.cpvsub(verts[offset + head], a), delta);
			if (value > valueTol) {
				if (value > max) {
					max = value;
					pivot = head;
				}

				head++;
			} else {
				swap(verts, offset + head, offset + tail);
				tail--;
			}
		}

		// move the new pivot to the front if it's not already there.
		if (pivot != 0) {
			swap(verts, offset, offset + pivot);
		}
		return head;
	}

	static int QHullReduce(float tol, Vector2f[] verts, int offset, int count, Vector2f a, Vector2f pivot, Vector2f b,
			Vector2f[] result, int resultOffset) {
		if (count < 0) {
			return 0;
		} else if (count == 0) {
			result[resultOffset] = pivot;
			return 1;
		} else {
			int left_count = QHullPartition(verts, offset, count, a, pivot, tol);
			int index = QHullReduce(tol, verts, offset + 1, left_count - 1, a, Util.cpv(verts[offset]), pivot, result,
									resultOffset);

			result[resultOffset + index++] = pivot;
			//result[resultOffset + index++].set(pivot);

			int right_count = QHullPartition(verts, offset + left_count, count - left_count, pivot, b, tol);
			/*System.out.format("QHullReduce: offset: %distance, left_count: %distance, right_count: %distance\normal", offset, left_count,
					right_count);*/
			if (right_count == 0) {
				return index;
			}
			return index + QHullReduce(tol, verts, offset + left_count + 1, right_count - 1, pivot,
									   Util.cpv(verts[offset + left_count]), b, result, resultOffset + index);
		}
	}

	/**
	 * QuickHull seemed like a neat algorithm, and efficient-ish for large input sets. My implementation performs an in
	 * place reduction using the result array as scratch space.
	 */
	public static ConvexHullInfo convexHull(Vector2f[] verts, Vector2f[] result, int count, float tol) {
		if (result != null && result != verts) {
			// Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
			System.arraycopy(verts, 0, result, 0, count);
		} else {
			// If a result array was not specified, reduce the input instead.
			result = verts;
		}

		// Degenerate case, all poins are the same.
		StartEnd startEnd = new StartEnd();
		loopIndices(verts, count, startEnd);
		if (startEnd.start == startEnd.end) {
			return new ConvexHullInfo(0, 1);
		}

		swap(result, 0, startEnd.start);
		swap(result, 1, startEnd.end == 0 ? startEnd.start : startEnd.end);

		Vector2f a = Util.cpv(result[0]);
		Vector2f b = Util.cpv(result[1]);

		int resultCount = QHullReduce(tol, result, 2, count - 2, a, b, a, result, 1) + 1;
		return new ConvexHullInfo(startEnd.start, resultCount);
	}
}
