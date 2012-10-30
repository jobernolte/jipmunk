package org.physics.jipmunk;

/** @author jobernolte */
public class ConvexHullUtil {

    public static class ConvexHullInfo {
        public final int first;
        public final int count;

        public ConvexHullInfo(int first, int count) {
            this.first = first;
            this.count = count;
        }

        public int getFirst() {
            return first;
        }

        public int getCount() {
            return count;
        }
    }

    static class StartEnd {
        int start;
        int end;
    }

    static void cpLoopIndexes(Vector2f[] verts, int count, StartEnd startEnd) {
        startEnd.start = startEnd.end = 0;
        Vector2f min = verts[0];
        Vector2f max = min;

        for (int i = 1; i < count; i++) {
            Vector2f v = verts[i];

            if (v.getX() < min.getX() || (v.getX() == min.getX() && v.getY() < min.getY())) {
                min = v;
                startEnd.start = i;
            } else if (v.getX() > max.getX() || (v.getX() == max.getX() && v.getY() > max.getY())) {
                max = v;
                startEnd.end = i;
            }
        }
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
            float value = Util.cpvcross(delta, Util.cpvsub(verts[head], a));
            if (value > valueTol) {
                if (value > max) {
                    max = value;
                    pivot = head;
                }

                head++;
            } else {
                //SWAP(verts[head], verts[tail]);
                Vector2f tmp = verts[head];
                verts[head] = verts[tail];
                verts[tail] = tmp;
                tail--;
            }
        }

        // move the new pivot to the front if it's not already there.
        if (pivot != 0) {
            // SWAP(verts[0], verts[pivot]);
            Vector2f tmp = verts[0];
            verts[0] = verts[pivot];
            verts[pivot] = tmp;
        }
        return head;
    }

    static int QHullReduce(float tol, Vector2f[] verts, int offset, int count, Vector2f a, Vector2f pivot, Vector2f b, Vector2f[] result, int resultOffset) {
        if (count < 0) {
            return 0;
        } else if (count == 0) {
            result[resultOffset] = pivot;
            return 1;
        } else {
            int left_count = QHullPartition(verts, 0, count, a, pivot, tol);
            int index = QHullReduce(tol, verts, offset + 1, left_count - 1, a, verts[0], pivot, result, 0);

            result[index++] = pivot;

            int right_count = QHullPartition(verts, offset + left_count, count - left_count, pivot, b, tol);
            return index + QHullReduce(tol, verts, offset + left_count + 1, right_count - 1, pivot, verts[left_count], b, result, resultOffset + index);
        }
    }

    // QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
// My implementation performs an in place reduction using the result array as scratch space.
    public static ConvexHullInfo cpConvexHull(int count, Vector2f[] verts, Vector2f[] result, float tol) {
        if (result != null) {
            // Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
            //memcpy(result, verts, count * sizeof(Vector2f));
            System.arraycopy(verts, 0, result, 0, count);
        } else {
            // If a result array was not specified, reduce the input instead.
            result = verts;
        }

        // Degenerate case, all poins are the same.
        StartEnd startEnd = new StartEnd();
        cpLoopIndexes(verts, count, startEnd);
        if (startEnd.start == startEnd.end) {
            return new ConvexHullInfo(0, 1);
        }

        {
            // SWAP(result[0], result[start]);
            Vector2f tmp = result[0];
            result[0] = result[startEnd.start];
            result[startEnd.start] = tmp;
        }
        {
            // SWAP(result[1], result[end == 0 ? start : end]);
            Vector2f tmp = result[1];
            int i = startEnd.end == 0 ? startEnd.start : startEnd.end;
            result[1] = result[i];
            result[i] = tmp;
        }

        Vector2f a = result[0];
        Vector2f b = result[1];

        int resultCount = QHullReduce(tol, result, 2, count - 2, a, b, a, result, 1) + 1;
        /*Assert.cpAssertSoft(cpPolyValidate(result, resultCount),
                "Internal error: cpConvexHull() and cpPolyValidate() did not agree."
                "Please report this error with as much info as you can.");*/
        return new ConvexHullInfo(startEnd.start, resultCount);
    }
}
