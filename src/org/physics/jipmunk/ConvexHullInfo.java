package org.physics.jipmunk;

/** @author jobernolte */
public class ConvexHullInfo {
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
