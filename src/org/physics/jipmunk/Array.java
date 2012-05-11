package org.physics.jipmunk;

import java.util.List;

/** @author jobernolte */
class Array {
	static <T> void cpArrayDeleteObj(List<T> array, T obj) {
		array.remove(obj);
	}

	public static <T> void cpArrayPush(List<T> array, T obj) {
		array.add(obj);
	}

	public static <T> void cpArrayFree(List<T> array) {
		array.clear();
	}
}
