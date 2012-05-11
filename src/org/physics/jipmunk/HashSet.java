package org.physics.jipmunk;

import java.util.Iterator;

/** @author jobernolte */
class HashSet {
	public static <T> void cpHashSetFilter(LongHashMap<T> set, HashSetFilterFunc<T> func) {
		Iterator<LongHashMap.Entry<T>> it = set.entrySet().iterator();
		while (it.hasNext()) {
			LongHashMap.Entry<T> entry = it.next();
			if (!func.filter(entry.getValue())) {
				it.remove();
			}
		}
	}

	//public static <T> void cpHashSetInsert(LongHashMap<T> set, int hash, )
}
