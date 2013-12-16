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

import java.util.Iterator;
import java.util.Map;

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

	public static <T> void cpHashSetFilter(IntHashMap<T> set, HashSetFilterFunc<T> func) {
		Iterator<IntHashMap.Entry<T>> it = set.entrySet().iterator();
		while (it.hasNext()) {
			IntHashMap.Entry<T> entry = it.next();
			if (!func.filter(entry.getValue())) {
				it.remove();
			}
		}
	}

	public static <K, V> void cpHashSetFilter(Map<K, V> set, HashSetFilterFunc<V> func) {
		Iterator<Map.Entry<K, V>> it = set.entrySet().iterator();
		while (it.hasNext()) {
			Map.Entry<K, V> entry = it.next();
			if (!func.filter(entry.getValue())) {
				it.remove();
			}
		}
	}


}
