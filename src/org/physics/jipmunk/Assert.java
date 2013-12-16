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

import java.util.logging.Logger;

/** @author jobernolte */
public class Assert {
	final static Logger logger = Logger.getLogger(Assert.class.getName());

	public static void cpAssertSoft(boolean condition, String message) {
		if (!condition) {
			logger.info("failed assertion: " + message);
		}
	}

	public static void cpAssertWarn(boolean condition, String message) {
		if (!condition) {
			logger.info("failed assertion: " + message);
			throw new AssertionError(message);
		}
	}

	public static void cpAssertHard(boolean condition, String message) {
		if (!condition) {
			logger.info("failed assertion: " + message);
			throw new AssertionError(message);
		}
	}

	public static void cpAssertSpaceUnlocked(Space space) {
		if (space.locked != 0) {
			throw new AssertionError("space is not unlocked");
		}
	}

	public static void cpAssertSaneBody(Body body) {
		body.sanityCheck();
	}
}
