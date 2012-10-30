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

package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;

import java.util.Random;

/**
 * @author chris_c based on work by jobernolte & Lembcke
 */
public class BouncyHexagons extends ExampleBase {

    // BouncyTerrain
    static Vector2f bouncy_terrain_verts[] = {
            Util.cpv(537.18f, 23.00f), Util.cpv(520.50f, 36.00f), Util.cpv(501.53f, 63.00f), Util.cpv(496.14f, 76.00f), Util.cpv(498.86f, 86.00f), Util.cpv(504.00f, 90.51f), Util.cpv(508.00f, 91.36f), Util.cpv(508.77f, 84.00f), Util.cpv(513.00f, 77.73f), Util.cpv(519.00f, 74.48f), Util.cpv(530.00f, 74.67f), Util.cpv(545.00f, 54.65f),
            Util.cpv(554.00f, 48.77f), Util.cpv(562.00f, 46.39f), Util.cpv(568.00f, 45.94f), Util.cpv(568.61f, 47.00f), Util.cpv(567.94f, 55.00f), Util.cpv(571.27f, 64.00f), Util.cpv(572.92f, 80.00f), Util.cpv(572.00f, 81.39f), Util.cpv(563.00f, 79.93f), Util.cpv(556.00f, 82.69f), Util.cpv(551.49f, 88.00f), Util.cpv(549.00f, 95.76f),
            Util.cpv(538.00f, 93.40f), Util.cpv(530.00f, 102.38f), Util.cpv(523.00f, 104.00f), Util.cpv(517.00f, 103.02f), Util.cpv(516.22f, 109.00f), Util.cpv(518.96f, 116.00f), Util.cpv(526.00f, 121.15f), Util.cpv(534.00f, 116.48f), Util.cpv(543.00f, 116.77f), Util.cpv(549.28f, 121.00f), Util.cpv(554.00f, 130.17f), Util.cpv(564.00f, 125.67f),
            Util.cpv(575.60f, 129.00f), Util.cpv(573.31f, 121.00f), Util.cpv(567.77f, 111.00f), Util.cpv(575.00f, 106.47f), Util.cpv(578.51f, 102.00f), Util.cpv(580.25f, 95.00f), Util.cpv(577.98f, 87.00f), Util.cpv(582.00f, 85.71f), Util.cpv(597.00f, 89.46f), Util.cpv(604.80f, 95.00f), Util.cpv(609.28f, 104.00f), Util.cpv(610.55f, 116.00f),
            Util.cpv(609.30f, 125.00f), Util.cpv(600.80f, 142.00f), Util.cpv(597.31f, 155.00f), Util.cpv(584.00f, 167.23f), Util.cpv(577.86f, 175.00f), Util.cpv(583.52f, 184.00f), Util.cpv(582.64f, 195.00f), Util.cpv(591.00f, 196.56f), Util.cpv(597.81f, 201.00f), Util.cpv(607.45f, 219.00f), Util.cpv(607.51f, 246.00f), Util.cpv(600.00f, 275.46f),
            Util.cpv(588.00f, 267.81f), Util.cpv(579.00f, 264.91f), Util.cpv(557.00f, 264.41f), Util.cpv(552.98f, 259.00f), Util.cpv(548.00f, 246.18f), Util.cpv(558.00f, 247.12f), Util.cpv(565.98f, 244.00f), Util.cpv(571.10f, 237.00f), Util.cpv(571.61f, 229.00f), Util.cpv(568.25f, 222.00f), Util.cpv(562.00f, 217.67f), Util.cpv(544.00f, 213.93f),
            Util.cpv(536.73f, 214.00f), Util.cpv(535.60f, 204.00f), Util.cpv(539.69f, 181.00f), Util.cpv(542.84f, 171.00f), Util.cpv(550.43f, 161.00f), Util.cpv(540.00f, 156.27f), Util.cpv(536.62f, 152.00f), Util.cpv(534.70f, 146.00f), Util.cpv(527.00f, 141.88f), Util.cpv(518.59f, 152.00f), Util.cpv(514.51f, 160.00f), Util.cpv(510.33f, 175.00f),
            Util.cpv(519.38f, 183.00f), Util.cpv(520.52f, 194.00f), Util.cpv(516.00f, 201.27f), Util.cpv(505.25f, 206.00f), Util.cpv(507.57f, 223.00f), Util.cpv(519.90f, 260.00f), Util.cpv(529.00f, 260.48f), Util.cpv(534.00f, 262.94f), Util.cpv(538.38f, 268.00f), Util.cpv(540.00f, 275.00f), Util.cpv(537.06f, 284.00f), Util.cpv(530.00f, 289.23f),
            Util.cpv(520.00f, 289.23f), Util.cpv(513.00f, 284.18f), Util.cpv(509.71f, 286.00f), Util.cpv(501.69f, 298.00f), Util.cpv(501.56f, 305.00f), Util.cpv(504.30f, 311.00f), Util.cpv(512.00f, 316.43f), Util.cpv(521.00f, 316.42f), Util.cpv(525.67f, 314.00f), Util.cpv(535.00f, 304.98f), Util.cpv(562.00f, 294.80f), Util.cpv(573.00f, 294.81f),
            Util.cpv(587.52f, 304.00f), Util.cpv(600.89f, 310.00f), Util.cpv(596.96f, 322.00f), Util.cpv(603.28f, 327.00f), Util.cpv(606.52f, 333.00f), Util.cpv(605.38f, 344.00f), Util.cpv(597.65f, 352.00f), Util.cpv(606.36f, 375.00f), Util.cpv(607.16f, 384.00f), Util.cpv(603.40f, 393.00f), Util.cpv(597.00f, 398.14f), Util.cpv(577.00f, 386.15f),
            Util.cpv(564.35f, 373.00f), Util.cpv(565.21f, 364.00f), Util.cpv(562.81f, 350.00f), Util.cpv(553.00f, 346.06f), Util.cpv(547.48f, 338.00f), Util.cpv(547.48f, 330.00f), Util.cpv(550.00f, 323.30f), Util.cpv(544.00f, 321.53f), Util.cpv(537.00f, 322.70f), Util.cpv(532.00f, 326.23f), Util.cpv(528.89f, 331.00f), Util.cpv(527.83f, 338.00f),
            Util.cpv(533.02f, 356.00f), Util.cpv(542.00f, 360.73f), Util.cpv(546.68f, 369.00f), Util.cpv(545.38f, 379.00f), Util.cpv(537.58f, 386.00f), Util.cpv(537.63f, 388.00f), Util.cpv(555.00f, 407.47f), Util.cpv(563.00f, 413.52f), Util.cpv(572.57f, 418.00f), Util.cpv(582.72f, 426.00f), Util.cpv(578.00f, 431.12f), Util.cpv(563.21f, 440.00f),
            Util.cpv(558.00f, 449.27f), Util.cpv(549.00f, 452.94f), Util.cpv(541.00f, 451.38f), Util.cpv(536.73f, 448.00f), Util.cpv(533.00f, 441.87f), Util.cpv(520.00f, 437.96f), Util.cpv(514.00f, 429.69f), Util.cpv(490.00f, 415.15f), Util.cpv(472.89f, 399.00f), Util.cpv(472.03f, 398.00f), Util.cpv(474.00f, 396.71f), Util.cpv(486.00f, 393.61f),
            Util.cpv(492.00f, 385.85f), Util.cpv(492.00f, 376.15f), Util.cpv(489.04f, 371.00f), Util.cpv(485.00f, 368.11f), Util.cpv(480.00f, 376.27f), Util.cpv(472.00f, 379.82f), Util.cpv(463.00f, 378.38f), Util.cpv(455.08f, 372.00f), Util.cpv(446.00f, 377.69f), Util.cpv(439.00f, 385.24f), Util.cpv(436.61f, 391.00f), Util.cpv(437.52f, 404.00f),
            Util.cpv(440.00f, 409.53f), Util.cpv(463.53f, 433.00f), Util.cpv(473.80f, 441.00f), Util.cpv(455.00f, 440.30f), Util.cpv(443.00f, 436.18f), Util.cpv(436.00f, 431.98f), Util.cpv(412.00f, 440.92f), Util.cpv(397.00f, 442.46f), Util.cpv(393.59f, 431.00f), Util.cpv(393.71f, 412.00f), Util.cpv(400.00f, 395.10f), Util.cpv(407.32f, 387.00f),
            Util.cpv(408.54f, 380.00f), Util.cpv(407.42f, 375.00f), Util.cpv(403.97f, 370.00f), Util.cpv(399.00f, 366.74f), Util.cpv(393.00f, 365.68f), Util.cpv(391.23f, 374.00f), Util.cpv(387.00f, 380.27f), Util.cpv(381.00f, 383.52f), Util.cpv(371.56f, 384.00f), Util.cpv(364.98f, 401.00f), Util.cpv(362.96f, 412.00f), Util.cpv(363.63f, 435.00f),
            Util.cpv(345.00f, 433.55f), Util.cpv(344.52f, 442.00f), Util.cpv(342.06f, 447.00f), Util.cpv(337.00f, 451.38f), Util.cpv(330.00f, 453.00f), Util.cpv(325.00f, 452.23f), Util.cpv(318.00f, 448.17f), Util.cpv(298.00f, 453.70f), Util.cpv(284.00f, 451.49f), Util.cpv(278.62f, 449.00f), Util.cpv(291.47f, 408.00f), Util.cpv(291.77f, 398.00f),
            Util.cpv(301.00f, 393.83f), Util.cpv(305.00f, 393.84f), Util.cpv(305.60f, 403.00f), Util.cpv(310.00f, 409.47f), Util.cpv(318.00f, 413.07f), Util.cpv(325.00f, 412.40f), Util.cpv(332.31f, 407.00f), Util.cpv(335.07f, 400.00f), Util.cpv(334.40f, 393.00f), Util.cpv(329.00f, 385.69f), Util.cpv(319.00f, 382.79f), Util.cpv(301.00f, 389.23f),
            Util.cpv(289.00f, 389.97f), Util.cpv(265.00f, 389.82f), Util.cpv(251.00f, 385.85f), Util.cpv(245.00f, 389.23f), Util.cpv(239.00f, 389.94f), Util.cpv(233.00f, 388.38f), Util.cpv(226.00f, 382.04f), Util.cpv(206.00f, 374.75f), Util.cpv(206.00f, 394.00f), Util.cpv(204.27f, 402.00f), Util.cpv(197.00f, 401.79f), Util.cpv(191.00f, 403.49f),
            Util.cpv(186.53f, 407.00f), Util.cpv(183.60f, 412.00f), Util.cpv(183.60f, 422.00f), Util.cpv(189.00f, 429.31f), Util.cpv(196.00f, 432.07f), Util.cpv(203.00f, 431.40f), Util.cpv(209.47f, 427.00f), Util.cpv(213.00f, 419.72f), Util.cpv(220.00f, 420.21f), Util.cpv(227.00f, 418.32f), Util.cpv(242.00f, 408.41f), Util.cpv(258.98f, 409.00f),
            Util.cpv(250.00f, 435.43f), Util.cpv(239.00f, 438.78f), Util.cpv(223.00f, 448.19f), Util.cpv(209.00f, 449.70f), Util.cpv(205.28f, 456.00f), Util.cpv(199.00f, 460.23f), Util.cpv(190.00f, 460.52f), Util.cpv(182.73f, 456.00f), Util.cpv(178.00f, 446.27f), Util.cpv(160.00f, 441.42f), Util.cpv(148.35f, 435.00f), Util.cpv(149.79f, 418.00f),
            Util.cpv(157.72f, 401.00f), Util.cpv(161.00f, 396.53f), Util.cpv(177.00f, 385.00f), Util.cpv(180.14f, 380.00f), Util.cpv(181.11f, 374.00f), Util.cpv(180.00f, 370.52f), Util.cpv(170.00f, 371.68f), Util.cpv(162.72f, 368.00f), Util.cpv(158.48f, 361.00f), Util.cpv(159.56f, 349.00f), Util.cpv(154.00f, 342.53f), Util.cpv(146.00f, 339.85f),
            Util.cpv(136.09f, 343.00f), Util.cpv(130.64f, 351.00f), Util.cpv(131.74f, 362.00f), Util.cpv(140.61f, 374.00f), Util.cpv(130.68f, 387.00f), Util.cpv(120.75f, 409.00f), Util.cpv(118.09f, 421.00f), Util.cpv(117.92f, 434.00f), Util.cpv(100.00f, 432.40f), Util.cpv(87.00f, 427.48f), Util.cpv(81.59f, 423.00f), Util.cpv(73.64f, 409.00f),
            Util.cpv(72.57f, 398.00f), Util.cpv(74.62f, 386.00f), Util.cpv(78.80f, 378.00f), Util.cpv(88.00f, 373.43f), Util.cpv(92.49f, 367.00f), Util.cpv(93.32f, 360.00f), Util.cpv(91.30f, 353.00f), Util.cpv(103.00f, 342.67f), Util.cpv(109.00f, 343.10f), Util.cpv(116.00f, 340.44f), Util.cpv(127.33f, 330.00f), Util.cpv(143.00f, 327.24f),
            Util.cpv(154.30f, 322.00f), Util.cpv(145.00f, 318.06f), Util.cpv(139.77f, 311.00f), Util.cpv(139.48f, 302.00f), Util.cpv(144.95f, 293.00f), Util.cpv(143.00f, 291.56f), Util.cpv(134.00f, 298.21f), Util.cpv(118.00f, 300.75f), Util.cpv(109.40f, 305.00f), Util.cpv(94.67f, 319.00f), Util.cpv(88.00f, 318.93f), Util.cpv(81.00f, 321.69f),
            Util.cpv(67.24f, 333.00f), Util.cpv(56.68f, 345.00f), Util.cpv(53.00f, 351.40f), Util.cpv(47.34f, 333.00f), Util.cpv(50.71f, 314.00f), Util.cpv(56.57f, 302.00f), Util.cpv(68.00f, 287.96f), Util.cpv(91.00f, 287.24f), Util.cpv(110.00f, 282.36f), Util.cpv(133.80f, 271.00f), Util.cpv(147.34f, 256.00f), Util.cpv(156.47f, 251.00f),
            Util.cpv(157.26f, 250.00f), Util.cpv(154.18f, 242.00f), Util.cpv(154.48f, 236.00f), Util.cpv(158.72f, 229.00f), Util.cpv(166.71f, 224.00f), Util.cpv(170.15f, 206.00f), Util.cpv(170.19f, 196.00f), Util.cpv(167.24f, 188.00f), Util.cpv(160.00f, 182.67f), Util.cpv(150.00f, 182.66f), Util.cpv(143.60f, 187.00f), Util.cpv(139.96f, 195.00f),
            Util.cpv(139.50f, 207.00f), Util.cpv(136.45f, 221.00f), Util.cpv(136.52f, 232.00f), Util.cpv(133.28f, 238.00f), Util.cpv(129.00f, 241.38f), Util.cpv(119.00f, 243.07f), Util.cpv(115.00f, 246.55f), Util.cpv(101.00f, 253.16f), Util.cpv(86.00f, 257.32f), Util.cpv(63.00f, 259.24f), Util.cpv(57.00f, 257.31f), Util.cpv(50.54f, 252.00f),
            Util.cpv(47.59f, 247.00f), Util.cpv(46.30f, 240.00f), Util.cpv(47.58f, 226.00f), Util.cpv(50.00f, 220.57f), Util.cpv(58.00f, 226.41f), Util.cpv(69.00f, 229.17f), Util.cpv(79.00f, 229.08f), Util.cpv(94.50f, 225.00f), Util.cpv(100.21f, 231.00f), Util.cpv(107.00f, 233.47f), Util.cpv(107.48f, 224.00f), Util.cpv(109.94f, 219.00f),
            Util.cpv(115.00f, 214.62f), Util.cpv(122.57f, 212.00f), Util.cpv(116.00f, 201.49f), Util.cpv(104.00f, 194.57f), Util.cpv(90.00f, 194.04f), Util.cpv(79.00f, 198.21f), Util.cpv(73.00f, 198.87f), Util.cpv(62.68f, 191.00f), Util.cpv(62.58f, 184.00f), Util.cpv(64.42f, 179.00f), Util.cpv(75.00f, 167.70f), Util.cpv(80.39f, 157.00f),
            Util.cpv(68.79f, 140.00f), Util.cpv(61.67f, 126.00f), Util.cpv(61.47f, 117.00f), Util.cpv(64.43f, 109.00f), Util.cpv(63.10f, 96.00f), Util.cpv(56.48f, 82.00f), Util.cpv(48.00f, 73.88f), Util.cpv(43.81f, 66.00f), Util.cpv(43.81f, 56.00f), Util.cpv(50.11f, 46.00f), Util.cpv(59.00f, 41.55f), Util.cpv(71.00f, 42.64f),
            Util.cpv(78.00f, 36.77f), Util.cpv(83.00f, 34.75f), Util.cpv(99.00f, 34.32f), Util.cpv(117.00f, 38.92f), Util.cpv(133.00f, 55.15f), Util.cpv(142.00f, 50.70f), Util.cpv(149.74f, 51.00f), Util.cpv(143.55f, 68.00f), Util.cpv(153.28f, 74.00f), Util.cpv(156.23f, 79.00f), Util.cpv(157.00f, 84.00f), Util.cpv(156.23f, 89.00f),
            Util.cpv(153.28f, 94.00f), Util.cpv(144.58f, 99.00f), Util.cpv(151.52f, 112.00f), Util.cpv(151.51f, 124.00f), Util.cpv(150.00f, 126.36f), Util.cpv(133.00f, 130.25f), Util.cpv(126.71f, 125.00f), Util.cpv(122.00f, 117.25f), Util.cpv(114.00f, 116.23f), Util.cpv(107.73f, 112.00f), Util.cpv(104.48f, 106.00f), Util.cpv(104.32f, 99.00f),
            Util.cpv(106.94f, 93.00f), Util.cpv(111.24f, 89.00f), Util.cpv(111.60f, 85.00f), Util.cpv(107.24f, 73.00f), Util.cpv(102.00f, 67.57f), Util.cpv(99.79f, 67.00f), Util.cpv(99.23f, 76.00f), Util.cpv(95.00f, 82.27f), Util.cpv(89.00f, 85.52f), Util.cpv(79.84f, 86.00f), Util.cpv(86.73f, 114.00f), Util.cpv(98.00f, 136.73f),
            Util.cpv(99.00f, 137.61f), Util.cpv(109.00f, 135.06f), Util.cpv(117.00f, 137.94f), Util.cpv(122.52f, 146.00f), Util.cpv(122.94f, 151.00f), Util.cpv(121.00f, 158.58f), Util.cpv(134.00f, 160.97f), Util.cpv(153.00f, 157.45f), Util.cpv(171.30f, 150.00f), Util.cpv(169.06f, 142.00f), Util.cpv(169.77f, 136.00f), Util.cpv(174.00f, 129.73f),
            Util.cpv(181.46f, 126.00f), Util.cpv(182.22f, 120.00f), Util.cpv(182.20f, 111.00f), Util.cpv(180.06f, 101.00f), Util.cpv(171.28f, 85.00f), Util.cpv(171.75f, 80.00f), Util.cpv(182.30f, 53.00f), Util.cpv(189.47f, 50.00f), Util.cpv(190.62f, 38.00f), Util.cpv(194.00f, 33.73f), Util.cpv(199.00f, 30.77f), Util.cpv(208.00f, 30.48f),
            Util.cpv(216.00f, 34.94f), Util.cpv(224.00f, 31.47f), Util.cpv(240.00f, 30.37f), Util.cpv(247.00f, 32.51f), Util.cpv(249.77f, 35.00f), Util.cpv(234.75f, 53.00f), Util.cpv(213.81f, 93.00f), Util.cpv(212.08f, 99.00f), Util.cpv(213.00f, 101.77f), Util.cpv(220.00f, 96.77f), Util.cpv(229.00f, 96.48f), Util.cpv(236.28f, 101.00f),
            Util.cpv(240.00f, 107.96f), Util.cpv(245.08f, 101.00f), Util.cpv(263.00f, 65.32f), Util.cpv(277.47f, 48.00f), Util.cpv(284.00f, 47.03f), Util.cpv(286.94f, 41.00f), Util.cpv(292.00f, 36.62f), Util.cpv(298.00f, 35.06f), Util.cpv(304.00f, 35.77f), Util.cpv(314.00f, 43.81f), Util.cpv(342.00f, 32.56f), Util.cpv(359.00f, 31.32f),
            Util.cpv(365.00f, 32.57f), Util.cpv(371.00f, 36.38f), Util.cpv(379.53f, 48.00f), Util.cpv(379.70f, 51.00f), Util.cpv(356.00f, 52.19f), Util.cpv(347.00f, 54.74f), Util.cpv(344.38f, 66.00f), Util.cpv(341.00f, 70.27f), Util.cpv(335.00f, 73.52f), Util.cpv(324.00f, 72.38f), Util.cpv(317.00f, 65.75f), Util.cpv(313.00f, 67.79f),
            Util.cpv(307.57f, 76.00f), Util.cpv(315.00f, 78.62f), Util.cpv(319.28f, 82.00f), Util.cpv(322.23f, 87.00f), Util.cpv(323.00f, 94.41f), Util.cpv(334.00f, 92.49f), Util.cpv(347.00f, 87.47f), Util.cpv(349.62f, 80.00f), Util.cpv(353.00f, 75.73f), Util.cpv(359.00f, 72.48f), Util.cpv(366.00f, 72.32f), Util.cpv(372.00f, 74.94f),
            Util.cpv(377.00f, 81.34f), Util.cpv(382.00f, 83.41f), Util.cpv(392.00f, 83.40f), Util.cpv(399.00f, 79.15f), Util.cpv(404.00f, 85.74f), Util.cpv(411.00f, 85.06f), Util.cpv(417.00f, 86.62f), Util.cpv(423.38f, 93.00f), Util.cpv(425.05f, 104.00f), Util.cpv(438.00f, 110.35f), Util.cpv(450.00f, 112.17f), Util.cpv(452.62f, 103.00f),
            Util.cpv(456.00f, 98.73f), Util.cpv(462.00f, 95.48f), Util.cpv(472.00f, 95.79f), Util.cpv(471.28f, 92.00f), Util.cpv(464.00f, 84.62f), Util.cpv(445.00f, 80.39f), Util.cpv(436.00f, 75.33f), Util.cpv(428.00f, 68.46f), Util.cpv(419.00f, 68.52f), Util.cpv(413.00f, 65.27f), Util.cpv(408.48f, 58.00f), Util.cpv(409.87f, 46.00f),
            Util.cpv(404.42f, 39.00f), Util.cpv(408.00f, 33.88f), Util.cpv(415.00f, 29.31f), Util.cpv(429.00f, 26.45f), Util.cpv(455.00f, 28.77f), Util.cpv(470.00f, 33.81f), Util.cpv(482.00f, 42.16f), Util.cpv(494.00f, 46.85f), Util.cpv(499.65f, 36.00f), Util.cpv(513.00f, 25.95f), Util.cpv(529.00f, 22.42f), Util.cpv(537.18f, 23.00f),
    };


    private static Random rgen = new Random();
    private Space space;

    @Override
    public Space init() {
        System.out.println("initializing Bouncy Hexagons");
        space = new Space();

        space.setIterations(12);
        space.setGravity(Util.cpv(0, 0));
        space.setSleepTimeThreshold(0.5f);
        space.setCollisionSlop(0.5f);

        Body body, staticBody = space.getStaticBody();
        Shape shape;

        // Create vertexes for a hexagon shape.
        final int NUM_VERTS = 6;
        Vector2f[] verts = new DefaultVector2f[NUM_VERTS];
        for (int i = 0; i < NUM_VERTS; i++) {
            float angle = -2f * (float) Math.PI * i / ((float) NUM_VERTS);
            verts[i] = Util.cpv(5f * (float) Math.cos(angle), 5f * (float) Math.sin(angle));
        }

        // Add lots of hexagons.
        for (int i = 0; i < 500; i++) {
            body = space.addBody(new Body(1.0f, Util.momentForPoly(1.0f, verts, Util.cpvzero())));
            float x = rgen.nextFloat() * 200f - 100f;
            float y = rgen.nextFloat() * 200f - 100f;
            body.setPosition(Util.cpv(x, y));
            body.setVelocity(Util.cpv(rgen.nextFloat() * 160f - 80f, rgen.nextFloat() * 160f - 80f));

            shape = space.addShape(new PolyShape(body, verts, Util.cpvzero()));
            shape.setElasticity(1.0f);
        }

        Vector2f offset = Util.cpv(-320, -240);
        Vector2f a, b;
        for (int i = 0; i < (bouncy_terrain_verts.length - 1); i++) {
            a = Util.cpv(bouncy_terrain_verts[i]);
            b = Util.cpv(bouncy_terrain_verts[i + 1]);
            shape = space.addShape(new SegmentShape(staticBody, Util.cpvadd(a, offset), Util.cpvadd(b, offset), 0.0f));
            shape.setElasticity(1.0f);
        }

        return space;
    }

    @Override
    public void update(long delta) {
        int steps = 3;
        float dt = 1.0f / 60.0f / (float) steps;

        for (int i = 0; i < steps; i++) {
            space.step(dt);
        }
    }

    public static void main(String[] args) {
        new BouncyHexagons().start(640, 480);
    }
}

