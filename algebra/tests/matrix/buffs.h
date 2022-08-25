/*
 * Phoenix-Pilot
 *
 * Buffers for matrix library unit tests
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */


/* A - small square matrix with integer values */
const unsigned int buffs_rowsA = 3, buffs_colsA = 3;

const float buffs_A[] = {
    -1, -2,  3,
     0,  2, -1,
    -1,  3,  0
};


/* B - small square matrix with integer values */
const unsigned int buffs_rowsB = 3, buffs_colsB = 3;

const float buffs_B[] = {
    1,  5,  1,
    2,  1,  2,
    3,  2,  3
};


/* C - small matrix with integer values*/
const unsigned int buffs_rowsC = 2, buffs_colsC = 3;

const float buffs_C[] = {
    0, 3, 5,
    5, 5, 2
};


/* D - small matrix with integer values*/
const unsigned int buffs_rowsD = 3, buffs_colsD = 2;

const float buffs_D[] = {
    3,  4,
    3, -2,
    4, -2
};


/* E - big matrix with float values */
const unsigned int buffs_rowsE = 20, buffs_colsE = 15;

const float buffs_E[] = {  
     78.95615,	 46.66450,	 96.14773,	 74.12088,	 74.88720,	  6.46915,	 53.16968,	-40.29660,	 -1.40914,	-24.74833,	-52.48458,	 34.02692,	 55.92622,	-45.76079,	 21.56489,
     76.43067,	-89.92965,	-33.09507,	 90.14247,	 46.25325,	 34.20910,	-24.32962,	 43.36492,	 -9.96188,	 91.01780,	  8.01281,	 48.33970,	 11.17384,	 -7.38102,	-96.72735,
     21.44963,	 27.27700,	 60.10502,	-37.28911,	 43.84365,	 18.49805,	 78.76087,	 97.75989,	 -1.92337,	 46.24458,	 21.56828,	-71.00521,	 53.42809,	-29.95182,	-32.96295,
     67.15677,	 10.61812,	 83.94018,	-55.02394,	 63.01558,	-86.62694,	 31.62592,	-16.56527,	 12.14964,	-16.95365,	-92.98379,	-28.39581,	-12.16005,	-30.67673,	 56.74463,
    -93.29795,	 13.28492,	 47.40115,	-84.17913,	-95.09192,	 51.05386,	 96.68683,	-20.22738,	-73.05975,	 37.57926,	-92.99695,	-41.95776,	 43.60677,	-45.40557,	 97.41441,
      2.40248,	 91.58967,	  8.21395,	-79.05149,	-89.11697,	-88.90335,	 11.14529,	-32.33563,	-32.25544,	-93.56707,	-21.75694,	 97.66116,	  1.43534,	-23.01376,	 87.52366,
    -60.29059,	 89.86046,	 -5.14560,	 53.66932,	-73.28500,	 90.23274,	 33.59484,	  3.53540,	-48.94070,	 -4.00987,	-68.20324,	  7.64371,	 24.76144,	-54.64690,	 32.80789,
     24.64867,	 26.72501,	-18.58118,	 45.67702,	 70.60465,	 55.25718,	-35.88838,	-21.33370,	-79.75388,	 17.24966,	-44.49713,	 32.85084,	 65.73957,	-76.28837,	  4.52809,
     44.21281,	 53.68042,	  0.94209,	-27.85270,	-33.71146,	 13.03607,	 18.47210,	-63.91245,	-94.98900,	 36.46611,	 39.76143,	  2.78590,	-83.01015,	-87.39184,	-82.31151,
    -98.29721,	-39.77412,	-91.59049,	-55.88468,	-92.36772,	 67.29914,	 49.64089,	-56.03084,	 96.34656,	 79.63599,	 14.03382,	-75.61517,	 66.09162,	-76.71757,	 86.85236,
    -64.25703,	 -6.97594,	 88.84141,	 65.37765,	 52.16607,	-50.49504,	 80.79414,	 73.84577,	-59.55733,	 62.99027,	-36.36835,	-30.87996,	 18.90707,	-55.99054,	 31.17618,
     94.25520,	 -8.13789,	 30.51608,	-50.26063,	 12.73708,	-80.24050,	-35.60950,	-18.98589,	-62.98830,	-33.26638,	-13.97939,	 54.56570,	 47.53514,	-66.17890,	 42.12020,
     65.70246,	-21.99977,	-68.43527,	 37.88990,	-26.84420,	-42.01969,	-94.38826,	 26.34452,	-71.66754,	 44.48669,	-55.27345,	-94.17436,	-83.12983,	 99.35118,	 33.52906,
      8.09638,	-67.31765,	-81.11762,	 61.93854,	-71.26161,	 80.21351,	 11.25338,	 26.67928,	-92.26951,	 95.68536,	-32.94845,	 75.27795,	-96.10089,	  9.51196,	-93.40333,
    -79.04829,	-33.59661,	 24.50834,	-26.98523,	 18.45786,	-71.55675,	-49.02644,	 88.14655,	 21.02259,	 14.82872,	 11.20553,	 97.78188,	-63.24182,	 33.25644,	-94.77354,
     22.43208,	-37.07981,	-44.27673,	 36.37943,	 89.31849,	  4.33515,	 89.42466,	 36.85958,	 -4.05201,	 83.45137,	-77.99463,	 64.75104,	-64.72626,	 -7.18604,	 -5.05744,
    -90.00741,	 75.24905,	 20.35746,	 -9.74547,	-37.69510,	-15.21690,	 42.87076,	-67.35473,	 -9.70579,	 37.06039,	 73.40603,	  7.63022,	 70.75394,	 -7.23655,	-32.85513,
     61.09290,	-72.26061,	 62.73610,	 45.52496,	-37.72517,	 -2.56187,	 -6.77651,	-60.28251,	-60.67866,	 17.35602,	 67.21946,	 50.75327,	-57.22459,	-14.04222,	 92.18982,
      1.86500,	-38.65328,	-15.28883,	-38.19950,	-57.35735,	 27.57501,	-91.81205,	-52.65141,	-32.72091,	 27.90737,	  7.40218,	 30.17098,	-70.91230,	-76.41243,	 11.21213,
    -33.39468,	-92.32901,	-15.92960,	 58.05569,	-39.72368,	 74.69751,	-43.86294,	 62.85577,	 20.53891,	 97.16222,	 98.07118,	-51.17309,	-63.54735,	-64.43356,	 18.78160
};


/* F - big matrix with float values */
const unsigned int buffs_rowsF = 15, buffs_colsF = 12;

const float buffs_F[] = { 
     61.48089,	-44.80124,	-49.33748,	 98.07285,	-55.78205,	 14.16248,	 46.01931,	 33.17280,	-36.05119,	 41.81020,	 -5.50665,	-60.96922,
     45.35729,	 87.08668,	 -1.61851,	-76.66452,	-26.06310,	 49.55097,	-52.17210,	-21.60139,	-52.77478,	 96.48536,	 53.59350,	 89.44353,
    -97.40409,	-36.28306,	 66.57804,	 25.44342,	-81.05998,	-49.04616,	 -2.33492,	 27.97118,	 35.50844,	-82.08937,	 -8.37861,	-34.25712,
     -7.53392,	-29.05773,	-34.31997,	 49.03210,	 99.81123,	-25.63574,	 63.40094,	  4.73289,	 -8.05271,	-21.53683,	 64.47193,	 43.55512,
    -40.92832,	 26.92834,	-97.35809,	-29.54060,	 88.23332,	 62.86358,	-15.44277,	-61.48131,	-46.57177,	-51.21540,	 84.81163,	 -1.47711,
    -53.04936,	-15.06756,	 41.19327,	-36.76358,	-37.25478,	 95.80091,	 -1.37586,	-47.67056,	 84.72178,	-16.40629,	-36.78621,	-12.53062,
     45.91220,	 84.04043,	-64.08653,	 34.51349,	-55.26862,	 -1.24850,	 82.29173,	-64.58869,	-62.21733,	 51.15921,	-27.86643,	-71.75306,
    -69.53260,	 52.03300,	 20.28623,	-75.56984,	  7.00192,	 65.19528,	-53.45280,	 92.10068,	 50.09239,	  5.44353,	-16.87527,	-49.76914,
    -68.81520,	 27.99405,	-91.88078,	 -0.05763,	-26.05631,	  3.14560,	 -8.64962,	 65.86499,	-72.80224,	 97.14176,	-24.57089,	-35.25910,
    -82.02578,	 83.19997,	 73.78858,	-83.33091,	 69.68811,	-92.02944,	 87.81585,	 36.73591,	-17.84779,	-35.67683,	 -4.91864,	 59.62460,
     88.90802,	 93.83860,	-23.67160,	 40.02814,	 35.34224,	 41.35678,	 88.12341,	-56.71537,	 77.41586,	  7.43982,	 89.58778,	-68.96012,
    -45.71117,	 13.46179,	-86.87269,	 26.45024,	-62.57004,	 61.10005,	 52.85763,	 98.95016,	 61.16001,	-73.94886,	 57.73218,	-33.69205,
    -71.27151,	 64.21733,	 56.70519,	 93.22075,	-67.39442,	 79.35260,	-60.59705,	-34.90471,	 81.73642,	 92.91479,	  2.01442,	-42.81778,
     17.42913,	 35.19914,	 26.43354,	-88.49276,	 27.82751,	 55.40078,	 17.38321,	 85.23450,	 35.21412,	 74.43137,	 22.36589,	 23.76190,
     23.38073,	 18.60795,	  9.82940,	-89.34235,	-14.77751,	-36.91180,	-11.59496,	-66.05795,	 99.71110,	-20.75876,	 -5.07695,	-98.43841
};
