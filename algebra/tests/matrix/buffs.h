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

#ifndef MATRIX_TEST_BUFF_H
#define MATRIX_TEST_BUFF_H

/* clang-format off */

/* A - small square matrix with integer values */
static const unsigned int buffs_rowsA = 3, buffs_colsA = 3;

static const float buffs_A[] = {
    -1, -2,  3,
     0,  2, -1,
    -1,  3,  0
};


/* B - small square matrix with integer values */
static const unsigned int buffs_rowsB = 3, buffs_colsB = 3;

static const float buffs_B[] = {
    1,  5,  1,
    2,  1,  2,
    3,  2,  3
};


/* This matrix is equal to A * B * A^T */
static const unsigned int buffs_rowsAsandB = 3, buffs_colsAsandB = 3;

static const float buffs_AsandB[] = {
    10, -6,  -7,
     2, -1,  -1,
    14, -9, -11
};


/* C - small matrix with integer values*/
static const unsigned int buffs_rowsC = 2, buffs_colsC = 3;

static const float buffs_C[] = {
    0, 3, 5,
    5, 5, 2
};


/* D - small matrix with integer values*/
static const unsigned int buffs_rowsD = 3, buffs_colsD = 2;

static const float buffs_D[] = {
    3,  4,
    3, -2,
    4, -2
};


/* Matrix equal to C*D */
static const unsigned int buffs_rowsCtimesD = 2, buffs_colsCtimesD = 2;

static const float buffs_CtimesD[] = {
    29, -16,
    38,   6
};


/* E - big matrix with float values */
static const unsigned int buffs_rowsE = 20, buffs_colsE = 15;

static const float buffs_E[] = {  
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
static const unsigned int buffs_rowsF = 15, buffs_colsF = 12;

static const float buffs_F[] = { 
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


/* This matrix is equal to E*F. We are using here precise result - that's why we are using this division notation */
static const unsigned int buffs_rowsEtimesF = 20, buffs_colsEtimesF = 12;

static const float buffs_EtimesF[] = {
      -47459568105833.0/5000000000.0,	  -25042928009033.0/5000000000.0,	-125994692452551.0/10000000000.0,	 208746765211743.0/10000000000.0,	  -17379447869169.0/1250000000.0,	  27368673009221.0/10000000000.0,	  16156610586023.0/10000000000.0,	  -46710191465987.0/5000000000.0,	  -35429443210031.0/5000000000.0,	  -19720846400867.0/5000000000.0,	      619958090013.0/80000000.0,	 -72645283874097.0/10000000000.0,
      -40349317454669.0/2500000000.0,	     -684108173151.0/156250000.0,	 -68474279614139.0/10000000000.0,	       973699618961.0/62500000.0,	 185471715520337.0/10000000000.0,	  39065634867133.0/10000000000.0,	 205764011708403.0/10000000000.0,	 174565579590703.0/10000000000.0,	 -17941491868577.0/10000000000.0,	  -69389546198621.0/5000000000.0,	  14226407187681.0/2000000000.0,	  39116511069879.0/10000000000.0,
      -12572958635169.0/1000000000.0,	   24437666185767.0/1250000000.0,	  87689585432199.0/10000000000.0,	 -12233924326301.0/10000000000.0,	  -32299123689627.0/5000000000.0,	   16388824504381.0/2500000000.0,	 -35380885159267.0/10000000000.0,	    -3475535817891.0/500000000.0,	  -17945150702003.0/5000000000.0,	  84039924416671.0/10000000000.0,	 -25563138376351.0/5000000000.0,	 -93889587775303.0/10000000000.0,
     -32865952097311.0/10000000000.0,	  -24289001536053.0/2500000000.0,	    -2624792003897.0/400000000.0,	   11634912163643.0/5000000000.0,	  -33189752473807.0/2500000000.0,	-154271721479719.0/10000000000.0,	  -51285913885311.0/5000000000.0,	        -15778861581.0/6250000.0,	    -5800728470711.0/312500000.0,	  -17747723516611.0/5000000000.0,	 -18263937105723.0/2500000000.0,	  -10917227064267.0/1250000000.0,
     -81139578537039.0/10000000000.0,	  46107665997301.0/10000000000.0,	 329429698316693.0/10000000000.0,	-158718840624291.0/10000000000.0,	  -49889508808481.0/2000000000.0,	  -80524902669039.0/5000000000.0,	-110406017706321.0/10000000000.0,	 -106429169168011.0/5000000000.0,	 133098159942243.0/10000000000.0,	 -32054056920303.0/10000000000.0,	 -54920431970619.0/2000000000.0,	 -47358516862343.0/10000000000.0,
      101300912561627.0/5000000000.0,	  -4423841165227.0/10000000000.0,	  -24696407738701.0/5000000000.0,	    5345125528963.0/2500000000.0,	 -155303851242409.0/5000000000.0,	     -537616971373.0/625000000.0,	-117495525461107.0/10000000000.0,	    3036833253443.0/2000000000.0,	  66539417855663.0/10000000000.0,	         14518034223.0/2500000.0,	   -337069168353.0/1000000000.0,	  -45396216678059.0/5000000000.0,
     -46914016886891.0/10000000000.0,	  11043927195163.0/10000000000.0,	 148485950335799.0/10000000000.0,	 -85393746250723.0/10000000000.0,	-104320096592873.0/10000000000.0,	  22891456295787.0/10000000000.0,	 -88307643998813.0/10000000000.0,	  -29766595262067.0/2500000000.0,	  81538082814399.0/10000000000.0,	  12952753643887.0/10000000000.0,	 -19162290856597.0/2500000000.0,	     5481117226211.0/500000000.0,
     -90856304407491.0/10000000000.0,	  -26988151261487.0/5000000000.0,	  17612465797093.0/10000000000.0,	     3408569642913.0/400000000.0,	  30307645845623.0/10000000000.0,	  89514173149837.0/10000000000.0,	 -56424126907217.0/10000000000.0,	-146262346253019.0/10000000000.0,	   32160252056021.0/5000000000.0,	    -1638911335897.0/125000000.0,	   9875903636109.0/1250000000.0,	  70826906201227.0/10000000000.0,
      103366453442477.0/5000000000.0,	 -52030972331117.0/10000000000.0,	  24980774756121.0/10000000000.0,	   27110362490627.0/2500000000.0,	   -2048321306431.0/2000000000.0,	  -22673641321969.0/2000000000.0,	   37613767325707.0/2500000000.0,	  -57184699930291.0/5000000000.0,	    -5760861024903.0/400000000.0,	-132872448112473.0/10000000000.0,	         7131571321.0/3200000.0,	 148702447559633.0/10000000000.0,
       -5743486111333.0/1250000000.0,	 164239720030069.0/10000000000.0,	   35038133780941.0/2500000000.0,	 -83490974109349.0/10000000000.0,	 -50328211269037.0/10000000000.0,	-137893743593251.0/10000000000.0,	  -3096899841587.0/10000000000.0,	       -241346752099.0/9765625.0,	     1583929319727.0/250000000.0,	   87388845798637.0/5000000000.0,	-115320555161989.0/5000000000.0,	 -86124453773997.0/10000000000.0,
    -187793828542891.0/10000000000.0,	   23786608500477.0/2500000000.0,	       748147864577.0/78125000.0,	  -31819512743669.0/5000000000.0,	   44605583936591.0/5000000000.0,	    -2032643718693.0/125000000.0,	  20388173160013.0/10000000000.0,	  -21242753632833.0/2500000000.0,	 -15324177381953.0/10000000000.0,	-184462800502921.0/10000000000.0,	   2437455774453.0/2500000000.0,	 -16238107803013.0/10000000000.0,
      60222477246879.0/10000000000.0,	-106081259896703.0/10000000000.0,	   -6490888928169.0/2000000000.0,	   26428980262019.0/1250000000.0,	-168404288363677.0/10000000000.0,	 -31862108431217.0/10000000000.0,	 -57409697124533.0/10000000000.0,	 -17493971132001.0/10000000000.0,	   13955384586899.0/2500000000.0,	  -24956005157649.0/2500000000.0,	    1815205829813.0/625000000.0,	    -5445408826437.0/400000000.0,
     156980963928333.0/10000000000.0,	      -503302430801.0/31250000.0,	   32004633176421.0/2000000000.0,	 -113035861399317.0/5000000000.0,	 276139134866189.0/10000000000.0,	  -81708396907247.0/5000000000.0,	  -9162854248827.0/10000000000.0,	   64906139300833.0/5000000000.0,	 -46284494253673.0/10000000000.0,	    -541004525621.0/2500000000.0,	 -20160408573271.0/5000000000.0,	   37665003645677.0/2000000000.0,
         -261387083951.0/312500000.0,	  -12688057728723.0/1250000000.0,	   11815688063349.0/2000000000.0,	 -42950427709271.0/10000000000.0,	   78343179039701.0/5000000000.0,	 -54107107588189.0/10000000000.0,	   64598020434511.0/2500000000.0,	    11953162879703.0/625000000.0,	      538829556143.0/625000000.0,	   -11124091597961.0/500000000.0,	   -1579298265367.0/312500000.0,	   18156431424827.0/1000000000.0,
      -85862378839223.0/5000000000.0,	   10440052404623.0/5000000000.0,	   -8172680094969.0/1250000000.0,	      -535869130957.0/50000000.0,	  95778143098509.0/10000000000.0,	    2280199927233.0/1000000000.0,	     999095419067.0/5000000000.0,	  172472611849313.0/5000000000.0,	 -21412317495951.0/10000000000.0,	-168447337672259.0/10000000000.0,	 79231671060783.0/10000000000.0,	  87029677072743.0/10000000000.0,
      -53489026706701.0/5000000000.0,	  39672076976289.0/10000000000.0,	  -99511791414329.0/5000000000.0,	      -789557775771.0/78125000.0,	 133204501274697.0/10000000000.0,	   -4268715236299.0/1000000000.0,	 171488657205277.0/10000000000.0,	   20024862870333.0/2500000000.0,	-175468926248111.0/10000000000.0,	  -74210106612887.0/5000000000.0,	 10819942070461.0/10000000000.0,	    3484864817821.0/2500000000.0,
       14200582546289.0/5000000000.0,	     4584580792837.0/200000000.0,	  96618437057531.0/10000000000.0,	    9775899524019.0/2500000000.0,	   -4695192989009.0/1000000000.0,	   -796570992677.0/10000000000.0,	     3235063510217.0/625000000.0,	  -34048986374477.0/2500000000.0,	   14734793250249.0/5000000000.0,	   26192770691447.0/2500000000.0,	  41274260100527.0/5000000000.0,	   44421464261721.0/5000000000.0,
     119827101873741.0/10000000000.0,	-130297718064983.0/10000000000.0,	    1283160295553.0/2000000000.0,	   55760109828349.0/5000000000.0,	   -6263160485331.0/5000000000.0,	-181269205255383.0/10000000000.0,	  126909491412681.0/5000000000.0,	   -4553722970767.0/1000000000.0,	   47125400738269.0/2500000000.0,	  -68228822768361.0/2500000000.0,	  28807345841069.0/5000000000.0,	-169096557241827.0/10000000000.0,
      36947468036203.0/10000000000.0,	  -93643668195409.0/5000000000.0,	       508640623699.0/62500000.0,	      184595829259.0/625000000.0,	    1514361235619.0/2500000000.0,	-152376283338813.0/10000000000.0,	   18213280141419.0/5000000000.0,	   2382632424939.0/10000000000.0,	   33765733982541.0/5000000000.0,	 -114657518579791.0/5000000000.0,	 -28428030639357.0/5000000000.0,	       282198642897.0/50000000.0,
     -83273854772559.0/10000000000.0,	    1136372614037.0/1250000000.0,	   60294569322277.0/5000000000.0,	  -45088836422489.0/5000000000.0,	  115238186485109.0/5000000000.0,	-141969064869371.0/10000000000.0,	   86723148742737.0/5000000000.0,	     -499695832287.0/250000000.0,	     5896919040381.0/400000000.0,	  -24234852190289.0/1250000000.0,	  -9146457036891.0/2500000000.0,	     -724203867487.0/156250000.0
};


static const unsigned int buffs_rowsG = 10, buffs_colsG = 15;

static const float buffs_G[] = {
    -71.23306,	 76.92959,	 97.35763,	-38.30976,	-71.08948,	 84.38139,	 54.72446,	 97.70814,	 81.37501,	 48.40553,	 97.50404,	-56.44275,	-74.61018,	-88.83393,	 15.27162,
     42.62024,	-64.32605,	-61.97475,	-81.28816,	 21.27332,	 39.05265,	 33.04993,	 76.55394,	 72.92666,	-93.31019,	-57.65328,	 -2.17061,	 71.13378,	 34.89744,	 59.85611,
     77.93165,	 41.16256,	-56.90262,	-65.18020,	-32.13754,	 20.70422,	 15.35551,	-30.62952,	 32.34400,	-34.58250,	-66.59382,	 77.89968,	-85.42574,	 63.89574,	-64.71333,
    -74.73176,	-41.45598,	-75.66110,	-84.82169,	-25.40665,	 32.96669,	-13.86574,	  1.37203,	-21.47871,	-89.65493,	-70.88043,	-66.68794,	-25.44491,	-24.21397,	 90.33519,
    -41.77715,	-22.67063,	 85.18075,	 45.17014,	-81.55433,	-85.24986,	-25.72988,	-67.89685,	 28.09271,	  0.39582,	-97.66182,	  8.33100,	-29.07447,	-80.53561,	 18.08030,
     62.17072,	-98.88202,	-70.77754,	 10.16033,	 57.01799,	-68.58443,	-87.31843,	  2.08512,	 53.27814,	 71.33640,	-47.61187,	-99.88750,	-73.68752,	 14.00475,	 68.35155,
      3.97927,	 50.40292,	 80.50544,	 17.44043,	 62.89419,	 56.81579,	-10.33868,	 54.03197,	 -5.37405,	 24.12153,	 31.36570,	  3.70497,	 57.17744,	-15.88377,	 61.58304,
     54.28003,	 72.69307,	 38.69736,	-65.24946,	 25.09608,	-25.28109,	 58.67525,	-24.23972,	 -4.64787,	 10.69936,	-90.40396,	-90.20452,	-78.64420,	 94.77823,	 20.34624,
     58.88127,	 99.79837,	 67.35919,	 87.48388,	 32.07215,	-13.57159,	-44.81538,	-81.56918,	-29.22721,	 27.17207,	 24.96566,	 17.58515,	-58.94980,	-72.37189,	 34.23760,
     22.09177,	 80.71322,	-68.78099,	-35.69381,	-49.26806,	 84.95800,	-87.67651,	-39.28627,	 77.10230,	-83.73856,	-69.64924,	-78.26606,	-67.29108,	  4.74067,	  1.13649
};


static const unsigned int buffs_rowsH = 15, buffs_colsH = 15;

static const float buffs_H[] = {
     99.67867,	 17.98695,	 43.36675,	 19.84845,	-82.28063,	 62.98004,	-30.31563,	 86.09034,	-90.91627,	  5.75828,	-16.45718,	-47.80303,	 27.29972,	 24.28029,	 36.49940,
    -24.94488,	-39.24248,	 45.70308,	 -0.58056,	 -6.29304,	 34.17753,	 32.56944,	 36.37329,	 54.36871,	 -4.97004,	 54.58158,	 -3.26883,	-48.16249,	 34.18541,	 70.71110,
    -67.58366,	 68.00804,	-99.77149,	-50.68994,	-26.81330,	-66.94190,	  4.53349,	-39.32047,	 70.33179,	  8.68163,	 32.91971,	-91.35144,	-62.16993,	 61.80249,	 46.14721,
    -87.83206,	-77.88370,	-29.41014,	-49.63063,	-99.43642,	-79.42071,	-63.69576,	 96.38860,	-22.25819,	-16.95727,	 71.28527,	  6.63395,	 63.08570,	 78.50666,	 91.92504,
     35.97524,	 29.68940,	 47.30226,	 16.75764,	-16.82804,	-37.00467,	 31.85432,	 80.11482,	-81.14059,	 20.16295,	 38.38806,	-12.07693,	 56.40466,	 49.28137,	 19.26776,
    -45.30794,	  4.47840,	-86.77153,	-22.78187,	 84.72503,	-26.96556,	-74.61709,	-42.43497,	-47.94259,	-17.53728,	 75.78811,	  1.22246,	 52.02452,	 84.41607,	 29.03548,
     97.74750,	-52.65865,	 53.22160,	-82.08096,	 84.32212,	 45.79999,	 63.60070,	 46.76422,	 48.86569,	 19.34060,	 91.11481,	-99.94247,	 57.23813,	 65.42709,	-50.27271,
    -36.97220,	 32.34478,	 34.13365,	-85.70981,	-45.24843,	-70.38599,	-62.26171,	 99.76303,	-33.07196,	 99.96417,	-54.38065,	-46.37688,	 28.48934,	-32.90519,	 -0.45902,
     70.93476,	 34.59869,	 31.65317,	 76.26252,	 32.78446,	 96.89118,	-84.56754,	-20.32521,	-67.63208,	  4.01436,	-40.16003,	-28.02289,	 -7.53257,	 50.28468,	 51.63588,
    -39.41504,	 69.16080,	  0.39062,	-34.66410,	 62.82210,	 70.01150,	 81.81866,	-82.07793,	-59.69720,	-97.80208,	 30.29300,	-46.62504,	-82.36000,	 19.95837,	 78.22123,
      8.33706,	-24.87575,	 14.42797,	 35.49217,	-95.10882,	-77.99232,	-69.06680,	-15.01246,	 77.24802,	-44.67450,	 -1.25154,	  5.12065,	-50.11029,	 28.03385,	-10.94539,
     12.38524,	 98.78298,	 94.56595,	 24.15342,	 95.49463,	 21.68097,	 17.28577,	 28.38860,	-17.45793,	 96.97092,	-54.10731,	 29.39638,	-46.64201,	-79.76484,	 83.52040,
      5.39941,	 59.26349,	 76.76387,	-58.42829,	 39.38072,	 74.76518,	-15.21213,	 54.78531,	 50.00434,	-14.56868,	 40.60926,	 53.10250,	 91.68220,	-53.76980,	 12.02886,
    -30.91879,	 12.80960,	-40.75792,	 97.16540,	 38.75903,	 99.19875,	 42.63202,	-59.26603,	 66.66872,	-42.80030,	-16.27343,	  4.70163,	-26.16226,	-76.26342,	-89.24624,
    -23.68011,	  5.48650,	 90.76039,	 48.14960,	-46.01896,	 28.96649,	-75.24915,	-69.63227,	 98.60089,	-95.30351,	 77.10099,	 98.48375,	-96.69790,	-78.62217,	-59.26076
};


/* This matrix is equal to G * H * G^T */
static const unsigned int buffs_rowsGsandH = 10, buffs_colsGsandH = 10;

static const float buffs_GsandH[] = {
    -1174375374260461151479.0/500000000000000.0,	  108372524575145042507.0/1000000000000000.0,	      10869287378728922819.0/6250000000000.0,	  934942707572815630853.0/200000000000000.0,	  -103649274907556186807.0/200000000000000.0,	 8439057422929778858267.0/1000000000000000.0,	   -113679618299820301641.0/25000000000000.0,	  2690891947886309030089.0/500000000000000.0,	 -1119398511252279181999.0/500000000000000.0,	 5380122835604353043513.0/1000000000000000.0,
      161015265948047646411.0/500000000000000.0,	  -393131420282807240183.0/200000000000000.0,	  -229280902096203832017.0/250000000000000.0,	-3331928373871870286521.0/500000000000000.0,	  -191815506321950143659.0/200000000000000.0,	-3164685452588012710161.0/1000000000000000.0,	  2577425921310280105279.0/500000000000000.0,	 -1424895654013803096329.0/500000000000000.0,	 4165230629379932556927.0/1000000000000000.0,	  -601487711992762950331.0/500000000000000.0,
     -499909185591625576949.0/250000000000000.0,	-1151797732735037116341.0/1000000000000000.0,	   -51336618674281949209.0/200000000000000.0,	 -299160240553819200069.0/125000000000000.0,	-2233907010329117830429.0/1000000000000000.0,	   693507897925449279867.0/500000000000000.0,	 3043941567743963544503.0/1000000000000000.0,	  1403108125542964740543.0/500000000000000.0,	    153016811589749201341.0/40000000000000.0,	  -495309813893293699899.0/500000000000000.0,
       -46386903521386745361.0/12500000000000.0,	 -1287074633515015701683.0/500000000000000.0,	 2263563242366693600669.0/1000000000000000.0,	-1054461692082731890673.0/250000000000000.0,	  1302045350703790633537.0/500000000000000.0,	-2030049578081618031349.0/1000000000000000.0,	-2544004016902160531159.0/1000000000000000.0,	-5503505119169757175123.0/1000000000000000.0,	   367819203526718675907.0/250000000000000.0,	-1174069983958569847771.0/1000000000000000.0,
       34807019175118010483.0/200000000000000.0,	   31065659179781388789.0/1000000000000000.0,	   692332261459316483317.0/500000000000000.0,	  797571892655142567037.0/250000000000000.0,	   646192170096323016579.0/500000000000000.0,	    111001566295186733223.0/62500000000000.0,	   -127355748491273754899.0/62500000000000.0,	 1580142865183630112331.0/1000000000000000.0,	   -10460010479129036951.0/200000000000000.0,	 2046564859498400118953.0/1000000000000000.0,
      -152786808804789723301.0/40000000000000.0,	-3215493632524926990757.0/1000000000000000.0,	  1037576945325622434911.0/500000000000000.0,	 933975039720354048981.0/1000000000000000.0,	    170053813287618507989.0/50000000000000.0,	 -2049481769012582038063.0/500000000000000.0,	  -603444837154171230073.0/250000000000000.0,	-1732059825087768916391.0/1000000000000000.0,	   588874163809755758679.0/200000000000000.0,	  1145534512977156640359.0/500000000000000.0,
     2044351087108814200201.0/500000000000000.0,	 -460809309014674927099.0/1000000000000000.0,	-1429207178228972578201.0/1000000000000000.0,	  229782342208929322093.0/200000000000000.0,	 -408621837009782935781.0/1000000000000000.0,	       -985643441576539009.0/2000000000000.0,	     42221242338989553707.0/40000000000000.0,	  -43542769775913090897.0/1000000000000000.0,	 -183295448450926905343.0/1000000000000000.0,	   117000972542134703557.0/125000000000000.0,
        11024362053440294921.0/10000000000000.0,	     32619735180422301571.0/40000000000000.0,	 2145983646953753254807.0/1000000000000000.0,	  -67127682664902944993.0/250000000000000.0,	    -59780685306443929829.0/15625000000000.0,	  -164296501136306396853.0/200000000000000.0,	  -370875058799941375333.0/200000000000000.0,	   880844712041706656223.0/500000000000000.0,	-1129194531550192789469.0/1000000000000000.0,	  1308817159058757092197.0/500000000000000.0,
      170968068172360403783.0/100000000000000.0,	   255278621999993871529.0/500000000000000.0,	  -294633571838603351761.0/500000000000000.0,	3646558929998091969909.0/1000000000000000.0,	 1154466174574322931813.0/1000000000000000.0,	 1849452985315688439713.0/1000000000000000.0,	-1905605433200491868377.0/1000000000000000.0,	   368743068425089756749.0/500000000000000.0,	    -32448394817520825427.0/20000000000000.0,	  222865928489139210657.0/1000000000000000.0,
    -1930330311375881143581.0/250000000000000.0,	  -472602800502084652917.0/500000000000000.0,	  803103887970166896837.0/1000000000000000.0,	    -949902249154167037.0/200000000000000.0,	    63790234900807923791.0/500000000000000.0,	   365267183102757413681.0/250000000000000.0,	-1846153534900935272733.0/1000000000000000.0,	  -505813712061073125699.0/125000000000000.0,	   339697579065229007799.0/500000000000000.0,	  611239962412601857077.0/1000000000000000.0
};


/* clang-format on */

#endif
