//#include "common.h"
#include "wavelet.h"

//#include <math.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef FWT_LOG2N
#define FWT_LOG2N	6
#endif

#define FWT_N		(1UL<<FWT_LOG2N)

#ifndef FWT_MIN
#define FWT_MIN		2
#endif

#if !defined(FWT_DAUB1) || !defined(FWT_DAUB2) || !defined(FWT_DAUB3) || !defined(FWT_DAUB4) || !defined(FWT_DAUB5) || !defined(FWT_DAUB6) || !defined(FWT_DAUB7) || !defined(FWT_DAUB8)
#define FWT_DAUB1
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef FWT_DAUB1

const FWT_FLTR hF[2] = { FLOAT_FWT( 0.7071067811865476), FLOAT_FWT(0.7071067811865476) };
const FWT_FLTR gF[2] = { FLOAT_FWT(-0.7071067811865476), FLOAT_FWT(0.7071067811865476) };

#elif defined(FWT_DAUB2)

const FWT_FLTR hF[4] = { FLOAT_FWT(-0.12940952255092145),	FLOAT_FWT(0.22414386804185735),	FLOAT_FWT(0.836516303737469),		FLOAT_FWT(0.48296291314469025)	};
const FWT_FLTR gF[4] = { FLOAT_FWT(-0.48296291314469025),	FLOAT_FWT(0.836516303737469),	FLOAT_FWT(-0.22414386804185735),	FLOAT_FWT(-0.12940952255092145) };

#elif defined(FWT_DAUB3)

const FWT_FLTR hF[6] = {	FLOAT_FWT( 0.0352262918821007), FLOAT_FWT(-0.0854412738822415), FLOAT_FWT(-0.1350110200103908),
							FLOAT_FWT( 0.4598775021193313), FLOAT_FWT( 0.8068915093133388), FLOAT_FWT( 0.3326705529509569)	};

const FWT_FLTR gF[6] = {	FLOAT_FWT(-0.3326705529509569), FLOAT_FWT( 0.8068915093133388), FLOAT_FWT(-0.4598775021193313), 
							FLOAT_FWT(-0.1350110200103908), FLOAT_FWT( 0.0854412738822415), FLOAT_FWT( 0.0352262918821007)	};
							 
#elif defined(FWT_DAUB4)

const FWT_FLTR hF[8] = {	FLOAT_FWT(-0.010597401784997), FLOAT_FWT(0.0328830116669829), FLOAT_FWT(0.0308413818359870), FLOAT_FWT(-0.187034811718881),
							FLOAT_FWT(-0.027983769416984), FLOAT_FWT(0.6308807679295904), FLOAT_FWT(0.7148465705525415), FLOAT_FWT(0.2303778133088552)		};

const FWT_FLTR gF[8] = {	FLOAT_FWT(-0.230377813308855), FLOAT_FWT(0.7148465705525415), FLOAT_FWT(-0.630880767929590), FLOAT_FWT(-0.027983769416984),
							FLOAT_FWT(0.1870348117188811), FLOAT_FWT(0.0308413818359870), FLOAT_FWT(-0.032883011666983), FLOAT_FWT(-0.010597401784997)		};

#elif defined(FWT_DAUB5)

const FWT_FLTR hF[10] = {	FLOAT_FWT( 0.003335725285002), FLOAT_FWT(-0.012580751999016), FLOAT_FWT(-0.006241490213012), FLOAT_FWT( 0.077571493840065),
							FLOAT_FWT(-0.032244869585030), FLOAT_FWT(-0.242294887066190), FLOAT_FWT( 0.138428145901103), FLOAT_FWT( 0.724308528438574),
							FLOAT_FWT( 0.603829269797473), FLOAT_FWT( 0.160102397974125)		};

const FWT_FLTR gF[10] = {	FLOAT_FWT(-0.160102397974125), FLOAT_FWT( 0.603829269797473), FLOAT_FWT(-0.724308528438574), FLOAT_FWT( 0.138428145901103),
							FLOAT_FWT( 0.242294887066190), FLOAT_FWT(-0.032244869585030), FLOAT_FWT(-0.077571493840065), FLOAT_FWT(-0.006241490213012),
							FLOAT_FWT( 0.012580751999016), FLOAT_FWT( 0.003335725285002)		};

#elif defined(FWT_DAUB6)

const FWT_FLTR hF[12] = {	FLOAT_FWT(-0.0010773010849956), FLOAT_FWT( 0.0047772575110107), FLOAT_FWT( 0.0005538422009938), FLOAT_FWT(-0.0315820393180312),
							FLOAT_FWT( 0.0275228655300163), FLOAT_FWT( 0.0975016055870794), FLOAT_FWT(-0.1297668675670956), FLOAT_FWT(-0.2262646939651691),
							FLOAT_FWT( 0.3152503517092432), FLOAT_FWT( 0.7511339080215775), FLOAT_FWT( 0.4946238903983854), FLOAT_FWT( 0.1115407433500802)		};

const FWT_FLTR gF[12] = {	FLOAT_FWT(-0.1115407433500802), FLOAT_FWT( 0.4946238903983854), FLOAT_FWT(-0.7511339080215775), FLOAT_FWT( 0.3152503517092432),
							FLOAT_FWT( 0.2262646939651691), FLOAT_FWT(-0.1297668675670956), FLOAT_FWT(-0.0975016055870794), FLOAT_FWT( 0.0275228655300163),
							FLOAT_FWT( 0.0315820393180312), FLOAT_FWT( 0.0005538422009938), FLOAT_FWT(-0.0047772575110107), FLOAT_FWT(-0.0010773010849956)		};

#elif defined(FWT_DAUB7)

const FWT_FLTR hF[14] = {	FLOAT_FWT( 0.0003537138000010),FLOAT_FWT(-0.0018016407039998),FLOAT_FWT( 0.0004295779730047),FLOAT_FWT( 0.0125509985560138),
							FLOAT_FWT(-0.0165745416310156),FLOAT_FWT(-0.0380299369350346),FLOAT_FWT( 0.0806126091510659),FLOAT_FWT( 0.0713092192670500),
							FLOAT_FWT(-0.2240361849941657),FLOAT_FWT(-0.1439060039291063),FLOAT_FWT( 0.4697822874053586),FLOAT_FWT( 0.7291320908465551),
							FLOAT_FWT( 0.3965393194823058),FLOAT_FWT( 0.0778520540850624)																	};

const FWT_FLTR gF[14] = {	FLOAT_FWT(-0.0778520540850624),FLOAT_FWT( 0.3965393194823058),FLOAT_FWT(-0.7291320908465551),FLOAT_FWT( 0.4697822874053586),
							FLOAT_FWT( 0.1439060039291063),FLOAT_FWT(-0.2240361849941657),FLOAT_FWT(-0.0713092192670500),FLOAT_FWT( 0.0806126091510659),
							FLOAT_FWT( 0.0380299369350346),FLOAT_FWT(-0.0165745416310156),FLOAT_FWT(-0.0125509985560138),FLOAT_FWT( 0.0004295779730047),
							FLOAT_FWT( 0.0018016407039998),FLOAT_FWT( 0.0003537138000010)																	};

#elif defined(FWT_DAUB8)

const FWT_FLTR hF[16] = {	FLOAT_FWT(-0.000117476784002),FLOAT_FWT( 0.000675449405999),FLOAT_FWT(-0.000391740372996),FLOAT_FWT(-0.004870352993011),
							FLOAT_FWT( 0.008746094047016),FLOAT_FWT( 0.013981027917016),FLOAT_FWT(-0.044088253931065),FLOAT_FWT(-0.017369301002022),
							FLOAT_FWT( 0.128747426620186),FLOAT_FWT( 0.000472484573998),FLOAT_FWT(-0.284015542962428),FLOAT_FWT(-0.015829105256024),
							FLOAT_FWT( 0.585354683654869),FLOAT_FWT( 0.675630736298013),FLOAT_FWT( 0.312871590914466),FLOAT_FWT( 0.054415842243082)		};

const FWT_FLTR gF[16] = {	FLOAT_FWT(-0.054415842243082),FLOAT_FWT( 0.312871590914466),FLOAT_FWT(-0.675630736298013),FLOAT_FWT( 0.585354683654869),
							FLOAT_FWT( 0.015829105256024),FLOAT_FWT(-0.284015542962428),FLOAT_FWT(-0.000472484573998),FLOAT_FWT( 0.128747426620186),
							FLOAT_FWT( 0.017369301002022),FLOAT_FWT(-0.044088253931065),FLOAT_FWT(-0.013981027917016),FLOAT_FWT( 0.008746094047016),
							FLOAT_FWT( 0.004870352993011),FLOAT_FWT(-0.000391740372996),FLOAT_FWT(-0.000675449405999),FLOAT_FWT(-0.000117476784002)		};

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Wavelet_step(FWT_DATA *f, u16 n, FWT_DATA *t);
static void InvWavelet_step(FWT_DATA *f, u16 n, FWT_DATA *t);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Wavelet transform

void Wavelet(FWT_DATA *f, u16 log2n)
{
   	if (log2n < 3 || log2n > FWT_LOG2N) return;  // Length is not power of 2
	
	u16 n = (1UL<<log2n);

	FWT_DATA temp[FWT_N];
	FWT_DATA *vec = f;
	FWT_DATA *tmp = temp;

    for (u16 m = n; m >= FWT_MIN; m >>= 1)
	{
		Wavelet_step(vec, m, tmp); 

		//FWT_DATA *t = vec;	vec = tmp; tmp = t;
	};

	//if (tmp != f) { for (int i = 0; i < n; i++) *(f++) = *(tmp++); };
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Wavelet_step(FWT_DATA *f, u16 n, FWT_DATA *t)
{
    const u16 nh = (n>>1);
    const u16 m = n-1;  // mask to compute modulo n (n is a power of two)

    for (u16 i = 0, j = 0; i < n; i += 2, ++j) // i \in [0,2,4,..,n-2];  j \in [0,1,2,..,n/2-1]
    {
        FWT_DATA s = 0,  d = 0;

        for (u16 k = 0; k < ArraySize(hF); ++k)
        {
            u16 w = (i+k) & m;

            s += MULT_FWT(hF[k] * f[w]);
            d += MULT_FWT(gF[k] * f[w]);
        };

        t[j] = s;
        t[nh+j] = d;
    };

	for (u16 i = n; i > 0; i--) *(f++) = *(t++);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Inverse wavelet transform

void InvWavelet(FWT_DATA *f, u16 log2n)
{
   	if (log2n < 3 || log2n > FWT_LOG2N) return;  // Length is not power of 2
	
	u16 n = (1UL<<log2n);

	FWT_DATA temp[FWT_N];
	FWT_DATA *vec = f;
	FWT_DATA *tmp = temp;

    for (u16 m = FWT_MIN; m <= n; m <<= 1)
	{
		InvWavelet_step(vec, m, tmp); 

		//FWT_DATA *t = vec;	vec = tmp; tmp = t;
	};

	//if (tmp != f) { for (int i = 0; i < n; i++) *(f++) = *(tmp++); };
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InvWavelet_step(FWT_DATA *f, u16 n, FWT_DATA *t)
{
    const u16 nh = (n>>1);
    const u16 m = n-1;  // mask to compute modulo n (n is a power of two)

	for (u16 i = 0; i < n; i++) t[i] = 0; // null(t, n);  // t[] := [0,0,...,0]

    for (u16 i = 0, j = 0; i < n; i+=2, ++j)
    {
        const FWT_DATA x = f[j],  y = f[nh+j];

        for (u16 k = 0; k < ArraySize(hF); ++k)
        {
            u16 w = (i+k) & m;

            t[w] += MULT_FWT(hF[k] * x);
            t[w] += MULT_FWT(gF[k] * y);
        }
    };

	for (u16 i = n; i > 0; i--) *(f++) = *(t++);
}
// -------------------------
 


