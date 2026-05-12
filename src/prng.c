/*
 * ============================================================================
 * prng.c — Deterministic Pseudo-Random Number Generator (PRNG)
 * ============================================================================
 *
 * PURPOSE:
 *   This file provides a high-quality, deterministic random number generator
 *   that the simulator relies on for ALL randomness. That includes:
 *     - Picking which of the 64 constellation symbols to transmit
 *     - Generating Gaussian (bell-curve) noise added to signals
 *
 *   "Deterministic" means: if you give it the same starting seed number,
 *   it will always produce the EXACT SAME sequence of random numbers.
 *   This is critical for reproducibility — you can re-run a simulation
 *   with the same seed and get identical results, which is essential
 *   for debugging and scientific verification.
 *
 * CORE ALGORITHM — xoshiro256** (pronounced "zoh-SHEE-roh 256 star-star"):
 *   - This is one of the best-known non-cryptographic PRNGs in use today
 *   - It was designed by David Blackman and Sebastiano Vigna
 *   - It maintains 256 bits of internal state (four 64-bit numbers)
 *   - It has a period of 2^256 - 1, meaning it can produce that many numbers
 *     before repeating. This is an astronomically large number
 *   - It passes all major statistical randomness tests (BigCrush, PractRand)
 *   - It is very fast: roughly 10 CPU cycles per 64-bit random number
 *
 * SEEDING — SplitMix64:
 *   - The user provides a single 32-bit seed (like 42 or 12345)
 *   - But xoshiro256** needs 256 bits (four 64-bit words) of initial state
 *   - SplitMix64 is a simpler PRNG used ONLY during initialization to
 *     "expand" the 32-bit seed into four well-distributed 64-bit values
 *   - This ensures that even similar seeds (like 0 and 1) produce
 *     completely different random sequences
 *
 * RANDOM DISTRIBUTIONS PROVIDED:
 *   1. prng_uniform()  → Returns a random decimal number in [0.0, 1.0)
 *   2. prng_gauss()    → Returns a Gaussian (normal) random number with
 *                         mean=0 and standard deviation=1 (the "standard normal")
 *   3. prng_uint32()   → Returns a random 32-bit unsigned integer (0 to ~4 billion)
 *
 * THREAD SAFETY:
 *   This module uses file-scope static variables (global state).
 *   It is NOT thread-safe. If multiple threads call these functions
 *   simultaneously, the state will become corrupted. This is fine for
 *   this simulator, which runs single-threaded.
 *
 * ============================================================================
 */

#include "prng.h"

#include <math.h>
#include <omp.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_OMP_THREADS 64

typedef struct {
    uint64_t s[4];
    int have_spare_gauss;
    double spare_gauss;
} PrngThreadState;

static PrngThreadState prng_threads[MAX_OMP_THREADS];
static int prng_parallel_initialized = 0;

static uint64_t s[4] = {
    0x180ec6d33cfd0abaULL,
    0xd5a61266f0c9392cULL,
    0xa9582618e03fc9aaULL,
    0x39abdc4529b1661cULL
};

static int have_spare_gauss = 0;
static double spare_gauss = 0.0;


static inline uint64_t rotl(const uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}

static uint64_t next_u64(void) {
    const uint64_t result = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return result;
}

static uint64_t next_u64_state(uint64_t *st) {
    const uint64_t result = rotl(st[1] * 5, 7) * 9;
    const uint64_t t = st[1] << 17;
    st[2] ^= st[0];
    st[3] ^= st[1];
    st[1] ^= st[2];
    st[0] ^= st[3];
    st[2] ^= t;
    st[3] = rotl(st[3], 45);
    return result;
}


static uint64_t splitmix64(uint64_t* x) {
    uint64_t z = (*x += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

void prng_seed(uint32_t seed) {
    uint64_t state = (uint64_t)seed;
    if (state == 0u) state = 1u;
    s[0] = splitmix64(&state);
    s[1] = splitmix64(&state);
    s[2] = splitmix64(&state);
    s[3] = splitmix64(&state);
    have_spare_gauss = 0;
}

double prng_uniform(void) {
    return (next_u64() >> 11) * 0x1.0p-53;
}

static double prng_uniform_state(uint64_t *st) {
    return (next_u64_state(st) >> 11) * 0x1.0p-53;
}

uint32_t prng_uint32(void) {
    return (uint32_t)(next_u64() >> 32);
}

static uint32_t prng_uint32_state(uint64_t *st) {
    return (uint32_t)(next_u64_state(st) >> 32);
}


#define ZIG_N 256
#define ZIG_K 7.273554360975853

static const double ziggurat_x[ZIG_N] = {
    3.9646979206058828e+00, 3.4867393982391743e+00, 3.1656189721379859e+00,
    2.9204368760874883e+00, 2.7195830053763579e+00, 2.5478056643704297e+00,
    2.3961839568055272e+00, 2.2592767705436925e+00, 2.1338065919226661e+00,
    2.0181925990161927e+00, 1.9112899891509811e+00, 1.8121514921606007e+00,
    1.7200574334054360e+00, 1.6344399120123558e+00, 1.5547538382709508e+00,
    1.4804862792004019e+00, 1.4111629640884028e+00, 1.3463437816359489e+00,
    1.2856264539241872e+00, 1.2286422319691539e+00, 1.1750598226911691e+00,
    1.1245789338593914e+00, 1.0769321574883506e+00, 1.0318862132602321e+00,
    0.98923893146087059, 0.94850988069230739, 0.90955087394046136,
    0.87222168954091788, 0.83639040733079929, 0.80193219435518635,
    0.76872799000583919, 0.73666403543781714, 0.70563139529278637,
    0.67552542646760243, 0.64624530969741056, 0.61769356097852134,
    0.58977554929547579, 0.56239909346062284, 0.53547409235480640,
    0.50891221489048910, 0.48262662763740037, 0.45653172777111485,
    0.43054288365840722, 0.40457624380240548, 0.37854854296091668,
    0.35237693085292770, 0.32597890301562108, 0.29927214753069132,
    0.27217439549591668, 0.24460334178992035, 0.21647656964540771,
    0.18771147646755927, 0.15822520490153592, 0.12793461253164632,
    0.096756017322175730, 0.064605083896961696, 0.031397005267994567,
    0.00000000000000000,
};

static const double ziggurat_f[ZIG_N] = {
    1.6671631788860967e-03, 3.0321373872519737e-03, 4.6645646430894051e-03,
    6.4799199102605646e-03, 8.4266106026373838e-03, 1.0479389809449384e-02,
    1.2677725203482698e-02, 1.4998208956354299e-02, 1.7424020464071958e-02,
    1.9946924796058357e-02, 2.2558961602053807e-02, 2.5252641636806918e-02,
    2.8021005560164035e-02, 3.0857673834859013e-02, 3.3756798122806915e-02,
    3.6713099312523988e-02, 3.9721797038098171e-02, 4.2778642104352941e-02,
    4.5879843946174319e-02, 4.9022091957890850e-02, 5.2202480642816505e-02,
    5.5418527035903370e-02, 5.8668187697709399e-02, 6.1949774650719897e-02,
    6.5261960649753156e-02, 6.8603782292309563e-02, 7.1974641703614609e-02,
    7.5374215835851067e-02, 7.8802463467150073e-02, 8.2259629866934790e-02,
    8.5746248824670864e-02, 8.9263142708601187e-02, 9.2811420459372183e-02,
    9.6392484702836961e-02, 1.0000793307751938e-01, 1.0366505864536175e-01,
    1.0736494794364829e-01, 1.1110883232609497e-01, 1.1489789925493789e-01,
    1.1873328914156051e-01, 1.2261608936901607e-01, 1.2654732676088870e-01,
    1.3052795027020893e-01, 1.3455882309273615e-01, 1.3864070534476622e-01,
    1.4277433668021295e-01, 1.4696032757934492e-01, 1.5119914124628609e-01,
    1.5549118496903942e-01, 1.5983680225919562e-01, 1.6423626305934348e-01,
    1.6868975636086310e-01, 1.7319738341445853e-01, 1.7775915018334608e-01,
    1.8237495862054827e-01, 1.8704459836904103e-01, 1.9176773949313332e-01,
    1.9654392415642093e-01, 2.0137255817714469e-01, 2.0625290382102955e-01,
    2.1118407134706059e-01, 2.1616501355134694e-01, 2.2119451769018016e-01,
    2.2627119837023168e-01, 2.3139339155384456e-01, 2.3655924796094352e-01,
    2.4176671779324899e-01, 2.4701354362576396e-01, 2.5229724179249308e-01,
    2.5761509338852999e-01, 2.6296413708045834e-01, 2.6834115084465322e-01,
    2.7374274264274504e-01, 2.7916533215333488e-01, 2.8460514138539257e-01,
    2.9005817729248336e-01, 2.9552022254684045e-01, 3.0098681763146172e-01,
    3.0645324498093253e-01, 3.1191451418652083e-01, 3.1736534416959472e-01,
    3.2280014606939607e-01, 3.2821300614828153e-01, 3.3359766946903504e-01,
    3.3894752364484368e-01, 3.4425558346902693e-01, 3.4951447352929563e-01,
    3.5471641254444677e-01, 3.5985319806054468e-01, 3.6491619047154630e-01,
    3.6989629664856500e-01, 3.7478395197909535e-01, 3.7956909423528293e-01,
    3.8424114309300868e-01, 3.8878897257022948e-01, 3.9320089014601612e-01,
    3.9746461909731504e-01, 4.0156717318681581e-01, 4.0549483730954039e-01,
    4.0923314704583337e-01, 4.1276677367704403e-01, 4.1607950794424604e-01,
    4.1915414612956028e-01, 4.2197247445605261e-01, 4.2451515529346960e-01,
    4.2676171330673085e-01, 4.2869042155290894e-01, 4.3027828514852066e-01,
    4.3150103078030896e-01, 4.3233298962092899e-01, 4.3274698053233372e-01,
    4.3271429344823678e-01, 4.3220457445142065e-01, 4.3118571228608927e-01,
    4.2962372899214775e-01, 4.2748275847100165e-01, 4.2472493462136662e-01,
    4.2131026936302518e-01, 4.1719653508096206e-01, 4.1233914352825018e-01,
    4.0669101804621821e-01, 4.0020236434567608e-01, 3.9282053556904966e-01,
    3.8448990230935818e-01, 3.7515161289536605e-01, 3.6474334167889177e-01,
    3.5319902064503441e-01, 3.4044856156053377e-01, 3.2641756303013504e-01,
    3.1102697954033835e-01, 2.9419284011854755e-01, 2.7582586153705969e-01,
    2.5583104121904062e-01, 2.3410713463612499e-01, 2.1054605853008394e-01,
    1.8503204722822094e-01, 1.5744073509744012e-01, 1.2763781721692898e-01,
    9.5478298457486811e-02, 6.0869525491158086e-02, 2.3067959778369141e-02,
    0.00000000000000000,
};

static const double ziggurat_fabsmin = 9.1838303002622427e-03;

static double ziggurat_sample(uint64_t *st, int *have_spare, double *spare) {
    if (*have_spare) {
        *have_spare = 0;
        return *spare;
    }
    while (1) {
        unsigned int ui = prng_uint32_state(st);
        int i = (int)(ui & 0xFF);
        double x = prng_uniform_state(st) * ziggurat_x[i];
        double y = prng_uniform_state(st) * ziggurat_f[i];
        if (i != 0) {
            if (y < ziggurat_f[i + 1] + (ziggurat_f[i] - ziggurat_f[i + 1]) * x / ziggurat_x[i]) {
                *spare = (ui & 0x200) ? -x : x;
                *have_spare = 1;
                return x;
            }
        } else {
            if (y < ziggurat_fabsmin * exp(-0.5 * x * x)) {
                *spare = (ui & 0x200) ? -x : x;
                *have_spare = 1;
                return x;
            }
        }
    }
}

double prng_gauss(void) {
    return ziggurat_sample(s, &have_spare_gauss, &spare_gauss);
}

void prng_init_parallel(uint32_t seed) {
    uint64_t state = (uint64_t)seed;
    if (state == 0u) state = 1u;

    int nthreads = omp_get_max_threads();
    if (nthreads > MAX_OMP_THREADS) nthreads = MAX_OMP_THREADS;

    for (int t = 0; t < nthreads; t++) {
        prng_threads[t].s[0] = splitmix64(&state);
        prng_threads[t].s[1] = splitmix64(&state);
        prng_threads[t].s[2] = splitmix64(&state);
        prng_threads[t].s[3] = splitmix64(&state);
        prng_threads[t].have_spare_gauss = 0;
        prng_threads[t].spare_gauss = 0.0;
    }
    prng_parallel_initialized = 1;
}

double prng_gauss_parallel(void) {
    int tid = omp_get_thread_num();
    PrngThreadState *ts = &prng_threads[tid];
    return ziggurat_sample(ts->s, &ts->have_spare_gauss, &ts->spare_gauss);
}
