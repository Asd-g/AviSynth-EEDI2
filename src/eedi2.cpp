#include <algorithm>
#include <memory>

#include "avisynth.h"

#define VS_RESTRICT __restrict

class EEDI2 : public GenericVideoFilter
{
    int _mthresh, _lthresh, _vthresh, _estr, _dstr, _maxd, _field, _map, _pp;
    unsigned fieldS, nt4, nt7, nt8, nt13, nt19;
    int8_t* limlut;
    int16_t* limlut2;
    int* cx2, * cy2, * cxy, * tmpc;
    VideoInfo vi2;
    bool has_at_least_v8;

    template<typename T>
    void buildEdgeMask(PVideoFrame& src, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void erode(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void dilate(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void removeSmallHorzGaps(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void calcDirections(PVideoFrame& src, PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void filterDirMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void expandDirMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void filterMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void markDirections2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void filterDirMap2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void expandDirMap2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void fillGaps2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void interpolateLattice(PVideoFrame& omsk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void postProcess(PVideoFrame& nmsk, PVideoFrame& omsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept;
    template<typename T>
    void gaussianBlur1(PVideoFrame& src, PVideoFrame& tmp, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept;
    template<typename T>
    void calcDerivatives(PVideoFrame& src, int* VS_RESTRICT x2, int* VS_RESTRICT y2, int* VS_RESTRICT xy,
        const int plane, const unsigned bitsPerSample, IScriptEnvironment* env) noexcept;
    template<typename T>
    void postProcessCorner(PVideoFrame& msk, PVideoFrame& dst, const int* x2, const int* y2, const int* xy,
        const int plane, const unsigned field, const unsigned bitsPerSample, IScriptEnvironment* env) noexcept;
    template<typename T>
    void process(PVideoFrame& src, PVideoFrame& dst, PVideoFrame& msk, PVideoFrame& tmp,
        PVideoFrame& dst2, PVideoFrame& dst2M, PVideoFrame& tmp2, PVideoFrame& tmp2_2, PVideoFrame& msk2,
        const unsigned field, IScriptEnvironment* env) noexcept;

public:
    EEDI2(PClip _child, int mthresh, int lthresh, int vthresh, int estr, int dstr, int maxd, int field, int map, int nt, int pp, IScriptEnvironment* env);
    PVideoFrame __stdcall GetFrame(int n, IScriptEnvironment* env);
    int __stdcall SetCacheHints(int cachehints, int frame_range)
    {
        return cachehints == CACHE_GET_MTMODE ? MT_MULTI_INSTANCE : 0;
    }
    ~EEDI2();
};

template<typename T>
void EEDI2::buildEdgeMask(PVideoFrame& src, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift = vi.BitsPerComponent() - 8;
    const T ten = 10 << shift;

    const unsigned stride = src->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = src->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = src->GetHeight(plane);
    const T* srcp = reinterpret_cast<const T*>(src->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memset(dstp, 0, dst->GetPitch(plane) * static_cast<int64_t>(height));

    srcp += stride;
    dstp += dst_stride;

    const T* srcpp = srcp - stride;
    const T* srcpn = srcp + stride;

    for (unsigned y = 1; y < height - 1; y++)
    {
        for (unsigned x = 1; x < width - 1; x++)
        {
            if ((std::abs(srcpp[x] - srcp[x]) < ten && std::abs(srcp[x] - srcpn[x]) < ten && std::abs(srcpp[x] - srcpn[x]) < ten) ||
                (std::abs(srcpp[x - 1] - srcp[x - 1]) < ten && std::abs(srcp[x - 1] - srcpn[x - 1]) < ten && std::abs(srcpp[x - 1] - srcpn[x - 1]) < ten &&
                    std::abs(srcpp[x + 1] - srcp[x + 1]) < ten && std::abs(srcp[x + 1] - srcpn[x + 1]) < ten && std::abs(srcpp[x + 1] - srcpn[x + 1]) < ten))
                continue;

            const unsigned sum = (srcpp[x - 1] + srcpp[x] + srcpp[x + 1] +
                srcp[x - 1] + srcp[x] + srcp[x + 1] +
                srcpn[x - 1] + srcpn[x] + srcpn[x + 1]) >> shift;
            const unsigned sumsq = (srcpp[x - 1] >> shift) * (srcpp[x - 1] >> shift) + (srcpp[x] >> shift) * (srcpp[x] >> shift) + (srcpp[x + 1] >> shift) * (srcpp[x + 1] >> shift) +
                (srcp[x - 1] >> shift) * (srcp[x - 1] >> shift) + (srcp[x] >> shift) * (srcp[x] >> shift) + (srcp[x + 1] >> shift) * (srcp[x + 1] >> shift) +
                (srcpn[x - 1] >> shift) * (srcpn[x - 1] >> shift) + (srcpn[x] >> shift) * (srcpn[x] >> shift) + (srcpn[x + 1] >> shift) * (srcpn[x + 1] >> shift);
            if (9 * sumsq - sum * sum < static_cast<unsigned>(_vthresh))
                continue;

            const unsigned Ix = std::abs(srcp[x + 1] - srcp[x - 1]) >> shift;
            const unsigned Iy = std::max({ std::abs(srcpp[x] - srcpn[x]), std::abs(srcpp[x] - srcp[x]), std::abs(srcp[x] - srcpn[x]) }) >> shift;
            if (Ix * Ix + Iy * Iy >= static_cast<unsigned>(_mthresh)) {
                dstp[x] = peak;
                continue;
            }

            const unsigned Ixx = std::abs(srcp[x - 1] - 2 * srcp[x] + srcp[x + 1]) >> shift;
            const unsigned Iyy = std::abs(srcpp[x] - 2 * srcp[x] + srcpn[x]) >> shift;
            if (Ixx + Iyy >= static_cast<unsigned>(_lthresh))
                dstp[x] = peak;
        }

        srcpp += stride;
        srcp += stride;
        srcpn += stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::erode(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T peak = (1 << vi.BitsPerComponent()) - 1;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), msk->GetReadPtr(plane), static_cast<int64_t>(msk->GetPitch(plane)) * msk->GetHeight(plane));

    mskp += stride;
    dstp += dst_stride;

    const T* mskpp = mskp - stride;
    const T* mskpn = mskp + stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (mskp[x] != peak)
                continue;

            unsigned count = 0;

            if (mskpp[x - 1] == peak)
                count++;
            if (mskpp[x] == peak)
                count++;
            if (mskpp[x + 1] == peak)
                count++;
            if (mskp[x - 1] == peak)
                count++;
            if (mskp[x + 1] == peak)
                count++;
            if (mskpn[x - 1] == peak)
                count++;
            if (mskpn[x] == peak)
                count++;
            if (mskpn[x + 1] == peak)
                count++;

            if (count < static_cast<unsigned>(_estr))
                dstp[x] = 0;
        }

        mskpp += stride;
        mskp += stride;
        mskpn += stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::dilate(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T peak = (1 << vi.BitsPerComponent()) - 1;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), msk->GetReadPtr(plane), static_cast<int64_t>(msk->GetPitch(plane)) * msk->GetHeight(plane));

    mskp += stride;
    dstp += dst_stride;

    const T* mskpp = mskp - stride;
    const T* mskpn = mskp + stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (mskp[x] != 0)
                continue;

            unsigned count = 0;

            if (mskpp[x - 1] == peak)
                count++;
            if (mskpp[x] == peak)
                count++;
            if (mskpp[x + 1] == peak)
                count++;
            if (mskp[x - 1] == peak)
                count++;
            if (mskp[x + 1] == peak)
                count++;
            if (mskpn[x - 1] == peak)
                count++;
            if (mskpn[x] == peak)
                count++;
            if (mskpn[x + 1] == peak)
                count++;

            if (count >= static_cast<unsigned>(_dstr))
                dstp[x] = peak;
        }

        mskpp += stride;
        mskp += stride;
        mskpn += stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::removeSmallHorzGaps(PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T peak = (1 << vi.BitsPerComponent()) - 1;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), msk->GetReadPtr(plane), static_cast<int64_t>(msk->GetPitch(plane)) * msk->GetHeight(plane));

    mskp += stride;
    dstp += dst_stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 3; x < width - 3; x++) {
            if (mskp[x]) {
                if (mskp[x - 3] || mskp[x - 2] || mskp[x - 1] ||
                    mskp[x + 1] || mskp[x + 2] || mskp[x + 3])
                    continue;
                dstp[x] = 0;
            }
            else {
                if ((mskp[x + 1] && (mskp[x - 1] || mskp[x - 2] || mskp[x - 3])) ||
                    (mskp[x + 2] && (mskp[x - 1] || mskp[x - 2])) ||
                    (mskp[x + 3] && mskp[x - 1]))
                    dstp[x] = peak;
            }
        }

        mskp += stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::calcDirections(PVideoFrame& src, PVideoFrame& msk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = src->GetPitch(plane) / sizeof(T);
    const unsigned msk_stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const int width = src->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = src->GetHeight(plane);
    const T* srcp = reinterpret_cast<const T*>(src->GetReadPtr(plane));
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    std::fill_n(dstp, dst_stride * height, peak);

    srcp += stride;
    mskp += msk_stride;
    dstp += dst_stride;

    const T* src2p = srcp - stride * static_cast<int64_t>(2);
    const T* srcpp = srcp - stride;
    const T* srcpn = srcp + stride;
    const T* src2n = srcp + stride * static_cast<int64_t>(2);
    const T* mskpp = mskp - msk_stride;
    const T* mskpn = mskp + msk_stride;

    const int maxd = _maxd >> (plane != PLANAR_Y ? vi.GetPlaneWidthSubsampling(PLANAR_U) : 0);

    for (unsigned y = 1; y < height - 1; y++) {
        for (int x = 1; x < width - 1; x++) {
            if (mskp[x] != peak || (mskp[x - 1] != peak && mskp[x + 1] != peak))
                continue;

            const int uStart = std::max(-x + 1, -maxd);
            const int uStop = std::min(width - 2 - x, maxd);
            const unsigned min0 = std::abs(srcp[x] - srcpn[x]) + std::abs(srcp[x] - srcpp[x]);
            unsigned minA = std::min(nt19, min0 * 9);
            unsigned minB = std::min(nt13, min0 * 6);
            unsigned minC = minA;
            unsigned minD = minB;
            unsigned minE = minB;
            int dirA = -5000, dirB = -5000, dirC = -5000, dirD = -5000, dirE = -5000;

            for (int u = uStart; u <= uStop; u++) {
                if ((y == 1 || mskpp[x - 1 + u] == peak || mskpp[x + u] == peak || mskpp[x + 1 + u] == peak) &&
                    (y == height - 2 || mskpn[x - 1 - u] == peak || mskpn[x - u] == peak || mskpn[x + 1 - u] == peak)) {
                    const unsigned diffsn = std::abs(srcp[x - 1] - srcpn[x - 1 - u]) + std::abs(srcp[x] - srcpn[x - u]) + std::abs(srcp[x + 1] - srcpn[x + 1 - u]);
                    const unsigned diffsp = std::abs(srcp[x - 1] - srcpp[x - 1 + u]) + std::abs(srcp[x] - srcpp[x + u]) + std::abs(srcp[x + 1] - srcpp[x + 1 + u]);
                    const unsigned diffps = std::abs(srcpp[x - 1] - srcp[x - 1 - u]) + std::abs(srcpp[x] - srcp[x - u]) + std::abs(srcpp[x + 1] - srcp[x + 1 - u]);
                    const unsigned diffns = std::abs(srcpn[x - 1] - srcp[x - 1 + u]) + std::abs(srcpn[x] - srcp[x + u]) + std::abs(srcpn[x + 1] - srcp[x + 1 + u]);
                    const unsigned diff = diffsn + diffsp + diffps + diffns;
                    unsigned diffD = diffsp + diffns;
                    unsigned diffE = diffsn + diffps;

                    if (diff < minB) {
                        dirB = u;
                        minB = diff;
                    }

                    if (y > 1) {
                        const unsigned diff2pp = std::abs(src2p[x - 1] - srcpp[x - 1 - u]) + std::abs(src2p[x] - srcpp[x - u]) + std::abs(src2p[x + 1] - srcpp[x + 1 - u]);
                        const unsigned diffp2p = std::abs(srcpp[x - 1] - src2p[x - 1 + u]) + std::abs(srcpp[x] - src2p[x + u]) + std::abs(srcpp[x + 1] - src2p[x + 1 + u]);
                        const unsigned diffA = diff + diff2pp + diffp2p;
                        diffD += diffp2p;
                        diffE += diff2pp;

                        if (diffA < minA) {
                            dirA = u;
                            minA = diffA;
                        }
                    }

                    if (y < height - 2) {
                        const unsigned diff2nn = std::abs(src2n[x - 1] - srcpn[x - 1 + u]) + std::abs(src2n[x] - srcpn[x + u]) + std::abs(src2n[x + 1] - srcpn[x + 1 + u]);
                        const unsigned diffn2n = std::abs(srcpn[x - 1] - src2n[x - 1 - u]) + std::abs(srcpn[x] - src2n[x - u]) + std::abs(srcpn[x + 1] - src2n[x + 1 - u]);
                        const unsigned diffC = diff + diff2nn + diffn2n;
                        diffD += diff2nn;
                        diffE += diffn2n;

                        if (diffC < minC) {
                            dirC = u;
                            minC = diffC;
                        }
                    }

                    if (diffD < minD) {
                        dirD = u;
                        minD = diffD;
                    }

                    if (diffE < minE) {
                        dirE = u;
                        minE = diffE;
                    }
                }
            }

            int order[5];
            unsigned k = 0;

            if (dirA != -5000)
                order[k++] = dirA;
            if (dirB != -5000)
                order[k++] = dirB;
            if (dirC != -5000)
                order[k++] = dirC;
            if (dirD != -5000)
                order[k++] = dirD;
            if (dirE != -5000)
                order[k++] = dirE;

            if (k > 1) {
                std::sort(order, order + k);

                const int mid = (k & 1) ? order[k / 2] : (order[(k - 1) / 2] + order[k / 2] + 1) / 2;
                const int lim = std::max(limlut[std::abs(mid)] / 4, 2);
                int sum = 0;
                unsigned count = 0;

                for (unsigned i = 0; i < k; i++) {
                    if (std::abs(order[i] - mid) <= lim) {
                        sum += order[i];
                        count++;
                    }
                }

                dstp[x] = (count > 1) ? neutral + (static_cast<int>(static_cast<float>(sum) / count) << shift2) : neutral;
            }
            else {
                dstp[x] = neutral;
            }
        }

        src2p += stride;
        srcpp += stride;
        srcp += stride;
        srcpn += stride;
        src2n += stride;
        mskpp += msk_stride;
        mskp += msk_stride;
        mskpn += msk_stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::filterDirMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride;
    dmskp += dmsk_stride;
    dstp += dst_stride;

    const T* dmskpp = dmskp - dmsk_stride;
    const T* dmskpn = dmskp + dmsk_stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (mskp[x] != peak)
                continue;

            int order[9];
            unsigned u = 0;

            if (dmskpp[x - 1] != peak)
                order[u++] = dmskpp[x - 1];
            if (dmskpp[x] != peak)
                order[u++] = dmskpp[x];
            if (dmskpp[x + 1] != peak)
                order[u++] = dmskpp[x + 1];
            if (dmskp[x - 1] != peak)
                order[u++] = dmskp[x - 1];
            if (dmskp[x] != peak)
                order[u++] = dmskp[x];
            if (dmskp[x + 1] != peak)
                order[u++] = dmskp[x + 1];
            if (dmskpn[x - 1] != peak)
                order[u++] = dmskpn[x - 1];
            if (dmskpn[x] != peak)
                order[u++] = dmskpn[x];
            if (dmskpn[x + 1] != peak)
                order[u++] = dmskpn[x + 1];

            if (u < 4) {
                dstp[x] = peak;
                continue;
            }

            std::sort(order, order + u);

            const int mid = (u & 1) ? order[u / 2] : (order[(u - 1) / 2] + order[u / 2] + 1) / 2;
            const int lim = limlut2[std::abs(mid - neutral) >> shift2];
            int sum = 0;
            unsigned count = 0;

            for (unsigned i = 0; i < u; i++) {
                if (std::abs(order[i] - mid) <= lim) {
                    sum += order[i];
                    count++;
                }
            }

            if (count < 4 || (count < 5 && dmskp[x] == peak)) {
                dstp[x] = peak;
                continue;
            }

            dstp[x] = static_cast<int>(static_cast<float>(sum + mid) / (count + 1) + 0.5f);
        }

        mskp += stride;
        dmskpp += dmsk_stride;
        dmskp += dmsk_stride;
        dmskpn += dmsk_stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::expandDirMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride;
    dmskp += dmsk_stride;
    dstp += dst_stride;

    const T* dmskpp = dmskp - dmsk_stride;
    const T* dmskpn = dmskp + dmsk_stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (dmskp[x] != peak || mskp[x] != peak)
                continue;

            int order[9];
            unsigned u = 0;

            if (dmskpp[x - 1] != peak)
                order[u++] = dmskpp[x - 1];
            if (dmskpp[x] != peak)
                order[u++] = dmskpp[x];
            if (dmskpp[x + 1] != peak)
                order[u++] = dmskpp[x + 1];
            if (dmskp[x - 1] != peak)
                order[u++] = dmskp[x - 1];
            if (dmskp[x + 1] != peak)
                order[u++] = dmskp[x + 1];
            if (dmskpn[x - 1] != peak)
                order[u++] = dmskpn[x - 1];
            if (dmskpn[x] != peak)
                order[u++] = dmskpn[x];
            if (dmskpn[x + 1] != peak)
                order[u++] = dmskpn[x + 1];

            if (u < 5)
                continue;

            std::sort(order, order + u);

            const int mid = (u & 1) ? order[u / 2] : (order[(u - 1) / 2] + order[u / 2] + 1) / 2;
            const int lim = limlut2[std::abs(mid - neutral) >> shift2];
            int sum = 0;
            unsigned count = 0;

            for (unsigned i = 0; i < u; i++) {
                if (std::abs(order[i] - mid) <= lim) {
                    sum += order[i];
                    count++;
                }
            }

            if (count < 5)
                continue;

            dstp[x] = static_cast<int>(static_cast<float>(sum + mid) / (count + 1) + 0.5f);
        }

        mskp += stride;
        dmskpp += dmsk_stride;
        dmskp += dmsk_stride;
        dmskpn += dmsk_stride;
        dstp += dst_stride;
    }
}

template<typename T>
void EEDI2::filterMap(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift = vi.BitsPerComponent() - 8;
    const int twleve = 12 << shift;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride;
    dmskp += dmsk_stride;
    dstp += dst_stride;

    const T* dmskpp = dmskp - dmsk_stride;
    const T* dmskpn = dmskp + dmsk_stride;

    for (unsigned y = 1; y < height - 1; y++) {
        for (int x = 1; x < static_cast<int>(width) - 1; x++) {
            if (dmskp[x] == peak || mskp[x] != peak)
                continue;

            int dir = (dmskp[x] - neutral) / 4;
            const int lim = std::max(std::abs(dir) * 2, twleve);
            dir >>= shift;
            bool ict = false, icb = false;

            if (dir < 0) {
                for (int j = std::max(-x, dir); j <= 0; j++) {
                    if ((std::abs(dmskpp[x + j] - dmskp[x]) > lim && dmskpp[x + j] != peak) ||
                        (dmskp[x + j] == peak && dmskpp[x + j] == peak) ||
                        (std::abs(dmskp[x + j] - dmskp[x]) > lim && dmskp[x + j] != peak)) {
                        ict = true;
                        break;
                    }
                }
            }
            else {
                for (int j = 0; j <= std::min(static_cast<int>(width) - x - 1, dir); j++) {
                    if ((std::abs(dmskpp[x + j] - dmskp[x]) > lim && dmskpp[x + j] != peak) ||
                        (dmskp[x + j] == peak && dmskpp[x + j] == peak) ||
                        (std::abs(dmskp[x + j] - dmskp[x]) > lim && dmskp[x + j] != peak)) {
                        ict = true;
                        break;
                    }
                }
            }

            if (ict) {
                if (dir < 0) {
                    for (int j = 0; j <= std::min(static_cast<int>(width) - x - 1, std::abs(dir)); j++) {
                        if ((std::abs(dmskpn[x + j] - dmskp[x]) > lim && dmskpn[x + j] != peak) ||
                            (dmskpn[x + j] == peak && dmskp[x + j] == peak) ||
                            (std::abs(dmskp[x + j] - dmskp[x]) > lim && dmskp[x + j] != peak)) {
                            icb = true;
                            break;
                        }
                    }
                }
                else {
                    for (int j = std::max(-x, -dir); j <= 0; j++) {
                        if ((std::abs(dmskpn[x + j] - dmskp[x]) > lim && dmskpn[x + j] != peak) ||
                            (dmskpn[x + j] == peak && dmskp[x + j] == peak) ||
                            (std::abs(dmskp[x + j] - dmskp[x]) > lim && dmskp[x + j] != peak)) {
                            icb = true;
                            break;
                        }
                    }
                }

                if (icb)
                    dstp[x] = peak;
            }
        }

        mskp += stride;
        dmskpp += dmsk_stride;
        dmskp += dmsk_stride;
        dmskpn += dmsk_stride;
        dstp += dst_stride;
    }
}

static void upscaleBy2(PVideoFrame& src, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    env->BitBlt(dst->GetWritePtr(plane) + dst->GetPitch(plane) * (static_cast<int64_t>(1) - field), dst->GetPitch(plane) * 2,
        src->GetReadPtr(plane), src->GetPitch(plane), src->GetRowSize(plane), src->GetHeight(plane));
}

template<typename T>
void EEDI2::markDirections2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    std::fill_n(dstp, dst_stride * height, peak);

    mskp += stride * (static_cast<int64_t>(1) - field);
    dmskp += dmsk_stride * (static_cast<int64_t>(1) - field);
    dstp += dst_stride * (static_cast<int64_t>(2) - field);

    const T* mskpn = mskp + stride * static_cast<int64_t>(2);
    const T* dmskpn = dmskp + dmsk_stride * static_cast<int64_t>(2);

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (mskp[x] != peak && mskpn[x] != peak)
                continue;

            int order[6];
            unsigned v = 0;

            if (dmskp[x - 1] != peak)
                order[v++] = dmskp[x - 1];
            if (dmskp[x] != peak)
                order[v++] = dmskp[x];
            if (dmskp[x + 1] != peak)
                order[v++] = dmskp[x + 1];
            if (dmskpn[x - 1] != peak)
                order[v++] = dmskpn[x - 1];
            if (dmskpn[x] != peak)
                order[v++] = dmskpn[x];
            if (dmskpn[x + 1] != peak)
                order[v++] = dmskpn[x + 1];

            if (v < 3) {
                continue;
            }
            else {
                std::sort(order, order + v);

                const int mid = (v & 1) ? order[v / 2] : (order[(v - 1) / 2] + order[v / 2] + 1) / 2;
                const int lim = limlut2[std::abs(mid - neutral) >> shift2];
                int sum = 0;
                unsigned count = 0;

                unsigned u = 0;
                if (std::abs(dmskp[x - 1] - dmskpn[x - 1]) <= lim || dmskp[x - 1] == peak || dmskpn[x - 1] == peak)
                    u++;
                if (std::abs(dmskp[x] - dmskpn[x]) <= lim || dmskp[x] == peak || dmskpn[x] == peak)
                    u++;
                if (std::abs(dmskp[x + 1] - dmskpn[x - 1]) <= lim || dmskp[x + 1] == peak || dmskpn[x + 1] == peak)
                    u++;
                if (u < 2)
                    continue;

                for (unsigned i = 0; i < v; i++) {
                    if (std::abs(order[i] - mid) <= lim) {
                        sum += order[i];
                        count++;
                    }
                }

                if (count < v - 2 || count < 2)
                    continue;

                dstp[x] = static_cast<int>(static_cast<float>(sum + mid) / (count + 1) + 0.5f);
            }
        }

        mskp += stride * static_cast<int64_t>(2);
        mskpn += stride * static_cast<int64_t>(2);
        dmskp += dmsk_stride * static_cast<int64_t>(2);
        dmskpn += dmsk_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::filterDirMap2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride * (static_cast<int64_t>(1) - field);
    dmskp += dmsk_stride * (static_cast<int64_t>(2) - field);
    dstp += dst_stride * (static_cast<int64_t>(2) - field);

    const T* mskpn = mskp + stride * static_cast<int64_t>(2);
    const T* dmskpp = dmskp - dmsk_stride * static_cast<int64_t>(2);
    const T* dmskpn = dmskp + dmsk_stride * static_cast<int64_t>(2);

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (mskp[x] != peak && mskpn[x] != peak)
                continue;

            int order[9];
            unsigned u = 0;

            if (y > 1) {
                if (dmskpp[x - 1] != peak)
                    order[u++] = dmskpp[x - 1];
                if (dmskpp[x] != peak)
                    order[u++] = dmskpp[x];
                if (dmskpp[x + 1] != peak)
                    order[u++] = dmskpp[x + 1];
            }

            if (dmskp[x - 1] != peak)
                order[u++] = dmskp[x - 1];
            if (dmskp[x] != peak)
                order[u++] = dmskp[x];
            if (dmskp[x + 1] != peak)
                order[u++] = dmskp[x + 1];

            if (y < height - 2) {
                if (dmskpn[x - 1] != peak)
                    order[u++] = dmskpn[x - 1];
                if (dmskpn[x] != peak)
                    order[u++] = dmskpn[x];
                if (dmskpn[x + 1] != peak)
                    order[u++] = dmskpn[x + 1];
            }

            if (u < 4) {
                dstp[x] = peak;
                continue;
            }

            std::sort(order, order + u);

            const int mid = (u & 1) ? order[u / 2] : (order[(u - 1) / 2] + order[u / 2] + 1) / 2;
            const int lim = limlut2[std::abs(mid - neutral) >> shift2];
            int sum = 0;
            unsigned count = 0;

            for (unsigned i = 0; i < u; i++) {
                if (std::abs(order[i] - mid) <= lim) {
                    sum += order[i];
                    count++;
                }
            }

            if (count < 4 || (count < 5 && dmskp[x] == peak)) {
                dstp[x] = peak;
                continue;
            }

            dstp[x] = static_cast<int>(static_cast<float>(sum + mid) / (count + 1) + 0.5f);
        }

        mskp += stride * static_cast<int64_t>(2);
        mskpn += stride * static_cast<int64_t>(2);
        dmskpp += dmsk_stride * static_cast<int64_t>(2);
        dmskp += dmsk_stride * static_cast<int64_t>(2);
        dmskpn += dmsk_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::expandDirMap2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride * (static_cast<int64_t>(1) - field);
    dmskp += dmsk_stride * (static_cast<int64_t>(2) - field);
    dstp += dst_stride * (static_cast<int64_t>(2) - field);

    const T* mskpn = mskp + stride * static_cast<int64_t>(2);
    const T* dmskpp = dmskp - dmsk_stride * static_cast<int64_t>(2);
    const T* dmskpn = dmskp + dmsk_stride * static_cast<int64_t>(2);

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (dmskp[x] != peak || (mskp[x] != peak && mskpn[x] != peak))
                continue;

            int order[9];
            unsigned u = 0;

            if (y > 1) {
                if (dmskpp[x - 1] != peak)
                    order[u++] = dmskpp[x - 1];
                if (dmskpp[x] != peak)
                    order[u++] = dmskpp[x];
                if (dmskpp[x + 1] != peak)
                    order[u++] = dmskpp[x + 1];
            }

            if (dmskp[x - 1] != peak)
                order[u++] = dmskp[x - 1];
            if (dmskp[x + 1] != peak)
                order[u++] = dmskp[x + 1];

            if (y < height - 2) {
                if (dmskpn[x - 1] != peak)
                    order[u++] = dmskpn[x - 1];
                if (dmskpn[x] != peak)
                    order[u++] = dmskpn[x];
                if (dmskpn[x + 1] != peak)
                    order[u++] = dmskpn[x + 1];
            }

            if (u < 5)
                continue;

            std::sort(order, order + u);

            const int mid = (u & 1) ? order[u / 2] : (order[(u - 1) / 2] + order[u / 2] + 1) / 2;
            const int lim = limlut2[std::abs(mid - neutral) >> shift2];
            int sum = 0;
            unsigned count = 0;

            for (unsigned i = 0; i < u; i++) {
                if (std::abs(order[i] - mid) <= lim) {
                    sum += order[i];
                    count++;
                }
            }

            if (count < 5)
                continue;

            dstp[x] = static_cast<int>(static_cast<float>(sum + mid) / (count + 1) + 0.5f);
        }

        mskp += stride * static_cast<int64_t>(2);
        mskpn += stride * static_cast<int64_t>(2);
        dmskpp += dmsk_stride * static_cast<int64_t>(2);
        dmskp += dmsk_stride * static_cast<int64_t>(2);
        dmskpn += dmsk_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::fillGaps2X(PVideoFrame& msk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift = vi.BitsPerComponent() - 8;
    const T shift2 = 2 + shift;
    const int eight = 8 << shift;
    const int twenty = 20 << shift;
    const int fiveHundred = 500 << shift;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    const T* dmskp = reinterpret_cast<const T*>(dmsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    memcpy(dst->GetWritePtr(plane), dmsk->GetReadPtr(plane), static_cast<int64_t>(dmsk->GetPitch(plane)) * dmsk->GetHeight(plane));

    mskp += stride * (static_cast<int64_t>(1) - field);
    dmskp += dmsk_stride * (static_cast<int64_t>(2) - field);
    dstp += dst_stride * (static_cast<int64_t>(2) - field);

    const T* mskpp = mskp - stride * static_cast<int64_t>(2);
    const T* mskpn = mskp + stride * static_cast<int64_t>(2);
    const T* mskpnn = mskpn + stride * static_cast<int64_t>(2);
    const T* dmskpp = dmskp - dmsk_stride * static_cast<int64_t>(2);
    const T* dmskpn = dmskp + dmsk_stride * static_cast<int64_t>(2);

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (unsigned x = 1; x < width - 1; x++) {
            if (dmskp[x] != peak || (mskp[x] != peak && mskpn[x] != peak))
                continue;

            unsigned u = x - 1, v = x + 1;
            int back = fiveHundred, forward = -fiveHundred;

            while (u) {
                if (dmskp[u] != peak) {
                    back = dmskp[u];
                    break;
                }
                if (mskp[u] != peak && mskpn[u] != peak)
                    break;
                u--;
            }

            while (v < width) {
                if (dmskp[v] != peak) {
                    forward = dmskp[v];
                    break;
                }
                if (mskp[v] != peak && mskpn[v] != peak)
                    break;
                v++;
            }

            bool tc = true, bc = true;
            int mint = fiveHundred, maxt = -twenty;
            int minb = fiveHundred, maxb = -twenty;

            for (unsigned j = u; j <= v; j++) {
                if (tc) {
                    if (y <= 2 || dmskpp[j] == peak || (mskpp[j] != peak && mskp[j] != peak)) {
                        tc = false;
                        mint = maxt = twenty;
                    }
                    else {
                        if (dmskpp[j] < mint)
                            mint = dmskpp[j];
                        if (dmskpp[j] > maxt)
                            maxt = dmskpp[j];
                    }
                }

                if (bc) {
                    if (y >= height - 3 || dmskpn[j] == peak || (mskpn[j] != peak && mskpnn[j] != peak)) {
                        bc = false;
                        minb = maxb = twenty;
                    }
                    else {
                        if (dmskpn[j] < minb)
                            minb = dmskpn[j];
                        if (dmskpn[j] > maxb)
                            maxb = dmskpn[j];
                    }
                }
            }

            if (maxt == -twenty)
                maxt = mint = twenty;
            if (maxb == -twenty)
                maxb = minb = twenty;

            const int thresh = std::max({ std::max(std::abs(forward - neutral), std::abs(back - neutral)) / 4, eight, std::abs(mint - maxt), std::abs(minb - maxb) });
            const unsigned lim = std::min(std::max(std::abs(forward - neutral), std::abs(back - neutral)) >> shift2, 6);
            if (std::abs(forward - back) <= thresh && (v - u - 1 <= lim || tc || bc)) {
                const float step = static_cast<float>(forward - back) / (v - u);
                for (unsigned j = 0; j < v - u - 1; j++)
                    dstp[u + j + 1] = back + static_cast<int>(j * step + 0.5);
            }
        }

        mskpp += stride * static_cast<int64_t>(2);
        mskp += stride * static_cast<int64_t>(2);
        mskpn += stride * static_cast<int64_t>(2);
        mskpnn += stride * static_cast<int64_t>(2);
        dmskpp += dmsk_stride * static_cast<int64_t>(2);
        dmskp += dmsk_stride * static_cast<int64_t>(2);
        dmskpn += dmsk_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::interpolateLattice(PVideoFrame& omsk, PVideoFrame& dmsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift = vi.BitsPerComponent() - 8;
    const T shift2 = 2 + shift;
    const T three = 3 << shift;
    const T nine = 9 << shift;

    const unsigned stride = omsk->GetPitch(plane) / sizeof(T);
    const unsigned dmsk_stride = dmsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const int width = omsk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = omsk->GetHeight(plane);
    const T* omskp = reinterpret_cast<const T*>(omsk->GetReadPtr(plane));
    T* VS_RESTRICT dmskp = reinterpret_cast<T*>(dmsk->GetWritePtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    if (field)
        std::copy_n(dstp + dst_stride * (height - static_cast<int64_t>(2)), width, dstp + dst_stride * (height - static_cast<int64_t>(1)));
    else
        std::copy_n(dstp + dst_stride, width, dstp);

    omskp += stride * (static_cast<int64_t>(1) - field);
    dmskp += dmsk_stride * (static_cast<int64_t>(2) - field);
    dstp += dst_stride * (static_cast<int64_t>(1) - field);

    const T* omskn = omskp + stride * static_cast<int64_t>(2);
    T* VS_RESTRICT dstpn = dstp + dst_stride;
    const T* dstpnn = dstp + dst_stride * static_cast<int64_t>(2);

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (int x = 0; x < width; x++) {
            int dir = dmskp[x];
            const int lim = limlut2[std::abs(dir - neutral) >> shift2];

            if (dir == peak || (std::abs(dmskp[x] - dmskp[x - 1]) > lim && std::abs(dmskp[x] - dmskp[x + 1]) > lim)) {
                dstpn[x] = (dstp[x] + dstpnn[x] + 1) / 2;
                if (dir != peak)
                    dmskp[x] = neutral;
                continue;
            }

            if (lim < nine) {
                const unsigned sum = (dstp[x - 1] + dstp[x] + dstp[x + 1] +
                    dstpnn[x - 1] + dstpnn[x] + dstpnn[x + 1]) >> shift;
                const unsigned sumsq = (dstp[x - 1] >> shift) * (dstp[x - 1] >> shift) + (dstp[x] >> shift) * (dstp[x] >> shift) + (dstp[x + 1] >> shift) * (dstp[x + 1] >> shift) +
                    (dstpnn[x - 1] >> shift) * (dstpnn[x - 1] >> shift) + (dstpnn[x] >> shift) * (dstpnn[x] >> shift) + (dstpnn[x + 1] >> shift) * (dstpnn[x + 1] >> shift);
                if (6 * sumsq - sum * sum < 576) {
                    dstpn[x] = (dstp[x] + dstpnn[x] + 1) / 2;
                    dmskp[x] = peak;
                    continue;
                }
            }

            if (x > 1 && x < width - 2 &&
                ((dstp[x] < std::max(dstp[x - 2], dstp[x - 1]) - three && dstp[x] < std::max(dstp[x + 2], dstp[x + 1]) - three &&
                    dstpnn[x] < std::max(dstpnn[x - 2], dstpnn[x - 1]) - three && dstpnn[x] < std::max(dstpnn[x + 2], dstpnn[x + 1]) - three) ||
                    (dstp[x] > std::min(dstp[x - 2], dstp[x - 1]) + three && dstp[x] > std::min(dstp[x + 2], dstp[x + 1]) + three &&
                        dstpnn[x] > std::min(dstpnn[x - 2], dstpnn[x - 1]) + three && dstpnn[x] > std::min(dstpnn[x + 2], dstpnn[x + 1]) + three))) {
                dstpn[x] = (dstp[x] + dstpnn[x] + 1) / 2;
                dmskp[x] = neutral;
                continue;
            }

            dir = (dir - neutral + (1 << (shift2 - 1))) >> shift2;
            const int uStart = (dir - 2 < 0) ? std::max({ -x + 1, dir - 2, -width + 2 + x }) : std::min({ x - 1, dir - 2, width - 2 - x });
            const int uStop = (dir + 2 < 0) ? std::max({ -x + 1, dir + 2, -width + 2 + x }) : std::min({ x - 1, dir + 2, width - 2 - x });
            unsigned min = nt8;
            unsigned val = (dstp[x] + dstpnn[x] + 1) / 2;

            for (int u = uStart; u <= uStop; u++) {
                const unsigned diff = std::abs(dstp[x - 1] - dstpnn[x - u - 1]) + std::abs(dstp[x] - dstpnn[x - u]) + std::abs(dstp[x + 1] - dstpnn[x - u + 1]) +
                    std::abs(dstpnn[x - 1] - dstp[x + u - 1]) + std::abs(dstpnn[x] - dstp[x + u]) + std::abs(dstpnn[x + 1] - dstp[x + u + 1]);
                if (diff < min &&
                    ((omskp[x - 1 + u] != peak && std::abs(omskp[x - 1 + u] - dmskp[x]) <= lim) ||
                        (omskp[x + u] != peak && std::abs(omskp[x + u] - dmskp[x]) <= lim) ||
                        (omskp[x + 1 + u] != peak && std::abs(omskp[x + 1 + u] - dmskp[x]) <= lim)) &&
                    ((omskn[x - 1 - u] != peak && std::abs(omskn[x - 1 - u] - dmskp[x]) <= lim) ||
                        (omskn[x - u] != peak && std::abs(omskn[x - u] - dmskp[x]) <= lim) ||
                        (omskn[x + 1 - u] != peak && std::abs(omskn[x + 1 - u] - dmskp[x]) <= lim))) {
                    const unsigned diff2 = std::abs(dstp[x + u / 2 - 1] - dstpnn[x - u / 2 - 1]) +
                        std::abs(dstp[x + u / 2] - dstpnn[x - u / 2]) +
                        std::abs(dstp[x + u / 2 + 1] - dstpnn[x - u / 2 + 1]);
                    if (diff2 < nt4 &&
                        (((std::abs(omskp[x + u / 2] - omskn[x - u / 2]) <= lim ||
                            std::abs(omskp[x + u / 2] - omskn[x - ((u + 1) / 2)]) <= lim) &&
                            omskp[x + u / 2] != peak) ||
                            ((std::abs(omskp[x + ((u + 1) / 2)] - omskn[x - u / 2]) <= lim ||
                                std::abs(omskp[x + ((u + 1) / 2)] - omskn[x - ((u + 1) / 2)]) <= lim) &&
                                omskp[x + ((u + 1) / 2)] != peak))) {
                        if ((std::abs(dmskp[x] - omskp[x + u / 2]) <= lim || std::abs(dmskp[x] - omskp[x + ((u + 1) / 2)]) <= lim) &&
                            (std::abs(dmskp[x] - omskn[x - u / 2]) <= lim || std::abs(dmskp[x] - omskn[x - ((u + 1) / 2)]) <= lim)) {
                            val = (dstp[x + u / 2] + dstp[x + ((u + 1) / 2)] +
                                dstpnn[x - u / 2] + dstpnn[x - ((u + 1) / 2)] + 2) / 4;
                            min = diff;
                            dir = u;
                        }
                    }
                }
            }

            if (min != nt8) {
                dstpn[x] = val;
                dmskp[x] = neutral + (dir << shift2);
            }
            else {
                const int dt = 4 >> (plane != PLANAR_Y ? vi.GetPlaneWidthSubsampling(PLANAR_U) : 0);
                const int uStart2 = std::max(-x + 1, -dt);
                const int uStop2 = std::min(width - 2 - x, dt);
                const unsigned minm = std::min(dstp[x], dstpnn[x]);
                const unsigned maxm = std::max(dstp[x], dstpnn[x]);
                min = nt7;

                for (int u = uStart2; u <= uStop2; u++) {
                    const int p1 = dstp[x + u / 2] + dstp[x + ((u + 1) / 2)];
                    const int p2 = dstpnn[x - u / 2] + dstpnn[x - ((u + 1) / 2)];
                    const unsigned diff = std::abs(dstp[x - 1] - dstpnn[x - u - 1]) + std::abs(dstp[x] - dstpnn[x - u]) + std::abs(dstp[x + 1] - dstpnn[x - u + 1]) +
                        std::abs(dstpnn[x - 1] - dstp[x + u - 1]) + std::abs(dstpnn[x] - dstp[x + u]) + std::abs(dstpnn[x + 1] - dstp[x + u + 1]) +
                        std::abs(p1 - p2);
                    if (diff < min) {
                        const unsigned valt = (p1 + p2 + 2) / 4;
                        if (valt >= minm && valt <= maxm) {
                            val = valt;
                            min = diff;
                            dir = u;
                        }
                    }
                }

                dstpn[x] = val;
                dmskp[x] = (min == nt7) ? neutral : neutral + (dir << shift2);
            }
        }

        omskp += stride * static_cast<int64_t>(2);
        omskn += stride * static_cast<int64_t>(2);
        dmskp += dmsk_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
        dstpn += dst_stride * static_cast<int64_t>(2);
        dstpnn += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::postProcess(PVideoFrame& nmsk, PVideoFrame& omsk, PVideoFrame& dst, const int plane, const unsigned field, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (vi.BitsPerComponent() - 1);
    const T peak = (1 << vi.BitsPerComponent()) - 1;
    const T shift2 = 2 + (vi.BitsPerComponent() - 8);

    const unsigned stride = nmsk->GetPitch(plane) / sizeof(T);
    const unsigned omsk_stride = omsk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = nmsk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = nmsk->GetHeight(plane);
    const T* nmskp = reinterpret_cast<const T*>(nmsk->GetReadPtr(plane));
    const T* omskp = reinterpret_cast<const T*>(omsk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    nmskp += stride * (static_cast<int64_t>(2) - field);
    omskp += omsk_stride * (static_cast<int64_t>(2) - field);
    dstp += dst_stride * (static_cast<int64_t>(2) - field);

    const T* dstpp = dstp - dst_stride;
    const T* dstpn = dstp + dst_stride;

    for (unsigned y = 2 - field; y < height - 1; y += 2) {
        for (unsigned x = 0; x < width; x++) {
            const int lim = limlut2[std::abs(nmskp[x] - neutral) >> shift2];
            if (std::abs(nmskp[x] - omskp[x]) > lim && omskp[x] != peak && omskp[x] != neutral)
                dstp[x] = (dstpp[x] + dstpn[x] + 1) / 2;
        }

        nmskp += stride * static_cast<int64_t>(2);
        omskp += omsk_stride * static_cast<int64_t>(2);
        dstpp += dst_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
        dstpn += dst_stride * static_cast<int64_t>(2);
    }
}

template<typename T>
void EEDI2::gaussianBlur1(PVideoFrame& src, PVideoFrame& tmp, PVideoFrame& dst, const int plane, IScriptEnvironment* env) noexcept
{
    const unsigned stride = src->GetPitch(plane) / sizeof(T);
    const unsigned tmp_stride = tmp->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = src->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = src->GetHeight(plane);
    const T* srcp = reinterpret_cast<const T*>(src->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(tmp->GetWritePtr(plane));

    for (unsigned y = 0; y < height; y++) {
        dstp[0] = (srcp[3] * 582u + srcp[2] * 7078u + srcp[1] * 31724u + srcp[0] * 26152u + 32768u) / 65536u;
        dstp[1] = (srcp[4] * 582u + srcp[3] * 7078u + (srcp[0] + srcp[2]) * 15862u + srcp[1] * 26152u + 32768u) / 65536u;
        dstp[2] = (srcp[5] * 582u + (srcp[0] + srcp[4]) * 3539u + (srcp[1] + srcp[3]) * 15862u + srcp[2] * 26152u + 32768u) / 65536u;
        unsigned x;
        for (x = 3; x < width - 3; x++)
            dstp[x] = ((srcp[x - 3] + srcp[x + 3]) * 291u + (srcp[x - 2] + srcp[x + 2]) * 3539u + (srcp[x - 1] + srcp[x + 1]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;
        dstp[x] = (srcp[x - 3] * 582u + (srcp[x - 2] + srcp[x + 2]) * 3539u + (srcp[x - 1] + srcp[x + 1]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u; x++;
        dstp[x] = (srcp[x - 3] * 582u + srcp[x - 2] * 7078u + (srcp[x - 1] + srcp[x + 1]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u; x++;
        dstp[x] = (srcp[x - 3] * 582u + srcp[x - 2] * 7078u + srcp[x - 1] * 31724u + srcp[x] * 26152u + 32768u) / 65536u;

        srcp += stride;
        dstp += tmp_stride;
    }

    srcp = reinterpret_cast<const T*>(tmp->GetReadPtr(plane));
    dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));
    const T* src3p = srcp - tmp_stride * static_cast<int64_t>(3);
    const T* src2p = srcp - tmp_stride * static_cast<int64_t>(2);
    const T* srcpp = srcp - tmp_stride;
    const T* srcpn = srcp + tmp_stride;
    const T* src2n = srcp + tmp_stride * static_cast<int64_t>(2);
    const T* src3n = srcp + tmp_stride * static_cast<int64_t>(3);

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3n[x] * 582u + src2n[x] * 7078u + srcpn[x] * 31724u + srcp[x] * 26152u + 32768u) / 65536u;

    src3p += tmp_stride;
    src2p += tmp_stride;
    srcpp += tmp_stride;
    srcp += tmp_stride;
    srcpn += tmp_stride;
    src2n += tmp_stride;
    src3n += tmp_stride;
    dstp += dst_stride;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3n[x] * 582u + src2n[x] * 7078u + (srcpp[x] + srcpn[x]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;

    src3p += tmp_stride;
    src2p += tmp_stride;
    srcpp += tmp_stride;
    srcp += tmp_stride;
    srcpn += tmp_stride;
    src2n += tmp_stride;
    src3n += tmp_stride;
    dstp += dst_stride;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3n[x] * 582u + (src2p[x] + src2n[x]) * 3539u + (srcpp[x] + srcpn[x]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;

    src3p += tmp_stride;
    src2p += tmp_stride;
    srcpp += tmp_stride;
    srcp += tmp_stride;
    srcpn += tmp_stride;
    src2n += tmp_stride;
    src3n += tmp_stride;
    dstp += dst_stride;

    for (unsigned y = 3; y < height - 3; y++) {
        for (unsigned x = 0; x < width; x++)
            dstp[x] = ((src3p[x] + src3n[x]) * 291u + (src2p[x] + src2n[x]) * 3539u + (srcpp[x] + srcpn[x]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;

        src3p += tmp_stride;
        src2p += tmp_stride;
        srcpp += tmp_stride;
        srcp += tmp_stride;
        srcpn += tmp_stride;
        src2n += tmp_stride;
        src3n += tmp_stride;
        dstp += dst_stride;
    }

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3p[x] * 582u + (src2p[x] + src2n[x]) * 3539u + (srcpp[x] + srcpn[x]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;

    src3p += tmp_stride;
    src2p += tmp_stride;
    srcpp += tmp_stride;
    srcp += tmp_stride;
    srcpn += tmp_stride;
    src2n += tmp_stride;
    src3n += tmp_stride;
    dstp += dst_stride;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3p[x] * 582u + src2p[x] * 7078u + (srcpp[x] + srcpn[x]) * 15862u + srcp[x] * 26152u + 32768u) / 65536u;

    src3p += tmp_stride;
    src2p += tmp_stride;
    srcpp += tmp_stride;
    srcp += tmp_stride;
    srcpn += tmp_stride;
    src2n += tmp_stride;
    src3n += tmp_stride;
    dstp += dst_stride;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src3p[x] * 582u + src2p[x] * 7078u + srcpp[x] * 31724u + srcp[x] * 26152u + 32768u) / 65536u;
}

template<typename T>
void EEDI2::calcDerivatives(PVideoFrame& src, int* VS_RESTRICT x2, int* VS_RESTRICT y2, int* VS_RESTRICT xy,
    const int plane, const unsigned bitsPerSample, IScriptEnvironment* env) noexcept
{
    const T shift = bitsPerSample - 8;

    const unsigned stride = src->GetPitch(plane) / sizeof(T);
    const unsigned width = src->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = src->GetHeight(plane);
    const T* srcp = reinterpret_cast<const T*>(src->GetReadPtr(plane));
    const T* srcpp = srcp - stride;
    const T* srcpn = srcp + stride;

    {
        const int Ix = (srcp[1] - srcp[0]) >> shift;
        const int Iy = (srcp[0] - srcpn[0]) >> shift;
        x2[0] = (Ix * Ix) / 2;
        y2[0] = (Iy * Iy) / 2;
        xy[0] = (Ix * Iy) / 2;
    }

    unsigned x;

    for (x = 1; x < width - 1; x++) {
        const int Ix = (srcp[x + 1] - srcp[x - 1]) >> shift;
        const int Iy = (srcp[x] - srcpn[x]) >> shift;
        x2[x] = (Ix * Ix) / 2;
        y2[x] = (Iy * Iy) / 2;
        xy[x] = (Ix * Iy) / 2;
    }

    {
        const int Ix = (srcp[x] - srcp[x - 1]) >> shift;
        const int Iy = (srcp[x] - srcpn[x]) >> shift;
        x2[x] = (Ix * Ix) / 2;
        y2[x] = (Iy * Iy) / 2;
        xy[x] = (Ix * Iy) / 2;
    }

    srcpp += stride;
    srcp += stride;
    srcpn += stride;
    x2 += width;
    y2 += width;
    xy += width;

    for (unsigned y = 1; y < height - 1; y++) {
        {
            const int Ix = (srcp[1] - srcp[0]) >> shift;
            const int Iy = (srcpp[0] - srcpn[0]) >> shift;
            x2[0] = (Ix * Ix) / 2;
            y2[0] = (Iy * Iy) / 2;
            xy[0] = (Ix * Iy) / 2;
        }

        for (x = 1; x < width - 1; x++) {
            const int Ix = (srcp[x + 1] - srcp[x - 1]) >> shift;
            const int Iy = (srcpp[x] - srcpn[x]) >> shift;
            x2[x] = (Ix * Ix) / 2;
            y2[x] = (Iy * Iy) / 2;
            xy[x] = (Ix * Iy) / 2;
        }

        {
            const int Ix = (srcp[x] - srcp[x - 1]) >> shift;
            const int Iy = (srcpp[x] - srcpn[x]) >> shift;
            x2[x] = (Ix * Ix) / 2;
            y2[x] = (Iy * Iy) / 2;
            xy[x] = (Ix * Iy) / 2;
        }

        srcpp += stride;
        srcp += stride;
        srcpn += stride;
        x2 += width;
        y2 += width;
        xy += width;
    }

    {
        const int Ix = (srcp[1] - srcp[0]) >> shift;
        const int Iy = (srcpp[0] - srcp[0]) >> shift;
        x2[0] = (Ix * Ix) / 2;
        y2[0] = (Iy * Iy) / 2;
        xy[0] = (Ix * Iy) / 2;
    }

    for (x = 1; x < width - 1; x++) {
        const int Ix = (srcp[x + 1] - srcp[x - 1]) >> shift;
        const int Iy = (srcpp[x] - srcp[x]) >> shift;
        x2[x] = (Ix * Ix) / 2;
        y2[x] = (Iy * Iy) / 2;
        xy[x] = (Ix * Iy) / 2;
    }

    {
        const int Ix = (srcp[x] - srcp[x - 1]) >> shift;
        const int Iy = (srcpp[x] - srcp[x]) >> shift;
        x2[x] = (Ix * Ix) / 2;
        y2[x] = (Iy * Iy) / 2;
        xy[x] = (Ix * Iy) / 2;
    }
}

static void gaussianBlurSqrt2(const int* src, int* tmp, int* dst, const unsigned width, const unsigned height) noexcept
{
    const int* srcp = src;
    int* VS_RESTRICT dstp = tmp;

    for (unsigned y = 0; y < height; y++) {
        unsigned x = 0;
        dstp[x] = (srcp[x + 4] * 678 + srcp[x + 3] * 3902 + srcp[x + 2] * 13618 + srcp[x + 1] * 28830 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x + 4] * 678 + srcp[x + 3] * 3902 + srcp[x + 2] * 13618 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x + 4] * 678 + srcp[x + 3] * 3902 + (srcp[x - 2] + srcp[x + 2]) * 6809 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x + 4] * 678 + (srcp[x - 3] + srcp[x + 3]) * 1951 + (srcp[x - 2] + srcp[x + 2]) * 6809 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536;
        for (x = 4; x < width - 4; x++)
            dstp[x] = ((srcp[x - 4] + srcp[x + 4]) * 339 + (srcp[x - 3] + srcp[x + 3]) * 1951 + (srcp[x - 2] + srcp[x + 2]) * 6809 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536;
        dstp[x] = (srcp[x - 4] * 678 + (srcp[x - 3] + srcp[x + 3]) * 1951 + (srcp[x - 2] + srcp[x + 2]) * 6809 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x - 4] * 678 + srcp[x - 3] * 3902 + (srcp[x - 2] + srcp[x + 2]) * 6809 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x - 4] * 678 + srcp[x - 3] * 3902 + srcp[x - 2] * 13618 + (srcp[x - 1] + srcp[x + 1]) * 14415 + srcp[x] * 18508 + 32768) / 65536; x++;
        dstp[x] = (srcp[x - 4] * 678 + srcp[x - 3] * 3902 + srcp[x - 2] * 13618 + srcp[x - 1] * 28830 + srcp[x] * 18508 + 32768) / 65536;

        srcp += width;
        dstp += width;
    }

    srcp = tmp;
    dstp = dst;
    const int* src4p = srcp - width * static_cast<int64_t>(4);
    const int* src3p = srcp - width * static_cast<int64_t>(3);
    const int* src2p = srcp - width * static_cast<int64_t>(2);
    const int* srcpp = srcp - width;
    const int* srcpn = srcp + width;
    const int* src2n = srcp + width * static_cast<int64_t>(2);
    const int* src3n = srcp + width * static_cast<int64_t>(3);
    const int* src4n = srcp + width * static_cast<int64_t>(4);

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4n[x] * 678 + src3n[x] * 3902 + src2n[x] * 13618 + srcpn[x] * 28830 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4n[x] * 678 + src3n[x] * 3902 + src2n[x] * 13618 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4n[x] * 678 + src3n[x] * 3902 + (src2p[x] + src2n[x]) * 6809 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4n[x] * 678 + (src3p[x] + src3n[x]) * 1951 + (src2p[x] + src2n[x]) * 6809 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned y = 4; y < height - 4; y++) {
        for (unsigned x = 0; x < width; x++)
            dstp[x] = ((src4p[x] + src4n[x]) * 339 + (src3p[x] + src3n[x]) * 1951 + (src2p[x] + src2n[x]) * 6809 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

        src4p += width;
        src3p += width;
        src2p += width;
        srcpp += width;
        srcp += width;
        srcpn += width;
        src2n += width;
        src3n += width;
        src4n += width;
        dstp += width;
    }

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4p[x] * 678 + (src3p[x] + src3n[x]) * 1951 + (src2p[x] + src2n[x]) * 6809 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4p[x] * 678 + src3p[x] * 3902 + (src2p[x] + src2n[x]) * 6809 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4p[x] * 678 + src3p[x] * 3902 + src2p[x] * 13618 + (srcpp[x] + srcpn[x]) * 14415 + srcp[x] * 18508 + 32768) / 65536;

    src4p += width;
    src3p += width;
    src2p += width;
    srcpp += width;
    srcp += width;
    srcpn += width;
    src2n += width;
    src3n += width;
    src4n += width;
    dstp += width;

    for (unsigned x = 0; x < width; x++)
        dstp[x] = (src4p[x] * 678 + src3p[x] * 3902 + src2p[x] * 13618 + srcpp[x] * 28830 + srcp[x] * 18508 + 32768) / 65536;
}

template<typename T>
void EEDI2::postProcessCorner(PVideoFrame& msk, PVideoFrame& dst, const int* x2, const int* y2, const int* xy,
    const int plane, const unsigned field, const unsigned bitsPerSample, IScriptEnvironment* env) noexcept
{
    const T neutral = 1 << (bitsPerSample - 1);
    const T peak = (1 << bitsPerSample) - 1;

    const unsigned stride = msk->GetPitch(plane) / sizeof(T);
    const unsigned dst_stride = dst->GetPitch(plane) / sizeof(T);
    const unsigned width = msk->GetRowSize(plane) / vi.ComponentSize();
    const unsigned height = msk->GetHeight(plane);
    const T* mskp = reinterpret_cast<const T*>(msk->GetReadPtr(plane));
    T* VS_RESTRICT dstp = reinterpret_cast<T*>(dst->GetWritePtr(plane));

    mskp += stride * (static_cast<int64_t>(8) - field);
    dstp += dst_stride * (static_cast<int64_t>(8) - field);
    x2 += width * static_cast<int64_t>(3);
    y2 += width * static_cast<int64_t>(3);
    xy += width * static_cast<int64_t>(3);

    const T* dstpp = dstp - dst_stride;
    const T* dstpn = dstp + dst_stride;
    const int* x2n = x2 + width;
    const int* y2n = y2 + width;
    const int* xyn = xy + width;

    for (unsigned y = 8 - field; y < height - 7; y += 2) {
        for (unsigned x = 4; x < width - 4; x++) {
            if (mskp[x] == peak || mskp[x] == neutral)
                continue;

            const int c1 = static_cast<int>(x2[x] * y2[x] - xy[x] * xy[x] - 0.09f * (x2[x] + y2[x]) * (x2[x] + y2[x]));
            const int c2 = static_cast<int>(x2n[x] * y2n[x] - xyn[x] * xyn[x] - 0.09f * (x2n[x] + y2n[x]) * (x2n[x] + y2n[x]));
            if (c1 > 775 || c2 > 775)
                dstp[x] = (dstpp[x] + dstpn[x] + 1) / 2;
        }

        mskp += stride * static_cast<int64_t>(2);
        dstpp += dst_stride * static_cast<int64_t>(2);
        dstp += dst_stride * static_cast<int64_t>(2);
        dstpn += dst_stride * static_cast<int64_t>(2);
        x2 += width;
        x2n += width;
        y2 += width;
        y2n += width;
        xy += width;
        xyn += width;
    }
}

template<typename T>
void EEDI2::process(PVideoFrame& src, PVideoFrame& dst, PVideoFrame& msk, PVideoFrame& tmp,
    PVideoFrame& dst2, PVideoFrame& dst2M, PVideoFrame& tmp2, PVideoFrame& tmp2_2, PVideoFrame& msk2,
    const unsigned field, IScriptEnvironment* env) noexcept
{
    int planes_y[4] = { PLANAR_Y, PLANAR_U, PLANAR_V, PLANAR_A };
    int planes_r[4] = { PLANAR_G, PLANAR_B, PLANAR_R, PLANAR_A };
    const int* current_planes = (vi.IsYUV() || vi.IsYUVA()) ? planes_y : planes_r;
    const int planecount = std::min(vi.NumComponents(), 3);
    for (int i = 0; i < planecount; i++)
    {
        const int plane = current_planes[i];

        buildEdgeMask<T>(src, msk, plane, env);
        erode<T>(msk, tmp, plane, env);
        dilate<T>(tmp, msk, plane, env);
        erode<T>(msk, tmp, plane, env);
        removeSmallHorzGaps<T>(tmp, msk, plane, env);

        if (_map != 1)
        {
            calcDirections<T>(src, msk, tmp, plane, env);
            filterDirMap<T>(msk, tmp, dst, plane, env);
            expandDirMap<T>(msk, dst, tmp, plane, env);
            filterMap<T>(msk, tmp, dst, plane, env);

            if (_map != 2)
            {
                memset(dst2->GetWritePtr(plane), 0, static_cast<int64_t>(dst2->GetPitch(plane)) * dst2->GetHeight(plane));

                upscaleBy2(src, dst2, plane, field, env);
                upscaleBy2(dst, tmp2_2, plane, field, env);
                upscaleBy2(msk, msk2, plane, field, env);
                markDirections2X<T>(msk2, tmp2_2, tmp2, plane, field, env);
                filterDirMap2X<T>(msk2, tmp2, dst2M, plane, field, env);
                expandDirMap2X<T>(msk2, dst2M, tmp2, plane, field, env);
                fillGaps2X<T>(msk2, tmp2, dst2M, plane, field, env);
                fillGaps2X<T>(msk2, dst2M, tmp2, plane, field, env);

                if (_map != 3)
                {
                    interpolateLattice<T>(tmp2_2, tmp2, dst2, plane, field, env);

                    if (_pp == 1 || _pp == 3)
                    {
                        memcpy(tmp2_2->GetWritePtr(plane), tmp2->GetReadPtr(plane), static_cast<int64_t>(tmp2->GetPitch(plane)) * tmp2->GetHeight(plane));

                        filterDirMap2X<T>(msk2, tmp2, dst2M, plane, field, env);
                        expandDirMap2X<T>(msk2, dst2M, tmp2, plane, field, env);
                        postProcess<T>(tmp2, tmp2_2, dst2, plane, field, env);
                    }

                    if (_pp == 2 || _pp == 3)
                    {
                        gaussianBlur1<T>(src, tmp, dst, plane, env);
                        calcDerivatives<T>(dst, cx2, cy2, cxy, plane, vi.BitsPerComponent(), env);
                        gaussianBlurSqrt2(cx2, tmpc, cx2, src->GetRowSize(plane) / vi.ComponentSize(), src->GetHeight(plane));
                        gaussianBlurSqrt2(cy2, tmpc, cy2, src->GetRowSize(plane) / vi.ComponentSize(), src->GetHeight(plane));
                        gaussianBlurSqrt2(cxy, tmpc, cxy, src->GetRowSize(plane) / vi.ComponentSize(), src->GetHeight(plane));
                        postProcessCorner<T>(tmp2_2, dst2, cx2, cy2, cxy, plane, field, vi.BitsPerComponent(), env);
                    }
                }
            }
        }
    }
}

EEDI2::EEDI2(PClip _child, int mthresh, int lthresh, int vthresh, int estr, int dstr, int maxd, int field, int map, int nt, int pp, IScriptEnvironment* env)
    : GenericVideoFilter(_child), _mthresh(mthresh), _lthresh(lthresh), _vthresh(vthresh), _estr(estr), _dstr(dstr), _maxd(maxd), _field(field), _map(map), _pp(pp)
{
    if ((vi.IsPlanar() && vi.BitsPerComponent() > 16) || !vi.IsPlanar())
        env->ThrowError("EEDI2: only 8..16-bit planar clip is supported.");

    if (vi.width < 8)
        env->ThrowError("EEDI2: the clip's width must be greater than or equal to 8.");

    if (vi.height < 7)
        env->ThrowError("EEDI2: the clip's height must be greater than or equal to 7.");

    if (_mthresh < 0)
        env->ThrowError("EEDI2: mthresh must be greater than or equal to 0.");

    if (_lthresh < 0)
        env->ThrowError("EEDI2: lthresh must be greater than or equal to 0.");

    if (_vthresh < 0)
        env->ThrowError("EEDI2: vthresh must be greater than or equal to 0.");

    if (_estr < 0)
        env->ThrowError("EEDI2: estr must be greater than or equal to 0.");

    if (_dstr < 0)
        env->ThrowError("EEDI2: dstr must be greater than or equal to 0.");

    if (_maxd < 1 || _maxd > 29)
        env->ThrowError("EEDI2: maxd must be between 1 and 29 (inclusive).");

    if (_field < -2 || _field > 3)
        env->ThrowError("EEDI2: field must be between -2 and 3 (inclusive).");

    if (_map < 0 || _map > 3)
        env->ThrowError("EEDI2: map must be 0, 1, 2 or 3.");

    if (nt < 0)
        env->ThrowError("EEDI2: nt must be greater than or equal to 0.");

    if (_pp < 0 || _pp > 3)
        env->ThrowError("EEDI2: pp must be 0, 1, 2 or 3.");

    has_at_least_v8 = true;
    try { env->CheckVersion(8); }
    catch (const AvisynthError&) { has_at_least_v8 = false; }

    fieldS = _field;

    if (_field < 0)
        _field = child->GetParity(0) ? 1 : 0;

    if (fieldS == -2)
        fieldS = _field == 0 ? 2 : 3;
    else if (fieldS == -1)
        fieldS = _field;
    else if (fieldS == 2)
        _field = 0;
    else if (fieldS == 3)
        _field = 1;

    cx2 = cy2 = cxy = tmpc = nullptr;

    vi2 = vi;

    if (_map == 0 || _map == 3)
    {
        vi.SetFieldBased(false);
        vi.height *= 2;
    }

    _mthresh *= _mthresh;
    _vthresh *= 81;

    const int8_t limlut[33]
    {
            6, 6, 7, 7, 8, 8, 9, 9, 9, 10,
            10, 11, 11, 12, 12, 12, 12, 12, 12, 12,
            12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
            12, -1, -1
    };

    EEDI2::limlut = new int8_t[33];
    limlut2 = new int16_t[33];
    std::copy_n(limlut, 33, EEDI2::limlut);

    const unsigned shift = vi.BitsPerComponent() - 8;
    nt <<= shift;
    for (unsigned i = 0; i < 33; i++)
        limlut2[i] = limlut[i] << shift;

    nt4 = nt * 4;
    nt7 = nt * 7;
    nt8 = nt * 8;
    nt13 = nt * 13;
    nt19 = nt * 19;
}

EEDI2::~EEDI2()
{
    delete[] limlut;
    delete[] limlut2;
}

PVideoFrame __stdcall EEDI2::GetFrame(int n, IScriptEnvironment* env)
{
    PVideoFrame src = child->GetFrame(n, env);
    PVideoFrame dst = env->NewVideoFrame(vi2);
    PVideoFrame msk = env->NewVideoFrame(vi2);
    PVideoFrame tmp = env->NewVideoFrame(vi2);

    PVideoFrame dst2 = nullptr, dst2M = nullptr, tmp2 = nullptr, tmp2_2 = nullptr, msk2 = nullptr;

    if (_map == 0 || _map == 3)
    {
        dst2 = env->NewVideoFrame(vi);
        dst2M = env->NewVideoFrame(vi);
        tmp2 = env->NewVideoFrame(vi);
        tmp2_2 = env->NewVideoFrame(vi);
        msk2 = env->NewVideoFrame(vi);

        if (_pp > 1 && _map == 0)
        {
            int pl;
            if (vi.IsRGB())
                pl = PLANAR_G;
            else
                pl = PLANAR_Y;

            if (has_at_least_v8)
            {
                cx2 = static_cast<int*>(env->Allocate(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32, AVS_POOLED_ALLOC));
                cy2 = static_cast<int*>(env->Allocate(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32, AVS_POOLED_ALLOC));
                cxy = static_cast<int*>(env->Allocate(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32, AVS_POOLED_ALLOC));
                tmpc = static_cast<int*>(env->Allocate(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32, AVS_POOLED_ALLOC));
            }
            else
            {
                cx2 = static_cast<int*>(_aligned_malloc(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32));
                cy2 = static_cast<int*>(_aligned_malloc(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32));
                cxy = static_cast<int*>(_aligned_malloc(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32));
                tmpc = static_cast<int*>(_aligned_malloc(static_cast<int64_t>(src->GetHeight(pl)) * src->GetPitch(pl) * sizeof(int), 32));
            }

            if (!cx2 || !cy2 || !cxy || !tmpc)
                env->ThrowError("EEDI2: malloc failure (pp > 1)!");
        }
    }

    unsigned field = _field;
    if (fieldS > 1)
        field = (n & 1) ? (fieldS == 2 ? 1 : 0) : (fieldS == 2 ? 0 : 1);

    if (vi.ComponentSize() == 1)
        process<uint8_t>(src, dst, msk, tmp, dst2, dst2M, tmp2, tmp2_2, msk2, field, env);
    else
        process<uint16_t>(src, dst, msk, tmp, dst2, dst2M, tmp2, tmp2_2, msk2, field, env);

    if (_map == 0)
        dst = dst2;
    else if (_map == 1)
        dst = msk;
    else if (_map == 3)
        dst = tmp2;

    if (has_at_least_v8)
    {
        env->copyFrameProps(src, dst);

        if (cx2)
            env->Free(cx2);

        if (cy2)
            env->Free(cy2);

        if (cxy)
            env->Free(cxy);

        if (tmpc)
            env->Free(tmpc);
    }
    else
    {
        if (cx2)
            _aligned_free(cx2);

        if (cy2)
            _aligned_free(cy2);

        if (cxy)
            _aligned_free(cxy);

        if (tmpc)
            _aligned_free(tmpc);
    }

    return dst;
}

AVSValue __cdecl Create_EEDI2(AVSValue args, void* user_data, IScriptEnvironment* env)
{
    return new EEDI2(
        args[0].AsClip(),
        args[1].AsInt(10),
        args[2].AsInt(20),
        args[3].AsInt(20),
        args[4].AsInt(2),
        args[5].AsInt(4),
        args[6].AsInt(24),
        args[7].AsInt(-1),
        args[8].AsInt(0),
        args[9].AsInt(50),
        args[10].AsInt(1),
        env);
}

const AVS_Linkage* AVS_linkage;

extern "C" __declspec(dllexport)
const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
    AVS_linkage = vectors;

    env->AddFunction("EEDI2", "c[mthresh]i[lthresh]i[vthresh]i[estr]i[dstr]i[maxd]i[field]i[map]i[nt]i[pp]i", Create_EEDI2, 0);
    return "EEDI2";
}
