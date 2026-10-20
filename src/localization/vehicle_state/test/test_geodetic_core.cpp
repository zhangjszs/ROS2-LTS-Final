// #79：WGS84 大地坐标 → ENU 纯 std 核的无 ROS 合成夹具（#24 第 1 条切片）。
//
// 参考值来源（与被测实现相互独立，容差全部显式写出）：
//   A. 手推闭式：把 WGS84 的 ECEF 定义与 ENU 旋转在给定原点上手推成解析式。
//      被测实现走「ECEF → 3×3 旋转」路径，期望值走「手推闭式」路径，不共用代码。
//   B. 大地测量学恒等式：卯酉圈曲率半径 N(φ)=a/(1-e²sin²φ)^{1/2}、
//      子午圈曲率半径 M(φ)=a(1-e²)/(1-e²sin²φ)^{3/2}，用于小位移的量级交叉校验。
//   C. 几何恒等式：同纬度、同经度只改高程时，天向分量恒等于 Δh。
//
// 椭球常数在本文件内独立声明（不引用被测头文件的常量），使交叉校验真实有效。
//
// 单位：纬度/经度为弧度，高程为米，输出 ENU 为米。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "geodetic_core.hpp"

namespace {

// ── 独立参考用的 WGS84 常数（与被测头文件同值，但独立声明）──────────────────
constexpr double kRefA = 6378137.0;
constexpr double kRefB = 6356752.3142;
constexpr double kRefF = (kRefA - kRefB) / kRefA;
constexpr double kRefE2 = kRefF * (2.0 - kRefF);

// 卯酉圈曲率半径 N(φ)（米）
double RefPrimeVerticalRadius(double lat) {
    const double s = std::sin(lat);
    return kRefA / std::sqrt(1.0 - kRefE2 * s * s);
}

// 子午圈曲率半径 M(φ)（米）
double RefMeridianRadius(double lat) {
    const double s = std::sin(lat);
    return kRefA * (1.0 - kRefE2) / std::pow(1.0 - kRefE2 * s * s, 1.5);
}

// ── 参考 A：手推闭式 ─────────────────────────────────────────────────────────
// 只改经度（h=h0、φ=φ0 保持不变）时，由 ECEF 定义手推可得：
//   east  = N(φ0)·cosφ0·sinΔλ
//   north = N(φ0)·cosφ0·(1-cosΔλ)·sinφ0
//   up    = N(φ0)·cos²φ0·(cosΔλ - 1)
// 只改纬度（λ=λ0、h=h0）时，手推得 east ≡ 0（旋转后 x/y 差同向，投影后抵消）。
// 只改高程（φ=φ0、λ=λ0）时，几何恒等式给出 up ≡ Δh、east ≡ north ≡ 0。
void RefEnuOnlyLonShift(double lat0, double dlon, double out[3]) {
    const double n = RefPrimeVerticalRadius(lat0);
    const double c = std::cos(lat0);
    out[0] = n * c * std::sin(dlon);
    out[1] = n * c * (1.0 - std::cos(dlon)) * std::sin(lat0);
    out[2] = n * c * c * (std::cos(dlon) - 1.0);
}

// 赤道 + 本初子午线原点（φ0=λ0=0、h0=0）下，ENU 旋转退化为最简解析式：
//   east  = Y        = (h+N)·cosφ·sinλ
//   north = Z        = (h+(1-e²)N)·sinφ
//   up    = X - a    = (h+N)·cosφ·cosλ - a
void RefEnuAtEquatorPrimeMeridian(double lat, double lon, double h, double out[3]) {
    const double n = RefPrimeVerticalRadius(lat);
    out[0] = (h + n) * std::cos(lat) * std::sin(lon);
    out[1] = (h + (1.0 - kRefE2) * n) * std::sin(lat);
    out[2] = (h + n) * std::cos(lat) * std::cos(lon) - kRefA;
}

// ── 被测调用的薄封装 ────────────────────────────────────────────────────────
void CallCore(double lat, double lon, double h, double lat0, double lon0, double h0, double out[3]) {
    vehicle_state_core::GeodeticToEnuCore(lat, lon, h, lat0, lon0, h0, out);
}

// 测试原点：北纬 30°、东经 120°（弧度）
constexpr double kOriginLat = 0.5235987755982988;  // 30°
constexpr double kOriginLon = 2.0943951023931953;  // 120°

// 容差（米）—— 逐条写明来源
constexpr double kTightTol = 1e-6;     // 手推闭式 vs 实现：浮点舍入差 ~1e-13 m，留 1e4 倍余量
constexpr double kHeightTol = 1e-9;    // 同纬度同经度只改高程：up ≡ Δh，残差仅舍入 ~1e-15 m
constexpr double kMeridianTol = 0.01;  // M(φ)·Δφ 一阶近似：二阶项 ~0.5·(dM/dφ)·Δφ² ≈ 2.8e-4 m

}  // namespace

// ── 1. 同点 → 严格原点 ─────────────────────────────────────────────────────

TEST(GeodeticCoreTest, SamePointIsExactlyOrigin) {
    double out[3] = {0.0, 0.0, 0.0};
    CallCore(kOriginLat, kOriginLon, 0.0, kOriginLat, kOriginLon, 0.0, out);
    EXPECT_DOUBLE_EQ(out[0], 0.0);
    EXPECT_DOUBLE_EQ(out[1], 0.0);
    EXPECT_DOUBLE_EQ(out[2], 0.0);
}

// ── 2. 东向偏移：符号为正、量级 = N·cosφ·Δλ ───────────────────────────────

TEST(GeodeticCoreTest, EastOffsetMatchesClosedForm) {
    constexpr double kDlon = 1e-4;  // 弧度 ≈ 637 m 东偏
    double out[3] = {0.0, 0.0, 0.0};
    CallCore(kOriginLat, kOriginLon + kDlon, 0.0, kOriginLat, kOriginLon, 0.0, out);

    double ref[3] = {0.0, 0.0, 0.0};
    RefEnuOnlyLonShift(kOriginLat, kDlon, ref);

    EXPECT_GT(out[0], 0.0);                  // 东偏 → x 为正
    EXPECT_NEAR(out[0], ref[0], kTightTol);  // 主分量：手推闭式
    EXPECT_NEAR(out[1], ref[1], kTightTol);  // 副分量（切平面投影二阶项）
    EXPECT_NEAR(out[2], ref[2], kTightTol);  // 副分量（椭球扁率的二阶效应）
    // 量级交叉校验：B 族恒等式 N(φ)·cosφ·Δλ
    const double ref_east_magnitude = RefPrimeVerticalRadius(kOriginLat) * std::cos(kOriginLat) * kDlon;
    EXPECT_NEAR(out[0], ref_east_magnitude, kTightTol);
}

// ── 3. 北向偏移：符号为正、量级 = M(φ)·Δφ ─────────────────────────────────

TEST(GeodeticCoreTest, NorthOffsetMatchesMeridianIdentity) {
    constexpr double kDlat = 1e-4;  // 弧度 ≈ 635 m 北偏
    double out[3] = {0.0, 0.0, 0.0};
    CallCore(kOriginLat + kDlat, kOriginLon, 0.0, kOriginLat, kOriginLon, 0.0, out);

    EXPECT_GT(out[1], 0.0);  // 北偏 → y 为正
    // B 族恒等式：子午圈曲率半径 M(φ)·Δφ
    EXPECT_NEAR(out[1], RefMeridianRadius(kOriginLat) * kDlat, kMeridianTol);
    // 手推闭式：只改纬度时 east 恒为 0
    EXPECT_NEAR(out[0], 0.0, kTightTol);
    // 天向分量：椭球扁率的真实效应（沿子午线北移会伴随约 3.7 m 的天向偏移），
    // 此处只做量级断言，不与闭式逐位对齐
    EXPECT_LT(std::abs(out[2]), 10.0);
}

// ── 4. 高度偏移：up ≡ Δh，水平分量 ≡ 0 ────────────────────────────────────

TEST(GeodeticCoreTest, AltitudeOffsetIsExactlyDeltaH) {
    constexpr double kDh = 5.0;
    double out[3] = {0.0, 0.0, 0.0};
    CallCore(kOriginLat, kOriginLon, kDh, kOriginLat, kOriginLon, 0.0, out);

    // C 族几何恒等式
    EXPECT_NEAR(out[2], kDh, kHeightTol);
    EXPECT_NEAR(out[0], 0.0, kTightTol);
    EXPECT_NEAR(out[1], 0.0, kTightTol);
}

// ── 5. 跨经纬度参考点（赤道 + 本初子午线原点 → 北纬 45°/东经 30°）──────────

TEST(GeodeticCoreTest, CrossReferencePointMatchesHandDerivedClosedForm) {
    constexpr double kLat = 0.7853981633974483;  // 45°
    constexpr double kLon = 0.5235987755982988;  // 30°
    constexpr double kH = 1000.0;

    double out[3] = {0.0, 0.0, 0.0};
    CallCore(kLat, kLon, kH, 0.0, 0.0, 0.0, out);

    double ref[3] = {0.0, 0.0, 0.0};
    RefEnuAtEquatorPrimeMeridian(kLat, kLon, kH, ref);

    EXPECT_NEAR(out[0], ref[0], kTightTol);
    EXPECT_NEAR(out[1], ref[1], kTightTol);
    EXPECT_NEAR(out[2], ref[2], kTightTol);
    // 量级健全性：点落在数千米量级，而非米级
    EXPECT_GT(std::abs(out[0]), 1.0e6);
    EXPECT_GT(std::abs(out[1]), 1.0e6);
    EXPECT_GT(std::abs(out[2]), 1.0e6);
}

// ── 6. 大纬度输入不产生 NaN ────────────────────────────────────────────────

TEST(GeodeticCoreTest, ExtremeLatitudesStayFinite) {
    for (const double lat : {1.3962634015954636, 0.0, -1.3962634015954636}) {  // ±80°
        double out[3] = {0.0, 0.0, 0.0};
        CallCore(lat, 0.7, 120.0, 0.0, 0.0, 0.0, out);
        EXPECT_TRUE(std::isfinite(out[0])) << "lat=" << lat;
        EXPECT_TRUE(std::isfinite(out[1])) << "lat=" << lat;
        EXPECT_TRUE(std::isfinite(out[2])) << "lat=" << lat;
    }
}

// ── 7. NaN 输入：按 IEEE-754 传播为 NaN 输出（显式定义，不静默 UB）─────────

TEST(GeodeticCoreTest, NanInputPropagatesToNanOutput) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double out[3] = {0.0, 0.0, 0.0};
    CallCore(nan, kOriginLon, 0.0, kOriginLat, kOriginLon, 0.0, out);
    EXPECT_TRUE(std::isnan(out[0]));
    EXPECT_TRUE(std::isnan(out[1]));
    EXPECT_TRUE(std::isnan(out[2]));
}

// ── 8. 确定性：固定输入双跑逐位一致（呼应 #24 的固定输入/顺序要求）────────

TEST(GeodeticCoreTest, DeterministicAcrossRepeatedCalls) {
    double first[3] = {0.0, 0.0, 0.0};
    double second[3] = {0.0, 0.0, 0.0};
    CallCore(0.9, 2.0, 300.0, kOriginLat, kOriginLon, 10.0, first);
    CallCore(0.9, 2.0, 300.0, kOriginLat, kOriginLon, 10.0, second);
    EXPECT_DOUBLE_EQ(first[0], second[0]);
    EXPECT_DOUBLE_EQ(first[1], second[1]);
    EXPECT_DOUBLE_EQ(first[2], second[2]);
}
