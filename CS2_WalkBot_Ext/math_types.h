#pragma once
#include <cmath>
#include <cassert>

// Forward declarations
class Vector;
class QAngle;

//=========================================================
// 3D Vector
//=========================================================
class Vector
{
public:
    float m_flX, m_flY, m_flZ;

    Vector() : m_flX(0), m_flY(0), m_flZ(0) {}
    Vector(float flX, float flY, float flZ) : m_flX(flX), m_flY(flY), m_flZ(flZ) {}

    void Init(float flX = 0.0f, float flY = 0.0f, float flZ = 0.0f) {
        m_flX = flX; m_flY = flY; m_flZ = flZ;
    }

    bool IsValid() const {
        return !std::isnan(m_flX) && !std::isnan(m_flY) && !std::isnan(m_flZ);
    }

    float operator[](int nIndex) const {
        return reinterpret_cast<const float*>(this)[nIndex];
    }

    float& operator[](int nIndex) {
        return reinterpret_cast<float*>(this)[nIndex];
    }

    Vector operator+(const Vector& v) const {
        return Vector(m_flX + v.m_flX, m_flY + v.m_flY, m_flZ + v.m_flZ);
    }

    Vector operator-(const Vector& v) const {
        return Vector(m_flX - v.m_flX, m_flY - v.m_flY, m_flZ - v.m_flZ);
    }

    Vector operator*(float f) const {
        return Vector(m_flX * f, m_flY * f, m_flZ * f);
    }

    Vector operator/(float f) const {
        return Vector(m_flX / f, m_flY / f, m_flZ / f);
    }

    Vector& operator+=(const Vector& v) {
        m_flX += v.m_flX; m_flY += v.m_flY; m_flZ += v.m_flZ; return *this;
    }

    Vector& operator-=(const Vector& v) {
        m_flX -= v.m_flX; m_flY -= v.m_flY; m_flZ -= v.m_flZ; return *this;
    }

    Vector& operator*=(float f) {
        m_flX *= f; m_flY *= f; m_flZ *= f; return *this;
    }

    bool operator==(const Vector& v) const {
        return m_flX == v.m_flX && m_flY == v.m_flY && m_flZ == v.m_flZ;
    }

    bool operator!=(const Vector& v) const {
        return !(*this == v);
    }

    float* Base() { return reinterpret_cast<float*>(this); }
    const float* Base() const { return reinterpret_cast<const float*>(this); }

    float Length() const { return std::sqrtf(LengthSqr()); }
    float LengthSqr() const { return m_flX * m_flX + m_flY * m_flY + m_flZ * m_flZ; }

    float Length2D() const { return std::sqrtf(Length2DSqr()); }
    float Length2DSqr() const { return m_flX * m_flX + m_flY * m_flY; }

    float Dot(const Vector& v) const { return m_flX * v.m_flX + m_flY * v.m_flY + m_flZ * v.m_flZ; }

    Vector Normalize() {
        float len = Length();
        if (len > 0.0f) { m_flX /= len; m_flY /= len; m_flZ /= len; }
        return *this;
    }

    float Distance(const Vector& v) const {
        return (*this - v).Length();
    }

    bool IsZero() const { return m_flX == 0.0f && m_flY == 0.0f && m_flZ == 0.0f; }

    QAngle ToAngles(Vector vecEnd) const;
};

//=========================================================
// QAngle (Pitch, Yaw, Roll in degrees)
//=========================================================
class QAngle
{
public:
    float m_flPitch, m_flYaw, m_flRoll;

    QAngle() : m_flPitch(0), m_flYaw(0), m_flRoll(0) {}
    QAngle(float p, float y, float r) : m_flPitch(p), m_flYaw(y), m_flRoll(r) {}

    void Init(float p = 0.0f, float y = 0.0f, float r = 0.0f) {
        m_flPitch = p; m_flYaw = y; m_flRoll = r;
    }

    bool IsValid() const {
        return !std::isnan(m_flPitch) && !std::isnan(m_flYaw) && !std::isnan(m_flRoll);
    }

    float operator[](int idx) const {
        return reinterpret_cast<const float*>(this)[idx];
    }

    float& operator[](int idx) {
        return reinterpret_cast<float*>(this)[idx];
    }

    QAngle operator+(const QAngle& a) const {
        return QAngle(m_flPitch + a.m_flPitch, m_flYaw + a.m_flYaw, m_flRoll + a.m_flRoll);
    }

    QAngle operator-(const QAngle& a) const {
        return QAngle(m_flPitch - a.m_flPitch, m_flYaw - a.m_flYaw, m_flRoll - a.m_flRoll);
    }

    QAngle operator*(float f) const {
        return QAngle(m_flPitch * f, m_flYaw * f, m_flRoll * f);
    }

    QAngle operator/(float f) const {
        return QAngle(m_flPitch / f, m_flYaw / f, m_flRoll / f);
    }

    QAngle& operator+=(const QAngle& a) {
        m_flPitch += a.m_flPitch; m_flYaw += a.m_flYaw; m_flRoll += a.m_flRoll; return *this;
    }

    QAngle& operator-=(const QAngle& a) {
        m_flPitch -= a.m_flPitch; m_flYaw -= a.m_flYaw; m_flRoll -= a.m_flRoll; return *this;
    }

    QAngle& operator*=(float f) {
        m_flPitch *= f; m_flYaw *= f; m_flRoll *= f; return *this;
    }

    bool operator==(const QAngle& a) const {
        return m_flPitch == a.m_flPitch && m_flYaw == a.m_flYaw && m_flRoll == a.m_flRoll;
    }

    bool operator!=(const QAngle& a) const {
        return !(*this == a);
    }

    float* Base() { return reinterpret_cast<float*>(this); }
    const float* Base() const { return reinterpret_cast<const float*>(this); }

    float Length() const { return std::sqrtf(LengthSqr()); }
    float LengthSqr() const { return m_flPitch * m_flPitch + m_flYaw * m_flYaw + m_flRoll * m_flRoll; }
};

inline QAngle Vector::ToAngles(Vector vecEnd) const
{
    const Vector vecStart = *this;
    const Vector vecDelta = vecEnd - vecStart;

    const float flLength = vecDelta.Length();
    if (flLength <= 0.0f)
        return QAngle();

    const float flPitch = static_cast<float>((std::acos(vecDelta.m_flZ / flLength) * (180.0 / 3.14159265358979323846)) - 90.0);
    const float flYaw = static_cast<float>(std::atan2(vecDelta.m_flY, vecDelta.m_flX) * (180.0 / 3.14159265358979323846));
    const float flRoll = 0.0f;

    return QAngle(flPitch, flYaw, flRoll);
}

#pragma pack(push, 4)

struct ViewMatrix_t
{
    ViewMatrix_t() = default;

    constexpr ViewMatrix_t(
        const float m00, const float m01, const float m02, const float m03,
        const float m10, const float m11, const float m12, const float m13,
        const float m20, const float m21, const float m22, const float m23,
        const float m30, const float m31, const float m32, const float m33)
    {
        arrData[0][0] = m00; arrData[0][1] = m01; arrData[0][2] = m02; arrData[0][3] = m03;
        arrData[1][0] = m10; arrData[1][1] = m11; arrData[1][2] = m12; arrData[1][3] = m13;
        arrData[2][0] = m20; arrData[2][1] = m21; arrData[2][2] = m22; arrData[2][3] = m23;
        arrData[3][0] = m30; arrData[3][1] = m31; arrData[3][2] = m32; arrData[3][3] = m33;
    }

    [[nodiscard]] float* operator[](const int nIndex)
    {
        return arrData[nIndex];
    }

    [[nodiscard]] const float* operator[](const int nIndex) const
    {
        return arrData[nIndex];
    }

    [[nodiscard]] bool operator==(const ViewMatrix_t& viewOther) const
    {
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                if (arrData[row][col] != viewOther.arrData[row][col])
                    return false;
            }
        }

        return true;
    }

    [[nodiscard]] bool operator!=(const ViewMatrix_t& viewOther) const
    {
        return !(*this == viewOther);
    }

    float arrData[4][4] = {};
};

#pragma pack(pop)
