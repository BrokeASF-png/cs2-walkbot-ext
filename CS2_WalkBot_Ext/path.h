#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include "math_types.h"

enum class EWaypointType : int
{
    Normal = 0,
    MultiBranch = 1
};

enum class EBranchSelectMode : int
{
    Sequential = 0,
    Random = 1,
    Cycle = 2
};

// Per-waypoint behavior flags (bitmask). Combine with bitwise OR.
enum EWaypointFlag : std::uint32_t
{
    WPF_NONE    = 0,
    WPF_CROUCH  = 1u << 0,  // force crouch while this waypoint is the active target
    WPF_JUMP    = 1u << 1,  // one-shot jump pulse on entry
    WPF_WALK    = 1u << 2,  // hold shift (walk key) while approaching
    WPF_SNIPER  = 1u << 3,  // reserved: slow precise hold
    WPF_LADDER  = 1u << 4,  // reserved
    WPF_PRECISE = 1u << 5,  // reserved: strict angle match
};

struct Waypoint
{
    Vector pos;
    QAngle angle;
    EWaypointType type = EWaypointType::Normal;
    EBranchSelectMode branchMode = EBranchSelectMode::Sequential;
    std::vector<int> nextIndices;

    // Extended kinetic / pacing fields. Defaults keep old paths.json loading unchanged.
    std::uint32_t flags = 0;          // EWaypointFlag bitmask
    float radius = 0.0f;              // <=0 means use global waypointReachDistance
    float waitTime = 0.0f;            // seconds to pause on arrival, 0 = no wait
    float desiredSpeed = 0.0f;        // 0..1 hint; <0.75 enables shift-walk
    bool  hasGazeTarget = false;      // reserved: aim override (not yet consumed)
    Vector gazeTarget{};
};

class PathManager
{
public:
    explicit PathManager(const std::string& filename = "paths.json");
    ~PathManager();

    void AddWaypoint(const Vector& pos, const QAngle& angle, EWaypointType type = EWaypointType::Normal);
    bool RemoveWaypoint(size_t index);
    bool MoveWaypointUp(size_t index);
    bool MoveWaypointDown(size_t index);
    bool SetWaypointTransform(size_t index, const Vector& pos, const QAngle& angle);
    bool SetWaypointType(size_t index, EWaypointType type);
    bool SetWaypointBranchMode(size_t index, EBranchSelectMode branchMode);
    bool AddBranchConnection(size_t index, int targetIndex);
    bool RemoveBranchConnection(size_t index, int targetIndex);

    bool SetWaypointFlags(size_t index, std::uint32_t flags);
    bool SetWaypointRadius(size_t index, float radius);
    bool SetWaypointWaitTime(size_t index, float seconds);
    bool SetWaypointDesiredSpeed(size_t index, float speed);
    bool SetWaypointGazeTarget(size_t index, bool has, const Vector& target);

    std::vector<int> GetResolvedNextIndices(size_t index) const;
    int PeekNextWaypointIndex(size_t index) const;
    int ResolveNextWaypointIndex(size_t index);
    void ResetRuntimeState();
    void LoadFromFile();
    void SaveToFile();

    const std::vector<Waypoint>& GetWaypoints() const { return waypoints; }
    size_t GetWaypointCount() const { return waypoints.size(); }
    void Clear();

private:
    [[nodiscard]] bool IsValidWaypointIndex(size_t index) const noexcept;
    void EnsureRuntimeState();
    void NormalizeWaypointLinks();
    void RemapIndicesAfterSwap(size_t firstIndex, size_t secondIndex);
    void NormalizeAndSave();

    std::vector<Waypoint> waypoints;
    std::vector<int> branchCycleCursor;
    std::string filepath;
};
