#include "path.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace
{
    bool TryReadVec3(const json& value, Vector& outVector)
    {
        if (!value.is_array())
            return false;

        if (!value[0].is_number() || !value[1].is_number() || !value[2].is_number())
            return false;

        outVector.m_flX = value[0].get<float>();
        outVector.m_flY = value[1].get<float>();
        outVector.m_flZ = value[2].get<float>();
        return true;
    }

    bool TryReadAngles(const json& value, QAngle& outAngles)
    {
        if (!value.is_array())
            return false;

        if (!value[0].is_number() || !value[1].is_number() || !value[2].is_number())
            return false;

        outAngles.m_flPitch = value[0].get<float>();
        outAngles.m_flYaw = value[1].get<float>();
        outAngles.m_flRoll = value[2].get<float>();
        return true;
    }

    EWaypointType ParseWaypointType(const json& value)
    {
        if (!value.is_number())
            return EWaypointType::Normal;

        const int rawType = value.get<int>();
        return rawType == static_cast<int>(EWaypointType::MultiBranch)
            ? EWaypointType::MultiBranch
            : EWaypointType::Normal;
    }

    EBranchSelectMode ParseBranchMode(const json& value)
    {
        if (!value.is_number())
            return EBranchSelectMode::Sequential;

        const int rawMode = value.get<int>();
        if (rawMode == static_cast<int>(EBranchSelectMode::Random))
            return EBranchSelectMode::Random;
        if (rawMode == static_cast<int>(EBranchSelectMode::Cycle))
            return EBranchSelectMode::Cycle;
        return EBranchSelectMode::Sequential;
    }
}

PathManager::PathManager(const std::string& filename)
    : filepath(filename)
{
    LoadFromFile();
}

PathManager::~PathManager()
{
    SaveToFile();
}

void PathManager::AddWaypoint(const Vector& pos, const QAngle& angle, EWaypointType type)
{
    waypoints.push_back({ pos, angle, type });
    NormalizeAndSave();
}

bool PathManager::RemoveWaypoint(size_t index)
{
    if (!IsValidWaypointIndex(index))
        return false;

    waypoints.erase(waypoints.begin() + static_cast<std::ptrdiff_t>(index));

    for (Waypoint& waypoint : waypoints)
    {
        for (int& targetIndex : waypoint.nextIndices)
        {
            if (targetIndex == static_cast<int>(index))
                targetIndex = -1;
            else if (targetIndex > static_cast<int>(index))
                --targetIndex;
        }
    }

    NormalizeAndSave();
    return true;
}

bool PathManager::MoveWaypointUp(size_t index)
{
    if (index == 0 || !IsValidWaypointIndex(index))
        return false;

    std::swap(waypoints[index - 1], waypoints[index]);
    RemapIndicesAfterSwap(index - 1, index);
    NormalizeAndSave();
    return true;
}

bool PathManager::MoveWaypointDown(size_t index)
{
    if (!IsValidWaypointIndex(index) || !IsValidWaypointIndex(index + 1))
        return false;

    std::swap(waypoints[index], waypoints[index + 1]);
    RemapIndicesAfterSwap(index, index + 1);
    NormalizeAndSave();
    return true;
}

bool PathManager::SetWaypointTransform(size_t index, const Vector& pos, const QAngle& angle)
{
    if (!IsValidWaypointIndex(index))
        return false;

    waypoints[index].pos = pos;
    waypoints[index].angle = angle;
    SaveToFile();
    return true;
}

bool PathManager::SetWaypointType(size_t index, EWaypointType type)
{
    if (!IsValidWaypointIndex(index))
        return false;

    Waypoint& waypoint = waypoints[index];
    waypoint.type = type;
    if (type != EWaypointType::MultiBranch)
    {
        waypoint.nextIndices.clear();
        waypoint.branchMode = EBranchSelectMode::Sequential;
    }

    NormalizeAndSave();
    return true;
}

bool PathManager::SetWaypointBranchMode(size_t index, EBranchSelectMode branchMode)
{
    if (!IsValidWaypointIndex(index))
        return false;

    Waypoint& waypoint = waypoints[index];
    if (waypoint.type != EWaypointType::MultiBranch)
    {
        waypoint.branchMode = EBranchSelectMode::Sequential;
        SaveToFile();
        return true;
    }

    waypoint.branchMode = branchMode;
    EnsureRuntimeState();
    branchCycleCursor[index] = 0;
    SaveToFile();
    return true;
}

bool PathManager::AddBranchConnection(size_t index, int targetIndex)
{
    if (!IsValidWaypointIndex(index))
        return false;

    Waypoint& waypoint = waypoints[index];
    if (waypoint.type != EWaypointType::MultiBranch)
        return false;

    if (targetIndex < 0 ||
        targetIndex >= static_cast<int>(waypoints.size()) ||
        targetIndex == static_cast<int>(index))
    {
        return false;
    }

    waypoint.nextIndices.push_back(targetIndex);
    NormalizeAndSave();
    return true;
}

bool PathManager::SetWaypointFlags(size_t index, std::uint32_t flags)
{
    if (!IsValidWaypointIndex(index))
        return false;

    waypoints[index].flags = flags;
    SaveToFile();
    return true;
}

bool PathManager::SetWaypointRadius(size_t index, float radius)
{
    if (!IsValidWaypointIndex(index))
        return false;

    if (radius < 0.0f)
        radius = 0.0f;
    waypoints[index].radius = radius;
    SaveToFile();
    return true;
}

bool PathManager::SetWaypointWaitTime(size_t index, float seconds)
{
    if (!IsValidWaypointIndex(index))
        return false;

    if (seconds < 0.0f)
        seconds = 0.0f;
    waypoints[index].waitTime = seconds;
    SaveToFile();
    return true;
}

bool PathManager::SetWaypointDesiredSpeed(size_t index, float speed)
{
    if (!IsValidWaypointIndex(index))
        return false;

    if (speed < 0.0f)
        speed = 0.0f;
    if (speed > 1.0f)
        speed = 1.0f;
    waypoints[index].desiredSpeed = speed;
    SaveToFile();
    return true;
}

bool PathManager::SetWaypointGazeTarget(size_t index, bool has, const Vector& target)
{
    if (!IsValidWaypointIndex(index))
        return false;

    waypoints[index].hasGazeTarget = has;
    waypoints[index].gazeTarget = target;
    SaveToFile();
    return true;
}

bool PathManager::RemoveBranchConnection(size_t index, int targetIndex)
{
    if (!IsValidWaypointIndex(index))
        return false;

    Waypoint& waypoint = waypoints[index];
    if (waypoint.type != EWaypointType::MultiBranch)
        return false;

    const auto it = std::remove(waypoint.nextIndices.begin(), waypoint.nextIndices.end(), targetIndex);
    if (it == waypoint.nextIndices.end())
        return false;

    waypoint.nextIndices.erase(it, waypoint.nextIndices.end());
    NormalizeAndSave();
    return true;
}

std::vector<int> PathManager::GetResolvedNextIndices(size_t index) const
{
    std::vector<int> nextIndices;
    if (!IsValidWaypointIndex(index))
        return nextIndices;

    const Waypoint& waypoint = waypoints[index];
    if (waypoint.type == EWaypointType::MultiBranch && !waypoint.nextIndices.empty())
        return waypoint.nextIndices;

    if (index + 1 < waypoints.size())
        nextIndices.push_back(static_cast<int>(index + 1));

    return nextIndices;
}

int PathManager::PeekNextWaypointIndex(size_t index) const
{
    if (!IsValidWaypointIndex(index) || index >= branchCycleCursor.size())
        return -1;

    const Waypoint& waypoint = waypoints[index];
    const std::vector<int> nextIndices = GetResolvedNextIndices(index);
    if (nextIndices.empty())
        return -1;

    if (waypoint.type == EWaypointType::MultiBranch &&
        waypoint.branchMode == EBranchSelectMode::Cycle &&
        nextIndices.size() > 1)
    {
        int cursor = branchCycleCursor[index];
        if (cursor < 0)
            cursor = 0;

        return nextIndices[static_cast<std::size_t>(cursor) % nextIndices.size()];
    }

    return nextIndices.front();
}

int PathManager::ResolveNextWaypointIndex(size_t index)
{
    if (!IsValidWaypointIndex(index))
        return -1;

    EnsureRuntimeState();

    const Waypoint& waypoint = waypoints[index];
    const std::vector<int> nextIndices = GetResolvedNextIndices(index);
    if (nextIndices.empty())
        return -1;

    if (waypoint.type != EWaypointType::MultiBranch || nextIndices.size() == 1)
        return PeekNextWaypointIndex(index);

    switch (waypoint.branchMode)
    {
    case EBranchSelectMode::Random:
        return nextIndices[static_cast<std::size_t>(std::rand() % static_cast<int>(nextIndices.size()))];

    case EBranchSelectMode::Cycle:
    {
        int& cursor = branchCycleCursor[index];
        if (cursor < 0)
            cursor = 0;

        const int resolvedIndex = nextIndices[static_cast<std::size_t>(cursor) % nextIndices.size()];
        cursor = (cursor + 1) % static_cast<int>(nextIndices.size());
        return resolvedIndex;
    }

    case EBranchSelectMode::Sequential:
    default:
        return nextIndices.front();
    }
}

void PathManager::ResetRuntimeState()
{
    EnsureRuntimeState();
    std::fill(branchCycleCursor.begin(), branchCycleCursor.end(), 0);
}

void PathManager::LoadFromFile()
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::printf("[*] No existing path file, starting fresh\n");
        return;
    }

    try
    {
        json data = json::parse(file);
        waypoints.clear();
        branchCycleCursor.clear();

        if (data.contains("waypoints") && data["waypoints"].is_array())
        {
            json waypointsJson = data["waypoints"];
            for (auto wpIt = waypointsJson.begin(); wpIt != waypointsJson.end(); ++wpIt)
            {
                const json wp = *wpIt;
                if (!wp.is_object() || !wp.contains("pos") || !wp.contains("angle"))
                    continue;

                Vector pos{};
                QAngle angle{};
                if (!TryReadVec3(wp["pos"], pos) || !TryReadAngles(wp["angle"], angle))
                    continue;

                Waypoint waypoint{};
                waypoint.pos = pos;
                waypoint.angle = angle;
                waypoint.type = wp.contains("type")
                    ? ParseWaypointType(wp["type"])
                    : EWaypointType::Normal;
                waypoint.branchMode = wp.contains("branch_mode")
                    ? ParseBranchMode(wp["branch_mode"])
                    : EBranchSelectMode::Sequential;

                if (wp.contains("next_indices") && wp["next_indices"].is_array())
                {
                    json nextIndicesJson = wp["next_indices"];
                    for (auto nextIt = nextIndicesJson.begin(); nextIt != nextIndicesJson.end(); ++nextIt)
                    {
                        const json nextIndex = *nextIt;
                        if (nextIndex.is_number())
                            waypoint.nextIndices.push_back(nextIndex.get<int>());
                    }
                }

                if (wp.contains("flags") && wp["flags"].is_number())
                {
                    const int rawFlags = wp["flags"].get<int>();
                    waypoint.flags = static_cast<std::uint32_t>(rawFlags < 0 ? 0 : rawFlags);
                }

                if (wp.contains("radius") && wp["radius"].is_number())
                {
                    const float radius = wp["radius"].get<float>();
                    waypoint.radius = radius > 0.0f ? radius : 0.0f;
                }

                if (wp.contains("wait_time") && wp["wait_time"].is_number())
                {
                    const float seconds = wp["wait_time"].get<float>();
                    waypoint.waitTime = seconds > 0.0f ? seconds : 0.0f;
                }

                if (wp.contains("desired_speed") && wp["desired_speed"].is_number())
                {
                    float speed = wp["desired_speed"].get<float>();
                    if (speed < 0.0f) speed = 0.0f;
                    if (speed > 1.0f) speed = 1.0f;
                    waypoint.desiredSpeed = speed;
                }

                if (wp.contains("gaze_target") && wp["gaze_target"].is_array())
                {
                    Vector gaze{};
                    if (TryReadVec3(wp["gaze_target"], gaze))
                    {
                        waypoint.gazeTarget = gaze;
                        waypoint.hasGazeTarget = true;
                    }
                }

                waypoints.push_back(waypoint);
            }
        }

        EnsureRuntimeState();
        NormalizeWaypointLinks();
        std::printf("[+] Loaded %zu waypoints from %s\n", waypoints.size(), filepath.c_str());
    }
    catch (const std::exception& e)
    {
        std::printf("[!] Error loading path file: %s\n", e.what());
    }
}

void PathManager::SaveToFile()
{
    try
    {
        EnsureRuntimeState();
        NormalizeWaypointLinks();

        json waypointsArr = json::array();

        for (const Waypoint& wp : waypoints)
        {
            json waypoint;
            json posArr = json::array();
            posArr.push_back(wp.pos.m_flX);
            posArr.push_back(wp.pos.m_flY);
            posArr.push_back(wp.pos.m_flZ);
            waypoint["pos"] = posArr;

            json angleArr = json::array();
            angleArr.push_back(wp.angle.m_flPitch);
            angleArr.push_back(wp.angle.m_flYaw);
            angleArr.push_back(wp.angle.m_flRoll);
            waypoint["angle"] = angleArr;
            waypoint["type"] = static_cast<int>(wp.type);
            waypoint["branch_mode"] = static_cast<int>(wp.branchMode);

            json nextIndicesArr = json::array();
            for (int nextIndex : wp.nextIndices)
                nextIndicesArr.push_back(nextIndex);
            waypoint["next_indices"] = nextIndicesArr;

            // Only serialize extended fields when non-default so untouched
            // waypoints keep the original on-disk layout.
            if (wp.flags != 0u)
                waypoint["flags"] = static_cast<int>(wp.flags);
            if (wp.radius > 0.0f)
                waypoint["radius"] = wp.radius;
            if (wp.waitTime > 0.0f)
                waypoint["wait_time"] = wp.waitTime;
            if (wp.desiredSpeed > 0.0f)
                waypoint["desired_speed"] = wp.desiredSpeed;
            if (wp.hasGazeTarget)
            {
                json gazeArr = json::array();
                gazeArr.push_back(wp.gazeTarget.m_flX);
                gazeArr.push_back(wp.gazeTarget.m_flY);
                gazeArr.push_back(wp.gazeTarget.m_flZ);
                waypoint["gaze_target"] = gazeArr;
            }

            waypointsArr.push_back(waypoint);
        }

        json data;
        data["waypoints"] = waypointsArr;

        std::ofstream file(filepath);
        if (!file.is_open())
        {
            std::printf("[!] Error saving path file: cannot open %s\n", filepath.c_str());
            return;
        }

        file << data.dump(2);
        std::printf("[+] Saved %zu waypoints to %s\n", waypoints.size(), filepath.c_str());
    }
    catch (const std::exception& e)
    {
        std::printf("[!] Error saving path file: %s\n", e.what());
    }
}

void PathManager::Clear()
{
    waypoints.clear();
    branchCycleCursor.clear();
    SaveToFile();
}

bool PathManager::IsValidWaypointIndex(size_t index) const noexcept
{
    return index < waypoints.size();
}

void PathManager::EnsureRuntimeState()
{
    branchCycleCursor.resize(waypoints.size(), 0);
}

void PathManager::NormalizeWaypointLinks()
{
    EnsureRuntimeState();

    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
        Waypoint& waypoint = waypoints[i];
        if (waypoint.type != EWaypointType::MultiBranch)
        {
            waypoint.nextIndices.clear();
            waypoint.branchMode = EBranchSelectMode::Sequential;
            branchCycleCursor[i] = 0;
            continue;
        }

        waypoint.nextIndices.erase(
            std::remove_if(
                waypoint.nextIndices.begin(),
                waypoint.nextIndices.end(),
                [this, i](int targetIndex)
                {
                    return targetIndex < 0 ||
                        targetIndex >= static_cast<int>(waypoints.size()) ||
                        targetIndex == static_cast<int>(i);
                }
            ),
            waypoint.nextIndices.end()
        );

        std::sort(waypoint.nextIndices.begin(), waypoint.nextIndices.end());
        waypoint.nextIndices.erase(
            std::unique(waypoint.nextIndices.begin(), waypoint.nextIndices.end()),
            waypoint.nextIndices.end()
        );

        if (branchCycleCursor[i] >= static_cast<int>(waypoint.nextIndices.size()))
            branchCycleCursor[i] = 0;
    }
}

void PathManager::RemapIndicesAfterSwap(std::size_t firstIndex, std::size_t secondIndex)
{
    for (Waypoint& waypoint : waypoints)
    {
        for (int& targetIndex : waypoint.nextIndices)
        {
            if (targetIndex == static_cast<int>(firstIndex))
                targetIndex = static_cast<int>(secondIndex);
            else if (targetIndex == static_cast<int>(secondIndex))
                targetIndex = static_cast<int>(firstIndex);
        }
    }
}

void PathManager::NormalizeAndSave()
{
    SaveToFile();
}
