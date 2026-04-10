#pragma once
#include <Windows.h>
#include <cstdint>

inline HANDLE  g_hProcess  = nullptr;
inline DWORD   g_dwProcessId = 0;

// Open cs2.exe by name, returns true on success
bool AttachCS2();
void DetachCS2();
HWND GetProcessWindow(DWORD processId);
HWND GetCS2Window();

// Check if attached
inline bool IsCS2Attached()
{
    return g_hProcess != nullptr;
}
