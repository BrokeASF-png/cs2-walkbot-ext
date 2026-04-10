#include "process.h"
#include <cstdio>
#include <TlHelp32.h>

struct WindowSearchContext
{
    DWORD processId = 0;
    HWND hWnd = nullptr;
};

static BOOL CALLBACK EnumWindowsProc(HWND hWnd, LPARAM lParam)
{
    WindowSearchContext* pContext = reinterpret_cast<WindowSearchContext*>(lParam);
    if (!pContext)
        return FALSE;

    DWORD windowProcessId = 0;
    GetWindowThreadProcessId(hWnd, &windowProcessId);

    if (windowProcessId != pContext->processId)
        return TRUE;

    if (!IsWindowVisible(hWnd) || GetWindow(hWnd, GW_OWNER) != nullptr)
        return TRUE;

    pContext->hWnd = hWnd;
    return FALSE;
}

DWORD FindProcessId(const wchar_t* processName)
{
    HANDLE hSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (hSnap == INVALID_HANDLE_VALUE)
        return 0;

    PROCESSENTRY32W pe{};
    pe.dwSize = sizeof(pe);

    DWORD pid = 0;
    if (Process32FirstW(hSnap, &pe))
    {
        do
        {
            if (_wcsicmp(pe.szExeFile, processName) == 0)
            {
                pid = pe.th32ProcessID;
                break;
            }
        } while (Process32NextW(hSnap, &pe));
    }

    CloseHandle(hSnap);
    return pid;
}

bool AttachCS2()
{
    if (g_hProcess)
        return true;

    printf("[*] Searching for cs2.exe ...\n");

    g_dwProcessId = FindProcessId(L"cs2.exe");
    if (g_dwProcessId == 0)
    {
        printf("[!] cs2.exe not found\n");
        return false;
    }

    printf("[+] Found cs2.exe (PID = %lu)\n", g_dwProcessId);

    g_hProcess = OpenProcess(
        PROCESS_VM_READ | PROCESS_VM_WRITE | PROCESS_VM_OPERATION | PROCESS_QUERY_INFORMATION,
        FALSE,
        g_dwProcessId
    );

    if (g_hProcess)
    {
        printf("[+] OpenProcess succeeded (HANDLE = %p)\n", g_hProcess);
        return true;
    }
    else
    {
        printf("[!] OpenProcess failed (err = %lu)\n", GetLastError());
        printf("    Ensure running as administrator\n");
        return false;
    }
}

void DetachCS2()
{
    if (g_hProcess)
    {
        CloseHandle(g_hProcess);
        g_hProcess = nullptr;
    }

    g_dwProcessId = 0;
}

HWND GetProcessWindow(DWORD processId)
{
    if (processId == 0)
        return nullptr;

    WindowSearchContext context;
    context.processId = processId;
    EnumWindows(EnumWindowsProc, reinterpret_cast<LPARAM>(&context));
    return context.hWnd;
}

HWND GetCS2Window()
{
    return GetProcessWindow(g_dwProcessId);
}
