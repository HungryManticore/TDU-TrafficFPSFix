/*
 * TDU_TrafficFPSFix.c  -  ASI plugin for Test Drive Unlimited (PC, TestDriveUnlimited.exe 1.66A,
 * sha256 1408778de813902587ad83438bce52a606f8895c5d74478762ad1c9cf92d5a96)
 *
 * Fixes AI/traffic cars swerving between lanes when the game runs above 60 fps.
 *
 * ---------------------------------------------------------------------------------------------
 * What is going on (static analysis of the shipped exe)
 * ---------------------------------------------------------------------------------------------
 * Every car is simulated with a VARIABLE time step: the frame delta is cut into chunks of at
 * most 0.1 s (0x9278E0, max chunk = float @0xFAA9A0) and each chunk runs
 *      Car::Update(car, dt, time)        0x926560
 *      PhysCar::Step(physCar, dt)        0x840640
 * so above 60 fps all AI code sees dt < 1/60 s. Nothing caps it.
 *
 * AI cars (race opponents *and* traffic) are steered by "pilot" objects embedded in the car
 * (car+0x230 = pilot A, car+0x400 = pilot B, selected through car+0x710). Once per physics
 * step the car asks the pilot for its inputs through a controller interface:
 *      pilot A: 0x936CD0 -> 0x936830 (compute)  -> 0x9378D0 (write throttle/brake/steer)
 *      pilot B: 0x9D7920 -> 0x9D75C0 (compute)  -> 0x9378D0
 * The compute routines run two PID controllers (generic PID: 0x9077F0, modes 0x9076D0 /
 * 0x9075F0; steer gains P=-4, D=-0.4 @0xF7C358/0xF7C368, speed gains @0xF7C35C/0xF7C36C)
 * and then smooth the steering output with a FIXED per-call factor:
 *      steer = pid * 0.2 + steer_prev * 0.8          (0x936AA6 / 0x9D78C4, factor = pilot+0x44)
 * That filter is tuned for 60 calls per second; at 144 fps its time constant is ~2.4x shorter,
 * at 240 fps ~4x. Together with the derivative term (dError/dt evaluated over ever smaller
 * steps) and the PID history ring (100 samples, 1 s window - shorter than 1 s above 100 fps)
 * the steering loop loses its damping and the car oscillates across the lane.
 *
 * ---------------------------------------------------------------------------------------------
 * The fix
 * ---------------------------------------------------------------------------------------------
 * The two `call compute` instructions (0x936CF7 and 0x9D7949) are redirected through a small
 * wrapper that runs the pilot's decision code at a fixed rate (default 60 Hz) with dt = 1/60:
 * dt is accumulated per pilot, the PIDs only run when a full 1/60 s has elapsed, and on the
 * frames in between the car simply keeps the last throttle/brake/steer values (0x9378D0 still
 * runs every step and hands them to the physics). That is exactly what the AI does at 60 fps,
 * so its behaviour no longer depends on the frame rate. Below the target rate nothing changes.
 * Player driving, physics and rendering are untouched.
 *
 * Install: copy TDU_TrafficFPSFix.asi + TDU_TrafficFPSFix.ini into the game's "scripts" folder
 *          (Ultimate ASI Loader = dinput8.dll is already present in this installation).
 */

/* ------------------------------------------------------------------ minimal Win32 decls ---- */
typedef int            BOOL;
typedef unsigned long  DWORD;
typedef void*          HANDLE;
typedef void*          HMODULE;

#define WINAPI __stdcall
#define IMPORT __declspec(dllimport)

IMPORT BOOL    WINAPI VirtualProtect(void* addr, unsigned long size, DWORD prot, DWORD* old);
IMPORT HMODULE WINAPI GetModuleHandleA(const char* name);
IMPORT DWORD   WINAPI GetModuleFileNameA(HMODULE mod, char* buf, DWORD size);
IMPORT unsigned WINAPI GetPrivateProfileIntA(const char* app, const char* key, int def, const char* file);
IMPORT HANDLE  WINAPI CreateFileA(const char* name, DWORD access, DWORD share, void* sa, DWORD disp, DWORD flags, HANDLE tmpl);
IMPORT BOOL    WINAPI WriteFile(HANDLE h, const void* buf, DWORD n, DWORD* written, void* ov);
IMPORT DWORD   WINAPI SetFilePointer(HANDLE h, long dist, long* high, DWORD method);
IMPORT BOOL    WINAPI CloseHandle(HANDLE h);
IMPORT BOOL    WINAPI FlushInstructionCache(HANDLE proc, const void* addr, unsigned long size);
IMPORT HANDLE  WINAPI GetCurrentProcess(void);
IMPORT BOOL    WINAPI DisableThreadLibraryCalls(HMODULE mod);
IMPORT DWORD   WINAPI GetTickCount(void);

#define PAGE_EXECUTE_READWRITE 0x40
#define GENERIC_WRITE          0x40000000
#define FILE_SHARE_READ        0x00000001
#define OPEN_ALWAYS            4
#define CREATE_ALWAYS          2
#define FILE_END               2
#define INVALID_HANDLE_VALUE   ((HANDLE)(long)-1)

#ifdef __clang__
int _fltused = 0;   /* freestanding build: no CRT */
#endif

/* ------------------------------------------------------------------ game addresses -------- */
/* Image base of this exe is 0x400000 and it has no relocations, so absolute VAs are fine.   */
#define SITE_PILOT_A   0x00936CF7u   /* call 0x936830 inside 0x936CD0 (pilot A controller)   */
#define TARGET_PILOT_A 0x00936830u
#define SITE_PILOT_B   0x009D7949u   /* call 0x9D75C0 inside 0x9D7920 (pilot B controller)   */
#define TARGET_PILOT_B 0x009D75C0u

/* ------------------------------------------------------------------ state ----------------- */
static HMODULE g_self;
static char    g_iniPath[260];
static char    g_logPath[260];
static int     g_log;
static float   g_step = 1.0f / 60.0f;    /* fixed AI step                                  */

unsigned g_origA = TARGET_PILOT_A;      /* read by the naked hooks                         */
unsigned g_origB = TARGET_PILOT_B;

#define TABLE_SIZE 1024u                 /* power of two; ~2 pilots per car, < 200 cars     */
static struct { void* key; float acc; } g_tbl[TABLE_SIZE];

static unsigned g_statRun, g_statHeld, g_statPass;
static DWORD    g_statTick;

/* ------------------------------------------------------------------ tiny helpers ---------- */
static int StrLen(const char* s) { int n = 0; while (s[n]) ++n; return n; }
static char* StrCpy(char* d, const char* s) { while ((*d = *s++) != 0) ++d; return d; }
static char* UToA(char* d, unsigned v)
{
    char tmp[12]; int n = 0;
    do { tmp[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) *d++ = tmp[--n];
    *d = 0;
    return d;
}
static char* HexToA(char* d, unsigned v)
{
    static const char hx[] = "0123456789ABCDEF";
    int i;
    *d++ = '0'; *d++ = 'x';
    for (i = 7; i >= 0; --i) *d++ = hx[(v >> (i * 4)) & 15u];
    *d = 0;
    return d;
}

static void Log(const char* msg)
{
    HANDLE h;
    DWORD w;
    if (!g_log) return;
    h = CreateFileA(g_logPath, GENERIC_WRITE, FILE_SHARE_READ, 0, OPEN_ALWAYS, 0x80, 0);
    if (h == INVALID_HANDLE_VALUE) return;
    SetFilePointer(h, 0, 0, FILE_END);
    WriteFile(h, msg, (DWORD)StrLen(msg), &w, 0);
    WriteFile(h, "\r\n", 2, &w, 0);
    CloseHandle(h);
}

/* ------------------------------------------------------------------ core logic ------------ */
static float* AccFor(void* pilot)
{
    unsigned h = ((unsigned)pilot >> 4) * 2654435761u;
    unsigned i, n;
    for (n = 0; n < TABLE_SIZE; ++n)
    {
        i = (h + n) & (TABLE_SIZE - 1u);
        if (g_tbl[i].key == pilot) return &g_tbl[i].acc;
        if (g_tbl[i].key == 0) { g_tbl[i].key = pilot; g_tbl[i].acc = 0.0f; return &g_tbl[i].acc; }
    }
    return 0; /* table full (never expected): fall back to original behaviour */
}

/* Car-level AI tick (driver state machine, route, pilot targets) */
static unsigned g_tickRun, g_tickHeld, g_tickPass, g_tickOther;
static unsigned g_typeSeen[8];
static unsigned g_driverMask = 1u;      /* bit n = rate-limit cars whose driver type is n   */
static int      g_fixLaneLogic = 1;

static void Stats(void)
{
    char line[256], *p;
    unsigned i;
    DWORD now = GetTickCount();
    if (now - g_statTick < 10000u) return;
    g_statTick = now;
    p = StrCpy(line, "last 10s: pilot steps run=");
    p = UToA(p, g_statRun);
    p = StrCpy(p, " held=");
    p = UToA(p, g_statHeld);
    p = StrCpy(p, " pass=");
    p = UToA(p, g_statPass);
    p = StrCpy(p, " | car AI ticks run=");
    p = UToA(p, g_tickRun);
    p = StrCpy(p, " held=");
    p = UToA(p, g_tickHeld);
    p = StrCpy(p, " pass=");
    p = UToA(p, g_tickPass);
    p = StrCpy(p, " untouched=");
    p = UToA(p, g_tickOther);
    p = StrCpy(p, " | driver types t0..t4:");
    for (i = 0; i < 5; ++i) { *p++ = ' '; p = UToA(p, g_typeSeen[i]); g_typeSeen[i] = 0; }
    Log(line);
    g_statRun = g_statHeld = g_statPass = 0;
    g_tickRun = g_tickHeld = g_tickPass = g_tickOther = 0;
}

/* Fixed-rate gate shared by both fixes. Returns the dt (raw float bits) to run the original
 * code with, or 0 if this step should be skipped (previous AI state is kept). */
static unsigned Gate(void* key, unsigned dtBits, unsigned* run, unsigned* held, unsigned* pass)
{
    union { unsigned u; float f; } in, out;
    float* acc;
    in.u = dtBits;

    if (g_log) Stats();

    if (!(in.f < g_step * 0.9f))     /* frame step ~>= 1/60 s: unchanged original behaviour */
    {
        acc = AccFor(key);
        if (acc) *acc = 0.0f;
        ++*pass;
        return dtBits;
    }

    acc = AccFor(key);
    if (!acc) return dtBits;

    *acc += in.f;
    if (*acc + 1e-6f < g_step)
    {
        ++*held;
        return 0;
    }

    *acc -= g_step;
    if (*acc > g_step) *acc = 0.0f;  /* never let a backlog build up */
    ++*run;
    out.f = g_step;
    return out.u;
}

/* Pilot compute (steering / speed PIDs). */
unsigned __fastcall Accumulate(void* pilot, unsigned dtBits)
{
    return Gate(pilot, dtBits, &g_statRun, &g_statHeld, &g_statPass);
}

/* Car::Update AI block (0x9265AE..0x9265F5): [car+0x22D8], route [car+0x2200],
 * driver [car+0x1E10] (traffic: lane-change state machine), pilot [car+0x710]. */
unsigned __fastcall CarAITick(unsigned char* car, unsigned dtBits)
{
    unsigned type = *(unsigned*)(car + 0x1E14);
    if (type < 8u) ++g_typeSeen[type];
    if (type >= 32u || !(g_driverMask & (1u << type)))
    {
        ++g_tickOther;
        if (g_log) Stats();
        return dtBits;
    }
    return Gate(car, dtBits, &g_tickRun, &g_tickHeld, &g_tickPass);
}

/* Naked wrappers that replace `call compute` at the two sites.
 * Register contract of the originals (custom, set up by the caller):
 *   ESI = pilot, EBX = physics car (A), EAX = physics car (B), [esp+4] = dt, callee pops 4.
 * We must keep EAX/EBX/ESI intact for the original; ECX/EDX are dead at both call sites. */
__declspec(naked) void HookPilotA(void)
{
    __asm {
        push eax
        mov  ecx, esi
        mov  edx, [esp + 8]          /* dt */
        call Accumulate
        test eax, eax
        jz   holdA
        mov  [esp + 8], eax          /* fixed dt */
        pop  eax
        jmp  dword ptr [g_origA]     /* original returns with ret 4 to our caller */
    holdA:
        pop  eax
        ret  4
    }
}

__declspec(naked) void HookPilotB(void)
{
    __asm {
        push eax
        mov  ecx, esi
        mov  edx, [esp + 8]
        call Accumulate
        test eax, eax
        jz   holdB
        mov  [esp + 8], eax
        pop  eax
        jmp  dword ptr [g_origB]
    holdB:
        pop  eax
        ret  4
    }
}

/* Replaces 0x9265AE..0x9265F5 inside Car::Update (0x926560).
 * Context there: EBX = car, EDI = dt (float bits), EBP = frame (time at [ebp+0x10]).
 * After 0x9265F5 the original only uses EBX/EBP and reloads ESI/EDI, so both are free. */
unsigned g_carAIRet = 0x009265F5u;

__declspec(naked) void HookCarAI(void)
{
    __asm {
        mov  esi, [ebp + 0x10]          /* time, as the original block does   */
        mov  ecx, ebx
        mov  edx, edi
        call CarAITick
        test eax, eax
        jz   skipAI
        mov  edi, eax                   /* dt for the four calls              */
        mov  esi, [ebp + 0x10]

        mov  ecx, [ebx + 0x22D8]
        test ecx, ecx
        jz   c1
        mov  eax, [ecx]
        push esi
        push edi
        call dword ptr [eax + 8]
    c1:
        mov  ecx, [ebx + 0x2200]
        test ecx, ecx
        jz   c2
        mov  edx, [ecx]
        push esi
        push edi
        call dword ptr [edx + 8]
    c2:
        mov  ecx, [ebx + 0x1E10]
        test ecx, ecx
        jz   c3
        mov  eax, [ecx]
        push esi
        push edi
        call dword ptr [eax + 8]
    c3:
        mov  ecx, [ebx + 0x710]
        test ecx, ecx
        jz   skipAI
        mov  edx, [ecx]
        push esi
        push edi
        call dword ptr [edx + 8]
    skipAI:
        jmp  dword ptr [g_carAIRet]
    }
}

/* ------------------------------------------------------------------ patching -------------- */
static const unsigned char kCarAIBlock[] = {
    0x8b,0x8b,0xd8,0x22,0x00,0x00,0x85,0xc9,0x8b,0x75,0x10,0x74,0x07,0x8b,0x01,0x56,0x57,0xff,0x50,0x08,
    0x8b,0x8b,0x00,0x22,0x00,0x00,0x85,0xc9,0x74,0x07,0x8b,0x11,0x56,0x57,0xff,0x52,0x08,
    0x8b,0x8b,0x10,0x1e,0x00,0x00,0x85,0xc9,0x74,0x07,0x8b,0x01,0x56,0x57,0xff,0x50,0x08,
    0x8b,0x8b,0x10,0x07,0x00,0x00,0x85,0xc9,0x74,0x07,0x8b,0x11,0x56,0x57,0xff,0x52,0x08
};
#define SITE_CAR_AI 0x009265AEu

static int PatchCarAI(void)
{
    unsigned char* p = (unsigned char*)SITE_CAR_AI;
    char line[160], *q;
    DWORD old;
    unsigned i;

    for (i = 0; i < sizeof(kCarAIBlock); ++i)
        if (p[i] != kCarAIBlock[i])
        {
            q = StrCpy(line, "SKIP car AI tick: unexpected bytes at ");
            HexToA(q, SITE_CAR_AI + i);
            Log(line);
            return 0;
        }
    if (!VirtualProtect(p, 6, PAGE_EXECUTE_READWRITE, &old)) return 0;
    p[0] = 0xE9;
    *(int*)(p + 1) = (int)((unsigned)HookCarAI - (SITE_CAR_AI + 5));
    p[5] = 0x90;
    VirtualProtect(p, 6, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p, 6);
    q = StrCpy(line, "OK   car AI tick (route/driver/lane-change FSM) hooked at ");
    HexToA(q, SITE_CAR_AI);
    Log(line);
    return 1;
}
static int PatchCall(unsigned site, unsigned expectedTarget, void* hook, const char* name)
{
    unsigned char* p = (unsigned char*)site;
    char line[160], *q;
    DWORD old;
    int rel;

    if (p[0] != 0xE8 || (unsigned)(site + 5 + *(int*)(p + 1)) != expectedTarget)
    {
        q = StrCpy(line, "SKIP ");
        q = StrCpy(q, name);
        q = StrCpy(q, ": unexpected bytes at ");
        HexToA(q, site);
        Log(line);
        return 0;
    }
    rel = (int)((unsigned)hook - (site + 5));
    if (!VirtualProtect(p, 5, PAGE_EXECUTE_READWRITE, &old)) return 0;
    *(int*)(p + 1) = rel;
    VirtualProtect(p, 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), p, 5);

    q = StrCpy(line, "OK   ");
    q = StrCpy(q, name);
    q = StrCpy(q, " hooked at ");
    HexToA(q, site);
    Log(line);
    return 1;
}

static void Init(void)
{
    char* dot = 0, *p;
    unsigned hz;
    HANDLE h;

    GetModuleFileNameA(g_self, g_iniPath, sizeof(g_iniPath));
    for (p = g_iniPath; *p; ++p) if (*p == '.') dot = p;
    if (!dot) return;
    StrCpy(g_logPath, g_iniPath);
    StrCpy(dot, ".ini");
    StrCpy(g_logPath + (dot - g_iniPath), ".log");

    if (!GetPrivateProfileIntA("TrafficFPSFix", "Enabled", 1, g_iniPath)) return;
    g_log = (int)GetPrivateProfileIntA("TrafficFPSFix", "Log", 0, g_iniPath);
    hz = GetPrivateProfileIntA("TrafficFPSFix", "AIRate", 60, g_iniPath);
    if (hz < 20u) hz = 20u;
    if (hz > 1000u) hz = 1000u;
    g_step = 1.0f / (float)hz;
    g_fixLaneLogic = (int)GetPrivateProfileIntA("TrafficFPSFix", "FixLaneLogic", 1, g_iniPath);
    g_driverMask = GetPrivateProfileIntA("TrafficFPSFix", "DriverTypes", 1, g_iniPath);

    if (g_log)
    {
        h = CreateFileA(g_logPath, GENERIC_WRITE, FILE_SHARE_READ, 0, CREATE_ALWAYS, 0x80, 0);
        if (h != INVALID_HANDLE_VALUE) CloseHandle(h);
        {
            char line[96];
            p = StrCpy(line, "TDU_TrafficFPSFix: AI rate ");
            p = UToA(p, hz);
            StrCpy(p, " Hz");
            Log(line);
        }
        g_statTick = GetTickCount();
    }

    /* Only touch memory if the host is exactly the supported exe (base 0x400000, PE timestamp
     * 0x468E36BE, SizeOfImage 0x12AF000). */
    {
        unsigned char* base = (unsigned char*)GetModuleHandleA(0);
        unsigned pe;
        if (base != (unsigned char*)0x400000u) { Log("SKIP: host exe base is not 0x400000"); return; }
        pe = *(unsigned*)(base + 0x3C);
        if (*(unsigned*)(base + pe + 8) != 0x468E36BEu || *(unsigned*)(base + pe + 24 + 56) != 0x012AF000u)
        {
            Log("SKIP: unsupported TestDriveUnlimited.exe build (expected 1.66A, timestamp 0x468E36BE)");
            return;
        }
    }

    PatchCall(SITE_PILOT_A, TARGET_PILOT_A, (void*)HookPilotA, "pilot A (0x936830)");
    PatchCall(SITE_PILOT_B, TARGET_PILOT_B, (void*)HookPilotB, "pilot B (0x9D75C0)");
    if (g_fixLaneLogic) PatchCarAI();
}

BOOL WINAPI DllMain(HMODULE mod, DWORD reason, void* reserved)
{
    (void)reserved;
    if (reason == 1 /* DLL_PROCESS_ATTACH */)
    {
        g_self = mod;
        DisableThreadLibraryCalls(mod);
        Init();
    }
    return 1;
}
