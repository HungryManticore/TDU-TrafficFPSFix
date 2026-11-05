# TDU_TrafficFPSFix

ASI plugin for **Test Drive Unlimited** (PC, `TestDriveUnlimited.exe` 1.66A,
sha256 `1408778de813902587ad83438bce52a606f8895c5d74478762ad1c9cf92d5a96`) that stops traffic and
AI cars from swerving between lanes when the game runs above 60 fps.

## Root cause (static analysis)

* Cars are simulated with a variable time step: the frame delta is cut into chunks of at most
  0.1 s (`0x9278E0`) and each chunk runs `Car::Update` (`0x926560`) and the physics-car step
  (`0x840640`). Above 60 fps every chunk is shorter than 1/60 s, and nothing caps it.
* AI and traffic cars are driven by "pilot" objects embedded in each car (`car+0x230`,
  `car+0x400`, selected via `car+0x710`). Every physics step the car asks the pilot for inputs
  (`0x936CD0` / `0x9D7920`), which runs the pilot's compute routine (`0x936830` / `0x9D75C0`):
  * a heading PID and a speed PID (generic PID `0x9077F0`; steer gains P = -4, D = -0.4 at
    `0xF7C358` / `0xF7C368`; derivative = dError/dt; 100-sample, 1 s integral history);
  * then a steering smoother with a **fixed per-call factor**
    `steer = pid*0.2 + prev*0.8` (`0x936AA6`, `0x9D78C4`; factor at `pilot+0x44`).
* That loop was tuned for 60 calls/s. At 144 fps the smoother's time constant is ~2.4x shorter,
  at 240 fps ~4x, the derivative is taken over ever smaller steps and the history window
  shrinks, so the steering loop loses its damping and the car oscillates across the lane.

## Fix

The two `call compute` instructions (`0x936CF7`, `0x9D7949`) are redirected to a wrapper that
evaluates the pilot at a fixed rate (default 60 Hz, dt = 1/60). dt is accumulated per pilot; on
physics steps in between, the car keeps its last throttle/brake/steer (`0x9378D0` still hands
them to the physics every step). That is exactly what the AI experiences at 60 fps. At or below
the target rate nothing changes. Player driving, physics and rendering are not touched.
Before patching, the plugin checks the bytes at both sites and skips them if they don't match.

## Fix 2: traffic lane-change logic

Even with the steering stabilised, traffic still changed lanes erratically. Traffic cars are run
by a state-machine driver at `car+0x720` (vtable `0xF0DAA8`, event handler `0x9DB3F0`, states at
`driver+0xC`: 1 ON ROAD, 9 LANE CHANGE, 10-14 BACK-OVERTAKE, junction states, ...). From ON ROAD
it enters LANE CHANGE when `0x93F980` returns true, and the manoeuvre itself is built by
`0x93F040` / `0x941370`. All of it is ticked once per frame from `Car::Update`
(`0x9265AE..0x9265F5`: `[car+0x22D8]`, route `[car+0x2200]`, driver `[car+0x1E10]`, pilot
`[car+0x710]`), with the frame dt and many instantaneous physics-derived inputs.

Rather than chase every frame-rate-sensitive input in that state machine, the plugin detours
that block (6-byte `jmp` at `0x9265AE`, original bytes verified first) and, for cars whose driver
type (`car+0x1E14`) is selected by `DriverTypes` (default: type 0 = traffic), runs the four
ticks at a fixed 60 Hz with dt = 1/60, which is exactly what the AI gets at 60 fps. Other cars,
physics and rendering still update every frame.

## Install

Copy `TDU_TrafficFPSFix.asi` and `TDU_TrafficFPSFix.ini` into the game's `scripts` folder
(needs Ultimate ASI Loader, e.g. as `dinput8.dll`).

## Build

* Windows / MSVC: `build.bat` from an x86 Native Tools prompt.
* Linux: `./build.sh` (clang + lld-link + llvm-dlltool).

## Status

Tested in-game on the 1.66A exe above: traffic no longer weaves or changes lanes erratically at
high frame rates. If you hit a case it doesn't cover, set `Log=1` and include
`TDU_TrafficFPSFix.log` with your report.

## AI disclosure

This plugin was developed with the help of an AI assistant (Claude, by Anthropic). The AI performed
the reverse engineering of `TestDriveUnlimited.exe` (static analysis with the REA/Ghidra tooling and
disassembly), identified the frame-rate-dependent AI code described above, and wrote the plugin
source, build scripts and this README. The fix was reviewed and tested in-game by the repository
owner.
