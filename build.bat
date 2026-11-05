@echo off
rem Build TDU_TrafficFPSFix.asi with MSVC -- MUST be 32-bit (the game is x86).
rem Run from an "x86 Native Tools Command Prompt for VS" (or call vcvars32.bat first).
cl /nologo /LD /O2 /MT /GS- /W3 TDU_TrafficFPSFix.c /link /MACHINE:X86 /SUBSYSTEM:WINDOWS /SAFESEH:NO /OUT:TDU_TrafficFPSFix.asi kernel32.lib
if errorlevel 1 goto :eof
echo Built TDU_TrafficFPSFix.asi - copy it with TDU_TrafficFPSFix.ini into the game's "scripts" folder.
