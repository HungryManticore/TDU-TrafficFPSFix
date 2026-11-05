#!/bin/sh
# Linux cross-build with clang + lld (no Windows SDK needed)
set -e
llvm-dlltool -m i386 -d kernel32.def -l kernel32.lib -k
clang --target=i686-pc-windows-msvc -fms-extensions -fasm-blocks -O2 -ffreestanding -fno-builtin \
      -fno-stack-protector -mno-sse -c TDU_TrafficFPSFix.c -o TDU_TrafficFPSFix.obj
lld-link /dll /nodefaultlib /entry:DllMain /subsystem:windows /machine:x86 /safeseh:no \
      /out:TDU_TrafficFPSFix.asi TDU_TrafficFPSFix.obj kernel32.lib
