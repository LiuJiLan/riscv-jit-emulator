# Project-root convenience Makefile — wraps cmake invocations for the 3 build types.
#
# 目的: 给非 CLion 用户 / CI / 命令行用户一个统一入口 (debug / release / tsan).
# CLion 用户继续走 GUI build profile, 不必 make.
#
# 关系:
#   - 主项目 CMakeLists.txt 已经 cover Debug / Release / Tsan 三个 build_type;
#     本 Makefile 只是 cmake 命令的 wrapper, 不动 CMakeLists.
#   - tags/Makefile 包 cmake -DCMAKE_BUILD_TYPE=Demo (WebTerm 演示用 fork),
#     跟本 Makefile 平行不冲突.
#   - tests/Makefile 是 fixture build (RV ELF / bin), 跟本 Makefile 完全独立.

BUILD_DIR_DEBUG   ?= cmake-build-debug
BUILD_DIR_RELEASE ?= cmake-build-release
BUILD_DIR_TSAN    ?= cmake-build-tsan
JOBS              ?= -j

.PHONY: help debug release tsan all clean clean-debug clean-release clean-tsan

help:
	@echo "Targets:"
	@echo "  debug        Build with ASan + UBSan (Debug build_type)."
	@echo "               Output: $(BUILD_DIR_DEBUG)/riscv_jit_emulator"
	@echo "  release      Build optimized -O2 (Release build_type, perf testing)."
	@echo "               Output: $(BUILD_DIR_RELEASE)/riscv_jit_emulator"
	@echo "  tsan         Build with TSan + UBSan (Tsan build_type, SMP race detection)."
	@echo "               Output: $(BUILD_DIR_TSAN)/riscv_jit_emulator"
	@echo "               Needs: sudo sysctl vm.mmap_rnd_bits=28 on Linux kernel 6.5+."
	@echo "  all          debug + release + tsan."
	@echo "  clean-debug / clean-release / clean-tsan / clean   Remove build dir(s)."
	@echo ""
	@echo "Notes:"
	@echo "  - Requires ninja (apt install ninja-build) — -G Ninja matches CLion default."
	@echo "  - CLion users: build via GUI profile, no need for this Makefile."
	@echo "  - tests/ fixture build: cd tests && make."
	@echo "  - Customize parallelism: make debug JOBS=\"-j8\"."

debug:
	cmake -B $(BUILD_DIR_DEBUG) -S . -DCMAKE_BUILD_TYPE=Debug -G Ninja
	cmake --build $(BUILD_DIR_DEBUG) $(JOBS)

release:
	cmake -B $(BUILD_DIR_RELEASE) -S . -DCMAKE_BUILD_TYPE=Release -G Ninja
	cmake --build $(BUILD_DIR_RELEASE) $(JOBS)

tsan:
	cmake -B $(BUILD_DIR_TSAN) -S . -DCMAKE_BUILD_TYPE=Tsan -G Ninja
	cmake --build $(BUILD_DIR_TSAN) $(JOBS)

all: debug release tsan

clean-debug:
	rm -rf $(BUILD_DIR_DEBUG)

clean-release:
	rm -rf $(BUILD_DIR_RELEASE)

clean-tsan:
	rm -rf $(BUILD_DIR_TSAN)

clean: clean-debug clean-release clean-tsan
