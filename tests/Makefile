# tests/Makefile —— 递归构建所有 fixture。
#
# 自动发现: 任何 tests/<milestone>/<sub>/<NN_xxx>/Makefile 都被纳入。
# 新加测试目录, 只要带 Makefile, 不用动这个文件。
#
# 用法:
#   make           递归 make 所有子目录(等价 make all)
#   make clean     递归 make clean
#   make -j8       并行(各子目录独立, 安全)
#
# CLion 用法:
#   Run Configuration → Before launch → 加 External Tool:
#     Program:           make
#     Arguments:         (空) 或 -j$(nproc)
#     Working directory: $ProjectFileDir$/tests

SUBDIRS := $(patsubst %/Makefile,%,$(wildcard */*/*/Makefile))

.PHONY: all clean list $(SUBDIRS)

all: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

clean:
	@for d in $(SUBDIRS); do $(MAKE) -C $$d clean; done

list:
	@echo "discovered test dirs:"
	@for d in $(SUBDIRS); do echo "  $$d"; done
