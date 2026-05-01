//
// Created by liujilan on 2026/4/28.
// a_01 loader 实现。
//

#include "loader.h"
#include "config.h"
#include "platform/ram.h"

#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// 从 fd 的 offset 处完整读 count 字节到 buf。
// pread 路线 —— bin 和 elf 共用同一份 IO 纪律(EINTR 重试 / 短读循环 / 意外 EOF 报错)。
// 成功返回 0, 失败返回 -1(内部已 fprintf)。
// `path` / `what` 仅用于报错文案。
static int read_full_at(int fd, void *buf, size_t count, off_t offset,
                        const char *path, const char *what) {
    uint8_t *p = (uint8_t *)buf;
    size_t remaining = count;
    off_t cur = offset;
    while (remaining > 0) {
        ssize_t n;
        do {
            n = pread(fd, p, remaining, cur);
        } while (n < 0 && errno == EINTR);
        if (n < 0) {
            fprintf(stderr,
                    "read_full_at: pread(%s) failed at offset %lld for %s: %s\n",
                    path, (long long)cur, what, strerror(errno));
            return -1;
        }
        if (n == 0) {
            fprintf(stderr,
                    "read_full_at: %s: unexpected EOF reading %s at offset %lld, %zu bytes left\n",
                    path, what, (long long)cur, remaining);
            return -1;
        }
        p += (size_t)n;
        cur += n;
        remaining -= (size_t)n;
    }
    return 0;
}

int guest_load_bin(const char *path, uint64_t dst) {
    if (dst < GUEST_RAM_START ||
        dst >= GUEST_RAM_START + GUEST_RAM_SIZE) {
        fprintf(stderr,
                "guest_load_bin: dst 0x%" PRIx64 " out of RAM range\n",
                dst);
        return -1;
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "guest_load_bin: open(%s) failed: %s\n",
                path, strerror(errno));
        return -1;
    }

    struct stat st;
    if (fstat(fd, &st) != 0) {
        fprintf(stderr, "guest_load_bin: fstat(%s) failed: %s\n",
                path, strerror(errno));
        close(fd);
        return -1;
    }

    size_t bytes = (size_t)st.st_size;
    size_t available = (size_t)((GUEST_RAM_START + GUEST_RAM_SIZE) - dst);
    if (bytes > available) {
        fprintf(stderr,
                "guest_load_bin: file too big: bytes=%zu, available=%zu\n",
                bytes, available);
        close(fd);
        return -1;
    }

    if (bytes > 0) {
        uint8_t *host_dst = gpa_to_hva_offset + dst;
        if (read_full_at(fd, host_dst, bytes, 0, path, ".bin contents") != 0) {
            close(fd);
            return -1;
        }
    }

    if (close(fd) != 0) {
        fprintf(stderr, "guest_load_bin: close(%s) failed: %s\n",
                path, strerror(errno));
        return -1;
    }

    return 0;
}

int guest_load_elf(const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "guest_load_elf: open(%s) failed: %s\n",
                path, strerror(errno));
        return -1;
    }

    // 1. 读 Elf32_Ehdr 全身(52B), 然后做所有头部校验。
    Elf32_Ehdr ehdr;
    if (read_full_at(fd, &ehdr, sizeof(ehdr), 0, path, "Elf32_Ehdr") != 0) {
        close(fd);
        return -1;
    }

    // 1a. magic
    if (ehdr.e_ident[EI_MAG0] != ELFMAG0 || ehdr.e_ident[EI_MAG1] != ELFMAG1 ||
        ehdr.e_ident[EI_MAG2] != ELFMAG2 || ehdr.e_ident[EI_MAG3] != ELFMAG3) {
        fprintf(stderr, "guest_load_elf: %s: not an ELF file (bad magic)\n",
                path);
        close(fd);
        return -1;
    }

    // 1b. class —— a_01: 当前模拟器写死 RV32, 只接 ELFCLASS32。
    //     未来上 RV64 后, 这里要变成"按当前模拟器配置选 32 / 64"。
    //     配置传递机制(宏 / 运行时变量)留到那时定, 不预先抽象。
    if (ehdr.e_ident[EI_CLASS] == ELFCLASS64) {
        fprintf(stderr,
                "guest_load_elf: %s: ELFCLASS64 not supported in current build (RV32 only)\n",
                path);
        close(fd);
        return -1;
    }
    if (ehdr.e_ident[EI_CLASS] != ELFCLASS32) {
        fprintf(stderr, "guest_load_elf: %s: invalid ELF class %u\n",
                path, ehdr.e_ident[EI_CLASS]);
        close(fd);
        return -1;
    }

    // 1c. data —— RISC-V 永远 LE
    if (ehdr.e_ident[EI_DATA] != ELFDATA2LSB) {
        fprintf(stderr,
                "guest_load_elf: %s: not little-endian (RISC-V is LE only)\n",
                path);
        close(fd);
        return -1;
    }

    // 1d. machine
    if (ehdr.e_machine != EM_RISCV) {
        fprintf(stderr,
                "guest_load_elf: %s: e_machine=%u, expected EM_RISCV (%u)\n",
                path, ehdr.e_machine, EM_RISCV);
        close(fd);
        return -1;
    }

    // 1e. type —— 只接 ET_EXEC; ET_DYN(PIE)需要 relocation, a_01 不做。
    if (ehdr.e_type != ET_EXEC) {
        fprintf(stderr,
                "guest_load_elf: %s: e_type=%u; only ET_EXEC supported "
                "(link with -no-pie or a bare-metal linker script)\n",
                path, ehdr.e_type);
        close(fd);
        return -1;
    }

    // 1f. program header table sanity
    if (ehdr.e_phentsize != sizeof(Elf32_Phdr) || ehdr.e_phnum == 0) {
        fprintf(stderr,
                "guest_load_elf: %s: invalid program header table "
                "(e_phentsize=%u, e_phnum=%u)\n",
                path, ehdr.e_phentsize, ehdr.e_phnum);
        close(fd);
        return -1;
    }

    // 2. 遍历 program headers, 处理 PT_LOAD。
    for (uint16_t i = 0; i < ehdr.e_phnum; i++) {
        Elf32_Phdr phdr;
        off_t phdr_off = (off_t)ehdr.e_phoff
                       + (off_t)i * (off_t)sizeof(Elf32_Phdr);
        char what[64];
        snprintf(what, sizeof(what), "Elf32_Phdr[%u]", i);
        if (read_full_at(fd, &phdr, sizeof(phdr), phdr_off, path, what) != 0) {
            close(fd);
            return -1;
        }

        if (phdr.p_type != PT_LOAD) continue;

        if (phdr.p_filesz > phdr.p_memsz) {
            fprintf(stderr,
                    "guest_load_elf: %s: PT_LOAD[%u] p_filesz (%u) > p_memsz (%u)\n",
                    path, i, phdr.p_filesz, phdr.p_memsz);
            close(fd);
            return -1;
        }

        // 范围检查: [p_paddr, p_paddr + p_memsz) ⊂ [GUEST_RAM_START, GUEST_RAM_START + GUEST_RAM_SIZE)
        // 用 uint64_t 算, 避开 Elf32 字段 32 位加法的溢出风险。
        uint64_t seg_start = (uint64_t)phdr.p_paddr;
        uint64_t seg_end   = seg_start + (uint64_t)phdr.p_memsz;
        if (seg_start < GUEST_RAM_START ||
            seg_end > GUEST_RAM_START + GUEST_RAM_SIZE) {
            fprintf(stderr,
                    "guest_load_elf: %s: PT_LOAD[%u] [0x%" PRIx64 ", 0x%" PRIx64 ") out of RAM range\n",
                    path, i, seg_start, seg_end);
            close(fd);
            return -1;
        }

        // 拷段内容; BSS 部分 (p_memsz - p_filesz) 由 guest 自身 startup 负责清零, loader 不动。
        if (phdr.p_filesz > 0) {
            uint8_t *host_dst = gpa_to_hva_offset + seg_start;
            char what2[64];
            snprintf(what2, sizeof(what2), "PT_LOAD[%u] contents", i);
            if (read_full_at(fd, host_dst, phdr.p_filesz,
                             (off_t)phdr.p_offset, path, what2) != 0) {
                close(fd);
                return -1;
            }
        }
    }

    if (close(fd) != 0) {
        fprintf(stderr, "guest_load_elf: close(%s) failed: %s\n",
                path, strerror(errno));
        return -1;
    }

    return 0;
}

int guest_is_elf(const char *path) {
    // 探测函数: silently return 0 表示"不是 / 不可读"。不 fprintf。
    int fd = open(path, O_RDONLY);
    if (fd < 0) return 0;

    unsigned char m[4];
    ssize_t n;
    do {
        n = pread(fd, m, 4, 0);
    } while (n < 0 && errno == EINTR);
    close(fd);
    if (n != 4) return 0;
    return (m[0] == ELFMAG0 && m[1] == ELFMAG1 &&
            m[2] == ELFMAG2 && m[3] == ELFMAG3) ? 1 : 0;
}
