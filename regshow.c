//
// Created by Petr Otoupal on 18/09/2019.
//

#include <stdint.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

void regshow(uint32_t reg, int fd)
{
    off_t target;
    void *map_base, *virt_addr;

    target = (off_t)reg;
    map_base = mmap(0, MAP_SIZE, PROT_READ, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (uint32_t *)(map_base + (target & MAP_MASK));
    printf("(0x%08X): 0x%08X\r\n", reg, *((uint32_t *)virt_addr));
    munmap(map_base, MAP_SIZE);
}

int main(int argc, char** argv) {
    int fd;
    uint32_t reg;
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf("Could not open /dev/mem, are you root?\n");
        return -1;
    }
    reg = (uint32_t) strtoul(argv[1], NULL, 16);
    regshow(reg,fd);
}

