/*
 * m4fwloader - based on mqx_upload_on_m4SoloX
 *              from Giuseppe Pagano
 *               
 * Tool to control M4 AMP core from Linux user-space
 * 
 * Copyright (C) 2015-2016 Giuseppe Pagano <giuseppe.pagano@seco.com>
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdint.h>

#define LogVerbose(...)          \
    do {                         \
        if (verbose)             \
            printf(__VA_ARGS__); \
    } while (0)
#define LogError(...) printf(__VA_ARGS__)

#define VERSION "1.0.1"
#define NAME_OF_UTILITY "i.MX M4 Loader"
#define HEADER NAME_OF_UTILITY " - M4 firmware loader v. " VERSION "\n"

#define IMX7D_SRC_M4RCR                   (0x3039000C) /* reset register */
#define IMX7D_STOP_CLEAR_MASK             (0xFFFFFF00)
#define IMX7D_STOP_SET_MASK               (0x000000AA)
#define IMX7D_START_CLEAR_MASK            (0xFFFFFFFF)
#define IMX7D_START_SET_MASK              (0x00000001)
#define IMX7D_MU_ATR1                     (0x30AA0004) /* rpmsg_mu_kick_addr */
#define IMX7D_M4_BOOTROM                  (0x00180000)
#define IMX7D_CCM_ANALOG_PLL_480          (0x303600B0)
#define IMX7D_CCM_CCGR1                   (0x30384010)
#define IMX7D_OCRAM_START_ADDR            (0x00900000)
#define IMX7D_OCRAM_END_ADDR              (0x00947FFF) /* OCRAM + OCRAM_EPDC + OCRAM_PXP */
#define IMX7D_OCRAM_M4_ALIAS_START_ADDR   (0x20200000)
#define IMX7D_OCRAM_M4_ALIAS_END_ADDR     (0x20247FFF) /* OCRAM + OCRAM_EPDC + OCRAM_PXP */

#define IMX6SX_SRC_SCR                     (0x020D8000) /* reset register */
#define IMX6SX_STOP_CLEAR_MASK             (0xFFFFFFEF)
#define IMX6SX_STOP_SET_MASK               (0x00400000)
#define IMX6SX_START_CLEAR_MASK            (0xFFFFFFFF)
#define IMX6SX_START_SET_MASK              (0x00400010)
#define IMX6SX_MU_ATR1                     (0x02294004) /* rpmsg_mu_kick_addr */
#define IMX6SX_M4_BOOTROM                  (0x007F8000)
#define IMX6SX_CCM_CCGR3                   (0x020C4074)
#define IMX6SX_OCRAM_START_ADDR            (0x00900000)
#define IMX6SX_OCRAM_END_ADDR              (0x0091FFFF) /* OCRAM 128KB */
#define IMX6SX_OCRAM_ALIAS_START_ADDR      (0x00920000)
#define IMX6SX_OCRAM_ALIAS_END_ADDR        (0x0093FFFF) /* OCRAM aliased 128KB */
#define IMX6SX_OCRAM_M4_START_ADDR         (0x20900000)
#define IMX6SX_OCRAM_M4_END_ADDR           (0x2091FFFF) /* OCRAM 128KB */
#define IMX6SX_OCRAM_M4_ALIAS_START_ADDR   (0x20920000)
#define IMX6SX_OCRAM_M4_ALIAS_END_ADDR     (0x2093FFFF) /* OCRAM aliased 128KB */

#define IMX_TCM_START_ADDR       (0x007F8000) /* TCML  32KB */
#define IMX_TCM_END_ADDR         (0x00807FFF) /* TCMU  32KB */
#define IMX_TCM_M4_START_ADDR    (0x1FFF8000) /* TCML  32KB */
#define IMX_TCM_M4_END_ADDR      (0x20007FFF) /* TCMU  32KB */

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)
#define SIZE_4BYTE 4UL
#define SIZE_16BYTE 16UL
#define MAP_OCRAM_SIZE 64 * 1024
#define MAP_OCRAM_MASK (MAP_OCRAM_SIZE - 1)
#define MAX_FILE_SIZE MAP_OCRAM_SIZE
#define MAX_RETRIES 8
#define MAX_FILE_SIZE_TCM (32 * 1024)

#define RETURN_CODE_OK 0
#define RETURN_CODE_ARGUMENTS_ERROR 1
#define RETURN_CODE_M4STOP_FAILED 2
#define RETURN_CODE_M4START_FAILED 3

struct soc_specific {
    char* detect_name;

    uint32_t src_m4reg_addr;

    uint32_t start_and;
    uint32_t start_or;
    uint32_t stop_and;
    uint32_t stop_or;

    uint32_t rpmsg_mu_kick_addr;

    void (*clk_enable)(int);

    uint32_t stack_pc_addr;
};

static int verbose = 0;

void regshow(uint32_t addr, char* name, int fd)
{
    off_t target;
    void *map_base, *virt_addr;

    target = (off_t)addr;
    map_base = mmap(0, MAP_SIZE, PROT_READ, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    LogVerbose("%s (0x%08X): 0x%08X\r\n", name, addr, *((unsigned long*)virt_addr));
    munmap(map_base, MAP_SIZE);
}

void imx6sx_clk_enable(int fd)
{
    off_t target;
    unsigned long read_result;
    void *map_base, *virt_addr;

    LogVerbose("i.MX6SX specific function for M4 clock enabling!\n");

    regshow(IMX6SX_CCM_CCGR3, "CCM_CCGR3", fd);
    target = (off_t)IMX6SX_CCM_CCGR3; /* M4 Clock gate*/
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    read_result = *((unsigned long*)virt_addr);
    *((unsigned long*)virt_addr) = read_result | 0x0000000C;
    munmap(map_base, MAP_SIZE);
    regshow(IMX6SX_CCM_CCGR3, "CCM_CCGR3", fd);
    LogVerbose("CCM_CCGR3 done\n");
}

void imx7d_clk_enable(int fd)
{
    off_t target;
    unsigned long read_result;
    void *map_base, *virt_addr;

    LogVerbose("i.MX7D specific function for M4 clock enabling!\n");

    regshow(IMX7D_CCM_ANALOG_PLL_480, "CCM_ANALOG_PLL_480", fd);
    /* Enable parent clock first! */
    target = (off_t)IMX7D_CCM_ANALOG_PLL_480;
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    /* clock enabled by clearing the bit!  */
    *((unsigned long*)virt_addr) = (*((unsigned long*)virt_addr)) & (~(1 << 5));
    munmap(map_base, MAP_SIZE);
    regshow(IMX7D_CCM_ANALOG_PLL_480, "CCM_ANALOG_PLL_480", fd);
    LogVerbose("CCM_ANALOG_PLL_480 done\n");

    /* ENABLE CLK */
    regshow(IMX7D_CCM_CCGR1, "CCM1_CCGR1", fd);
    target = (off_t)(IMX7D_CCM_CCGR1+4); /* CCM_CCGR1_SET */
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    *((unsigned long*)virt_addr) = 0x00000003;
    munmap(map_base, MAP_SIZE);
    regshow(IMX7D_CCM_CCGR1, "CCM1_CCGR1", fd);
    LogVerbose("CCM_CCGR1_SET done\n");
}

static struct soc_specific socs[] = {
    {
        "i.MX7 Dual",
        IMX7D_SRC_M4RCR,
        IMX7D_STOP_CLEAR_MASK,
        IMX7D_STOP_SET_MASK,
        IMX7D_START_CLEAR_MASK,
        IMX7D_START_SET_MASK,
        IMX7D_MU_ATR1,

        imx7d_clk_enable,

        IMX7D_M4_BOOTROM
    },
    {
        "i.MX6 SoloX",
        IMX6SX_SRC_SCR,
        IMX6SX_STOP_CLEAR_MASK,
        IMX6SX_STOP_SET_MASK,
        IMX6SX_START_CLEAR_MASK,
        IMX6SX_START_SET_MASK,
        IMX6SX_MU_ATR1,

        imx6sx_clk_enable,

        IMX6SX_M4_BOOTROM
    }
};

static int currentSoC;

void rpmsg_mu_kick(int fd, int socid, uint32_t vq_id)
{
    off_t target;
    void *map_base, *virt_addr;

    if (!socs[socid].rpmsg_mu_kick_addr)
        return;

    target = (off_t)socs[socid].rpmsg_mu_kick_addr;
    map_base = mmap(0, SIZE_4BYTE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    vq_id = (vq_id << 16);
    *((unsigned long*)virt_addr) = vq_id;
    munmap(map_base, SIZE_4BYTE);
}

void ungate_m4_clk(int fd, int socid)
{
    socs[socid].clk_enable(fd);
}

void stop_cpu(int fd, int socid)
{
    unsigned long read_result;
    off_t target;
    void *map_base, *virt_addr;

    if (!socs[socid].src_m4reg_addr)
        return;

    regshow(socs[socid].src_m4reg_addr, "STOP - before", fd);
    target = (off_t)socs[socid].src_m4reg_addr;
    map_base = mmap(0, SIZE_4BYTE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    read_result = *((unsigned long*)virt_addr);
    *((unsigned long*)virt_addr) = (read_result & (socs[socid].stop_and)) | socs[socid].stop_or;
    munmap(virt_addr, SIZE_4BYTE);
    regshow(socs[socid].src_m4reg_addr, "STOP - after", fd);
}

void start_cpu(int fd, int socid)
{
    unsigned long read_result;
    off_t target;
    void *map_base, *virt_addr;

    if (!socs[socid].src_m4reg_addr)
        return;

    regshow(socs[socid].src_m4reg_addr, "START - before", fd);
    target = (off_t)socs[socid].src_m4reg_addr;
    map_base = mmap(0, SIZE_4BYTE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    read_result = *((unsigned long*)virt_addr);
    *((unsigned long*)virt_addr) = (read_result & (socs[socid].start_and)) | socs[socid].start_or;
    munmap(virt_addr, SIZE_4BYTE);
    regshow(socs[socid].src_m4reg_addr, "START -after", fd);
}

void set_stack_pc(int fd, int socid, unsigned int stack, unsigned int pc)
{
    off_t target = (off_t)socs[socid].stack_pc_addr;
    unsigned long read_result;
    void *map_base, *virt_addr;
    map_base = mmap(0, SIZE_16BYTE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
    virt_addr = (unsigned char*)(map_base + (target & MAP_MASK));
    *((unsigned long*)virt_addr) = stack;
    virt_addr = (unsigned char*)(map_base + ((target + 0x4) & MAP_MASK));
    *((unsigned long*)virt_addr) = pc;
    munmap(map_base, SIZE_16BYTE);
}

int check_load_addr(unsigned long* ldaddr, char* mt, unsigned long pc)
{
  /* Check if where we loading to looks familiar. Also if the address looks like an M4 address,
   * return A7/A9 instead. Return 0(false) on any error.
   * NOTE: Only does very minimal check for case your PC value and thus taret load address looks
   * suspicious or wonkey, not much more. */
  if (0 == ldaddr || 0 == mt){
    LogError("%s - a NULL passed to load address checker:  bailing out.\n", NAME_OF_UTILITY);
    return 0;
  }

  int ret = 0; // false

  switch(currentSoC)
  {
    default:
      LogError("%s - SoC id is not set to anything I know, bailing out.\n", NAME_OF_UTILITY);
      break;
    case 0:  //iMX7D
      if ( (IMX7D_OCRAM_START_ADDR < pc && pc < IMX7D_OCRAM_END_ADDR)  ||
          (IMX7D_OCRAM_M4_ALIAS_START_ADDR < pc && pc < IMX7D_OCRAM_M4_ALIAS_END_ADDR) ){
        ret = 1;
        *ldaddr = IMX7D_OCRAM_START_ADDR + (pc & 0x000f0000);  /* Align */
        *mt = 'o';
      }
      else if ( (IMX_TCM_START_ADDR < pc && pc < IMX_TCM_END_ADDR) ||
                (IMX_TCM_M4_START_ADDR < pc && pc < IMX_TCM_M4_END_ADDR) ){
        ret = 1;
        //Note: expects you to load it to TCML, by-the-book
        *ldaddr = IMX_TCM_START_ADDR; /* Align */
        *mt = 't';
      }
      break;
    case 1: //iMX6Solo
      if (  (IMX6SX_OCRAM_START_ADDR < pc && pc < IMX6SX_OCRAM_END_ADDR)  ||
            (IMX6SX_OCRAM_ALIAS_START_ADDR < pc && pc < IMX6SX_OCRAM_ALIAS_END_ADDR) ||
            (IMX6SX_OCRAM_M4_START_ADDR < pc && pc < IMX6SX_OCRAM_M4_END_ADDR) ||
            (IMX6SX_OCRAM_M4_ALIAS_START_ADDR < pc && pc < IMX6SX_OCRAM_M4_ALIAS_END_ADDR ) ) {
        *ldaddr = IMX6SX_OCRAM_START_ADDR + (pc & 0x000f0000);  /* Align */
        ret = 1;
        *mt = 'o';
      }
      else if ( (IMX_TCM_START_ADDR < pc && pc < IMX_TCM_END_ADDR) ||
                (IMX_TCM_M4_START_ADDR < pc && pc < IMX_TCM_M4_END_ADDR) ) {
        ret = 1;
        //Note: expects you to load it to TCML, by-the-book
        *ldaddr = IMX_TCM_START_ADDR; /* Align */
        *mt = 't';
      }
      break;
  }//switch(..)

  return ret;
}

int load_m4_fw(int fd, int socid, char* filepath, unsigned long loadaddr)
{
    int n;
    int size;
    FILE* fdf;
    off_t target;
    uint8_t* filebuffer;
    void *map_base, *virt_addr;
    unsigned long stack, pc;
    char mt;

    fdf = fopen(filepath, "rb");
    fseek(fdf, 0, SEEK_END);
    size = ftell(fdf);
    fseek(fdf, 0, SEEK_SET);
    if (size > MAX_FILE_SIZE) {
        LogError("%s - File size too big, can't load: %d > %d \n", NAME_OF_UTILITY, size, MAX_FILE_SIZE);
        return -2;
    }

    filebuffer = malloc(size + 1);
    if (size != fread(filebuffer, sizeof(char), size, fdf)) {
        free(filebuffer);
        return -2;
    }

    fclose(fdf);

    stack = (filebuffer[0] | (filebuffer[1] << 8) | (filebuffer[2] << 16) | (filebuffer[3] << 24));
    pc = (filebuffer[4] | (filebuffer[5] << 8) | (filebuffer[6] << 16) | (filebuffer[7] << 24));

    if (0 == loadaddr)
    {
      if ( !check_load_addr(&loadaddr, &mt, pc)) {
        LogError("%s - Failed the check for a valid target load address. Bailing out...\n", NAME_OF_UTILITY);
        free(filebuffer);
        return -2;
      }
      if ('o' == mt) {
        LogVerbose("%s - Your target memory type is .. %s\n", NAME_OF_UTILITY, "OCRAM");
      }
      else if ('t' == mt){
        LogVerbose("%s: Your target memory type is .. %s\n", NAME_OF_UTILITY, "TCM");
        if (size > MAX_FILE_SIZE_TCM ){
          LogError("%s - File size too big for TCM, can't load: %d > %d \n", NAME_OF_UTILITY, size, MAX_FILE_SIZE_TCM);
          free(filebuffer);
          return -2;
        }
      }
      else {
        LogError("%s - Could not determine your target memory type, thus bailing out .. \n", NAME_OF_UTILITY);
        free(filebuffer);
        return -2;
      }
    }

    LogVerbose("%s - FILENAME = %s; loadaddr = 0x%08x\n", NAME_OF_UTILITY, filepath, loadaddr);

    map_base = mmap(0, MAP_OCRAM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, loadaddr & ~MAP_OCRAM_MASK);
    LogVerbose("%s - start - end (0x%08x - 0x%08x)\n", NAME_OF_UTILITY, loadaddr & ~MAP_OCRAM_MASK, (loadaddr & ~MAP_OCRAM_MASK) + MAP_OCRAM_SIZE);
    virt_addr = (unsigned char*)(map_base + (loadaddr & MAP_OCRAM_MASK));
    memcpy(virt_addr, filebuffer, size);
    munmap(map_base, MAP_OCRAM_SIZE);

    LogVerbose("Will set PC and STACK...");
    set_stack_pc(fd, socid, stack, pc);
    LogVerbose("...Done\n");
    free(filebuffer);

    return size;
}

int get_board_id(void)
{
    int i;
    char out[512];
    int result = -1;
    FILE* fp;

    fp = fopen("/proc/cpuinfo", "r");
    if (fp == NULL)
        return result;

    while (fgets(out, sizeof(out) - 1, fp) != NULL) {
        if (strstr(out, "Hardware")) {
            for (i = 0; i < (sizeof(socs) / sizeof(struct soc_specific)); i++) {
                if (strstr(out, socs[i].detect_name)) {
                    result = i;
                    break;
                }
            }
            break;
        }
    }

    fclose(fp);
    return result;
}

int main(int argc, char** argv)
{
    int fd, n;
    unsigned long loadaddr;
    char* p;
    char m4IsStopped = 0;
    char m4IsRunning = 0;
    int m4TraceFlags = 0;
    int m4Retry;
    char* filepath = argv[1];
    currentSoC = -1;

    if (argc < 2) {
        LogError(HEADER);
        LogError("-- %s -- \nUsage:\n"
                 "%s [filename.bin] [0xLOADADDR] [--verbose]  # loads new firmware\n"
                 "or: %s stop                    # holds the auxiliary core in reset\n"
                 "or: %s start                   # releases the auxiliary core from reset\n"
                 "or: %s kick [n]                # triggers interrupt on RPMsg virtqueue n\n"
                 "\nSpecifying 0xLOADADDR = 0 makes the program determine the load address from the image file.\n"
                 "You probably want to use it with 0(zero) for load address as this may be safer.\n",
            NAME_OF_UTILITY, argv[0], argv[0], argv[0], argv[0], argv[0]);
        LogError("\nNOTE1: For TCM memory, can only load 32KB image\n");
        LogError("NOTE2: LIMITATION: For OCRAM, currently can only load 64KB image\n\n");
        return RETURN_CODE_ARGUMENTS_ERROR;
    }

    currentSoC = get_board_id();
    if (currentSoC == -1) {
        LogError(HEADER);
        LogError("Board is not supported.\n");
        return RETURN_CODE_ARGUMENTS_ERROR;
    }

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        LogError(HEADER);
        LogError("Could not open /dev/mem, are you root?\n");
        return RETURN_CODE_ARGUMENTS_ERROR;
    }

    /* PARTIAL COMMANDS */
    if (!strcmp(argv[1], "stop")) {
        stop_cpu(fd, currentSoC);
        return RETURN_CODE_OK;
    }
    else if (!strcmp(argv[1], "start")) {
        start_cpu(fd, currentSoC);
        return RETURN_CODE_OK;
    }
    else if (!strcmp(argv[1], "kick")) {
        if (argc < 3) {
            LogError(HEADER);
            LogError("%s - Usage: %s kick {vq_id to kick}\n", NAME_OF_UTILITY, argv[0]);
            return RETURN_CODE_ARGUMENTS_ERROR;
        }
        rpmsg_mu_kick(fd, currentSoC, strtoul(argv[2], &p, 16));
        return RETURN_CODE_OK;
    }

    /* FW LOADING */
    if (argc < 3) {
        LogError(HEADER);
        LogError("%s - Usage: %s [yourfwname.bin] [0xLOADADDR] [--verbose]\n", NAME_OF_UTILITY, argv[0]);
        return RETURN_CODE_ARGUMENTS_ERROR;
    }

    if (access(filepath, F_OK) == -1) {
        LogError("File %s not found.\n", argv[1]);
        return RETURN_CODE_ARGUMENTS_ERROR;
    }

    loadaddr = strtoul(argv[2], &p, 16);

    if (argc == 4) {
        if (!strcmp(argv[3], "--verbose")) {
            verbose = 1;
        }
        else {
            LogError(HEADER);
            LogError("%s - Usage: %s [yourfwname.bin] [0xLOADADDR] [--verbose]\n", NAME_OF_UTILITY, argv[0]);
            return RETURN_CODE_ARGUMENTS_ERROR;
        }
    }

    LogVerbose("LoadAddr is: %X\n", loadaddr);
    LogVerbose("Will stop CPU now...\n");
    stop_cpu(fd, currentSoC);
    LogVerbose("Will ungate M4 clock source...\n");
    ungate_m4_clk(fd, currentSoC);
    LogVerbose("Will load M4 firmware...\n");
    load_m4_fw(fd, currentSoC, filepath, loadaddr);
    LogVerbose("Will start CPU now...\n");
    start_cpu(fd, currentSoC);
    LogVerbose("Done!\n");
    close(fd);
    return RETURN_CODE_OK;
}
