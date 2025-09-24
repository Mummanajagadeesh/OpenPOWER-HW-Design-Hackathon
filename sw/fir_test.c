// sw/fir_test.c
// Minimal FIR accelerator test for Microwatt
// Maps the registers in fir_wb.v and pushes samples

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

// Adjust this base address to where FIR is mapped in your memory map
#define FIR_BASE 0x40000000UL

#define FIR_CONTROL  0x00
#define FIR_STATUS   0x04
#define FIR_TAPS     0x08
#define FIR_COEF(n)  (0x10 + ((n)*4))
#define FIR_IN_FIFO  0x80
#define FIR_OUT_FIFO 0x84

volatile uint32_t *fir;

// Simple memory-mapped I/O helpers
static inline void write_reg(uint32_t addr, uint32_t value) {
    fir[addr/4] = value;
}

static inline uint32_t read_reg(uint32_t addr) {
    return fir[addr/4];
}

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    // Map 4KB for FIR
    fir = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, FIR_BASE);
    if (fir == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    // Configure FIR: 3 taps
    write_reg(FIR_TAPS, 3);

    // Coefficients: 1,2,3
    write_reg(FIR_COEF(0), 1);
    write_reg(FIR_COEF(1), 2);
    write_reg(FIR_COEF(2), 3);

    // Sample input array
    int32_t samples[8] = {0,1,2,3,4,5,6,7};
    int i;
    for (i=0; i<8; i++) {
        // Push sample
        write_reg(FIR_IN_FIFO, samples[i]);
        // Pulse start
        write_reg(FIR_CONTROL, 0x1);

        // Wait for OUT_VALID
        uint32_t status = 0;
        do {
            status = read_reg(FIR_STATUS);
        } while ((status & 0x2) == 0);

        // Read output
        int32_t out = read_reg(FIR_OUT_FIFO);

        // Compute golden reference
        int32_t a = (i>=0) ? samples[i] : 0;
        int32_t b = (i-1>=0) ? samples[i-1] : 0;
        int32_t c = (i-2>=0) ? samples[i-2] : 0;
        int32_t golden = a*1 + b*2 + c*3;

        printf("i=%d out=%d golden=%d\n", i, out, golden);
    }

    munmap((void*)fir, 0x1000);
    close(fd);
    return 0;
}
