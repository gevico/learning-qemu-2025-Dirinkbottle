#ifndef HW_G233SPI_H
#define HW_G233SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_G233_SPI "g233_spi"
#define G233_SPI_NUM_CS 4

typedef struct G233SPIState G233SPIState;
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIState, G233_SPI)

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *bus;

    qemu_irq plic_irq;
    qemu_irq cs_lines[G233_SPI_NUM_CS];

    uint32_t SPI_CR1;
    uint32_t SPI_CR2;
    uint32_t SPI_SR;
    uint32_t SPI_DR;
    uint32_t SPI_CSCTRL;

    bool cs_enabled[G233_SPI_NUM_CS];
    bool cs_active[G233_SPI_NUM_CS];
    uint8_t rx_buf;
    bool rx_valid;

};

#endif