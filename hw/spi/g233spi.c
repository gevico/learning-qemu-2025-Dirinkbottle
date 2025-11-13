#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/spi/g233spi.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define G233_SPI_MMIO_SIZE 0x1000

enum {
    G233_SPI_REG_CR1     = 0x00,
    G233_SPI_REG_CR2     = 0x04,
    G233_SPI_REG_SR      = 0x08,
    G233_SPI_REG_DR      = 0x0c,
    G233_SPI_REG_CSCTRL  = 0x10,
};

#define G233_SPI_SR_RXNE     BIT(0)
#define G233_SPI_SR_TXE      BIT(1)
#define G233_SPI_SR_UNDERRUN BIT(2)
#define G233_SPI_SR_OVERRUN  BIT(3)
#define G233_SPI_SR_BSY      BIT(7)



#define G233_SPI_CR1_SPE   BIT(6)

#define G233_SPI_CR2_SSOE   BIT(4)
#define G233_SPI_CR2_ERRIE  BIT(5)
#define G233_SPI_CR2_RXNEIE BIT(6)
#define G233_SPI_CR2_TXEIE  BIT(7)

#define G233_SPI_CS_ENABLE_MASK   0x0f
#define G233_SPI_CS_ACTIVE_MASK   0x0f
#define G233_SPI_CS_ACTIVE_SHIFT  4

static uint64_t g233spi_read(void *opaque, hwaddr addr, unsigned size);
static void g233spi_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size);
static void g233spi_update(G233SPIState *s);
static void g233spi_reset(DeviceState *dev);
static void g233spi_realize(DeviceState *dev, Error **errp);
static void g233spi_class_init(ObjectClass *klass, const void *data);
static void g233spi_instance_init(Object *obj);

static const MemoryRegionOps g233_spi_ops = {
    .read = g233spi_read,
    .write = g233spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void g233spi_drive_cs(G233SPIState *s, int index, bool active)
{
    bool level = active ? 0 : 1; //低电平是有效的 active->true是level电平为0

    if (index >= 0 && index < G233_SPI_NUM_CS && s->cs_lines[index]) {/* s->cs_lines[index]不能舍弃 判断在4个gpio没有全连接的情况 */
        qemu_set_irq(s->cs_lines[index], level); //发送电平给对应从机flash 目前是全部拉低电平实现
    }
}

static void g233spi_refresh_cs(G233SPIState *s)
{
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        bool active = s->cs_enabled[i] && s->cs_active[i];
        g233spi_drive_cs(s, i, active);
    }
}

/* 有使能并且激活的就写入这个 */
static bool g233spi_any_cs_active(G233SPIState *s)
{
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        if (s->cs_enabled[i] && s->cs_active[i]) {
            return true;
        }
    }
    return false;
}




static uint64_t g233spi_read(void *opaque, hwaddr addr, unsigned size)
{
    G233SPIState *s = opaque;
    uint32_t value = 0;

    switch (addr) {
    case G233_SPI_REG_CR1:
        value = s->SPI_CR1;
        break;
    case G233_SPI_REG_CR2:
        value = s->SPI_CR2;
        break;
    case G233_SPI_REG_SR:
        value = s->SPI_SR;
        break;
    case G233_SPI_REG_DR:
        
        /* 溢出检查 */
        if (!(s->SPI_SR & G233_SPI_SR_RXNE) || !s->rx_valid)
        {
            /* 溢出 */
            s->SPI_SR |= G233_SPI_SR_UNDERRUN;
            g233spi_update(s);  // 更新错误状态
            break;
        }

        value = s->SPI_DR;
        s->SPI_SR &= ~G233_SPI_SR_RXNE;
        s->rx_valid = false;
        g233spi_update(s);
        break;
    case G233_SPI_REG_CSCTRL:
        value = s->SPI_CSCTRL;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "g233.spi: invalid read at offset 0x%" HWADDR_PRIx
                      " (size=%u)\n",
                      addr, size);
        break;
    }


    return value;
}

static void g233spi_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    G233SPIState *s = opaque;
    uint32_t val32 = value;

    switch (addr) {
    case G233_SPI_REG_CR1:
        s->SPI_CR1 = val32;
        break;
    case G233_SPI_REG_CR2:
        s->SPI_CR2 = val32;
        break;
    case G233_SPI_REG_SR:
        //部分寄存器 overrun underrun 实现写1置位
        if (val32 & G233_SPI_SR_UNDERRUN)
        {
            s->SPI_SR &= ~G233_SPI_SR_UNDERRUN;
        }
        if (val32 & G233_SPI_SR_OVERRUN)
        {
            s->SPI_SR &= ~G233_SPI_SR_OVERRUN;
        }

        break;
    case G233_SPI_REG_DR:
        if (!(s->SPI_CR1 & G233_SPI_CR1_SPE)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "g233.spi: write to DR while SPI disabled\n");
            break;
        }

        /* 溢出检查 */
        /**
         * 
         *  SPI 溢出发生在以下情况：
            接收缓冲区有数据未读取（RXNE = 1）
            继续发送新数据
            新数据到达但无法存储到已满的接收缓冲区
            触发溢出错误标志（OVERRUN = 1）
         */
        if ((s->SPI_SR & G233_SPI_SR_RXNE) || !(s->SPI_SR & G233_SPI_SR_TXE))
        {
            /* 溢出 */
            s->SPI_SR |= G233_SPI_SR_OVERRUN;
        }
        

        /* 此时发送缓冲区有数据 */
        s->SPI_SR &= ~G233_SPI_SR_TXE; 

        /* 设置繁忙 */
        s->SPI_SR |= G233_SPI_SR_BSY;

        /* 写入数据到spi_dr寄存器 */
        s->SPI_DR = val32 & 0xff;

        if (g233spi_any_cs_active(s) && s->bus) {
            uint32_t rx = ssi_transfer(s->bus, s->SPI_DR);
            s->SPI_DR = rx & 0xff;
            s->rx_buf = s->SPI_DR;
            s->rx_valid = true;
            s->SPI_SR |= G233_SPI_SR_RXNE;
        } else {
            s->SPI_SR &= ~G233_SPI_SR_RXNE;
            s->rx_valid = false;
        }

        s->SPI_SR |= G233_SPI_SR_TXE;
        /* 当前处于空闲状态 */
        s->SPI_SR &= ~G233_SPI_SR_BSY;
        break;
    case G233_SPI_REG_CSCTRL:
    {
        uint32_t enable_mask = val32 & G233_SPI_CS_ENABLE_MASK;
        uint32_t active_mask = (val32 >> G233_SPI_CS_ACTIVE_SHIFT) &
                               G233_SPI_CS_ACTIVE_MASK;

        s->SPI_CSCTRL = val32;

        for (int i = 0; i < G233_SPI_NUM_CS; i++) {//激活对应从机 flash
            s->cs_enabled[i] = enable_mask & BIT(i);
            s->cs_active[i] = active_mask & BIT(i);
        }

        g233spi_refresh_cs(s);
        break;
    }
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "g233.spi: invalid write at offset 0x%" HWADDR_PRIx
                      " value=0x%" PRIx64 " (size=%u)\n",
                      addr, value, size);
        break;
    }

    g233spi_update(s);
}

static void g233spi_update(G233SPIState *s)
{
    if (!s->rx_valid) {
        s->SPI_SR &= ~G233_SPI_SR_RXNE;
    }

    uint8_t sr = s->SPI_SR;
    uint8_t cr2 = s->SPI_CR2;
    bool irq_pending = false;


    /* 错误优先 */
    if ((cr2 & G233_SPI_CR2_ERRIE) && ((sr & G233_SPI_SR_OVERRUN) || (sr & G233_SPI_SR_UNDERRUN)))
    {
       irq_pending =true;
    }

    /* 发送缓冲区为空 */
    if ((cr2 & G233_SPI_CR2_TXEIE) && (sr & G233_SPI_SR_TXE))
    {
        irq_pending =true;
    }

    /* 接受到新数据 */
    if ((cr2 & G233_SPI_CR2_RXNEIE) && !(sr & G233_SPI_SR_TXE))
    {
        irq_pending =true;
    }
    

    

    qemu_set_irq(s->plic_irq, irq_pending?1:0); //通知cpu
}

static void g233spi_reset(DeviceState *dev)
{
    G233SPIState *s = G233_SPI(dev);

    s->SPI_CR1 = 0x00000000;
    s->SPI_CR2 = 0x00000000;
    s->SPI_SR = G233_SPI_SR_TXE;//初始化为空
    s->SPI_DR = 0x0000000C; //实验要求
    s->SPI_CSCTRL = 0x00000000;
    s->rx_buf = 0;
    s->rx_valid = false;

    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        s->cs_enabled[i] = false;
        s->cs_active[i] = false;
    }

    g233spi_refresh_cs(s);
    g233spi_update(s);
}

static void g233spi_realize(DeviceState *dev, Error **errp)
{
    /* 当前实现无需额外步骤，保留占位以便后续扩展 */
}

static void g233spi_instance_init(Object *obj)
{
    G233SPIState *s = G233_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &g233_spi_ops, s,
                          "g233.spi", G233_SPI_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->plic_irq);

    s->bus = ssi_create_bus(DEVICE(obj), "g233.spi.bus");
    /* GPIO 与 IRQ 使用的是同一套抽象（qemu_irq）！！！！！！ */
    qdev_init_gpio_out_named(DEVICE(obj), s->cs_lines, "cs",
                             G233_SPI_NUM_CS);

    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        if (s->cs_lines[i]) {
            qemu_set_irq(s->cs_lines[i], 1);
        }
    }
}

static void g233spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233spi_realize;
    device_class_set_legacy_reset(dc, g233spi_reset);
}

static const TypeInfo g233_spi_typeinfo = {
    .name = TYPE_G233_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(G233SPIState),
    .instance_init = g233spi_instance_init,
    .class_init = g233spi_class_init,
};

static void g233spi_register_types(void)
{
    type_register_static(&g233_spi_typeinfo);
}

type_init(g233spi_register_types);