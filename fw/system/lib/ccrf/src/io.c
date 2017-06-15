#include <cc/io.h>
#include <cc/spi.h>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define swap16(v) __builtin_bswap16(v)
#else
#define swap16(v) (v)
#endif

u8 cc_strobe(cc_dev_t dev, u8 cmd)
{
    return cc_spi_io(dev, 0, cmd, NULL, NULL, 0);
}

void cc_read(cc_dev_t dev, u16 addr, u8 *buf, u32 len)
{
    const u8 flag = CC1200_ACCESS_READ | CC1200_ACCESS_BURST;//(len > 1 ? CC1200_ACCESS_BURST : CC1200_ACCESS_SINGLE);
    cc_spi_io(dev, flag, addr, NULL, buf, len);
}

void cc_read_e(cc_dev_t dev, u16 addr, u8 *buf, u32 len)
{
    const u8 flag = CC1200_ACCESS_READ | (len > 1 ? CC1200_ACCESS_BURST : CC1200_ACCESS_SINGLE);
    cc_spi_io(dev, flag, addr, NULL, buf, len);
}

void cc_write(cc_dev_t dev, u16 addr, u8 *buf, u32 len)
{
    const u8 flag = CC1200_ACCESS_WRITE | CC1200_ACCESS_BURST;//(len > 1 ? CC1200_ACCESS_BURST : CC1200_ACCESS_SINGLE);
    cc_spi_io(dev, flag, addr, buf, NULL, len);
}

u8 cc_get(cc_dev_t dev, u16 addr)
{
    u8 val = 0;
    cc_read(dev, addr, &val, sizeof(val));
    return val;
}

u16 cc_get16(cc_dev_t dev, u16 addr)
{
    u16 val = 0;
    cc_read(dev, addr, (u8 *) &val, sizeof(val));
    return swap16(val);
}

void cc_set(cc_dev_t dev, u16 addr, u8 val)
{
    cc_write(dev, addr, &val, sizeof(val));
}

void cc_set16(cc_dev_t dev, u16 addr, u16 val)
{
    val = swap16(val);
    cc_write(dev, addr, (u8 *) &val, sizeof(val));
}

void cc_update(cc_dev_t dev, u16 addr, u8 mask, u8 val)
{
    u8 cur = cc_get(dev, addr);
    val = (cur & ~mask) | (val & mask);
    if (val != cur) cc_set(dev, addr, val);
}

void cc_update16(cc_dev_t dev, u16 addr, u16 mask, u16 val)
{
    u16 cur = cc_get16(dev, addr);
    val = (cur & ~mask) | (val & mask);
    if (val != cur) cc_set16(dev, addr, val);
}

void cc_fifo_read(cc_dev_t dev, u8 *buf, u32 len)
{
    cc_read(dev, CC1200_FIFO_ACCESS, buf, len);
}

void cc_fifo_write(cc_dev_t dev, u8 *buf, u32 len)
{
    cc_write(dev, CC1200_FIFO_ACCESS, buf, len);
}
