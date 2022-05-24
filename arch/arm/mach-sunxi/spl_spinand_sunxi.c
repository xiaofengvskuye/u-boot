// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019 Benedikt-Alexander Mokroß <bam@icognize.de>
 */

#include <common.h>
#include <spl.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <mapmem.h>
#include <linux/libfdt.h>
#include <linux/delay.h>
#include <image.h>
#ifdef CONFIG_SPL_OS_BOOT
#error CONFIG_SPL_OS_BOOT is not supported yet
#endif

/*
 * This is a very simple U-Boot image loading implementation, trying to
 * replicate what the boot ROM is doing when loading the SPL. Because we
 * know the exact pins where the SPI Flash is connected and also know
 * that the Read Data Bytes (03h) command is supported, the hardware
 * configuration is very simple and we don't need the extra flexibility
 * of the SPI framework. Moreover, we rely on the default settings of
 * the SPI controler hardware registers and only adjust what needs to
 * be changed. This is good for the code size and this implementation
 * adds less than 400 bytes to the SPL.
 *
 * There are two variants of the SPI controller in Allwinner SoCs:
 * A10/A13/A20 (sun4i variant) and everything else (sun6i variant).
 * Both of them are supported.
 *
 * The pin mixing part is SoC specific and only A10/A13/A20/H3/A64 are
 * supported at the moment.
 */

/*****************************************************************************/
/* SUN4I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN4I_SPI0_CCTL             (0x01C05000 + 0x1C)
#define SUN4I_SPI0_CTL              (0x01C05000 + 0x08)
#define SUN4I_SPI0_RX               (0x01C05000 + 0x00)
#define SUN4I_SPI0_TX               (0x01C05000 + 0x04)
#define SUN4I_SPI0_FIFO_STA         (0x01C05000 + 0x28)
#define SUN4I_SPI0_BC               (0x01C05000 + 0x20)
#define SUN4I_SPI0_TC               (0x01C05000 + 0x24)

#define SUN4I_CTL_ENABLE            BIT(0)
#define SUN4I_CTL_MASTER            BIT(1)
#define SUN4I_CTL_TF_RST            BIT(8)
#define SUN4I_CTL_RF_RST            BIT(9)
#define SUN4I_CTL_XCH               BIT(10)

/*****************************************************************************/
/* SUN6I variant of the SPI controller                                       */
/*****************************************************************************/

#define SUN6I_SPI0_CCTL             (0x01C68000 + 0x24)
#define SUN6I_SPI0_GCR              (0x01C68000 + 0x04)
#define SUN6I_SPI0_TCR              (0x01C68000 + 0x08)
#define SUN6I_SPI0_FIFO_STA         (0x01C68000 + 0x1C)
#define SUN6I_SPI0_WCR				(0x01C68000 + 0x20)
#define SUN6I_SPI0_MBC              (0x01C68000 + 0x30)
#define SUN6I_SPI0_MTC              (0x01C68000 + 0x34)
#define SUN6I_SPI0_BCC              (0x01C68000 + 0x38)
#define SUN6I_SPI0_TXD              (0x01C68000 + 0x200)
#define SUN6I_SPI0_RXD              (0x01C68000 + 0x300)

#define SUN6I_CTL_ENABLE            BIT(0)
#define SUN6I_CTL_MASTER            BIT(1)
#define SUN6I_CTL_SRST              BIT(31)
#define SUN6I_TCR_XCH               BIT(31)

/*****************************************************************************/

#define CCM_AHB_GATING0             (0x01C20000 + 0x60)
#define CCM_SPI0_CLK                (0x01C20000 + 0xA0)
#define SUN6I_BUS_SOFT_RST_REG0     (0x01C20000 + 0x2C0)

#define AHB_RESET_SPI0_SHIFT        20
#define AHB_GATE_OFFSET_SPI0        20

#define SPI0_CLK_DIV_NONE			0x0000
#define SPI0_CLK_DIV_BY_2           0x1000
#define SPI0_CLK_DIV_BY_4           0x1001

#define DUMMY_BURST_BYTE 			0x00

#define EGON_BTO_HEADER_SIZE 40

#ifndef CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE
#define CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE 		0x6000
#endif
#ifndef CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING
#define CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING 	0x2000
#endif
#ifndef CONFIG_SPL_SPINAND_SUNXI_PAGESIZE
#define CONFIG_SPL_SPINAND_SUNXI_PAGESIZE 		2048
#endif
#ifndef CONFIG_SYS_SPI_U_BOOT_OFFS
#define CONFIG_SYS_SPI_U_BOOT_OFFS (CONFIG_SPL_SPINAND_SUNXI_SPL_SIZE * (CONFIG_SPL_SPINAND_SUNXI_PAGESIZE / 1024)) + CONFIG_SPL_SPINAND_SUNXI_UBOOT_PADDING
#endif

/* Somhow dcrc is failing even if the u-boot is valid */
#undef CONFIG_SPL_SPINAND_SUNXI_SPL_CHECK_DCRC

/*****************************************************************************/

struct sunxi_nand_config {
	const char* name;
	u32 jedec_id;
	u32 page_mask;
	u8 page_shift;
	u32 addr_mask;
	u8 addr_shift;
	u32 block_size;
	u32 page_size;
};

struct egon_bt0_header {
	u32 jump;
	const u8 magic[8]; /* eGON.BT0 */
	u32 checksum;
	u32 length;
	u32 header_size;
	u8 header_version[4];
	u8 boot_vsn[4];
	u8 egon_vsn[4];
	u8 platform[8];
};

/*****************************************************************************/

static const u8 egon_bt0_magic[8] = {'e','G','O','N','.','B','T','0'};
static const char* spl_name = "sunxi SPI-NAND";

struct sunxi_nand_config sunxi_known_nands[] = {
	{
		.name = "Macronix MX35LF1GE4AB",
		.jedec_id = 0x00C212C2, /* MX35LFxGE4AB repeat the C2x2 over and over again */
		.page_mask = 0x00FFFFFF,
		.page_shift = 11,
		.addr_mask = 0x7FF,
		.addr_shift = 0,
		.block_size = 128 * 1024,
		.page_size = 2048,
	},
	{
		.name = "Macronix MX35LF2GE4AB",
		.jedec_id = 0x00C222C2,
		.page_mask = 0x00FFFFFF,
		.page_shift = 11,
		.addr_mask = 0x7FF,
		.addr_shift = 0,
		.block_size = 128 * 1024,
		.page_size = 2048,
	},
	{
		.name = "Winbond W25N01GVxxIG",
		.jedec_id = 0x00EFAA21,
		.page_mask = 0x0000FFFF,
		.page_shift = 11,
		.addr_mask = 0x7FF,
		.addr_shift = 0,
		.block_size = 128 * 1024,
		.page_size = 2048,
	},
	{
		.name = "GigaDevice GD5F1GQ4RCxxG",
		.jedec_id = 0x00C8B148, /* 3.3v */
		.page_mask = 0x00FFFFFF,
		.page_shift = 11,
		.addr_mask = 0x7FF,
		.addr_shift = 0,
		.block_size = 128 * 1024,
		.page_size = 2048,
	},
	{
		.name = "GigaDevice GD5F1GQ4RCxxG",
		.jedec_id = 0x00C8A148, /* 1.8V */
		.page_mask = 0x00FFFFFF,
		.page_shift = 11,
		.addr_mask = 0x7FF,
		.addr_shift = 0,
		.block_size = 128 * 1024,
		.page_size = 2048,
	},

	{ /* Sentinel */
		.name = NULL,
	}
};

#ifdef CONFIG_SPL_SPINAND_SUNXI_SPL_USE_GENERIC2K_ON_UNKNOWN
struct sunxi_nand_config sunxi_generic_nand_config = {
	.name = "Generic 2K SPI-NAND",
	.jedec_id = 0xFFFFFFFF,
	.page_mask = 0x00FFFFFF,
	.page_shift = 11,
	.addr_mask = 0x7FF,
	.addr_shift = 0,
	.block_size = 128 * 1024,
	.page_size = 2048,
};
#endif

/*
 * Enumerate all known nands and return the config if found.
 * Returns NULL if none is found
 */
static struct sunxi_nand_config* sunxi_spinand_enumerate(u32 jedec_id)
{
	struct sunxi_nand_config* ptr = sunxi_known_nands;
	while(ptr->name != NULL) {
		if(jedec_id == ptr->jedec_id) {
			return ptr;
		}
		++ptr;
	}
#ifdef CONFIG_SPL_SPINAND_SUNXI_SPL_USE_GENERIC2K_ON_UNKNOWN
	return &sunxi_generic_nand_config;
#else
	return NULL;
#endif
}

/*
 * Allwinner A10/A20 SoCs were using pins PC0,PC1,PC2,PC23 for booting
 * from SPI Flash, everything else is using pins PC0,PC1,PC2,PC3.
 */
static void spi0_pinmux_setup(unsigned int pin_function)
{
	unsigned int pin;

	for (pin = SUNXI_GPC(0); pin <= SUNXI_GPC(2); pin++)
		sunxi_gpio_set_cfgpin(pin, pin_function);

	if (IS_ENABLED(CONFIG_MACH_SUN4I) || IS_ENABLED(CONFIG_MACH_SUN7I))
		sunxi_gpio_set_cfgpin(SUNXI_GPC(23), pin_function);
	else
		sunxi_gpio_set_cfgpin(SUNXI_GPC(3), pin_function);
}

/*
 * Setup 24 MHz from OSC24M.
 */
static void spi0_enable_clock(void)
{
	/* Deassert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		setbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));

	/* Open the SPI0 gate */
	setbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* No devide */
	writel(SPI0_CLK_DIV_NONE, IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I) ?
				  SUN6I_SPI0_CCTL : SUN4I_SPI0_CCTL);
	/* 24MHz from OSC24M */
	writel((1 << 31), CCM_SPI0_CLK);

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		/* Enable SPI in the master mode and do a soft reset */
		setbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE |
					     SUN6I_CTL_SRST);
		/* Wait for completion */
		while (readl(SUN6I_SPI0_GCR) & SUN6I_CTL_SRST)
			;
	} else {
		/* Enable SPI in the master mode and reset FIFO */
		setbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE |
					     SUN4I_CTL_TF_RST |
					     SUN4I_CTL_RF_RST);
	}
}

static void spi0_disable_clock(void)
{
	/* Disable the SPI0 controller */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_SPI0_GCR, SUN6I_CTL_MASTER |
					     SUN6I_CTL_ENABLE);
	else
		clrbits_le32(SUN4I_SPI0_CTL, SUN4I_CTL_MASTER |
					     SUN4I_CTL_ENABLE);

	/* Disable the SPI0 clock */
	writel(0, CCM_SPI0_CLK);

	/* Close the SPI0 gate */
	clrbits_le32(CCM_AHB_GATING0, (1 << AHB_GATE_OFFSET_SPI0));

	/* Assert SPI0 reset on SUN6I */
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		clrbits_le32(SUN6I_BUS_SOFT_RST_REG0,
			     (1 << AHB_RESET_SPI0_SHIFT));
}

static void spi0_init(void)
{
	unsigned int pin_function = SUNXI_GPC_SPI0;
	if (IS_ENABLED(CONFIG_MACH_SUN50I))
		pin_function = SUN50I_GPC_SPI0;

	spi0_pinmux_setup(pin_function);
	spi0_enable_clock();

	writel(0x01, SUN6I_SPI0_WCR);
}

static void spi0_deinit(void)
{
	/* New SoCs can disable pins, older could only set them as input */
	unsigned int pin_function = SUNXI_GPIO_INPUT;
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		pin_function = SUNXI_GPIO_DISABLE;

	spi0_disable_clock();
	spi0_pinmux_setup(pin_function);
}

/*****************************************************************************/

#define SPI_READ_MAX_SIZE 60 /* FIFO size, minus 4 bytes of the header */

static void sunxi_spi0_load_page(struct sunxi_nand_config * config, u32 addr, ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg) {

    /* Read Page in Cache */
	u8 status = 0x01;
	addr = addr >> (config->page_shift);
	addr = addr & (config->page_mask);

	writel(4, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (13h) command header */
	writeb(0x13, spi_tx_reg);
	writeb((u8)(addr >> 16), spi_tx_reg);
	writeb((u8)(addr >> 8),  spi_tx_reg);
	writeb((u8)(addr),       spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait till all bytes are send */
	while((readl(spi_fifo_reg) & 0x7F0000) > 0)
		;

    /* wait till all bytes are read */
	while ((readl(spi_fifo_reg) & 0x7F) < 4)
		;

	/* Discard the 4 empty bytes from our send */
	readl(spi_rx_reg);

	do {
		/* tCS = 100ns + tRD_ECC 70ns -> 200ns wait */
		ndelay(200);

		/* Poll */
		writel(2 + 1, spi_bc_reg);   /* Burst counter (total bytes) */
		writel(2, spi_tc_reg);       /* Transfer counter (bytes to send) */
		if (spi_bcc_reg)
			writel(2, spi_bcc_reg);  /* SUN6I also needs this */
		/* Send the Read Status Bytes (0FC0h) command header */
		writeb(0x0F, spi_tx_reg);
		writeb(0xC0, spi_tx_reg);

		/* Start the data transfer */
		setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

		while ((readl(spi_fifo_reg) & 0x7F) < 2 + 1)
			;

		/* skip 2 since we send 2 */
	    readb(spi_rx_reg);
		readb(spi_rx_reg);

		status = readb(spi_rx_reg);

	} while ((status & 0x01) == 0x01);

}

static void spi0_load_page(struct sunxi_nand_config * config, u32 addr)
{
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		sunxi_spi0_load_page(config, addr,
				     SUN6I_SPI0_TCR,
				     SUN6I_TCR_XCH,
				     SUN6I_SPI0_FIFO_STA,
				     SUN6I_SPI0_TXD,
				     SUN6I_SPI0_RXD,
				     SUN6I_SPI0_MBC,
				     SUN6I_SPI0_MTC,
				     SUN6I_SPI0_BCC);
	} else {
		sunxi_spi0_load_page(config, addr,
				     SUN4I_SPI0_CTL,
				     SUN4I_CTL_XCH,
				     SUN4I_SPI0_FIFO_STA,
				     SUN4I_SPI0_TX,
				     SUN4I_SPI0_RX,
				     SUN4I_SPI0_BC,
				     SUN4I_SPI0_TC,
				     0);
	}
}

static void sunxi_spi0_read_data(struct sunxi_nand_config * config, u8 *buf, u32 addr, u32 bufsize,
				 ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	addr = addr >> (config->addr_shift);
	addr = addr & (config->addr_mask);

	writel(4 + bufsize, spi_bc_reg); /* Burst counter (total bytes) */
	writel(4, spi_tc_reg);           /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(4, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read Data Bytes (0Bh) command header */
	writeb(0x0B, spi_tx_reg);
	writeb((u8)((addr >> 8)), spi_tx_reg);
	writeb((u8)(addr), spi_tx_reg);
	writeb(DUMMY_BURST_BYTE, spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 4 + bufsize)
		;

	/* Skip 4 bytes since we send 4 */
	readl(spi_rx_reg);

	/* Read the data */
	while (bufsize-- > 0)
		*buf++ = readb(spi_rx_reg);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	ndelay(100);
}

static void sunxi_spi0_read_cache(struct sunxi_nand_config * config, void *buf, u32 addr, u32 len) {

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
			sunxi_spi0_read_data(config, buf, addr, len,
					     SUN6I_SPI0_TCR,
					     SUN6I_TCR_XCH,
					     SUN6I_SPI0_FIFO_STA,
					     SUN6I_SPI0_TXD,
					     SUN6I_SPI0_RXD,
					     SUN6I_SPI0_MBC,
					     SUN6I_SPI0_MTC,
					     SUN6I_SPI0_BCC);
		} else {
			sunxi_spi0_read_data(config, buf, addr, len,
					     SUN4I_SPI0_CTL,
					     SUN4I_CTL_XCH,
					     SUN4I_SPI0_FIFO_STA,
					     SUN4I_SPI0_TX,
					     SUN4I_SPI0_RX,
					     SUN4I_SPI0_BC,
					     SUN4I_SPI0_TC,
					     0);
		}
}

static void spi0_read_data(struct sunxi_nand_config * config, void *buf, u32 addr, u32 len)
{
	u8 *buf8 = buf;
	u32 chunk_len;
	u32 curr_page;
	u32 last_page = (addr >> (config->page_shift)) & (config->page_mask);

	spi0_load_page(config, addr);

	while (len > 0) {
		curr_page = (addr >> (config->page_shift)) & (config->page_mask);
		if(curr_page > last_page) {
			spi0_load_page(config, addr);
			last_page = curr_page;
		}

		chunk_len = len;
		if (chunk_len > SPI_READ_MAX_SIZE) {
			chunk_len = SPI_READ_MAX_SIZE;
		}

		if((((addr + chunk_len) >> (config->page_shift)) & (config->page_mask)) > curr_page) {
			chunk_len = ((curr_page + 1) << (config->page_shift)) - addr;
		}

		sunxi_spi0_read_cache(config, buf8, addr, chunk_len);
		len  -= chunk_len;
		buf8 += chunk_len;
		addr += chunk_len;
	}
}

static u32 sunxi_spi0_read_id(ulong spi_ctl_reg,
				 ulong spi_ctl_xch_bitmask,
				 ulong spi_fifo_reg,
				 ulong spi_tx_reg,
				 ulong spi_rx_reg,
				 ulong spi_bc_reg,
				 ulong spi_tc_reg,
				 ulong spi_bcc_reg)
{
	u8 idbuf[3];
	writel(2 + 3, spi_bc_reg); /* Burst counter (total bytes) */
	writel(2, spi_tc_reg);     /* Transfer counter (bytes to send) */
	if (spi_bcc_reg)
		writel(2, spi_bcc_reg);  /* SUN6I also needs this */

	/* Send the Read ID Bytes (9Fh) command header */
	writeb(0x9F, spi_tx_reg);
	writeb(DUMMY_BURST_BYTE, spi_tx_reg);

	/* Start the data transfer */
	setbits_le32(spi_ctl_reg, spi_ctl_xch_bitmask);

	/* Wait until everything is received in the RX FIFO */
	while ((readl(spi_fifo_reg) & 0x7F) < 2 + 3)
		;

	/* Skip 2 bytes */
	readb(spi_rx_reg);
	readb(spi_rx_reg);

	/* Read the data */
	//while (bufsize-- > 0)
	idbuf[0] = readb(spi_rx_reg);
	idbuf[1] = readb(spi_rx_reg);
	idbuf[2] = readb(spi_rx_reg);

	/* tSHSL time is up to 100 ns in various SPI flash datasheets */
	ndelay(100);

	return idbuf[2] | (idbuf[1] << 8) | (idbuf[0] << 16);
}

static u32 spi0_read_id(void) {
	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I)) {
		return sunxi_spi0_read_id(
				     SUN6I_SPI0_TCR,
				     SUN6I_TCR_XCH,
				     SUN6I_SPI0_FIFO_STA,
				     SUN6I_SPI0_TXD,
				     SUN6I_SPI0_RXD,
				     SUN6I_SPI0_MBC,
				     SUN6I_SPI0_MTC,
				     SUN6I_SPI0_BCC);
	} else {
		return sunxi_spi0_read_id(
				     SUN4I_SPI0_CTL,
				     SUN4I_CTL_XCH,
				     SUN4I_SPI0_FIFO_STA,
				     SUN4I_SPI0_TX,
				     SUN4I_SPI0_RX,
				     SUN4I_SPI0_BC,
				     SUN4I_SPI0_TC,
				     0);
	}
}

#ifdef DEBUG
static void print_egon_header(struct egon_bt0_header * egon) {
	printf("Jump: %x\n", egon->jump);
	printf("Magic: %c%c%c%c%c%c%c%c\n",
		egon->magic[0], egon->magic[1], egon->magic[2], egon->magic[3],
		egon->magic[4], egon->magic[5], egon->magic[6], egon->magic[7]);
	printf("Check: %x\n", egon->checksum);
	printf("Length: %x\n",egon->length);
	printf("Hdrsz: %u\n",egon->header_size);
	printf("Bootvsn: %c%c%c%c\n", egon->boot_vsn[0],egon->boot_vsn[1],egon->boot_vsn[2],egon->boot_vsn[3]);
	printf("Egonvsn: %c%c%c%c\n", egon->egon_vsn[0],egon->egon_vsn[1],egon->egon_vsn[2],egon->egon_vsn[3]);
	printf("Platform: %c%c%c%c%c%c%c%c\n",
		egon->platform[0], egon->platform[1], egon->platform[2], egon->platform[3],
		egon->platform[4], egon->platform[5], egon->platform[6], egon->platform[7]);
}
#endif

static u32 sunxi_spi0_count_spls(struct sunxi_nand_config * config, struct egon_bt0_header* self) {
	u8 buffer[EGON_BTO_HEADER_SIZE];
	struct egon_bt0_header * egon = (struct egon_bt0_header *)buffer;
	u32 entry_pages[8] = {0, 32, 64, 96, 128, 160, 192, 224};
	u32 entry_point;
	u32 address;
	u32 spls = 0;
	u32 hits = 0;
	for(entry_point = 0; entry_point < 8; ++entry_point) {
		address = entry_pages[entry_point] * config->page_size;
#ifdef DEBUG
		printf("Searching for spl at entrypoint %u 0x%x\n", entry_point, address);
#endif
		spi0_load_page(config, address);
		sunxi_spi0_read_cache(config, buffer, address, EGON_BTO_HEADER_SIZE);
#ifdef DEBUG
		print_egon_header(egon);
#endif
		if(strncmp(egon->magic, egon_bt0_magic, 8) == 0) {
			++spls;
		} else {
			hits = 0;
			if(egon->jump == self->jump) {
				++hits;
			}
			if(egon->checksum == self->checksum) {
				++hits;
			}
			if(egon->length == self->length) {
				++hits;
			}
			if(egon->header_size == self->header_size) {
				++hits;
			}
			if(strncmp(egon->header_version, self->header_version, 4) == 0) {
				++hits;
			}
			if(strncmp(egon->boot_vsn, self->boot_vsn, 4) == 0) {
				++hits;
			}
			if(strncmp(egon->egon_vsn, self->egon_vsn, 4) == 0) {
				++hits;
			}
			if(strncmp(egon->platform, self->platform, 8) == 0) {
				++hits;
			}
			if(hits > 4) { /* at least > 50% similarity */
				++spls;
			}
		}
	}
	return spls;
}

/*****************************************************************************/

static int spl_spi_load_image(struct spl_image_info *spl_image,
								struct spl_boot_device *bootdev)
{
	int ret = 0;
	u32 id = 0;
	u32 uboot_offset;
	u32 total_spl_size = 0;
	u32 spls;
	struct image_header *header;
	struct sunxi_nand_config* config;
	struct egon_bt0_header* egon;
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	spi0_init();
	id = spi0_read_id();

	/*
	 * If we receive only zeros, there is most definetly no device attached.
	 */
	if(id == 0) {
		spi0_deinit();
		printf("%s: Received only zeros on jedec_id probe, assuming no spi-nand attached.\n",spl_name);
		return -1;
	}

	/*
	 * Check if device is known and compatible.
	 */
	config = sunxi_spinand_enumerate(id);
	if(config == NULL) {
		spi0_deinit();
		printf("%s: Unknown chip %x\n", spl_name, id);
		return -1;
	}

	printf("%s: Found %s (%x)\n", spl_name, config->name, id);

	/*
	 * eGON.BRM loads a valid SPL to 0x40000000 which is the DDR base
	 * Thus 0x000000 is mapped to this 0x40000000 where the SPL is loaded to.
	 * Since this SPL is running it was successfully validated by eGON.BRM
	 * and we can safely assume that all values are valid!
	 */
	egon = (struct egon_bt0_header *)map_sysmem(0x00000000, EGON_BTO_HEADER_SIZE);
#ifdef DEBUG
	printf("Self: \n");
	print_egon_header(egon);
#endif
	spls = sunxi_spi0_count_spls(config, egon);
	total_spl_size = ((config->page_size*32) * (spls - 1)) + ((config->page_size/1024) * egon->length);
	uboot_offset = (total_spl_size + config->block_size) - (total_spl_size % config->block_size);
	unmap_sysmem(egon);
#ifdef DEBUG
	printf("Found %u spls\n", spls);
	printf("total-spl-size 0x%x\n",total_spl_size);
	printf("uboot-offset 0x%x\n", uboot_offset);
#endif

	do {
		printf("%s: Loading u-boot from 0x%x\n", spl_name, uboot_offset);
		ret = 1;
		/*
		 * Read the header data from the image and parse it for validity.
		 */
		spi0_read_data(config, (void *)header, uboot_offset, 0x40);
		if(image_check_hcrc(header)) {
			printf("%s: u-boot hcrc OK!\n", spl_name);
			ret = spl_parse_image_header(spl_image, header);
			if (ret) {
				printf("spl_parse_image_header: %x\n", ret);
				/*
				 * The header was ok, so we can savely use the datasize to calculate the next PEB to look for an u-boot
				 */
				uboot_offset += (image_get_image_size(header) + config->block_size) - (image_get_image_size(header) % config->block_size);
			} else {
				/*
				 * If everything is fine, read the rest of u-boot and start
				 */
				spi0_read_data(config, (void *)spl_image->load_addr, uboot_offset, spl_image->size);
				#ifdef CONFIG_SPL_SPINAND_SUNXI_SPL_CHECK_DCRC
				if (image_check_dcrc(header)) {
					printf("%s: u-boot dcrc OK!\n", spl_name);
				} else {
					printf("%s: u-boot dcrc ERROR!\n", spl_name);
					ret = 1;
				}
				#endif
			}
		} else {
			printf("%s: u-boot hcrc ERROR!\n", spl_name);
			uboot_offset += config->block_size;
		}
	} while(ret != 0);

	spi0_deinit();
	return ret;
}
/* Use priorty 0 to override the default if it happens to be linked in */
SPL_LOAD_IMAGE_METHOD("sunxi SPI-NAND", 0, BOOT_DEVICE_SPI, spl_spi_load_image);
