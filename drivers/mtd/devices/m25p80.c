/*
 * MTD SPI driver for ST M25Pxx (and similar) serial flash chips
 *
 * Author: Mike Lavender, mike@steroidmicros.com
 *
 * Copyright (c) 2005, Intec Automation Inc.
 *
 * Some parts are based on lart.c by Abraham Van Der Merwe
 *
 * Cleaned up and generalized based on mtd_dataflash.c
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/spi-nor.h>

#define	MAX_CMD_SIZE		16
struct m25p {
	struct spi_device	*spi;
	struct spi_nor		spi_nor;
	u8			command[MAX_CMD_SIZE];
};

static inline int m25p80_proto2nbits(enum spi_nor_protocol proto,
				     unsigned *code_nbits,
				     unsigned *addr_nbits,
				     unsigned *data_nbits)
{
	if (code_nbits)
		*code_nbits = SNOR_PROTO_CODE_FROM_PROTO(proto);
	if (addr_nbits)
		*addr_nbits = SNOR_PROTO_ADDR_FROM_PROTO(proto);
	if (data_nbits)
		*data_nbits = SNOR_PROTO_DATA_FROM_PROTO(proto);

	return 0;
}

static int m25p80_read_reg(struct spi_nor *nor, u8 code, u8 *val, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned code_nbits, data_nbits;
	struct spi_transfer xfers[2];
	int ret;

	/* Check the total length of command op code and data. */
	if (len + 1 > MAX_CMD_SIZE)
		return -EINVAL;

	/* Get transfer protocols (addr_nbits is not relevant here). */
	ret = m25p80_proto2nbits(nor->reg_proto,
				 &code_nbits, NULL, &data_nbits);
	if (ret < 0)
		return ret;

	/* Set up transfers. */
	memset(xfers, 0, sizeof(xfers));

	flash->command[0] = code;
	xfers[0].len = 1;
	xfers[0].tx_buf = flash->command;
	xfers[0].tx_nbits = code_nbits;

	xfers[1].len = len;
	xfers[1].rx_buf = &flash->command[1];
	xfers[1].rx_nbits = data_nbits;

	/* Process command. */
	ret = spi_sync_transfer(spi, xfers, 2);
	if (ret < 0)
		dev_err(&spi->dev, "error %d reading %x\n", ret, code);
	else
		memcpy(val, &flash->command[1], len);

	return ret;
}

static void m25p_addr2cmd(struct spi_nor *nor, unsigned int addr, u8 *cmd)
{
	/* opcode is in cmd[0] */
	cmd[1] = addr >> (nor->addr_width * 8 -  8);
	cmd[2] = addr >> (nor->addr_width * 8 - 16);
	cmd[3] = addr >> (nor->addr_width * 8 - 24);
	cmd[4] = addr >> (nor->addr_width * 8 - 32);
}

static int m25p_cmdsz(struct spi_nor *nor)
{
	return 1 + nor->addr_width;
}

static int m25p80_write_reg(struct spi_nor *nor, u8 opcode, u8 *buf, int len)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned code_nbits, data_nbits, num_xfers = 1;
	struct spi_transfer xfers[2];
	int ret;

	/* Check the total length of command op code and data. */
	if (buf && (len + 1 > MAX_CMD_SIZE))
		return -EINVAL;

	/* Get transfer protocols (addr_nbits is not relevant here). */
	ret = m25p80_proto2nbits(nor->reg_proto,
				 &code_nbits, NULL, &data_nbits);
	if (ret < 0)
		return ret;

	/* Set up transfer(s). */
	memset(xfers, 0, sizeof(xfers));

	flash->command[0] = opcode;
	xfers[0].len = 1;
	xfers[0].tx_buf = flash->command;
	xfers[0].tx_nbits = code_nbits;

	if (buf) {
		memcpy(&flash->command[1], buf, len);
		if (data_nbits == code_nbits) {
			xfers[0].len += len;
		} else {
			xfers[1].len = len;
			xfers[1].tx_buf = &flash->command[1];
			xfers[1].tx_nbits = data_nbits;
			num_xfers++;
		}
	}

	/* Process command. */
	return spi_sync_transfer(spi, xfers, num_xfers);
}

static void m25p80_write(struct spi_nor *nor, loff_t to, size_t len,
			size_t *retlen, const u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned code_nbits, addr_nbits, data_nbits, num_xfers = 1;
	struct spi_transfer xfers[3];
	struct spi_message m;
	int cmd_sz = m25p_cmdsz(nor);
	int ret;

	if (nor->program_opcode == SPINOR_OP_AAI_WP && nor->sst_write_second)
		cmd_sz = 1;

	/* Get transfer protocols. */
	ret = m25p80_proto2nbits(nor->write_proto,
				 &code_nbits, &addr_nbits, &data_nbits);
	if (ret < 0)
		return;

	/* Set up transfers. */
	memset(xfers, 0, sizeof(xfers));

	flash->command[0] = nor->program_opcode;
	xfers[0].len = 1;
	xfers[0].tx_buf = flash->command;
	xfers[0].tx_nbits = code_nbits;

	if (cmd_sz > 1) {
		m25p_addr2cmd(nor, to, flash->command);
		if (addr_nbits == code_nbits) {
			xfers[0].len += nor->addr_width;
		} else {
			xfers[1].len = nor->addr_width;
			xfers[1].tx_buf = &flash->command[1];
			xfers[1].tx_nbits = addr_nbits;
			num_xfers++;
		}
	}

	xfers[num_xfers].len = len;
	xfers[num_xfers].tx_buf = buf;
	xfers[num_xfers].tx_nbits = data_nbits;
	num_xfers++;

	/* Process command. */
	spi_message_init_with_transfers(&m, xfers, num_xfers);
	spi_sync(spi, &m);

	*retlen += m.actual_length - cmd_sz;
}

/*
 * Read an address range from the nor chip.  The address range
 * may be any size provided it is within the physical boundaries.
 */
static int m25p80_read(struct spi_nor *nor, loff_t from, size_t len,
			size_t *retlen, u_char *buf)
{
	struct m25p *flash = nor->priv;
	struct spi_device *spi = flash->spi;
	unsigned code_nbits, addr_nbits, data_nbits, num_xfers = 1;
	unsigned int dummy = nor->read_dummy;
	int ret;
	struct spi_transfer xfers[3];
	struct spi_message m;

	/* Get transfer protocols. */
	ret = m25p80_proto2nbits(nor->read_proto,
				 &code_nbits, &addr_nbits, &data_nbits);
	if (ret < 0)
		return ret;

	/* convert the dummy cycles to the number of bytes */
	dummy = (dummy * addr_nbits) / 8;

	/* Set up transfers. */
	memset(xfers, 0, sizeof(xfers));

	flash->command[0] = nor->read_opcode;
	xfers[0].len = 1;
	xfers[0].tx_buf = flash->command;
	xfers[0].tx_nbits = code_nbits;

	m25p_addr2cmd(nor, from, flash->command);
	/*
	 * Clear all dummy/mode cycle bits to avoid sending some manufacturer
	 * specific pattern, which might make the memory enter its Continuous
	 * Read mode by mistake.
	 */
	memset(flash->command + 1 + nor->addr_width, 0, dummy);

	if (addr_nbits == code_nbits) {
		xfers[0].len += nor->addr_width + dummy;
	} else {
		xfers[1].len = nor->addr_width + dummy;
		xfers[1].tx_buf = &flash->command[1];
		xfers[1].tx_nbits = addr_nbits;
		num_xfers++;
	}

	xfers[num_xfers].len = len;
	xfers[num_xfers].rx_buf = buf;
	xfers[num_xfers].rx_nbits = data_nbits;
	num_xfers++;

	/* Process command. */
	spi_message_init_with_transfers(&m, xfers, num_xfers);
	spi_sync(spi, &m);

	*retlen = m.actual_length - m25p_cmdsz(nor) - dummy;
	return 0;
}

static int m25p80_erase(struct spi_nor *nor, loff_t offset)
{
	struct m25p *flash = nor->priv;

	dev_dbg(nor->dev, "%dKiB at 0x%08x\n",
		flash->spi_nor.mtd.erasesize / 1024, (u32)offset);

	/* Set up command buffer. */
	flash->command[0] = nor->erase_opcode;
	m25p_addr2cmd(nor, offset, flash->command);

	spi_write(flash->spi, flash->command, m25p_cmdsz(nor));

	return 0;
}

/*
 * board specific setup should have ensured the SPI clock used here
 * matches what the READ command supports, at least until this driver
 * understands FAST_READ (for clocks over 25 MHz).
 */
static int m25p_probe(struct spi_device *spi)
{
	struct mtd_part_parser_data	ppdata;
	struct flash_platform_data	*data;
	struct m25p *flash;
	struct spi_nor *nor;
	struct spi_nor_modes modes = {
		.rd_modes = SNOR_MODE_SLOW,
		.wr_modes = SNOR_MODE_1_1_1,
	};
	char *flash_name = NULL;
	int ret;

	data = dev_get_platdata(&spi->dev);

	flash = devm_kzalloc(&spi->dev, sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	nor = &flash->spi_nor;

	/* install the hooks */
	nor->read = m25p80_read;
	nor->write = m25p80_write;
	nor->erase = m25p80_erase;
	nor->write_reg = m25p80_write_reg;
	nor->read_reg = m25p80_read_reg;

	nor->dev = &spi->dev;
	nor->flash_node = spi->dev.of_node;
	nor->priv = flash;

	spi_set_drvdata(spi, flash);
	flash->spi = spi;

	if (spi->mode & SPI_RX_QUAD)
		modes.rd_modes |= SNOR_MODE_1_1_4;
	else if (spi->mode & SPI_RX_DUAL)
		modes.rd_modes |= SNOR_MODE_1_1_2;

	if (data && data->name)
		nor->mtd.name = data->name;

	/* For some (historical?) reason many platforms provide two different
	 * names in flash_platform_data: "name" and "type". Quite often name is
	 * set to "m25p80" and then "type" provides a real chip name.
	 * If that's the case, respect "type" and ignore a "name".
	 */
	if (data && data->type)
		flash_name = data->type;
	else
		flash_name = spi->modalias;

	ret = spi_nor_scan(nor, flash_name, &modes);
	if (ret)
		return ret;

	ppdata.of_node = spi->dev.of_node;

	return mtd_device_parse_register(&nor->mtd, NULL, &ppdata,
			data ? data->parts : NULL,
			data ? data->nr_parts : 0);
}


static int m25p_remove(struct spi_device *spi)
{
	struct m25p	*flash = spi_get_drvdata(spi);

	/* Clean up MTD stuff. */
	return mtd_device_unregister(&flash->spi_nor.mtd);
}

/*
 * Do NOT add to this array without reading the following:
 *
 * Historically, many flash devices are bound to this driver by their name. But
 * since most of these flash are compatible to some extent, and their
 * differences can often be differentiated by the JEDEC read-ID command, we
 * encourage new users to add support to the spi-nor library, and simply bind
 * against a generic string here (e.g., "jedec,spi-nor").
 *
 * Many flash names are kept here in this list (as well as in spi-nor.c) to
 * keep them available as module aliases for existing platforms.
 */
static const struct spi_device_id m25p_ids[] = {
	/*
	 * Entries not used in DTs that should be safe to drop after replacing
	 * them with "nor-jedec" in platform data.
	 */
	{"s25sl064a"},	{"w25x16"},	{"m25p10"},	{"m25px64"},

	/*
	 * Entries that were used in DTs without "nor-jedec" fallback and should
	 * be kept for backward compatibility.
	 */
	{"at25df321a"},	{"at25df641"},	{"at26df081a"},
	{"mr25h256"},
	{"mx25l4005a"},	{"mx25l1606e"},	{"mx25l6405d"},	{"mx25l12805d"},
	{"mx25l25635e"},{"mx66l51235l"},
	{"n25q064"},	{"n25q128a11"},	{"n25q128a13"},	{"n25q512a"},
	{"s25fl256s1"},	{"s25fl512s"},	{"s25sl12801"},	{"s25fl008k"},
	{"s25fl064k"},
	{"sst25vf040b"},{"sst25vf016b"},{"sst25vf032b"},{"sst25wf040"},
	{"m25p40"},	{"m25p80"},	{"m25p16"},	{"m25p32"},
	{"m25p64"},	{"m25p128"},
	{"w25x80"},	{"w25x32"},	{"w25q32"},	{"w25q32dw"},
	{"w25q80bl"},	{"w25q128"},	{"w25q256"},

	/* Flashes that can't be detected using JEDEC */
	{"m25p05-nonjedec"},	{"m25p10-nonjedec"},	{"m25p20-nonjedec"},
	{"m25p40-nonjedec"},	{"m25p80-nonjedec"},	{"m25p16-nonjedec"},
	{"m25p32-nonjedec"},	{"m25p64-nonjedec"},	{"m25p128-nonjedec"},

	{ },
};
MODULE_DEVICE_TABLE(spi, m25p_ids);

static const struct of_device_id m25p_of_table[] = {
	/*
	 * Generic compatibility for SPI NOR that can be identified by the
	 * JEDEC READ ID opcode (0x9F). Use this, if possible.
	 */
	{ .compatible = "jedec,spi-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, m25p_of_table);

static struct spi_driver m25p80_driver = {
	.driver = {
		.name	= "m25p80",
		.of_match_table = m25p_of_table,
	},
	.id_table	= m25p_ids,
	.probe	= m25p_probe,
	.remove	= m25p_remove,

	/* REVISIT: many of these chips have deep power-down modes, which
	 * should clearly be entered on suspend() to minimize power use.
	 * And also when they're otherwise idle...
	 */
};

module_spi_driver(m25p80_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Lavender");
MODULE_DESCRIPTION("MTD SPI driver for ST M25Pxx flash chips");
