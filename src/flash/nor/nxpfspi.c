/* SPDX-License-Identifier: GPL-2.0 */
/*
 * NXP FlexSPI Flash driver
 *
 * Copyright (C) 2020 Tobias Waldekranz <tobias@waldekranz.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/bits.h>

#include "imp.h"
#include "spi.h"

struct nxpfspi {
	const struct flash_device *model;
	uint32_t regs;
	bool probed;
};

enum nxpfspi_reg {
	MCR0 = 0x00,
	MCR1 = 0x04,
	MCR2 = 0x08,
	AHBCR = 0x0C,
	INTEN = 0x10,
	INTR = 0x14,
	LUTKEY = 0x18,
	LUTCR = 0x1C,
	AHBRXBUF0CR0 = 0x20,
	AHBRXBUF1CR0 = 0x24,
	AHBRXBUF2CR0 = 0x28,
	AHBRXBUF3CR0 = 0x2C,
	AHBRXBUF4CR0 = 0x30,
	AHBRXBUF5CR0 = 0x34,
	AHBRXBUF6CR0 = 0x38,
	AHBRXBUF7CR0 = 0x3C,
	FLSHA1CR0 = 0x60,
	FLSHA2CR0 = 0x64,
	FLSHB1CR0 = 0x68,
	FLSHB2CR0 = 0x6C,
	FLSHA1CR1 = 0x70,
	FLSHA2CR1 = 0x74,
	FLSHB1CR1 = 0x78,
	FLSHB2CR1 = 0x7C,
	FLSHA1CR2 = 0x80,
	FLSHA2CR2 = 0x84,
	FLSHB1CR2 = 0x88,
	FLSHB2CR2 = 0x8C,
	FLSHCR4 = 0x94,
	IPCR0 = 0xA0,
	IPCR1 = 0xA4,
	IPCMD = 0xB0,
	DLPR = 0xB4,
	IPRXFCR = 0xB8,
	IPTXFCR = 0xBC,
	DLLACR = 0xC0,
	DLLBCR = 0xC4,
	STS0 = 0xE0,
	STS1 = 0xE4,
	STS2 = 0xE8,
	AHBSPNDSTS = 0xEC,
	IPRXFSTS = 0xF0,
	IPTXFSTS = 0xF4,
	RFDR0 = 0x100,
	TFDR0 = 0x180,
	LUT0 = 0x200,
};

enum nxpfspi_lutop {
	LUT_STOP  = 0x00,
	LUT_CMD   = 0x01,
	LUT_RADDR = 0x02,
	LUT_WRITE = 0x08,
	LUT_READ  = 0x09,
	LUT_DATSZ = 0x0b,
};

#define lutinsn(_op, _arg) (((_op) << 10) | (_arg))

static bool nxpfspi_is_probed(struct flash_bank *bank)
{
	struct nxpfspi *fspi = bank->driver_priv;

	if (!fspi->probed) {
		LOG_ERROR("Flash bank not probed");
		return false;
	}

	return true;
}

static inline int nxpfspi_read(struct flash_bank *bank, enum nxpfspi_reg reg,
			       uint32_t *val)
{
	struct nxpfspi *fspi = bank->driver_priv;
	int err;

	err = target_read_u32(bank->target, fspi->regs + reg, val);
	if (err)
		LOG_ERROR("Unable to read register");

	return err;
}

static inline int nxpfspi_write(struct flash_bank *bank, enum nxpfspi_reg reg,
				uint32_t val)
{
	struct nxpfspi *fspi = bank->driver_priv;
	int err;

	err = target_write_u32(bank->target, fspi->regs + reg, val);
	if (err)
		LOG_ERROR("Unable to write register");

	return err;
}

static inline int nxpfspi_wait(struct flash_bank *bank, enum nxpfspi_reg reg,
			       uint32_t val, uint32_t mask, int timeout_ms)
{
	uint32_t now;
	int err;

	for (; timeout_ms >= 0; timeout_ms--) {
		err = nxpfspi_read(bank, reg, &now);
		if (err)
			return err;

		if ((now & mask) == val)
			return ERROR_OK;

		alive_sleep(1);
	}

	LOG_ERROR("Timed out waiting for register %#2x to match "
		  "%#8x (mask %#8x), last read %#8x", reg, val, mask, now);
	return ERROR_FAIL;
}



static inline int nxpfspi_lut_write(struct flash_bank *bank, uint8_t seqid,
				    uint8_t index, uint16_t insn)
{
	struct nxpfspi *fspi = bank->driver_priv;

	return target_write_u16(bank->target,
				fspi->regs + LUT0 + (seqid << 4) + (index << 1),
				insn);
}


static int nxpfspi_lut_set_lock(struct flash_bank *bank, bool locked)
{
	int err;

	err = nxpfspi_write(bank, LUTKEY, 0x5af05af0);
	if (err)
		return err;

	return nxpfspi_write(bank, LUTCR, locked ? 1 : 2);
}

static int nxpfspi_lut_set(struct flash_bank *bank, const uint16_t *insn)
{
	int err, werr, i;

	err = nxpfspi_lut_set_lock(bank, false);
	if (err)
		return err;

	for (i = 0, werr = 0; *insn && !werr; insn++, i++) {
		/* BootROM seems to be using LUT0, so we use the last
		 * one (15) so as to not step on its toes. */
		werr = nxpfspi_lut_write(bank, 15, i, *insn);
	}

	/* Write final STOP instruction. */
	werr = nxpfspi_lut_write(bank, 15, i, *insn);

	err = nxpfspi_lut_set_lock(bank, true);
	return werr ? : err;
}

static int nxpfspi_lut_exec(struct flash_bank *bank, uint32_t addr,
			    uint16_t datsz, int timeout_ms)
{
	int err = 0;

	err = err ? : nxpfspi_write(bank, IPCR0, addr);
	err = err ? : nxpfspi_write(bank, IPCR1, (15 << 16) | datsz);
	err = err ? : nxpfspi_write(bank, IPCMD, BIT(0));
	err = err ? : nxpfspi_wait(bank, INTR, BIT(0), BIT(0), timeout_ms);
	return err;
}

static int nxpfspi_fifo_read(struct flash_bank *bank, uint8_t *buf, size_t count)
{
	struct nxpfspi *fspi = bank->driver_priv;

	return target_read_buffer(bank->target, fspi->regs + RFDR0, count, buf);
}

static int nxpfspi_fifo_write(struct flash_bank *bank, const uint8_t *buf, size_t count)
{
	struct nxpfspi *fspi = bank->driver_priv;

	return target_write_buffer(bank->target, fspi->regs + TFDR0, count, buf);
}

static int nxpfspi_cmd(struct flash_bank *bank, uint8_t cmd)
{
	uint16_t cmd_prog[] = {
		lutinsn(LUT_CMD,  cmd),
		lutinsn(LUT_STOP, 0),
	};
	int err;

	err = nxpfspi_lut_set(bank, cmd_prog);
	err = err ? : nxpfspi_lut_exec(bank, 0, 0, 10);
	return err;
}

static int nxpfspi_erase_wait(struct flash_bank *bank, int timeout_ms)
{
	static const uint16_t read_status_prog[] = {
		lutinsn(LUT_CMD,  SPIFLASH_READ_STATUS),
		lutinsn(LUT_READ, 0),
		lutinsn(LUT_STOP, 0),
	};
	uint8_t status;
	int err;

	err = nxpfspi_lut_set(bank, read_status_prog);

	for (; timeout_ms >= 0; timeout_ms--) {
		err = err ? : nxpfspi_lut_exec(bank, 0, 1, 10);
		err = err ? : nxpfspi_fifo_read(bank, &status, 1);
		if (err)
			break;

		if (!(status & BIT(0)))
			return ERROR_OK;

		alive_sleep(1);
	}

	LOG_ERROR("Timeout waiting for erase completion");
	return err;
}


static int nxpfspi_flash_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	static const uint16_t erase_prog[] = {
		lutinsn(LUT_CMD,   SPIFLASH_SECTOR_ERASE),
		lutinsn(LUT_RADDR, 24),
		lutinsn(LUT_STOP,   0),
	};

	struct flash_sector *sect;
	int err;

	if (!nxpfspi_is_probed(bank))
		return ERROR_FLASH_BANK_NOT_PROBED;

	for (sect = &bank->sectors[first]; first <= last; first++, sect++) {
		err = nxpfspi_cmd(bank, SPIFLASH_WRITE_ENABLE);
		err = err ? : nxpfspi_lut_set(bank, erase_prog);
		err = err ? : nxpfspi_lut_exec(bank, sect->offset, 0, 10);
		err = err ? : nxpfspi_erase_wait(bank, 1000);
		if (err)
			break;
	}

	return ERROR_OK;
}

static int nxpfspi_flash_erase_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

static int nxpfspi_flash_write(struct flash_bank *bank, const uint8_t *buffer,
			       uint32_t offset, uint32_t left)
{

	static const uint16_t write_prog[] = {
		lutinsn(LUT_CMD,   SPIFLASH_PAGE_PROGRAM),
		lutinsn(LUT_RADDR, 24),
		lutinsn(LUT_WRITE,  0),
		lutinsn(LUT_STOP,   0),
	};

	uint32_t count;
	int err = 0;

	if (!nxpfspi_is_probed(bank))
		return ERROR_FLASH_BANK_NOT_PROBED;

	while (left && !err) {
		count = left > 128 ? 128 : left;

		err = nxpfspi_cmd(bank, SPIFLASH_WRITE_ENABLE);
		err = err ? : nxpfspi_lut_set(bank, write_prog);
		err = err ? : nxpfspi_write(
			bank, IPTXFCR, (((count >> 3) - 1) << 2) | BIT(0));
		err = err ? : nxpfspi_wait(bank, INTR, BIT(6), BIT(6), 10);
		err = err ? : nxpfspi_fifo_write(bank, buffer, count);
		err = err ? : nxpfspi_write(bank, INTR, BIT(6));
		err = err ? : nxpfspi_lut_exec(bank, offset, count, 10);

		offset += count;
		buffer += count;
		left -= count;
	}

	return err;
}

int nxpfspi_flash_read(struct flash_bank *bank, uint8_t *buffer,
				uint32_t offset, uint32_t left)
{
	static const uint16_t read_prog[] = {
		lutinsn(LUT_CMD,   SPIFLASH_READ),
		lutinsn(LUT_RADDR, 24),
		lutinsn(LUT_READ,   0),
		lutinsn(LUT_STOP,   0),
	};

	uint32_t count;
	int err;

	if (!nxpfspi_is_probed(bank))
		return ERROR_FLASH_BANK_NOT_PROBED;

	err = nxpfspi_lut_set(bank, read_prog);
	if (err)
		return err;

	while (left && !err) {
		count = left > 128 ? 128 : left;

		err = nxpfspi_lut_exec(bank, offset, count, 10);

		err = err ? : nxpfspi_write(bank, IPRXFCR,
					    (((count >> 3) - 1) << 2) | BIT(0));
		err = err ? : nxpfspi_wait(bank, INTR, BIT(5), BIT(5), 10);
		err = err ? : nxpfspi_fifo_read(bank, buffer, count);
		err = err ? : nxpfspi_write(bank, INTR, BIT(5));

		offset += count;
		buffer += count;
		left -= count;
	}

	return err;
}

static int nxpfspi_flash_probe(struct flash_bank *bank)
{
	static const uint16_t readid_prog[] = {
		lutinsn(LUT_CMD, SPIFLASH_READ_ID),
		lutinsn(LUT_READ, 0),
		lutinsn(LUT_STOP, 0),
	};

	struct nxpfspi *fspi = bank->driver_priv;
	const struct flash_device *model;
	struct flash_sector *sect;
	uint32_t id, sectsz;
	uint8_t idbuf[3];
	unsigned sectid;
	int err;

	fspi->probed = false;

	err = nxpfspi_cmd(bank, SPIFLASH_RESET_ENABLE);
	err = err ? : nxpfspi_cmd(bank, SPIFLASH_RESET_MEMORY);
	if (err)
		return err;

	err = nxpfspi_lut_set(bank, readid_prog);
	err = err ? : nxpfspi_lut_exec(bank, 0, 3, 10);
	err = err ? : nxpfspi_fifo_read(bank, idbuf, 3);
	if (err)
		return err;

	id = idbuf[0] | (idbuf[1] << 8) | (idbuf[2] << 16);

	for (model = flash_devices; model->name ; model++) {
		if (model->device_id == id) {
			fspi->model = model;
			break;
		}
	}

	if (!fspi->model) {
		LOG_ERROR("Unknown FLASH, ID 0x%08" PRIx32, id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found FLASH, \'%s\' ID 0x%08" PRIx32,
		 fspi->model->name, fspi->model->device_id);

	bank->size = fspi->model->size_in_bytes;
	sectsz = fspi->model->sectorsize ? : fspi->model->size_in_bytes;
	bank->num_sectors = bank->size / sectsz;

	sect = calloc(bank->num_sectors, sizeof(*sect));
	if (!sect) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	bank->sectors = sect;
	for (sectid = 0; sectid < bank->num_sectors; sectid++, sect++) {
		sect->offset = sectid * sectsz;
		sect->size = sectsz;
		sect->is_erased = -1;
	}

	fspi->probed = true;
	return ERROR_OK;
}

static int nxpfspi_flash_auto_probe(struct flash_bank *bank)
{
	struct nxpfspi *fspi = bank->driver_priv;

	if (fspi->probed)
		return ERROR_OK;

	return nxpfspi_flash_probe(bank);
}

static int nxpfspi_flash_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct nxpfspi *fspi = bank->driver_priv;

	if (!nxpfspi_is_probed(bank))
		return ERROR_FLASH_BANK_NOT_PROBED;

	snprintf(buf, buf_size, "\nNXP FlexSPI Controller:\n"
		"  Device \'%s\' ID 0x%08" PRIx32 "\n",
		fspi->model->name, fspi->model->device_id);

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(nxpfspi_flash_bank_command)
{
	struct nxpfspi *fspi;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fspi = calloc(1, sizeof(*fspi));
	if (!fspi) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	/* We only need to know the base address for the registers. */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], fspi->regs);
	bank->driver_priv = fspi;
	return ERROR_OK;
}

const struct flash_driver nxpfspi_flash = {
	.name               = "nxpfspi",
	.erase              = nxpfspi_flash_erase,
	.erase_check        = nxpfspi_flash_erase_check,
	.write              = nxpfspi_flash_write,
	.read               = nxpfspi_flash_read,
	.probe              = nxpfspi_flash_probe,
	.auto_probe         = nxpfspi_flash_auto_probe,
	.info               = nxpfspi_flash_info,
	.flash_bank_command = nxpfspi_flash_bank_command,
	.free_driver_priv   = default_flash_free_driver_priv,
};
