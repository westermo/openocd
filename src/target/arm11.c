/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
 *                                                                         *
 *   Copyright (C) 2008,2009 Oyvind Harboe oyvind.harboe@zylin.com         *
 *                                                                         *
 *   Copyright (C) 2008 Georg Acher <acher@in.tum.de>                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "etm.h"
#include "breakpoints.h"
#include "arm11_dbgtap.h"
#include "arm_simulator.h"
#include "time_support.h"
#include "target_type.h"
#include "algorithm.h"
#include "register.h"


#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static bool arm11_config_memwrite_burst = true;
static bool arm11_config_memwrite_error_fatal = true;
static uint32_t arm11_vcr = 0;
static bool arm11_config_step_irq_enable = false;
static bool arm11_config_hardware_step = false;

static int arm11_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints);


/** Check and if necessary take control of the system
 *
 * \param arm11		Target state variable.
 */
static int arm11_check_init(struct arm11_common *arm11)
{
	CHECK_RETVAL(arm11_read_DSCR(arm11));
	LOG_DEBUG("DSCR %08x", (unsigned) arm11->dscr);

	if (!(arm11->dscr & ARM11_DSCR_MODE_SELECT))
	{
		LOG_DEBUG("Bringing target into debug mode");

		arm11->dscr |= ARM11_DSCR_MODE_SELECT;		/* Halt debug-mode */
		arm11_write_DSCR(arm11, arm11->dscr);

		/* add further reset initialization here */

		arm11->simulate_reset_on_next_halt = true;

		if (arm11->dscr & ARM11_DSCR_CORE_HALTED)
		{
			/** \todo TODO: this needs further scrutiny because
			  * arm11_debug_entry() never gets called.  (WHY NOT?)
			  * As a result we don't read the actual register states from
			  * the target.
			  */

			arm11->arm.target->state = TARGET_HALTED;
			arm11->arm.target->debug_reason =
					arm11_get_DSCR_debug_reason(arm11->dscr);
		}
		else
		{
			arm11->arm.target->state = TARGET_RUNNING;
			arm11->arm.target->debug_reason = DBG_REASON_NOTHALTED;
		}

		arm11_sc7_clear_vbw(arm11);
	}

	return ERROR_OK;
}

/**
 * Save processor state.  This is called after a HALT instruction
 * succeeds, and on other occasions the processor enters debug mode
 * (breakpoint, watchpoint, etc).  Caller has updated arm11->dscr.
 */
static int arm11_debug_entry(struct arm11_common *arm11)
{
	int retval;

	arm11->arm.target->state = TARGET_HALTED;
	arm11->arm.target->debug_reason =
			arm11_get_DSCR_debug_reason(arm11->dscr);

	/* REVISIT entire cache should already be invalid !!! */
	register_cache_invalidate(arm11->arm.core_cache);

	/* See e.g. ARM1136 TRM, "14.8.4 Entering Debug state" */

	/* maybe save wDTR (pending DCC write to debug SW, e.g. libdcc) */
	arm11->is_wdtr_saved = !!(arm11->dscr & ARM11_DSCR_WDTR_FULL);
	if (arm11->is_wdtr_saved)
	{
		arm11_add_debug_SCAN_N(arm11, 0x05, ARM11_TAP_DEFAULT);

		arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

		struct scan_field	chain5_fields[3];

		arm11_setup_field(arm11, 32, NULL,
				&arm11->saved_wdtr, chain5_fields + 0);
		arm11_setup_field(arm11,  1, NULL, NULL,		chain5_fields + 1);
		arm11_setup_field(arm11,  1, NULL, NULL,		chain5_fields + 2);

		arm11_add_dr_scan_vc(ARRAY_SIZE(chain5_fields), chain5_fields, TAP_DRPAUSE);

	}

	/* DSCR: set ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE
	 *
	 * ARM1176 spec says this is needed only for wDTR/rDTR's "ITR mode",
	 * but not to issue ITRs. ARM1136 seems to require this to issue
	 * ITR's as well...
	 */

	arm11_write_DSCR(arm11, ARM11_DSCR_EXECUTE_ARM_INSTRUCTION_ENABLE
				| arm11->dscr);


	/* From the spec:
	   Before executing any instruction in debug state you have to drain the write buffer.
	   This ensures that no imprecise Data Aborts can return at a later point:*/

	/** \todo TODO: Test drain write buffer. */

#if 0
	while (1)
	{
		/* MRC p14,0,R0,c5,c10,0 */
		//	arm11_run_instr_no_data1(arm11, /*0xee150e1a*/0xe320f000);

		/* mcr	   15, 0, r0, cr7, cr10, {4} */
		arm11_run_instr_no_data1(arm11, 0xee070f9a);

		uint32_t dscr = arm11_read_DSCR(arm11);

		LOG_DEBUG("DRAIN, DSCR %08x", dscr);

		if (dscr & ARM11_DSCR_STICKY_IMPRECISE_DATA_ABORT)
		{
			arm11_run_instr_no_data1(arm11, 0xe320f000);

			dscr = arm11_read_DSCR(arm11);

			LOG_DEBUG("DRAIN, DSCR %08x (DONE)", dscr);

			break;
		}
	}
#endif

	/* Save registers.
	 *
	 * NOTE:  ARM1136 TRM suggests saving just R0 here now, then
	 * CPSR and PC after the rDTR stuff.  We do it all at once.
	 */
	retval = arm_dpm_read_current_registers(&arm11->dpm);
	if (retval != ERROR_OK)
		LOG_ERROR("DPM REG READ -- fail %d", retval);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* maybe save rDTR (pending DCC read from debug SW, e.g. libdcc) */
	arm11->is_rdtr_saved = !!(arm11->dscr & ARM11_DSCR_RDTR_FULL);
	if (arm11->is_rdtr_saved)
	{
		/* MRC p14,0,R0,c0,c5,0 (move rDTR -> r0 (-> wDTR -> local var)) */
		retval = arm11_run_instr_data_from_core_via_r0(arm11,
				0xEE100E15, &arm11->saved_rdtr);
		if (retval != ERROR_OK)
			return retval;
	}

	/* REVISIT Now that we've saved core state, there's may also
	 * be MMU and cache state to care about ...
	 */

	if (arm11->simulate_reset_on_next_halt)
	{
		arm11->simulate_reset_on_next_halt = false;

		LOG_DEBUG("Reset c1 Control Register");

		/* Write 0 (reset value) to Control register 0 to disable MMU/Cache etc. */

		/* MCR p15,0,R0,c1,c0,0 */
		retval = arm11_run_instr_data_to_core_via_r0(arm11, 0xee010f10, 0);
		if (retval != ERROR_OK)
			return retval;

	}

	retval = arm11_run_instr_data_finish(arm11);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/**
 * Restore processor state.  This is called in preparation for
 * the RESTART function.
 */
static int arm11_leave_debug_state(struct arm11_common *arm11, bool bpwp)
{
	int retval;

	/* See e.g. ARM1136 TRM, "14.8.5 Leaving Debug state" */

	/* NOTE:  the ARM1136 TRM suggests restoring all registers
	 * except R0/PC/CPSR right now.  Instead, we do them all
	 * at once, just a bit later on.
	 */

	/* REVISIT once we start caring about MMU and cache state,
	 * address it here ...
	 */

	/* spec says clear wDTR and rDTR; we assume they are clear as
	   otherwise our programming would be sloppy */
	{
		CHECK_RETVAL(arm11_read_DSCR(arm11));

		if (arm11->dscr & (ARM11_DSCR_RDTR_FULL | ARM11_DSCR_WDTR_FULL))
		{
			/*
			The wDTR/rDTR two registers that are used to send/receive data to/from
			the core in tandem with corresponding instruction codes that are
			written into the core. The RDTR FULL/WDTR FULL flag indicates that the
			registers hold data that was written by one side (CPU or JTAG) and not
			read out by the other side.
			*/
			LOG_ERROR("wDTR/rDTR inconsistent (DSCR %08x)",
					(unsigned) arm11->dscr);
			return ERROR_FAIL;
		}
	}

	/* maybe restore original wDTR */
	if (arm11->is_wdtr_saved)
	{
		retval = arm11_run_instr_data_prepare(arm11);
		if (retval != ERROR_OK)
			return retval;

		/* MCR p14,0,R0,c0,c5,0 */
		retval = arm11_run_instr_data_to_core_via_r0(arm11,
				0xee000e15, arm11->saved_wdtr);
		if (retval != ERROR_OK)
			return retval;

		retval = arm11_run_instr_data_finish(arm11);
		if (retval != ERROR_OK)
			return retval;
	}

	/* restore CPSR, PC, and R0 ... after flushing any modified
	 * registers.
	 */
	retval = arm_dpm_write_dirty_registers(&arm11->dpm, bpwp);

	register_cache_invalidate(arm11->arm.core_cache);

	/* restore DSCR */
	arm11_write_DSCR(arm11, arm11->dscr);

	/* maybe restore rDTR */
	if (arm11->is_rdtr_saved)
	{
		arm11_add_debug_SCAN_N(arm11, 0x05, ARM11_TAP_DEFAULT);

		arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

		struct scan_field	chain5_fields[3];

		uint8_t			Ready		= 0;	/* ignored */
		uint8_t			Valid		= 0;	/* ignored */

		arm11_setup_field(arm11, 32, &arm11->saved_rdtr,
				NULL, chain5_fields + 0);
		arm11_setup_field(arm11,  1, &Ready,	NULL, chain5_fields + 1);
		arm11_setup_field(arm11,  1, &Valid,	NULL, chain5_fields + 2);

		arm11_add_dr_scan_vc(ARRAY_SIZE(chain5_fields), chain5_fields, TAP_DRPAUSE);
	}

	/* now processor is ready to RESTART */

	return ERROR_OK;
}

/* poll current target status */
static int arm11_poll(struct target *target)
{
	int retval;
	struct arm11_common *arm11 = target_to_arm11(target);

	CHECK_RETVAL(arm11_check_init(arm11));

	if (arm11->dscr & ARM11_DSCR_CORE_HALTED)
	{
		if (target->state != TARGET_HALTED)
		{
			enum target_state old_state = target->state;

			LOG_DEBUG("enter TARGET_HALTED");
			retval = arm11_debug_entry(arm11);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target,
				old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED);
		}
	}
	else
	{
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING)
		{
			LOG_DEBUG("enter TARGET_RUNNING");
			target->state			= TARGET_RUNNING;
			target->debug_reason	= DBG_REASON_NOTHALTED;
		}
	}

	return ERROR_OK;
}
/* architecture specific status reply */
static int arm11_arch_state(struct target *target)
{
	int retval;

	retval = armv4_5_arch_state(target);

	/* REVISIT also display ARM11-specific MMU and cache status ... */

	return retval;
}

/* target request support */
static int arm11_target_request_data(struct target *target,
		uint32_t size, uint8_t *buffer)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/* target execution control */
static int arm11_halt(struct target *target)
{
	struct arm11_common *arm11 = target_to_arm11(target);

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state == TARGET_UNKNOWN)
	{
		arm11->simulate_reset_on_next_halt = true;
	}

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	arm11_add_IR(arm11, ARM11_HALT, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	int i = 0;

	while (1)
	{
		CHECK_RETVAL(arm11_read_DSCR(arm11));

		if (arm11->dscr & ARM11_DSCR_CORE_HALTED)
			break;


		long long then = 0;
		if (i == 1000)
		{
			then = timeval_ms();
		}
		if (i >= 1000)
		{
			if ((timeval_ms()-then) > 1000)
			{
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}
		i++;
	}

	enum target_state old_state	= target->state;

	arm11_debug_entry(arm11);

	CHECK_RETVAL(
		target_call_event_callbacks(target,
			old_state == TARGET_DEBUG_RUNNING ? TARGET_EVENT_DEBUG_HALTED : TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static uint32_t
arm11_nextpc(struct arm11_common *arm11, int current, uint32_t address)
{
	void *value = arm11->arm.core_cache->reg_list[15].value;

	if (!current)
		buf_set_u32(value, 0, 32, address);
	else
		address = buf_get_u32(value, 0, 32);

	return address;
}

static int arm11_resume(struct target *target, int current,
		uint32_t address, int handle_breakpoints, int debug_execution)
{
	//	  LOG_DEBUG("current %d  address %08x  handle_breakpoints %d  debug_execution %d",
	//	current, address, handle_breakpoints, debug_execution);

	struct arm11_common *arm11 = target_to_arm11(target);

	LOG_DEBUG("target->state: %s",
		target_state_name(target));


	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	address = arm11_nextpc(arm11, current, address);

	LOG_DEBUG("RESUME PC %08" PRIx32 "%s", address, !current ? "!" : "");

	/* clear breakpoints/watchpoints and VCR*/
	arm11_sc7_clear_vbw(arm11);

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* Set up breakpoints */
	if (handle_breakpoints)
	{
		/* check if one matches PC and step over it if necessary */

		struct breakpoint *	bp;

		for (bp = target->breakpoints; bp; bp = bp->next)
		{
			if (bp->address == address)
			{
				LOG_DEBUG("must step over %08" PRIx32 "", bp->address);
				arm11_step(target, 1, 0, 0);
				break;
			}
		}

		/* set all breakpoints */

		unsigned brp_num = 0;

		for (bp = target->breakpoints; bp; bp = bp->next)
		{
			struct arm11_sc7_action	brp[2];

			brp[0].write	= 1;
			brp[0].address	= ARM11_SC7_BVR0 + brp_num;
			brp[0].value	= bp->address;
			brp[1].write	= 1;
			brp[1].address	= ARM11_SC7_BCR0 + brp_num;
			brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5) | (0 << 14) | (0 << 16) | (0 << 20) | (0 << 21);

			arm11_sc7_run(arm11, brp, ARRAY_SIZE(brp));

			LOG_DEBUG("Add BP %d at %08" PRIx32, brp_num,
					bp->address);

			brp_num++;
		}

		arm11_sc7_set_vcr(arm11, arm11_vcr);
	}

	arm11_leave_debug_state(arm11, handle_breakpoints);

	arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	int i = 0;
	while (1)
	{
		CHECK_RETVAL(arm11_read_DSCR(arm11));

		LOG_DEBUG("DSCR %08x", (unsigned) arm11->dscr);

		if (arm11->dscr & ARM11_DSCR_CORE_RESTARTED)
			break;


		long long then = 0;
		if (i == 1000)
		{
			then = timeval_ms();
		}
		if (i >= 1000)
		{
			if ((timeval_ms()-then) > 1000)
			{
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}
		i++;
	}

	if (!debug_execution)
	{
		target->state			= TARGET_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;

		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));
	}
	else
	{
		target->state			= TARGET_DEBUG_RUNNING;
		target->debug_reason	= DBG_REASON_NOTHALTED;

		CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_RESUMED));
	}

	return ERROR_OK;
}

static int arm11_step(struct target *target, int current,
		uint32_t address, int handle_breakpoints)
{
	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct arm11_common *arm11 = target_to_arm11(target);

	address = arm11_nextpc(arm11, current, address);

	LOG_DEBUG("STEP PC %08" PRIx32 "%s", address, !current ? "!" : "");


	/** \todo TODO: Thumb not supported here */

	uint32_t	next_instruction;

	CHECK_RETVAL(arm11_read_memory_word(arm11, address, &next_instruction));

	/* skip over BKPT */
	if ((next_instruction & 0xFFF00070) == 0xe1200070)
	{
		address = arm11_nextpc(arm11, 0, address + 4);
		LOG_DEBUG("Skipping BKPT");
	}
	/* skip over Wait for interrupt / Standby */
	/* mcr	15, 0, r?, cr7, cr0, {4} */
	else if ((next_instruction & 0xFFFF0FFF) == 0xee070f90)
	{
		address = arm11_nextpc(arm11, 0, address + 4);
		LOG_DEBUG("Skipping WFI");
	}
	/* ignore B to self */
	else if ((next_instruction & 0xFEFFFFFF) == 0xeafffffe)
	{
		LOG_DEBUG("Not stepping jump to self");
	}
	else
	{
		/** \todo TODO: check if break-/watchpoints make any sense at all in combination
		* with this. */

		/** \todo TODO: check if disabling IRQs might be a good idea here. Alternatively
		* the VCR might be something worth looking into. */


		/* Set up breakpoint for stepping */

		struct arm11_sc7_action	brp[2];

		brp[0].write	= 1;
		brp[0].address	= ARM11_SC7_BVR0;
		brp[1].write	= 1;
		brp[1].address	= ARM11_SC7_BCR0;

		if (arm11_config_hardware_step)
		{
			/* Hardware single stepping ("instruction address
			 * mismatch") is used if enabled.  It's not quite
			 * exactly "run one instruction"; "branch to here"
			 * loops won't break, neither will some other cases,
			 * but it's probably the best default.
			 *
			 * Hardware single stepping isn't supported on v6
			 * debug modules.  ARM1176 and v7 can support it...
			 *
			 * FIXME Thumb stepping likely needs to use 0x03
			 * or 0xc0 byte masks, not 0x0f.
			 */
			 brp[0].value	= address;
			 brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5)
					| (0 << 14) | (0 << 16) | (0 << 20)
					| (2 << 21);
		} else
		{
			/* Sets a breakpoint on the next PC, as calculated
			 * by instruction set simulation.
			 *
			 * REVISIT stepping Thumb on ARM1156 requires Thumb2
			 * support from the simulator.
			 */
			uint32_t next_pc;
			int retval;

			retval = arm_simulate_step(target, &next_pc);
			if (retval != ERROR_OK)
				return retval;

			brp[0].value	= next_pc;
			brp[1].value	= 0x1 | (3 << 1) | (0x0F << 5)
					| (0 << 14) | (0 << 16) | (0 << 20)
					| (0 << 21);
		}

		CHECK_RETVAL(arm11_sc7_run(arm11, brp, ARRAY_SIZE(brp)));

		/* resume */


		if (arm11_config_step_irq_enable)
			/* this disable should be redundant ... */
			arm11->dscr &= ~ARM11_DSCR_INTERRUPTS_DISABLE;
		else
			arm11->dscr |= ARM11_DSCR_INTERRUPTS_DISABLE;


		CHECK_RETVAL(arm11_leave_debug_state(arm11, handle_breakpoints));

		arm11_add_IR(arm11, ARM11_RESTART, TAP_IDLE);

		CHECK_RETVAL(jtag_execute_queue());

		/* wait for halt */
		int i = 0;

		while (1)
		{
			const uint32_t mask = ARM11_DSCR_CORE_RESTARTED
					| ARM11_DSCR_CORE_HALTED;

			CHECK_RETVAL(arm11_read_DSCR(arm11));
			LOG_DEBUG("DSCR %08x e", (unsigned) arm11->dscr);

			if ((arm11->dscr & mask) == mask)
				break;

			long long then = 0;
			if (i == 1000)
			{
				then = timeval_ms();
			}
			if (i >= 1000)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}
			i++;
		}

		/* clear breakpoint */
		arm11_sc7_clear_vbw(arm11);

		/* save state */
		CHECK_RETVAL(arm11_debug_entry(arm11));

		/* restore default state */
		arm11->dscr &= ~ARM11_DSCR_INTERRUPTS_DISABLE;

	}

	target->debug_reason	= DBG_REASON_SINGLESTEP;

	CHECK_RETVAL(target_call_event_callbacks(target, TARGET_EVENT_HALTED));

	return ERROR_OK;
}

static int arm11_assert_reset(struct target *target)
{
	int retval;
	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_check_init(arm11);
	if (retval != ERROR_OK)
		return retval;

	target->state = TARGET_UNKNOWN;

	/* we would very much like to reset into the halted, state,
	 * but resetting and halting is second best... */
	if (target->reset_halt)
	{
		CHECK_RETVAL(target_halt(target));
	}


	/* srst is funny. We can not do *anything* else while it's asserted
	 * and it has unkonwn side effects. Make sure no other code runs
	 * meanwhile.
	 *
	 * Code below assumes srst:
	 *
	 * - Causes power-on-reset (but of what parts of the system?). Bug
	 * in arm11?
	 *
	 * - Messes us TAP state without asserting trst.
	 *
	 * - There is another bug in the arm11 core. When you generate an access to
	 * external logic (for example ddr controller via AHB bus) and that block
	 * is not configured (perhaps it is still held in reset), that transaction
	 * will never complete. This will hang arm11 core but it will also hang
	 * JTAG controller. Nothing, short of srst assertion will bring it out of
	 * this.
	 *
	 * Mysteries:
	 *
	 * - What should the PC be after an srst reset when starting in the halted
	 * state?
	 */

	jtag_add_reset(0, 1);
	jtag_add_reset(0, 0);

	/* How long do we have to wait? */
	jtag_add_sleep(5000);

	/* un-mess up TAP state */
	jtag_add_tlr();

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

static int arm11_deassert_reset(struct target *target)
{
	return ERROR_OK;
}

static int arm11_soft_reset_halt(struct target *target)
{
	LOG_WARNING("Not implemented: %s", __func__);

	return ERROR_FAIL;
}

/* target memory access
 * size: 1 = byte (8bit), 2 = half-word (16bit), 4 = word (32bit)
 * count: number of items of <size>
 *
 * arm11_config_memrw_no_increment - in the future we may want to be able
 * to read/write a range of data to a "port". a "port" is an action on
 * read memory address for some peripheral.
 */
static int arm11_read_memory_inner(struct target *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer,
		bool arm11_config_memrw_no_increment)
{
	/** \todo TODO: check if buffer cast to uint32_t* and uint16_t* might cause alignment problems */
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08" PRIx32 "  SIZE %08" PRIx32 "  COUNT %08" PRIx32 "", address, size, count);

	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);
	if (retval != ERROR_OK)
		return retval;

	switch (size)
	{
	case 1:
		arm11->arm.core_cache->reg_list[1].dirty = true;

		for (size_t i = 0; i < count; i++)
		{
			/* ldrb    r1, [r0], #1 */
			/* ldrb    r1, [r0] */
			arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe4d01001 : 0xe5d01000);

			uint32_t res;
			/* MCR p14,0,R1,c0,c5,0 */
			arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

			*buffer++ = res;
		}

		break;

	case 2:
		{
			arm11->arm.core_cache->reg_list[1].dirty = true;

			for (size_t i = 0; i < count; i++)
			{
				/* ldrh    r1, [r0], #2 */
				arm11_run_instr_no_data1(arm11,
					!arm11_config_memrw_no_increment ? 0xe0d010b2 : 0xe1d010b0);

				uint32_t res;

				/* MCR p14,0,R1,c0,c5,0 */
				arm11_run_instr_data_from_core(arm11, 0xEE001E15, &res, 1);

				uint16_t svalue = res;
				memcpy(buffer + i * sizeof(uint16_t), &svalue, sizeof(uint16_t));
			}

			break;
		}

	case 4:
		{
		uint32_t instr = !arm11_config_memrw_no_increment ? 0xecb05e01 : 0xed905e00;
		/** \todo TODO: buffer cast to uint32_t* causes alignment warnings */
		uint32_t *words = (uint32_t *)buffer;

		/* LDC p14,c5,[R0],#4 */
		/* LDC p14,c5,[R0] */
		arm11_run_instr_data_from_core(arm11, instr, words, count);
		break;
		}
	}

	return arm11_run_instr_data_finish(arm11);
}

static int arm11_read_memory(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	return arm11_read_memory_inner(target, address, size, count, buffer, false);
}

/*
* no_increment - in the future we may want to be able
* to read/write a range of data to a "port". a "port" is an action on
* read memory address for some peripheral.
*/
static int arm11_write_memory_inner(struct target *target,
		uint32_t address, uint32_t size,
		uint32_t count, uint8_t *buffer,
		bool no_increment)
{
	int retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("ADDR %08" PRIx32 "  SIZE %08" PRIx32 "  COUNT %08" PRIx32 "", address, size, count);

	struct arm11_common *arm11 = target_to_arm11(target);

	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xee100e15, address);
	if (retval != ERROR_OK)
		return retval;

	/* burst writes are not used for single words as those may well be
	 * reset init script writes.
	 *
	 * The other advantage is that as burst writes are default, we'll
	 * now exercise both burst and non-burst code paths with the
	 * default settings, increasing code coverage.
	 */
	bool burst = arm11_config_memwrite_burst && (count > 1);

	switch (size)
	{
	case 1:
		{
			arm11->arm.core_cache->reg_list[1].dirty = true;

			for (size_t i = 0; i < count; i++)
			{
				/* MRC p14,0,r1,c0,c5,0 */
				retval = arm11_run_instr_data_to_core1(arm11, 0xee101e15, *buffer++);
				if (retval != ERROR_OK)
					return retval;

				/* strb    r1, [r0], #1 */
				/* strb    r1, [r0] */
				retval = arm11_run_instr_no_data1(arm11,
					!no_increment
						? 0xe4c01001
						: 0xe5c01000);
				if (retval != ERROR_OK)
					return retval;
			}

			break;
		}

	case 2:
		{
			arm11->arm.core_cache->reg_list[1].dirty = true;

			for (size_t i = 0; i < count; i++)
			{
				uint16_t value;
				memcpy(&value, buffer + i * sizeof(uint16_t), sizeof(uint16_t));

				/* MRC p14,0,r1,c0,c5,0 */
				retval = arm11_run_instr_data_to_core1(arm11, 0xee101e15, value);
				if (retval != ERROR_OK)
					return retval;

				/* strh    r1, [r0], #2 */
				/* strh    r1, [r0] */
				retval = arm11_run_instr_no_data1(arm11,
					!no_increment
						? 0xe0c010b2
						: 0xe1c010b0);
				if (retval != ERROR_OK)
					return retval;
			}

			break;
		}

	case 4: {
		uint32_t instr = !no_increment ? 0xeca05e01 : 0xed805e00;

		/** \todo TODO: buffer cast to uint32_t* causes alignment warnings */
		uint32_t *words = (uint32_t*)buffer;

		if (!burst)
		{
			/* STC p14,c5,[R0],#4 */
			/* STC p14,c5,[R0]*/
			retval = arm11_run_instr_data_to_core(arm11, instr, words, count);
			if (retval != ERROR_OK)
				return retval;
		}
		else
		{
			/* STC p14,c5,[R0],#4 */
			/* STC p14,c5,[R0]*/
			retval = arm11_run_instr_data_to_core_noack(arm11, instr, words, count);
			if (retval != ERROR_OK)
				return retval;
		}

		break;
	}
	}

	/* r0 verification */
	if (!no_increment)
	{
		uint32_t r0;

		/* MCR p14,0,R0,c0,c5,0 */
		retval = arm11_run_instr_data_from_core(arm11, 0xEE000E15, &r0, 1);
		if (retval != ERROR_OK)
			return retval;

		if (address + size * count != r0)
		{
			LOG_ERROR("Data transfer failed. Expected end "
					"address 0x%08x, got 0x%08x",
					(unsigned) (address + size * count),
					(unsigned) r0);

			if (burst)
				LOG_ERROR("use 'arm11 memwrite burst disable' to disable fast burst mode");

			if (arm11_config_memwrite_error_fatal)
				return ERROR_FAIL;
		}
	}

	return arm11_run_instr_data_finish(arm11);
}

static int arm11_write_memory(struct target *target,
		uint32_t address, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	/* pointer increment matters only for multi-unit writes ...
	 * not e.g. to a "reset the chip" controller.
	 */
	return arm11_write_memory_inner(target, address, size,
			count, buffer, count == 1);
}

/* write target memory in multiples of 4 byte, optimized for writing large quantities of data */
static int arm11_bulk_write_memory(struct target *target,
		uint32_t address, uint32_t count, uint8_t *buffer)
{
	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target was not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return arm11_write_memory(target, address, 4, count, buffer);
}

/* target break-/watchpoint control
* rw: 0 = write, 1 = read, 2 = access
*/
static int arm11_add_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct arm11_common *arm11 = target_to_arm11(target);

#if 0
	if (breakpoint->type == BKPT_SOFT)
	{
		LOG_INFO("sw breakpoint requested, but software breakpoints not enabled");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
#endif

	if (!arm11->free_brps)
	{
		LOG_DEBUG("no breakpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->length != 4)
	{
		LOG_DEBUG("only breakpoints of four bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	arm11->free_brps--;

	return ERROR_OK;
}

static int arm11_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	struct arm11_common *arm11 = target_to_arm11(target);

	arm11->free_brps++;

	return ERROR_OK;
}

static int arm11_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm11_common *arm11;

	if (target->tap == NULL)
		return ERROR_FAIL;

	if (target->tap->ir_length != 5)
	{
		LOG_ERROR("'target arm11' expects IR LENGTH = 5");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	arm11 = calloc(1, sizeof *arm11);
	if (!arm11)
		return ERROR_FAIL;

	armv4_5_init_arch_info(target, &arm11->arm);

	arm11->jtag_info.tap = target->tap;
	arm11->jtag_info.scann_size = 5;
	arm11->jtag_info.scann_instr = ARM11_SCAN_N;
	/* cur_scan_chain == 0 */
	arm11->jtag_info.intest_instr = ARM11_INTEST;

	return ERROR_OK;
}

static int arm11_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	/* Initialize anything we can set up without talking to the target */
	return ERROR_OK;
}

/* talk to the target and set things up */
static int arm11_examine(struct target *target)
{
	int retval;
	char *type;
	struct arm11_common *arm11 = target_to_arm11(target);
	uint32_t didr, device_id;
	uint8_t implementor;

	/* FIXME split into do-first-time and do-every-time logic ... */

	/* check IDCODE */

	arm11_add_IR(arm11, ARM11_IDCODE, ARM11_TAP_DEFAULT);

	struct scan_field		idcode_field;

	arm11_setup_field(arm11, 32, NULL, &device_id, &idcode_field);

	arm11_add_dr_scan_vc(1, &idcode_field, TAP_DRPAUSE);

	/* check DIDR */

	arm11_add_debug_SCAN_N(arm11, 0x00, ARM11_TAP_DEFAULT);

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field		chain0_fields[2];

	arm11_setup_field(arm11, 32, NULL, &didr, chain0_fields + 0);
	arm11_setup_field(arm11,  8, NULL, &implementor, chain0_fields + 1);

	arm11_add_dr_scan_vc(ARRAY_SIZE(chain0_fields), chain0_fields, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	switch (device_id & 0x0FFFF000)
	{
	case 0x07B36000:
		type = "ARM1136";
		break;
	case 0x07B56000:
		type = "ARM1156";
		break;
	case 0x07B76000:
		arm11->arm.core_type = ARM_MODE_MON;
		type = "ARM1176";
		break;
	default:
		LOG_ERROR("'target arm11' expects IDCODE 0x*7B*7****");
		return ERROR_FAIL;
	}
	LOG_INFO("found %s", type);

	/* unlikely this could ever fail, but ... */
	switch ((didr >> 16) & 0x0F) {
	case ARM11_DEBUG_V6:
	case ARM11_DEBUG_V61:		/* supports security extensions */
		break;
	default:
		LOG_ERROR("Only ARM v6 and v6.1 debug supported.");
		return ERROR_FAIL;
	}

	arm11->brp = ((didr >> 24) & 0x0F) + 1;
	arm11->wrp = ((didr >> 28) & 0x0F) + 1;

	/** \todo TODO: reserve one brp slot if we allow breakpoints during step */
	arm11->free_brps = arm11->brp;

	LOG_DEBUG("IDCODE %08" PRIx32 " IMPLEMENTOR %02x DIDR %08" PRIx32,
			device_id, implementor, didr);

	/* as a side-effect this reads DSCR and thus
	 * clears the ARM11_DSCR_STICKY_PRECISE_DATA_ABORT / Sticky Precise Data Abort Flag
	 * as suggested by the spec.
	 */

	retval = arm11_check_init(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* Build register cache "late", after target_init(), since we
	 * want to know if this core supports Secure Monitor mode.
	 */
	if (!target_was_examined(target))
		retval = arm11_dpm_init(arm11, didr);

	/* ETM on ARM11 still uses original scanchain 6 access mode */
	if (arm11->arm.etm && !target_was_examined(target)) {
		*register_get_last_cache_p(&target->reg_cache) =
			etm_build_reg_cache(target, &arm11->jtag_info,
					arm11->arm.etm);
		retval = etm_setup(target);
	}

	target_set_examined(target);

	return ERROR_OK;
}


/* FIXME all these BOOL_WRAPPER things should be modifying
 * per-instance state, not shared state; ditto the vector
 * catch register support.  Scan chains with multiple cores
 * should be able to say "work with this core like this,
 * that core like that".  Example, ARM11 MPCore ...
 */

#define ARM11_BOOL_WRAPPER(name, print_name)	\
		COMMAND_HANDLER(arm11_handle_bool_##name) \
		{ \
			return CALL_COMMAND_HANDLER(handle_command_parse_bool, \
					&arm11_config_##name, print_name); \
		}

ARM11_BOOL_WRAPPER(memwrite_burst, "memory write burst mode")
ARM11_BOOL_WRAPPER(memwrite_error_fatal, "fatal error mode for memory writes")
ARM11_BOOL_WRAPPER(step_irq_enable, "IRQs while stepping")
ARM11_BOOL_WRAPPER(hardware_step, "hardware single step")

COMMAND_HANDLER(arm11_handle_vcr)
{
	switch (CMD_ARGC) {
	case 0:
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], arm11_vcr);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	LOG_INFO("VCR 0x%08" PRIx32 "", arm11_vcr);
	return ERROR_OK;
}

static const struct command_registration arm11_mw_command_handlers[] = {
	{
		.name = "burst",
		.handler = &arm11_handle_bool_memwrite_burst,
		.mode = COMMAND_ANY,
		.help = "Enable/Disable non-standard but fast burst mode"
			" (default: enabled)",
	},
	{
		.name = "error_fatal",
		.handler = &arm11_handle_bool_memwrite_error_fatal,
		.mode = COMMAND_ANY,
		.help = "Terminate program if transfer error was found"
			" (default: enabled)",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration arm11_any_command_handlers[] = {
	{
		/* "hardware_step" is only here to check if the default
		 * simulate + breakpoint implementation is broken.
		 * TEMPORARY! NOT DOCUMENTED! */
		.name = "hardware_step",
		.handler = &arm11_handle_bool_hardware_step,
		.mode = COMMAND_ANY,
		.help = "DEBUG ONLY - Hardware single stepping"
			" (default: disabled)",
		.usage = "(enable|disable)",
	},
	{
		.name = "memwrite",
		.mode = COMMAND_ANY,
		.help = "memwrite command group",
		.chain = arm11_mw_command_handlers,
	},
	{
		.name = "step_irq_enable",
		.handler = &arm11_handle_bool_step_irq_enable,
		.mode = COMMAND_ANY,
		.help = "Enable interrupts while stepping"
			" (default: disabled)",
	},
	{
		.name = "vcr",
		.handler = &arm11_handle_vcr,
		.mode = COMMAND_ANY,
		.help = "Control (Interrupt) Vector Catch Register",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration arm11_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = etm_command_handlers,
	},
	{
		.name = "arm11",
		.mode = COMMAND_ANY,
		.help = "ARM11 command group",
		.chain = arm11_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM11xx targets. */
struct target_type arm11_target = {
	.name =			"arm11",

	.poll =			arm11_poll,
	.arch_state =		arm11_arch_state,

	.target_request_data =	arm11_target_request_data,

	.halt =			arm11_halt,
	.resume =		arm11_resume,
	.step =			arm11_step,

	.assert_reset =		arm11_assert_reset,
	.deassert_reset =	arm11_deassert_reset,
	.soft_reset_halt =	arm11_soft_reset_halt,

	.get_gdb_reg_list =	armv4_5_get_gdb_reg_list,

	.read_memory =		arm11_read_memory,
	.write_memory =		arm11_write_memory,

	.bulk_write_memory =	arm11_bulk_write_memory,

	.checksum_memory =	arm_checksum_memory,
	.blank_check_memory =	arm_blank_check_memory,

	.add_breakpoint =	arm11_add_breakpoint,
	.remove_breakpoint =	arm11_remove_breakpoint,

	.run_algorithm =	armv4_5_run_algorithm,

	.commands =		arm11_command_handlers,
	.target_create =	arm11_target_create,
	.init_target =		arm11_init_target,
	.examine =		arm11_examine,
};
