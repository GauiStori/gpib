/***************************************************************************
                                   tms9914.h
                             -------------------
    begin                : Feb 2002
    copyright            : (C) 2002 by Frank Mori Hess
    email                : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _TMS9914_H
#define _TMS9914_H

#include <linux/types.h>
#include <linux/interrupt.h>
#include "gpib_state_machines.h"
#include "gpib_types.h"

enum tms9914_holdoff_mode
{
	TMS9914_HOLDOFF_NONE,
	TMS9914_HOLDOFF_EOI,
	TMS9914_HOLDOFF_ALL,
};
/* struct used to provide variables local to a tms9914 chip */
typedef struct tms9914_private_struct tms9914_private_t;
struct tms9914_private_struct
{
	void *iobase;
	unsigned int offset;	// offset between successive tms9914 io addresses
	unsigned int dma_channel;
	// software copy of bits written to interrupt mask registers
	uint8_t imr0_bits, imr1_bits;
	// bits written to address mode register
	uint8_t admr_bits;
	uint8_t auxa_bits;	// bits written to auxilliary register A
	// used to keep track of board's state, bit definitions given below
	unsigned long state;
	uint8_t eos;	// eos character
	short eos_flags;
	uint8_t spoll_status;
	enum tms9914_holdoff_mode holdoff_mode;
	unsigned int ppoll_line;
	enum talker_function_state talker_state;
	enum listener_function_state listener_state;
	unsigned ppoll_sense : 1;
	unsigned ppoll_enable : 1;
	unsigned ppoll_configure_state : 1;
	unsigned primary_listen_addressed : 1;
	unsigned primary_talk_addressed : 1;
	unsigned holdoff_on_end : 1;
	unsigned holdoff_on_all : 1;
	unsigned holdoff_active : 1;
	// wrappers for outb, inb, readb, or writeb
	uint8_t (*read_byte)(tms9914_private_t *priv, unsigned int register_number);
	void (*write_byte)(tms9914_private_t *priv, uint8_t byte, unsigned int
		register_number);
};

// slightly shorter way to access read_byte and write_byte
static inline uint8_t read_byte(tms9914_private_t *priv, unsigned int register_number)
{
	return priv->read_byte(priv, register_number);
}
static inline void write_byte(tms9914_private_t *priv, uint8_t byte, unsigned int register_number)
{
	priv->write_byte(priv, byte, register_number);
}

// tms9914_private_t.state bit numbers
enum
{
	PIO_IN_PROGRESS_BN,	// pio transfer in progress
	DMA_READ_IN_PROGRESS_BN,	// dma read transfer in progress
	DMA_WRITE_IN_PROGRESS_BN,	// dma write transfer in progress
	READ_READY_BN,	// board has data byte available to read
	WRITE_READY_BN,	// board is ready to send a data byte
	COMMAND_READY_BN,	// board is ready to send a command byte
	RECEIVED_END_BN,	// received END
	BUS_ERROR_BN,	// bus error
	DEV_CLEAR_BN,	// device clear received
};

// interface functions
int tms9914_read(gpib_board_t *board, tms9914_private_t *priv,
	uint8_t *buffer, size_t length, int *end, size_t *bytes_read);
int tms9914_write(gpib_board_t *board, tms9914_private_t *priv,
	uint8_t *buffer, size_t length, int send_eoi, size_t *bytes_written);
int tms9914_command(gpib_board_t *board, tms9914_private_t *priv,
	uint8_t *buffer, size_t length, size_t *bytes_written);
int tms9914_take_control(gpib_board_t *board, tms9914_private_t *priv,
	int syncronous);
/* alternate version of tms9914_take_control which works around buggy tcs
 * implementation. */
int tms9914_take_control_workaround(gpib_board_t *board, tms9914_private_t *priv,
	int syncronous);
int tms9914_go_to_standby(gpib_board_t *board, tms9914_private_t *priv);
void tms9914_request_system_control( gpib_board_t *board, tms9914_private_t *priv,
	int request_control );
void tms9914_interface_clear(gpib_board_t *board, tms9914_private_t *priv, int assert);
void tms9914_remote_enable(gpib_board_t *board, tms9914_private_t *priv, int enable);
int tms9914_enable_eos(gpib_board_t *board, tms9914_private_t *priv,
	uint8_t eos_bytes, int compare_8_bits);
void tms9914_disable_eos(gpib_board_t *board, tms9914_private_t *priv);
unsigned int tms9914_update_status( gpib_board_t *board, tms9914_private_t *priv,
	unsigned int clear_mask );
int tms9914_primary_address(gpib_board_t *board,
	tms9914_private_t *priv, unsigned int address);
int tms9914_secondary_address(gpib_board_t *board, tms9914_private_t *priv,
	unsigned int address, int enable);
int tms9914_parallel_poll(gpib_board_t *board, tms9914_private_t *priv, uint8_t *result);
void tms9914_parallel_poll_configure( gpib_board_t *board,
	tms9914_private_t *priv, uint8_t config );
void tms9914_parallel_poll_response( gpib_board_t *board,
	tms9914_private_t *priv, int ist );
void tms9914_serial_poll_response(gpib_board_t *board, tms9914_private_t *priv, uint8_t status);
uint8_t tms9914_serial_poll_status( gpib_board_t *board, tms9914_private_t *priv );
int tms9914_line_status( const gpib_board_t *board, tms9914_private_t *priv );
unsigned int tms9914_t1_delay( gpib_board_t *board, tms9914_private_t *priv,
	unsigned int nano_sec );
void tms9914_return_to_local( const gpib_board_t *board, tms9914_private_t *priv );

// utility functions
void tms9914_board_reset(tms9914_private_t *priv);
void tms9914_online( gpib_board_t *board, tms9914_private_t *priv );
void tms9914_release_holdoff(tms9914_private_t *priv);
void tms9914_set_holdoff_mode(tms9914_private_t *priv, enum tms9914_holdoff_mode mode);

// wrappers for io functions
uint8_t tms9914_ioport_read_byte(tms9914_private_t *priv, unsigned int register_num);
void tms9914_ioport_write_byte(tms9914_private_t *priv, uint8_t data, unsigned int register_num);
uint8_t tms9914_iomem_read_byte(tms9914_private_t *priv, unsigned int register_num);
void tms9914_iomem_write_byte(tms9914_private_t *priv, uint8_t data, unsigned int register_num);

// interrupt service routine
irqreturn_t tms9914_interrupt(gpib_board_t *board, tms9914_private_t *priv );
irqreturn_t tms9914_interrupt_have_status(gpib_board_t *board, tms9914_private_t *priv, int status1,
		int status2);

// tms9914 has 8 registers
static const int tms9914_num_registers = 8;

/* tms9914 register numbers (might need to be multiplied by
 * a board-dependent offset to get actually io address offset)
 */
// write registers
enum
{
	IMR0 = 0,	/* interrupt mask 0          */
	IMR1 = 1,	/* interrupt mask 1          */
	AUXCR = 3,	/* auxiliary command         */
	ADR = 4,	// address register
	SPMR = 5,	// serial poll mode register
	PPR = 6,	/* parallel poll             */
	CDOR = 7,	/* data out register         */
};
// read registers
enum
{
	ISR0 = 0,	/* interrupt status 0          */
	ISR1 = 1,	/* interrupt status 1          */
	ADSR = 2,	/* address status               */
	BSR = 3,	/* bus status */
	CPTR = 6,	/* command pass thru           */
	DIR = 7,	/* data in register            */
};

//bit definitions common to tms9914 compatible registers

/* ISR0   - Register bits */
enum isr0_bits
{
	HR_MAC = ( 1 << 0 ),   /* My Address Change           */
	HR_RLC = ( 1 << 1 ),   /* Remote/Local change         */
	HR_SPAS = ( 1 << 2 ),   /* Serial Poll active State    */
	HR_END = ( 1 << 3 ),   /* END (EOI or EOS)            */
	HR_BO = ( 1 << 4 ),   /* Byte Out                    */
	HR_BI = ( 1 << 5 ),   /* Byte In                     */
};

/* IMR0   - Register bits */
enum imr0_bits
{
	HR_MACIE = ( 1 << 0 ),   /*        */
	HR_RLCIE = ( 1 << 1 ),   /*        */
	HR_SPASIE = ( 1 << 2 ),   /*        */
	HR_ENDIE = ( 1 << 3 ),   /*        */
	HR_BOIE = ( 1 << 4 ),   /*        */
	HR_BIIE = ( 1 << 5 ),   /*        */
};

/* ISR1   - Register bits */
enum isr1_bits
{
	HR_IFC = ( 1 << 0 ),   /* IFC asserted                */
	HR_SRQ = ( 1 << 1 ),   /* SRQ asserted                */
	HR_MA = ( 1 << 2 ),   /* My Adress                   */
	HR_DCAS = ( 1 << 3 ),   /* Device Clear active State   */
	HR_APT = ( 1 << 4 ),   /* Adress pass Through         */
	HR_UNC = ( 1 << 5 ),   /* Unrecognized Command        */
	HR_ERR = ( 1 << 6 ),   /* Data Transmission Error     */
	HR_GET = ( 1 << 7 ),   /* Group execute Trigger       */
};

/* IMR1   - Register bits */
enum imr1_bits
{
	HR_IFCIE = ( 1 << 0 ),   /*        */
	HR_SRQIE = ( 1 << 1 ),   /*        */
	HR_MAIE = ( 1 << 2 ),   /*        */
	HR_DCASIE = ( 1 << 3 ),   /*        */
	HR_APTIE = ( 1 << 4 ),   /*        */
	HR_UNCIE = ( 1 << 5 ),   /*        */
	HR_ERRIE = ( 1 << 6 ),   /*        */
	HR_GETIE = ( 1 << 7 ),   /*        */
};

/* ADSR   - Register bits */
enum adsr_bits
{
	HR_ULPA = ( 1 << 0 ),   /* Store last address LSB      */
	HR_TA = ( 1 << 1 ),   /* Talker Adressed             */
	HR_LA = ( 1 << 2 ),   /* Listener adressed           */
	HR_TPAS = ( 1 << 3 ),   /* talker primary adress state */
	HR_LPAS = ( 1 << 4 ),   /* listener    "               */
	HR_ATN = ( 1 << 5 ),   /* ATN active                  */
	HR_LLO = ( 1 << 6 ),   /* LLO active                  */
	HR_REM = ( 1 << 7 ),   /* REM active                  */
};

/* ADR   - Register bits */
enum adr_bits
{
	ADDRESS_MASK = 0x1f,	/* mask to specify lower 5 bits for ADR */
	HR_DAT = ( 1 << 5 ),   /* disable talker */
	HR_DAL = ( 1 << 6 ),   /* disable listener */
	HR_EDPA = ( 1 << 7 ),   /* enable dual primary addressing */
};

enum bus_status_bits
{
	BSR_REN_BIT = 0x1,
	BSR_IFC_BIT = 0x2,
	BSR_SRQ_BIT = 0x4,
	BSR_EOI_BIT = 0x8,
	BSR_NRFD_BIT = 0x10,
	BSR_NDAC_BIT = 0x20,
	BSR_DAV_BIT = 0x40,
	BSR_ATN_BIT = 0x80,
};

/*---------------------------------------------------------*/
/* TMS 9914 Auxiliary Commands                             */
/*---------------------------------------------------------*/

enum aux_cmd_bits
{
	AUX_CS = 0x80,	/* set bit instead of clearing it, used with commands marked 'd' below */
	AUX_CHIP_RESET = 0x0,	/* d Chip reset                   */
	AUX_INVAL = 0x1,	// release dac holdoff, invalid command byte
	AUX_VAL = ( AUX_INVAL | AUX_CS ),	// release dac holdoff, valid command byte
	AUX_RHDF = 0x2,	/* X Release RFD holdoff          */
	AUX_HLDA = 0x3,	/* d holdoff on all data          */
	AUX_HLDE = 0x4,	/* d holdoff on EOI only          */
	AUX_NBAF = 0x5,	/* X Set new byte availiable false */
	AUX_FGET = 0x6,	/* d force GET                    */
	AUX_RTL = 0x7,	/* d return to local              */
	AUX_SEOI = 0x8,	/* X send EOI with next byte      */
	AUX_LON = 0x9,	/* d Listen only                  */
	AUX_TON = 0xa,	/* d Talk only                    */
	AUX_GTS = 0xb,	/* X goto standby                 */
	AUX_TCA = 0xc,	/* X take control asynchronously  */
	AUX_TCS = 0xd,	/* X take    "     synchronously  */
	AUX_RPP = 0xe,	/* d Request parallel poll        */
	AUX_SIC = 0xf,	/* d send interface clear         */
	AUX_SRE = 0x10,	/* d send remote enable           */
	AUX_RQC = 0x11,	/* X request control              */
	AUX_RLC = 0x12,	/* X release control              */
	AUX_DAI = 0x13,	/* d disable all interrupts       */
	AUX_PTS = 0x14,	/* X pass through next secondary  */
	AUX_STDL = 0x15,	/* d short T1 delay                 */
	AUX_SHDW = 0x16,	/* d shadow handshake             */
	AUX_VSTDL = 0x17,	/* d very short T1 delay (smj9914 extension) */
	AUX_RSV2 = 0x18,	/* d request service bit 2 (smj9914 extension) */
};

#endif	//_TMS9914_H
