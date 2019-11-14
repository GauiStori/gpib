
#include "gpibP.h"


/*
 * IBRSV
 * Request service from the CIC and/or set the serial poll
 * status byte.
 */
int ibrsv2( gpib_board_t *board, uint8_t status_byte, int new_reason_for_service )
{
	int board_status = ibstatus( board );
	const unsigned MSS = status_byte & request_service_bit;
	
	if( ( board_status & CIC ) )
	{
		printk("gpib: interface requested service while CIC\n");
		return -EINVAL;
	}

	if( MSS == 0 && new_reason_for_service ) 
	{
		return -EINVAL;
	}
	
	if(board->interface->serial_poll_response2)
	{
		board->interface->serial_poll_response2( board, status_byte, new_reason_for_service );
	// fall back on simpler serial_poll_response if the behavior would be the same	
	} else if( board->interface->serial_poll_response && ( MSS == 0 || ( MSS && new_reason_for_service ) ) )
	{
		board->interface->serial_poll_response( board, status_byte );
	} else 
	{
		return -EOPNOTSUPP;
	}
	
	return 0;
}
