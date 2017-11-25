/*
 * mac.cpp
 *
 *  Created on: 30 Sep 2016
 *      Author: sid
 */
#include <math.h>

#include "../fanet_stack/fmac.h"
#include "../fanet_stack/sx1272.h"

/* get next frame which can be sent out */
//todo: this is potentially dangerous, as frm may be deleted in another place.
Frame* MacFifo::get_nexttx()
{
	int next;
	noInterrupts();
	for(next = 0; next<fifo.size(); next++)
		if(fifo.get(next)->next_tx < millis())
			break;
	Frame *frm;
	if(next == fifo.size())
		frm = NULL;
	else
		frm = fifo.get(next);
	interrupts();

	return frm;
}

Frame* MacFifo::frame_in_list(Frame *frm)
{
	noInterrupts();

	for(int i=0; i<fifo.size(); i++)
	{
		Frame *frm_list = fifo.get(i);
		if(*frm_list == *frm)
		{
			interrupts();
			return frm_list;
		}
	}

	interrupts();

	return NULL;
}

Frame* MacFifo::front()
{
	noInterrupts();
	Frame *frm = fifo.shift();
	interrupts();

	return frm;
}

/* add frame to fifo */
int MacFifo::add(Frame *frm)
{
	noInterrupts();

	/* buffer full */
	/* note: ACKs will always fit */
	if(fifo.size() >= MAC_FIFO_SIZE && frm->type != FRM_TYPE_ACK)
	{
		interrupts();
		return -1;
	}

	/* only one ack_requested from us to a specific address at a time is allowed in the queue */
	//in order not to screw with the awaiting of ACK
	if(frm->ack_requested)
	{
		for(int i=0; i<fifo.size(); i++)
		{
			//note: this never succeeds for received packets -> tx condition only
			Frame *ffrm = fifo.get(i);
			if(frm->ack_requested && ffrm->src == fmac.my_addr && ffrm->dest == frm->dest)
			{
				interrupts();
				return -2;
			}
		}
	}

	if(frm->type == FRM_TYPE_ACK)
		/* add to front */
		fifo.unshift(frm);
	else
		/* add to tail */
		fifo.add(frm);

	interrupts();
	return 0;
}

/* remove frame from linked list and delete it */
bool MacFifo::remove_delete(Frame *frm)
{
	bool found = false;

	noInterrupts();
	for(int i=0; i<fifo.size() && !found; i++)
		if(frm == fifo.get(i))
		{
			delete fifo.remove(i);
			found = true;
		}
	interrupts();

	return found;
}

/* remove any pending frame that waits on an ACK from a host */
bool MacFifo::remove_delete_acked_frame(MacAddr dest)
{
	bool found = false;
	noInterrupts();

	for(int i=0; i<fifo.size(); i++)
	{
		Frame* frm = fifo.get(i);
		if(frm->ack_requested && frm->dest == dest)
		{
			delete fifo.remove(i);
			found = true;
		}
	}
	interrupts();
	return found;
}

/* this is executed in a non-linear fashion */
void FanetMac::frame_received(int length)
{
	/* quickly read registers */
	num_received = sx1272.getFrame(rx_frame, sizeof(rx_frame));
	int rssi = sx1272.getRssi();

#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.print(F("### Mac Rx: "));
	SerialDEBUG.print(num_received, DEC);
	SerialDEBUG.print(F(" @ "));
	SerialDEBUG.print(rssi);
	SerialDEBUG.print(F(" "));

	for(int i=0; i<num_received; i++)
	{
		SerialDEBUG.print(rx_frame[i], HEX);
		if(i<num_received-1)
			SerialDEBUG.print(":");
	}
	SerialDEBUG.println();
#endif

	/* build frame from stream */
	Frame *frm = new Frame(num_received, rx_frame);
	frm->rssi = rssi;

	/* add to fifo */
	if(rx_fifo.add(frm) < 0)
		delete frm;
}

/* wrapper to fit callback into c++ */
void FanetMac::frame_rx_wrapper(int length)
{
	fmac.frame_received(length);
}

FanetMac::FanetMac() : my_timer(MAC_SLOT_MS, state_wrapper)
{
	num_received = 0;
}

bool FanetMac::begin(Fapp &app)
{
	myApp = &app;

	/* configure phy radio */
	if(sx1272.begin() == false)
		return false;
	sx1272.setBandwidth(BW_250);
	sx1272.setSpreadingFactor(SF_7);
	sx1272.setCodingRate(CR_5);
	sx1272.setExplicitHeader(true);
	sx1272.setPayloadCrc(true);
	sx1272.setLnaGain(LNAGAIN_G1_MAX, true);
	sx1272.setIrqReceiver(frame_rx_wrapper);

	/* region specific. default is EU */
	sx_region_t region = {.channel = CH_868_200, .dBm = 14};
	sx1272.setRegion(region);

	/* enter sleep mode */
	sx1272.setArmed(false);

	/* start state machine */
	my_timer.Start();

	/* start random machine */
#if defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_VARIANT_COMPLIANCE)
	/* use the device ID */
	volatile uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
	volatile uint32_t *ptr2 = (volatile uint32_t *)0x0080A040;
	volatile uint32_t *ptr3 = (volatile uint32_t *)0x0080A044;
	volatile uint32_t *ptr4 = (volatile uint32_t *)0x0080A048;
	randomSeed(millis() + *ptr1 + *ptr2 + *ptr3 + *ptr4);
#elif defined(STM32L432xx)
	uint32_t uid[3];
	STM32.getUID(uid);
	randomSeed(millis() + uid[0] + uid[1] + uid[2]);
#else
	randomSeed(millis());
#endif

	return true;
}

/* wrapper to fit callback into c++ */
void FanetMac::state_wrapper()
{
	/* only handle stuff during none-sleep mode */
	if(!sx1272.isArmed())
		return;

	fmac.handle_rx();
	fmac.handle_tx();
}

bool FanetMac::isNeighbor(MacAddr addr)
{
	for(int i=0; i<neighbors.size(); i++)
		if(neighbors.get(i)->addr == addr)
			return true;

	return false;
}

/*
 * Generates ACK frame
 */
void FanetMac::ack(Frame* frm)
{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.println(F("### generating ACK"));
#endif

	/* generate reply */
	Frame *ack = new Frame(my_addr);
	ack->type = FRM_TYPE_ACK;
	ack->dest = frm->src;

	/* only do a 2 hop ACK in case it was requested and we received it via a two hop link (= forward bit is not set anymore) */
	if(frm->ack_requested == MAC_ACK_TWOHOP && !frm->forward)
		ack->forward = true;

	/* add to front of fifo */
	//note: this will not fail by define
	if(tx_fifo.add(ack) != 0)
		delete ack;
}

/*
 * Processes stuff from rx_fifo
 */
void FanetMac::handle_rx()
{
	/* nothing to do */
	if(rx_fifo.size() == 0)
	{
		/* clean neighbors list */
		for(int i=0; i<neighbors.size(); i++)
		{
			if(neighbors.get(i)->isaround() == false)
				delete neighbors.remove(i);
		}

		return;
	}

	Frame *frm = rx_fifo.front();

	/* build up neighbors list */
	bool neighbor_known = false;
	for(int i=0; i<neighbors.size(); i++)
	{
		if(neighbors.get(i)->addr == frm->src)
		{
			/* update presents */
			neighbors.get(i)->seen();
			neighbor_known = true;
			break;
		}
	}
	/* neighbor unknown until now, add to list */
	if(neighbor_known == false)
	{
		/* too many neighbors, delete oldest member */
		if(neighbors.size() > MAC_NEIGHBOR_SIZE)
			delete neighbors.shift();

		neighbors.add(new NeighborNode(frm->src));
	}


	/* is the frame a forwarded one and is it still in the tx queue? */
	Frame *frm_list = tx_fifo.frame_in_list(frm);
	if(frm_list != NULL)
	{
		/* frame already in tx queue */

		if(frm->rssi > frm_list->rssi + MAC_FORWARD_MIN_DB_BOOST)
		{
			/* somebody broadcasted it already towards our direction */
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.println(F("### rx frame better than org. dropping both."));
#endif
			/* received frame is at least 20dB better than the original -> no need to rebroadcast */
			tx_fifo.remove_delete(frm_list);
		}
		else
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.println(F("### adjusting tx time"));
#endif
			/* adjusting new departure time */
			frm_list->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);
		}
	}
	else
	{
		if((frm->dest == MacAddr() || frm->dest == my_addr) && frm->src != my_addr)
		{
			/* a relevant frame */
			if(frm->type == FRM_TYPE_ACK)
			{
				if(tx_fifo.remove_delete_acked_frame(frm->src) && myApp != NULL)
					myApp->handle_acked(true, frm->src);
			}
			else
			{
				/* generate ACK */
				if(frm->ack_requested)
					ack(frm);

				/* forward frame */
				if(myApp != NULL)
					myApp->handle_frame(frm);
			}
		}

		/* Forward frame */
		if(frm->forward && tx_fifo.size() < MAC_FIFO_SIZE - 3 && frm->rssi <= MAC_FORWARD_MAX_RSSI_DBM &&
				(frm->dest == MacAddr() || isNeighbor(frm->dest)))
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.println(F("### adding new forward frame"));
#endif
			/* prevent from re-forwarding */
			frm->forward = false;

			/* generate new tx time */
			frm->next_tx = millis() + random(MAC_FORWARD_DELAY_MIN, MAC_FORWARD_DELAY_MAX);

			/* add to list */
			tx_fifo.add(frm);
			return;
		}
	}

	/* discard frame */
	delete frm;
}

/*
 * get a from from tx_fifo (or the app layer) and transmit it
 */
void FanetMac::handle_tx()
{
	/* still in backoff */
	if(millis() < csma_next_tx)
		return;

	/* find next send-able packet */
	/* this breaks the layering. however, this approach is much more efficient as the app layer now has a much higher priority */
	Frame* frm;
	bool app_tx = false;
	if(myApp->is_broadcast_ready(neighbors.size()))
//todo: check if first element of txfifo is not an ACK!
	{
		/* the app wants to broadcast the glider state */
		frm = myApp->get_frame();
		if(frm == NULL)
			return;

//todo?? set forward bit only if no inet base station is available, this MAY break the layers
		if(neighbors.size() <= MAC_MAXNEIGHBORS_4_TRACKING_2HOP)
			frm->forward = true;
		else
			frm->forward = false;

		app_tx = true;
	}
	else
	{
		/* get a from from the fifo */
		frm = tx_fifo.get_nexttx();
		if(frm == NULL)
			return;

		/* frame w/o a received ack and no more re-transmissions left */
		if(frm->ack_requested && frm->num_tx <= 0)
		{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
			SerialDEBUG.print(F("### Frame, 0x"));
			SerialDEBUG.print(frm->type, HEX);
			SerialDEBUG.println(F(" NACK!"));
#endif
			if(myApp != NULL)
				myApp->handle_acked(false, frm->dest);
			tx_fifo.remove_delete(frm);
			return;
		}

		/* unicast frame w/o forwarding and it is not a direct neighbor */
		if(frm->forward == false && frm->dest != MacAddr() && isNeighbor(frm->dest) == false)
			frm->forward = true;

		app_tx = false;
	}

	/* serialize frame */
	uint8_t* buffer;
	int blength = frm->serialize(buffer);
	if(blength < 0)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.println(F("### Problem serialization. removing."));
#endif
		/* problem while assembling the frame */
		if(app_tx)
			delete frm;
		else
			tx_fifo.remove_delete(frm);
		return;
	}

#if defined(SerialDEBUG) && MAC_debug_mode > 0
	SerialDEBUG.print(F("### Sending, 0x"));
	SerialDEBUG.print(frm->type, HEX);
	SerialDEBUG.print(F("..."));
#endif

#if defined(SerialDEBUG) && MAC_debug_mode > 1
	/* print hole packet */
	SerialDEBUG.print(F(" "));
	for(int i=0; i<blength; i++)
	{
		SerialDEBUG.print(buffer[i], HEX);
		if(i<blength-1)
			SerialDEBUG.print(F(":"));
	}
	SerialDEBUG.print(F(" "));
#endif

	/* for only a few nodes around, increase the coding rate to ensure a more robust transmission */
	if(neighbors.size() < MAC_CODING48_THRESHOLD)
		sx1272.setCodingRate(CR_8);
	else
		sx1272.setCodingRate(CR_5);

	/* channel free and transmit? */
	int tx_ret = sx1272.sendFrame(buffer, blength);
	delete[] buffer;

	if(tx_ret == TX_OK)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.println(F("done."));
#endif

		if(app_tx)
		{
			/* app tx */
			myApp->broadcast_successful(frm->type);
			delete frm;
		}
		else
		{
			/* fifo tx */

			/* transmission successful */
			if(!frm->ack_requested)
			{
				/* remove frame from FIFO */
				tx_fifo.remove_delete(frm);
			}
			else
			{
				/* update next transmission */
				if(--frm->num_tx > 0)
					frm->next_tx = millis() + (MAC_TX_RETRANSMISSION_TIME * (MAC_TX_RETRANSMISSION_RETRYS - frm->num_tx));
				else
					frm->next_tx = millis() + MAC_TX_ACKTIMEOUT;
			}
		}

		/* ready for a new transmission */
		csma_backoff_exp = MAC_TX_BACKOFF_EXP_MIN;
		csma_next_tx = millis() + MAC_TX_MINTIME;
	}
	else if(tx_ret == TX_RX_ONGOING)
	{
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.println(F("rx, abort."));
#endif

		if(app_tx)
			delete frm;

		/* channel busy, increment backoff exp */
		if(csma_backoff_exp < MAC_TX_BACKOFF_EXP_MAX)
			csma_backoff_exp++;

		/* next tx try */
		csma_next_tx = millis() + random(1<<(MAC_TX_BACKOFF_EXP_MIN-1), 1<<csma_backoff_exp);

#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.print(F("### backoff ("));
		SerialDEBUG.print(csma_next_tx - millis());
		SerialDEBUG.println(F("ms)"));
#endif
	}
	else
	{
		/* ignoring TX_TX_ONGOING */
#if defined(SerialDEBUG) && MAC_debug_mode > 0
		SerialDEBUG.println(F("### wow."));
#endif

		if(app_tx)
			delete frm;
	}
}

Frame::Frame()
{
	src = fmac.my_addr;
};

FanetMac fmac = FanetMac();
