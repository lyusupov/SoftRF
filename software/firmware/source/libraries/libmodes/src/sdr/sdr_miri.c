// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// sdr_miri.c: libmirisdr-4 support
//
// Copyright (c) 2014-2017 Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2017 FlightAware LLC
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#if defined(RASPBERRY_PI)

#include "sdr/common.h"
#include "sdr/sdr_miri.h"

#include <unistd.h>
#include "mirisdr.h"

#define DEFAULT_ASYNC_BUF_NUMBER	32
#define DEFAULT_BUF_LENGTH		(16 * 16384)

static struct { 
    mirisdr_dev_t *dev;
    iq_convert_fn converter;
    struct converter_state *converter_state;
    mirisdr_hw_flavour_t hw_flavour;
    bool verbose;
    int gain;
    int max_gain;
    char *transfer_type;
    uint32_t bandwidth;
} MIRI;

//
// =============================== miriSDR handling ==========================
//

void miriInitConfig()
{
    MIRI.dev = NULL;
    MIRI.converter = NULL;
    MIRI.converter_state = NULL;
    MIRI.gain = 0;
    MIRI.max_gain = 0;
    MIRI.bandwidth = 5e6;
    MIRI.transfer_type = "ISOC";
    MIRI.hw_flavour = MIRISDR_HW_DEFAULT;
}

void miriShowHelp()
{
    printf("      Mirics SDR-specific options  (use with --device-type miri)\n");
    printf("\n");
    printf("--bandwidth <hz>               (default: 5000000)\n");
    printf("--bulk-transfer                (default: isochronous)\n");
    printf("--hardware-flavour <string>    (hardware specific: \"default\", \"sdrplay\")\n");
    printf("\n");
}

bool miriHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j+1 < argc);
    if (!strcmp(argv[j], "--bandwidth") && more) {
        MIRI.bandwidth = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--bulk-transfer")) {
        MIRI.transfer_type = "BULK";
    } else if (!strcmp(argv[j], "--hardware-flavour") && more) {
        if (!strcasecmp(argv[++j], "sdrplay")) {
            MIRI.hw_flavour = MIRISDR_HW_SDRPLAY;
        }
    } else {
        return false;
    }

    *jptr = j;
    return true;
}

bool miriOpen(void)
{
    uint32_t dev_index = 0;
    uint32_t freq = state.freq;
    uint32_t sample_rate = state.sample_rate;

    int r, i = 0;
    int gain =  state.gain;
    char vendor[256] = { 0 }, product[256] = { 0 }, serial[256] = { 0 };

    int device_count = mirisdr_get_device_count();
    if (!device_count) {
	fprintf(stderr, "No supported devices found.\n");
	return false;
    }

    printf("Found %d device(s):\n", device_count);
    for (i = 0; i < device_count; i++) {
	mirisdr_get_device_usb_strings(i, vendor, product, serial);
	fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
    }

    printf("\n");

    r = mirisdr_open(&MIRI.dev, dev_index);
    if (r < 0) {
	fprintf(stderr, "Failed to open mirisdr device #%d.\n", dev_index);
	return false;
    }

    MIRI.max_gain = mirisdr_get_tuner_gains(MIRI.dev, NULL) -1;
    if (MIRI.max_gain > 0) {
        printf("Supported gain values: 0 - %d \n", MIRI.max_gain);
    }

    /* Set the hardware flavour */
    mirisdr_set_hw_flavour(MIRI.dev,  MIRI.hw_flavour);

    /* Set the sample rate */
    r = mirisdr_set_sample_rate(MIRI.dev, sample_rate);
    if (r < 0)
	fprintf(stderr, "WARNING: Failed to set sample rate to %u.\n", sample_rate);

    /* Set the frequency */
    r = mirisdr_set_center_freq(MIRI.dev, freq);
    if (r < 0)
	fprintf(stderr, "WARNING: Failed to set frequency to %u Hz.\n", freq);

    /* Set sample format */
    mirisdr_set_sample_format(MIRI.dev, "252_S16"); 

    /* Set USB transfer type */
    mirisdr_set_transfer(MIRI.dev, MIRI.transfer_type );

    /* Set IF mode to ZERO*/
    mirisdr_set_if_freq(MIRI.dev, 0);

    /* Set bandwidth */
    r = mirisdr_set_bandwidth(MIRI.dev, MIRI.bandwidth);
    if (r < 0)
        fprintf(stderr, "WARNING: Failed to set bandwidth to %u Hz.\n", MIRI.bandwidth);

    /* Set gain */
    miriSetGain(gain);

    /* Reset buffer before we start reading from it */
    r = mirisdr_reset_buffer(MIRI.dev);
    if (r < 0)
        fprintf(stderr, "WARNING: Failed to reset buffers.\n");

    MIRI.converter = init_converter(INPUT_SC16, 
                                     state.sample_rate,
                                     state.dc_filter,
                                     &MIRI.converter_state);
    if (!MIRI.converter) {
        fprintf(stderr, "MIRI: can't initialize sample converter\n");
        goto error;
    }

    return true;

 error:
    if (MIRI.dev != NULL) {
        mirisdr_close(MIRI.dev);
        MIRI.dev = NULL;
    }

    return false;
}


static void miriCallback(unsigned char *buf, uint32_t len, void *ctx)
{

    static unsigned dropped = 0;
    static uint64_t sampleCounter = 0;

    MODES_NOTUSED(ctx);

    sdrMonitor();

    if (state.exit) {
        mirisdr_cancel_async(MIRI.dev); 
        return;
    }

    unsigned samples_read = len/4;

    if (len < DEFAULT_BUF_LENGTH) {
        fprintf(stderr, "miri: len < DEFAULT_BUF_LENGTH, samples_read: %d len: %d buffer \n", samples_read, len);
        return;
    }

    struct mag_buf *outbuf = fifo_acquire(0 /* don't wait */);
    if (!outbuf) {
        // FIFO is full. Drop this block.
        dropped += samples_read;
        sampleCounter += samples_read;
        return;
    }

    outbuf->flags = 0;

    if (dropped) {
        // We previously dropped some samples due to no buffers being available
        outbuf->flags |= MAGBUF_DISCONTINUOUS;
        outbuf->dropped = dropped;
    }

    dropped = 0;

    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = sampleCounter * 12e6 / state.sample_rate;
    sampleCounter += samples_read;

    // Get the approx system time for the start of this block
    uint64_t block_duration = 1e3 * samples_read / state.sample_rate;
    outbuf->sysTimestamp = mstime() - block_duration;

    // Convert the new data
    unsigned to_convert = samples_read;
    if (to_convert + outbuf->overlap > outbuf->totalLength) {
        // how did that happen?
        to_convert = outbuf->totalLength - outbuf->overlap;
        dropped = samples_read - to_convert;
    }

    MIRI.converter(buf, &outbuf->data[outbuf->overlap], to_convert, MIRI.converter_state, &outbuf->mean_level, &outbuf->mean_power);
    outbuf->validLength = outbuf->overlap + to_convert;

    // Push to the demodulation thread
    fifo_enqueue(outbuf);
}

void miriRun()
{
    if (!MIRI.dev) {
        return;
    }

    uint32_t out_block_size = DEFAULT_BUF_LENGTH;

    fprintf(stderr, "miri:  out_block_size: %d buffer size %ld \n", out_block_size, (long)(out_block_size * sizeof(uint8_t)));

    int r = mirisdr_read_async(MIRI.dev, miriCallback, NULL,
                      DEFAULT_ASYNC_BUF_NUMBER,
                      out_block_size);

    if (!state.exit)
        fprintf(stderr, "miri: mirisdr_read_async returned unexpectedly. (error %d) \n", r);

}


void miriStop() {
    if (MIRI.dev) {
	    mirisdr_cancel_async(MIRI.dev);
    }
}

void miriClose()
{
    if (MIRI.dev) {
	    mirisdr_close(MIRI.dev);
        MIRI.dev = NULL;
    }

    if (MIRI.converter) {
        cleanup_converter(MIRI.converter_state);
        MIRI.converter = NULL;
        MIRI.converter_state = NULL;
    }
}

int miriGetGain()
{
    return MIRI.gain;
}

int miriGetMaxGain()
{
    return MIRI.max_gain;
}

double miriGetGainDb(int step)
{
    if (step < 0)
        step = 0;
    if (step >= MIRI.max_gain)
        step = MIRI.max_gain;
    return step;
}

int miriSetGain(int step)
{
    int r;

    if (step < 0) {
        step = 0;
    }
    if (step >= MIRI.max_gain) {
        step = MIRI.max_gain;
    }

	r = mirisdr_set_tuner_gain(MIRI.dev, step);
	if (r < 0)
		fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
	else
        MIRI.gain = step;

    return MIRI.gain;
}

#endif /* RASPBERRY_PI */
