
/* Quick and dirty ukhasnet-style transmitter for hackrf */
/* - Philip Heron <phil@sanslogic.co.uk>                 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <libhackrf/hackrf.h>

/* RF details */
#define FREQUENCY  (869500000)     /* 869.5MHz centre frequency	           */
#define DEVIATION  (12000)         /* +/- 12KHz deviation (24KHz total)    */
#define BITRATE    (2000)          /* 2000 baud                            */
#define SPS        (1000)	   /* Samples per symbol/bit               */
#define SAMPLERATE (BITRATE * SPS) /* == 2 MHz sample rate, min for HackRF */
#define FSTEP      (2 * M_PI * DEVIATION / SAMPLERATE)

/* Packet details */
#define INVERT_BITS  (0)           /* 0-send normal, 1-send inverted */
#define PREAMBLE_SYM (0xAA)
#define PREAMBLE_LEN (3)
#define SYNC_WORD    (0x2DAA)
#define MAX_LENGTH   (0xFF)
#define CRC_INIT     (0x1D0F)
#define MAX_PACKET (PREAMBLE_LEN + 2 + MAX_LENGTH + 2)

/* HackRF specifics -- number of silent samples to send first and last */
#define HACKRF_FIRST_DELAY (SAMPLERATE / 16)
#define HACKRF_LAST_DELAY (SAMPLERATE / 4)

/* Memory required for max packet, bytes */
#define MAX_WAVE ((HACKRF_FIRST_DELAY + (MAX_PACKET * 8 * SPS) + HACKRF_LAST_DELAY) * 2)

/* Globals -- urgh */
volatile int8_t baseband_packet[MAX_WAVE];
volatile int8_t *baseband_sample = NULL;
volatile ssize_t baseband_length = MAX_WAVE;

volatile char do_exit = 0;

/* RF lowpass FIR filter coefficients */
#define FIR_LEN (16)
static float _fir_co[FIR_LEN / 2] = {
	0.0097049,
	0.01456294,
	0.02828953,
	0.04855131,
	0.07186047,
	0.09418267,
	0.11164295,
	0.12120522
};

static float _fir(int len, float *co, float *h, float sample)
{
	float r = 0;
	int i, hlen;
	
	hlen = len / 2;
	
	for(i = 0; i < len; i++)
	{
		if(i < len - 1) h[i] = h[i + 1];
		else h[i] = sample;
		
		if(i < hlen) r += h[i] * co[i];
		else r += h[i] * co[hlen - (i - hlen)];
	}
	
	return(r);
}

/* CRC function from avr-libc */
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data)
{
	int i;
	
	crc = crc ^ ((uint16_t) data << 8);
	for(i = 0; i < 8; i++)
	{
		if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
		else crc <<= 1;
	}
	
	return(crc);
}

void ukhasnet_packet(void *data, size_t length)
{
	uint8_t packet[MAX_PACKET], *d;
	uint16_t crc = CRC_INIT;
	int i;
	
	for(i = 0; i < PREAMBLE_LEN; i++)
		packet[i] = PREAMBLE_SYM;
	
	packet[i++] = (SYNC_WORD >> 8) & 0xFF;
	packet[i++] = SYNC_WORD & 0xFF;
	
	crc = crc_xmodem_update(crc, (uint8_t) length);
	packet[i++] = length;
	
	for(d = data; length; length--)
	{
		crc = crc_xmodem_update(crc, *d);
		packet[i++] = *(d++);
	}
	
	crc ^= 0xFFFF;
	
	packet[i++] = (crc >> 8) & 0xFF;
	packet[i++] = crc & 0xFF;
	
	/* Prepare the baseband */
	if(1)
	{
		float ph = 0, fi, fq;
		float fih[FIR_LEN], fqh[FIR_LEN];
		int j, b, s, bit;
		int samp = 0;
		
		/* Clear the fir filter buffer */
		for(b = 0; b < FIR_LEN; b++)
			fih[b] = fqh[b] = 0;
		
		samp = 0;
		for(j = 0; j < HACKRF_FIRST_DELAY * 2; j++)
			baseband_packet[samp++] = 0;
		
		/* i == the number of bytes to transmit, set above */
		for(j = 0; j < i; j++)
		{
			fprintf(stderr, "0x%02X ", packet[j]);
			for(b = 7; b >= 0; b--)
			{
				bit = (packet[j] >> b) & 1;
				for(s = 0; s < SPS; s++)
				{
					fi = cos(ph);
					fq = sin(ph);
					
					#if INVERT_BITS
					ph += (bit ? -FSTEP : FSTEP);
					#else
					ph += (bit ? FSTEP : -FSTEP);
					#endif
					
					fi = _fir(FIR_LEN, _fir_co, fih, fi);
					fq = _fir(FIR_LEN, _fir_co, fqh, fq);
					
					baseband_packet[samp++] = fi * 127;
					baseband_packet[samp++] = fq * 127;
				}
			}
		}
		fprintf(stderr, "\n");

		for(j = 0; j < HACKRF_LAST_DELAY * 2; j++)
			baseband_packet[samp++] = 0;
		
		baseband_sample = baseband_packet;
		baseband_length = samp;
		
		/*{
			FILE *f = fopen("packet.bin", "wb");
			fwrite((void *) baseband_packet, 1, baseband_length, f);
			fclose(f);
		}*/
	}
}

int tx_callback(hackrf_transfer* transfer)
{
	size_t bytes_to_read;
	
	if(baseband_length == 0)
	{
		/* Packet has finished. Send one last
		 * blank buffer in to avoid the hackrf
		 * transmitting noise before the TX
		 * shuts down */
		memset(transfer->buffer, 0, transfer->valid_length);
		baseband_length--;
		return(0);
	}
	else if(baseband_length < 0)
	{
		/* All done. Signal to the main thread to end TX */
		do_exit = -1;
		return(-1);
	}
	
	bytes_to_read = transfer->valid_length;
	if(bytes_to_read > baseband_length)
		bytes_to_read = baseband_length;
	
	/* Copy requested sample data */
	memcpy(transfer->buffer, (void *) baseband_sample, bytes_to_read);
	baseband_sample += bytes_to_read;
	baseband_length -= bytes_to_read;
	
	/* Zero remaining buffer */
	if(bytes_to_read < transfer->valid_length)
		memset(transfer->buffer + bytes_to_read, 0, transfer->valid_length - bytes_to_read);
	
	return(0);
}

void sigint_callback_handler(int signum) 
{
	fprintf(stdout, "Caught signal %d\n", signum);
	do_exit = 1;
}

int tx_packet(void)
{
	int r;
	hackrf_device* device = NULL;
	
	uint32_t sample_rate_hz = SAMPLERATE;
	uint32_t baseband_filter_bw_hz = 0;
	unsigned int txvga_gain = 47;
	
	/* Compute default value depending on sample rate */
	baseband_filter_bw_hz = hackrf_compute_baseband_filter_bw_round_down_lt(sample_rate_hz);
	
	r = hackrf_init();
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_init() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	r = hackrf_open(&device);
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_open() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	/* Catch all the signals */
	signal(SIGINT, &sigint_callback_handler);
	signal(SIGILL, &sigint_callback_handler);
	signal(SIGFPE, &sigint_callback_handler);
	signal(SIGSEGV, &sigint_callback_handler);
	signal(SIGTERM, &sigint_callback_handler);
	signal(SIGABRT, &sigint_callback_handler);
	
	r = hackrf_set_sample_rate_manual(device, sample_rate_hz, 1);
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_sample_rate_set() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	r = hackrf_set_baseband_filter_bandwidth(device, baseband_filter_bw_hz);
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_baseband_filter_bandwidth_set() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	r  = hackrf_set_txvga_gain(device, txvga_gain);
	r |= hackrf_start_tx(device, tx_callback, NULL);
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_start_?x() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	r = hackrf_set_freq(device, FREQUENCY);
	if(r != HACKRF_SUCCESS)
	{
		fprintf(stderr, "hackrf_set_freq() failed: %s (%d)\n", hackrf_error_name(r), r);
		return(-1);
	}
	
	/* TODO: Timeout */
	while(do_exit == 0)
		usleep(100000);
	
	//r = hackrf_is_streaming(device);	
	
	r = hackrf_stop_tx(device);
	if(r != HACKRF_SUCCESS)
		fprintf(stderr, "hackrf_stop_tx() failed: %s (%d)\n", hackrf_error_name(r), r);
	
	r = hackrf_close(device);
	if(r != HACKRF_SUCCESS)
		fprintf(stderr, "hackrf_close() failed: %s (%d)\n", hackrf_error_name(r), r);
	
	hackrf_exit();
	
	return(-1);
}

int main(int argc, char *argv[])
{
	if(argc != 2) return(-1);
	
	ukhasnet_packet(argv[1], strlen(argv[1]));
	tx_packet();
	
	return(0);
}

