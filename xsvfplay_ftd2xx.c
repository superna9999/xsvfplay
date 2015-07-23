/*
 *  Lib(X)SVF  -  A library for implementing SVF and XSVF JTAG players
 *
 *  Copyright (C) 2009  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2009  Clifford Wolf <clifford@clifford.at>
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *
 *  A JTAG SVF/XSVF Player based on libxsvf for the FTDI FT232H, FT2232H and
 *  FT4232H High Speed USB to Multipurpose UART/FIFO ICs.
 *
 *  This also serves as an example program for using libxvsf with asynchonous
 *  hardware interfaces. Have a look at 'xsvftool-gpio.c' for a simple libxsvf
 *  example for synchonous interfaces (such as register mapped GPIOs).
 *
 *  IMPORTANT NOTE: You need libftdi [1] (version 0.16 or newer) installed
 *  to build this program.
 *
 *  To run it at full speed you need a version of libftdi that has been
 *  compiled with '--with-async-mode', and must enable all four defines
 *  BLOCK_WRITE, ASYNC_WRITE, BACKGROUND_READ and INTERLACED_READ_WRITE below.
 *
 *  [1] http://www.intra2net.com/en/developer/libftdi/
 */

#include "libxsvf.h"

#define BUFFER_SIZE (1024*16)

#define BLOCK_WRITE
// #define ASYNC_WRITE
// #define BACKGROUND_READ
// #define INTERLACED_READ_WRITE

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <errno.h>
#ifdef FORWIN
#include <windows.h>
#endif
#include <ftd2xx.h>
#include <math.h>
#ifdef BACKGROUND_READ
#  include <pthread.h>
#endif

/* FTDI Specifics */
char jtag_port_name[256] = "FTDI SPARTAN6 B";
unsigned jtag_port_pos = -1;
char control_port_name[256] = "FTDI SPARTAN6 D";
int control_mask = 0x3;
int control_value = 0x1;    // Flash access disabled, Spartan JTAG Enabled

struct read_job_s;
struct udata_s;
struct buffer_s;

typedef void job_handler_t(struct udata_s *u, struct read_job_s *job, unsigned char *data);

struct read_job_s {
	struct read_job_s *next;
	int data_len, bits_len;
	struct buffer_s *buffer;
	job_handler_t *handler;
	unsigned int command_id;
};

struct buffer_s {
	unsigned int tms:1;
	unsigned int tdi:1;
	unsigned int tdi_enable:1;
	unsigned int tdo:1;
	unsigned int tdo_enable:1;
	unsigned int rmask:1;
};

struct udata_s {
	FILE *f;
	FT_HANDLE ftdic;
	int buffer_size;
	struct buffer_s buffer[BUFFER_SIZE];
	struct read_job_s *job_fifo_out, *job_fifo_in;
	int last_tms;
	int last_tdo;
	int buffer_i;
	int retval_i;
	int retval[256];
	int error_rc;
	int verbose;
	int syncmode;
	int forcemode;
	int frequency;
#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	int total_job_bits;
	int writer_wait_flag;
	pthread_mutex_t writer_wait_flag_mutex;
#  endif
	int reader_terminate;
	pthread_mutex_t read_write_mutex;
	pthread_cond_t read_more_cond;
	pthread_cond_t read_done_cond;
	pthread_t read_thread;
#endif
#ifdef BLOCK_WRITE
	int ftdibuf_len;
	unsigned char ftdibuf[4096];
#endif
    off_t filesize;
    int progress;
    int no_enable_jtag;
    unsigned PID;
    unsigned VID;
};

static FILE *dumpfile = NULL;

static void write_dumpfile(int wr, unsigned char *buf, int size, unsigned int command_id)
{
	int i;
	if (!dumpfile)
		return;
	fprintf(dumpfile, "%s[%u] %04x:", wr ? "SEND" : "RECV", command_id, size);
	for (i = 0; i < size; i++)
		fprintf(dumpfile, " %02x", buf[i]);
	fprintf(dumpfile, "\n");
}

static int my_ftdi_read_data(FT_HANDLE ftdi, unsigned char *buf, int size, unsigned int command_id)
{
	int pos = 0;
    DWORD r;
	int poll_count = 0;
	while (pos < size) {
        r = -1;
        int rc = FT_Read(ftdi, buf+pos, size-pos, &r);
		if (rc < 0 || r == -1) {
			fprintf(stderr, "[***] ftdi_read_data returned error (rc=%d, r=%d).\n", rc, r);
			break;
		}
		// this check should only be needed for very low JTAG clock frequencies
		if (r == 0) {
			if (++poll_count > 8) {
				fprintf(stderr, "[***] my_ftdi_read_data gives up polling <id=%u, pos=%u, size=%u>.\n", command_id, pos, size);
				break;
			}
			// fprintf(stderr, "[%d/8] my_ftdi_read_data with len=%d polling at %d..\n", poll_count, size, pos);
			usleep(4096 << poll_count);
		}
		pos += r;
	}
	write_dumpfile(0, buf, pos, command_id);
	return pos;
}

static int my_ftdi_write_data(struct udata_s *u, unsigned char *buf, int size, int sync)
{
#ifdef BLOCK_WRITE
	int rc, total_queued = 0;
    DWORD w;

	sync = 1;

	while (size > 0)
	{
		if (u->ftdibuf_len == 4096) {
			if (dumpfile)
				fprintf(dumpfile, "WRITE %d BYTES (buffer full)\n", u->ftdibuf_len);
#ifdef ASYNC_WRITE
			//rc = ftdi_write_data_async(&u->ftdic, u->ftdibuf, u->ftdibuf_len);
#else
			//rc = ftdi_write_data(&u->ftdic, u->ftdibuf, u->ftdibuf_len);
#endif
            rc = FT_Write(u->ftdic, u->ftdibuf, u->ftdibuf_len, &w);
			if (rc != FT_OK || w != u->ftdibuf_len)
				return -1;
			u->ftdibuf_len = 0;
		}

		int chunksize = 4096 - u->ftdibuf_len;
		if (chunksize > size)
			chunksize = size;

		memcpy(u->ftdibuf + u->ftdibuf_len, buf, chunksize);
		u->ftdibuf_len += chunksize;
		total_queued += chunksize;
		size -= chunksize;
		buf += chunksize;
	}

	if (sync && u->ftdibuf_len > 0) {
		if (dumpfile)
			fprintf(dumpfile, "WRITE %d BYTES (sync)\n", u->ftdibuf_len);
#ifdef ASYNC_WRITE
		//rc = ftdi_write_data_async(&u->ftdic, u->ftdibuf, u->ftdibuf_len);
#else
		//rc = ftdi_write_data(&u->ftdic, u->ftdibuf, u->ftdibuf_len);
#endif
        rc = FT_Write(u->ftdic, u->ftdibuf, u->ftdibuf_len, &w);
		if (rc != FT_OK || w != u->ftdibuf_len)
			return -1;
		u->ftdibuf_len = 0;
	}

	return total_queued;
#else
   int rc, w;
#  ifdef ASYNC_WRITE
	//return ftdi_write_data_async(&u->ftdic, buf, size);
#  else
	//return ftdi_write_data(&u->ftdic, buf, size);
#  endif
   if(FT_Write(u->ftdic, buf, size, &w) == FT_OK)
       return w;
   return -1;
#endif
}

static struct read_job_s *new_read_job(struct udata_s *u, int data_len, int bits_len, struct buffer_s *buffer, job_handler_t *handler)
{
	struct read_job_s *job = calloc(1, sizeof(struct read_job_s));
	static unsigned int command_count = 0;

	job->data_len = data_len;
	job->bits_len = bits_len;
	job->buffer = calloc(bits_len, sizeof(struct buffer_s));
	memcpy(job->buffer, buffer, bits_len*sizeof(struct buffer_s));
	job->handler = handler;
	job->command_id = command_count++;

	if (u->job_fifo_in)
		u->job_fifo_in->next = job;
	if (!u->job_fifo_out)
		u->job_fifo_out = job;
	u->job_fifo_in = job;

#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	u->total_job_bits += bits_len;
#  endif
#endif

	return job;
}

static void transfer_tms_job_handler(struct udata_s *u, struct read_job_s *job, unsigned char *data)
{
	int i;
	for (i=0; i<job->bits_len; i++) {
		// seams like output is align to the MSB in the byte and is LSB first
		int bitpos = i + (8 - job->bits_len);
		int line_tdo = (*data & (1 << bitpos)) != 0 ? 1 : 0;
		if (job->buffer[i].tdo_enable && job->buffer[i].tdo != line_tdo)
			u->error_rc = -1;
		if (job->buffer[i].rmask && u->retval_i < 256)
			u->retval[u->retval_i++] = line_tdo;
		u->last_tdo = line_tdo;
	}
}

static void transfer_tms(struct udata_s *u, struct buffer_s *d, int tdi, int len)
{
	int i, rc;

	unsigned char data_command[] = {
		0x6e, len-1, tdi << 7, 0x87
	};

	for (i=0; i<len; i++)
		data_command[2] |= d[i].tms << i;
	data_command[2] |= d[len-1].tms << len;
	u->last_tms = d[len-1].tms;

	struct read_job_s *rj = new_read_job(u, 1, len, d, &transfer_tms_job_handler);

	write_dumpfile(1, data_command, sizeof(data_command), rj->command_id);
	rc = my_ftdi_write_data(u, data_command, sizeof(data_command), 0);
	if (rc != sizeof(data_command)) {
		fprintf(stderr, "IO Error: Transfer tms write failed: (rc=%d/%d)\n",
				rc, (int)sizeof(data_command));
		u->error_rc = -1;
	}
}

static void transfer_tdi_job_handler(struct udata_s *u, struct read_job_s *job, unsigned char *data)
{
	int i, j, k;
	int bytes = job->bits_len / 8;
	int bits = job->bits_len % 8;

	for (i=0, j=0; j<bytes; j++) {
		for (k=0; k<8; k++, i++) {
			int line_tdo = (data[j] & (1 << k)) != 0 ? 1 : 0;
			if (job->buffer[i].tdo_enable && job->buffer[i].tdo != line_tdo)
				if (!u->forcemode)
					u->error_rc = -1;
			if (job->buffer[j*8+k].rmask && u->retval_i < 256)
				u->retval[u->retval_i++] = line_tdo;
		}
	}
	for (j=0; j<bits; j++, i++) {
		int bitpos = j + (8 - bits);
		int line_tdo = (data[bytes] & (1 << bitpos)) != 0 ? 1 : 0;
		if (job->buffer[i].tdo_enable && job->buffer[i].tdo != line_tdo)
			if (!u->forcemode)
				u->error_rc = -1;
		if (job->buffer[i].rmask && u->retval_i < 256)
			u->retval[u->retval_i++] = line_tdo;
		u->last_tdo = line_tdo;
	}
}

static void transfer_tdi(struct udata_s *u, struct buffer_s *d, int len)
{
	int bytes = len / 8;
	int bits = len % 8;

	int command_len = 1;
	int data_len = 0;
	if (bytes) {
		command_len += 3 + bytes;
		data_len += bytes;
	}
	if (bits) {
		command_len += 3;
		data_len++;
	}

	int i, j, k, rc;
	unsigned char command[command_len];

	i = 0;
	if (bytes) {
		command[i++] = 0x39;
		command[i++] = (bytes-1) & 0xff;
		command[i++] = (bytes-1) >> 8;
		for (j=0; j<bytes; j++, i++) {
			command[i] = 0;
			for (k=0; k<8; k++)
				command[i] |= d[j*8+k].tdi << k;
		}
	}
	if (bits) {
		command[i++] = 0x3b;
		command[i++] = bits-1;
		command[i] = 0;
		for (j=0; j<bits; j++)
			command[i] |= d[bytes*8+j].tdi << j;
		i++;
	}
	command[i] = 0x87;
	assert(i+1 == command_len);

	struct read_job_s *rj = new_read_job(u, data_len, len, d, &transfer_tdi_job_handler);

	write_dumpfile(1, command, command_len, rj->command_id);
	rc = my_ftdi_write_data(u, command, command_len, 0);
	if (rc != command_len) {
		fprintf(stderr, "IO Error: Transfer tdi write failed: (rc=%d/%d)\n",
				rc, command_len);
		u->error_rc = -1;
	}
}

static void process_next_read_job(struct udata_s *u)
{
	if (!u->job_fifo_out)
		return;

#ifdef ASYNC_WRITE
	//ftdi_async_complete(&u->ftdic,1);
#endif

	struct read_job_s *job = u->job_fifo_out;
	
	u->job_fifo_out = job->next;
	if (!u->job_fifo_out)
		u->job_fifo_in = NULL;
	
	unsigned char data[job->data_len];
	if (my_ftdi_read_data(u->ftdic, data, job->data_len, job->command_id) != job->data_len) {
		fprintf(stderr, "IO Error: FTDI/USB read failed!\n");
		u->error_rc = -1;
	} else {
		job->handler(u, job, data);
	}

#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	u->total_job_bits -= job->bits_len;
#  endif
#endif
	
	free(job->buffer);
	free(job);
}

#ifdef BACKGROUND_READ
static void *reader_main(void *arg)
{
	struct udata_s *u = arg;
	pthread_mutex_lock(&u->read_write_mutex);
	while (!u->reader_terminate) {
		while (u->job_fifo_out) {
			process_next_read_job(u);
#ifdef INTERLACED_READ_WRITE
			if (u->total_job_bits <= u->buffer_size/2) {
				pthread_mutex_lock(&u->writer_wait_flag_mutex);
				int writer_is_waiting = u->writer_wait_flag;
				pthread_mutex_unlock(&u->writer_wait_flag_mutex);
				if (writer_is_waiting)
					break;
			}
#endif
		}
		pthread_cond_signal(&u->read_done_cond);
		pthread_cond_wait(&u->read_more_cond, &u->read_write_mutex);
	}
	pthread_mutex_unlock(&u->read_write_mutex);
	return NULL;
}
#endif

static void buffer_flush(struct udata_s *u)
{
    if(u->filesize > 0 && u->progress > 0)
    {
        long pos = ftell(u->f);
        printf("\r Progress : [%3d%%] %li/%lu\r", (int)(((float)pos/(float)u->filesize)*100), pos, u->filesize);
        fflush(stdout);
    }
#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	pthread_mutex_lock(&u->writer_wait_flag_mutex);
	u->writer_wait_flag = 1;
	pthread_mutex_unlock(&u->writer_wait_flag_mutex);
	pthread_mutex_lock(&u->read_write_mutex);
	while (u->total_job_bits > u->buffer_size/2) {
		pthread_cond_wait(&u->read_done_cond, &u->read_write_mutex);
	}
#  else
	pthread_mutex_lock(&u->read_write_mutex);
	while (u->job_fifo_out) {
		pthread_cond_wait(&u->read_done_cond, &u->read_write_mutex);
	}
#  endif
#endif
	int pos = 0;
	while (pos < u->buffer_i)
	{
		struct buffer_s b = u->buffer[pos];
		if (u->last_tms != b.tms) {
			int len = u->buffer_i - pos;
			len = len > 6 ? 6 : len;
			int tdi=-1, i;
			for (i=0; i<len; i++) {
				if (!u->buffer[pos+i].tdi_enable)
					continue;
				if (tdi < 0)
					tdi = u->buffer[pos+i].tdi;
				if (tdi != u->buffer[pos+i].tdi)
					len = i;
			}
			// printf("transfer_tms <len=%d, tdi=%d>\n", len, tdi < 0 ? 1 : tdi);
			transfer_tms(u, u->buffer+pos, (tdi & 1), len);
			pos += len;
			continue;
		}
		int len = u->buffer_i - pos;
		int i;
		for (i=0; i<len; i++) {
			if (u->buffer[pos+i].tms != u->last_tms)
				len = i;
		}
		// printf("transfer_tdi <len=%d, tms=%d>\n", len, u->last_tms);
		transfer_tdi(u, u->buffer+pos, len);
		pos += len;
	}
	u->buffer_i = 0;

#ifdef BLOCK_WRITE
	int rc = my_ftdi_write_data(u, NULL, 0, 1);
	if (rc != 0) {
		fprintf(stderr, "IO Error: Ftdi write failed\n");
		u->error_rc = -1;
	}
#endif

#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	pthread_mutex_lock(&u->writer_wait_flag_mutex);
	u->writer_wait_flag = 0;
	pthread_mutex_unlock(&u->writer_wait_flag_mutex);
#  endif
	pthread_mutex_unlock(&u->read_write_mutex);
	pthread_cond_signal(&u->read_more_cond);
#else
	while (u->job_fifo_out)
		process_next_read_job(u);
#endif
}

static void buffer_sync(struct udata_s *u)
{
	buffer_flush(u);
#ifdef BACKGROUND_READ
	pthread_mutex_lock(&u->read_write_mutex);
	while (u->job_fifo_out) {
		pthread_cond_wait(&u->read_done_cond, &u->read_write_mutex);
	}
	pthread_mutex_unlock(&u->read_write_mutex);
#endif
}

static void buffer_add(struct udata_s *u, int tms, int tdi, int tdo, int rmask)
{
	u->buffer[u->buffer_i].tms = tms;
	u->buffer[u->buffer_i].tdi = tdi;
	u->buffer[u->buffer_i].tdi_enable = tdi >= 0;
	u->buffer[u->buffer_i].tdo = tdo;
	u->buffer[u->buffer_i].tdo_enable = tdo >= 0;
	u->buffer[u->buffer_i].rmask = rmask;
	u->buffer_i++;

	if (u->buffer_i >= u->buffer_size)
		buffer_flush(u);
}

FT_HANDLE	ftHandleCtrl;
#include <stdint.h>

#define MAX_DEVICES 32

void listFTDI()
{
    FT_STATUS   ftStatus;         
    int iNumDevs = 0, i;
    char *  pcBufLD[MAX_DEVICES + 1]; 
    char cBufLD[MAX_DEVICES][64];

    for(i = 0; i < MAX_DEVICES; i++) {
        pcBufLD[i] = cBufLD[i];
    }   
    pcBufLD[MAX_DEVICES] = NULL;
    
    ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, FT_LIST_ALL |FT_OPEN_BY_DESCRIPTION);
        
    if(ftStatus != FT_OK) {
        printf("Error: FT_ListDevices(%d)\n", ftStatus);
        exit(1);
    }   
    printf("%d devices\n", iNumDevs);
    
    for(i = 0; ( (i <MAX_DEVICES) && (i < iNumDevs) ); i++) {
        printf("Device %d - '%s'\n", i, cBufLD[i]);
    }
}

void enableJTAG()
{
    FT_STATUS	ftStatus;
    DWORD written;
    uint8_t buf;

    // CTRL
    ftStatus = FT_OpenEx(control_port_name, FT_OPEN_BY_DESCRIPTION, &ftHandleCtrl);
    if(ftStatus != FT_OK) {
        printf("Error Ctrl: FT_Open(%d)\n", ftStatus);
        return;
    }

    //FT_ResetDevice(ftHandleCtrl);

    ftStatus = FT_SetBitMode(ftHandleCtrl, control_mask, 0x4);
    if(ftStatus != FT_OK) {
        printf("Error Ctrl: FT_SetBitMode(%d)\n", ftStatus);
        return;
    }

    FT_Purge(ftHandleCtrl, FT_PURGE_RX | FT_PURGE_TX);

    // Enable JTAG
    buf = control_value;
    ftStatus = FT_Write(ftHandleCtrl, &buf, 1, &written);
    if(ftStatus != FT_OK || written == 0) {
        printf("Error Ctrl: FT_Write(%d, %d)\n", ftStatus, written);
        return;
    }

    // Read data back
    FT_Read(ftHandleCtrl, &buf, 1, &written);
    printf("Wrote %02X Read %02X\n", control_value, buf&control_mask);

    buf = control_value;
    ftStatus = FT_Write(ftHandleCtrl, &buf, 1, &written);
    if(ftStatus != FT_OK || written == 0) {
        printf("Error Ctrl: FT_Write(%d, %d)\n", ftStatus, written);
        return;
    }

    // Read data back
    FT_Read(ftHandleCtrl, &buf, 1, &written);
    printf("Wrote %02X Read %02X\n", control_value, buf&control_mask);

   usleep(10000);
}

static int h_setup(struct libxsvf_host *h)
{
	DWORD w;
    FT_STATUS ret;

	struct udata_s *u = h->user_data;
	u->buffer_size = BUFFER_SIZE;
#ifdef BLOCK_WRITE
	u->ftdibuf_len = 0;
#endif

    if(u->no_enable_jtag == 0)
    {
        enableJTAG();
    }

    if (jtag_port_pos >= 0)
        ret = FT_Open(jtag_port_pos, &u->ftdic);
    else
        ret = FT_OpenEx(jtag_port_name, FT_OPEN_BY_DESCRIPTION, &u->ftdic);
        
    if (ret != FT_OK) {
        fprintf(stderr, "Failed to Open FTDI JTAG Interface(%s or %d) ret %d\n", jtag_port_name, jtag_port_pos, ret);
        return -1;
    }

    if(FT_ResetDevice(u->ftdic) != FT_OK ||
       FT_SetBitMode(u->ftdic, 0x00, 0) != FT_OK ||
       FT_SetLatencyTimer(u->ftdic, 2) != FT_OK ||
       FT_SetTimeouts(u->ftdic, 5000, 5000) != FT_OK ||
       FT_SetBitMode(u->ftdic, 0x0B, 2) != FT_OK)
        return -1;

	unsigned char amontec_init_commands[] = {
        0x8B, 0x97, 0x8D, // 12Mhz internal clk, no adaptative clocking
		0x86, 0x02, 0x00, // initial clk freq (2 MHz)
		0x80, 0x08, 0x1b, // initial line states
		0x85, // disable loopback
	};

	write_dumpfile(1, amontec_init_commands, sizeof(amontec_init_commands), 0);
	if (FT_Write(u->ftdic, amontec_init_commands, sizeof(amontec_init_commands), &w) != FT_OK) {
		fprintf(stderr, "IO Error: Interface setup failed (init commands)\n");
		return -1;
	}

	if (u->frequency > 0)
		h->set_frequency(h, u->frequency);

	u->job_fifo_out = NULL;
	u->job_fifo_in = NULL;
	u->last_tms = -1;
	u->last_tdo = -1;
	u->buffer_i = 0;
	u->error_rc = 0;

#ifdef BACKGROUND_READ
#  ifdef INTERLACED_READ_WRITE
	u->total_job_bits = 0;
	u->writer_wait_flag = 0;
	pthread_mutex_init(&u->writer_wait_flag_mutex, NULL);
#  endif
	u->reader_terminate = 0;
	pthread_mutex_init(&u->read_write_mutex, NULL);
	pthread_cond_init(&u->read_more_cond, NULL);
	pthread_cond_init(&u->read_done_cond, NULL);
	pthread_create(&u->read_thread, NULL, &reader_main, u);
#endif

	return 0;
}

static int h_shutdown(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	buffer_sync(u);
#ifdef BACKGROUND_READ
	pthread_mutex_lock(&u->read_write_mutex);
	u->reader_terminate = 1;
	pthread_cond_signal(&u->read_more_cond);
	pthread_mutex_unlock(&u->read_write_mutex);

	pthread_join(u->read_thread, NULL);
	pthread_cond_destroy(&u->read_done_cond);
	pthread_cond_destroy(&u->read_more_cond);
	pthread_mutex_destroy(&u->read_write_mutex);
#  ifdef INTERLACED_READ_WRITE
	pthread_mutex_destroy(&u->writer_wait_flag_mutex);
#  endif
#endif
	return u->error_rc;
}

static void h_udelay(struct libxsvf_host *h, long usecs, int tms, long num_tck)
{
	struct udata_s *u = h->user_data;
	buffer_sync(u);
	if (num_tck > 0) {
		struct timeval tv1, tv2;
		gettimeofday(&tv1, NULL);
		while (num_tck > 0) {
			buffer_add(u, tms, -1, -1, 0);
			num_tck--;
		}
		buffer_sync(u);
		gettimeofday(&tv2, NULL);
		if (tv2.tv_sec > tv1.tv_sec) {
			usecs -= (1000000 - tv1.tv_usec) + (tv2.tv_sec - tv1.tv_sec - 1) * 1000000;
			tv1.tv_usec = 0;
		}
		usecs -= tv2.tv_usec - tv1.tv_usec;
	}
	if (usecs > 0) {
		usleep(usecs);
	}
}

static int h_getbyte(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	return fgetc(u->f);
}

static int h_sync(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	buffer_sync(u);
	int rc = u->error_rc;
	u->error_rc = 0;
	return rc;
}

static int h_pulse_tck(struct libxsvf_host *h, int tms, int tdi, int tdo, int rmask, int sync)
{
	struct udata_s *u = h->user_data;
	if (u->syncmode)
		sync = 1;
	buffer_add(u, tms, tdi, tdo, rmask);
	if (sync) {
		buffer_sync(u);
		int rc = u->error_rc < 0 ? u->error_rc : u->last_tdo;
		u->error_rc = 0;
		return rc;
	}
	return u->error_rc < 0 ? u->error_rc : 1;
}

static int h_set_frequency(struct libxsvf_host *h, int v)
{
    int rc;
	struct udata_s *u = h->user_data;
	if (u->syncmode && v > 10000)
		v = 10000;
	unsigned char sethighspeed_command[] = { 0x8A, 0x97, 0x8D };
	unsigned char setfreq_command[] = { 0x86, 0x02, 0x00 };
    int div;
    if(v > 12e6)
    {
        fprintf(stderr, "Warning : Using High-Speed config, only for FT2232H and FT4232H\n");
	    write_dumpfile(1, sethighspeed_command, sizeof(sethighspeed_command), 0);
        rc = my_ftdi_write_data(u, sethighspeed_command, sizeof(sethighspeed_command), 1);
        if (rc != sizeof(sethighspeed_command)) {
            fprintf(stderr, "IO Error: Set frequency write failed: (rc=%d/%d)\n",
                    rc, (int)sizeof(sethighspeed_command));
            u->error_rc = -1;
        }
        div = fmax(ceil(60e6 / (2*v) - 1), 0);
    }
    else
	    div = fmax(ceil(12e6 / (2*v) - 1), 0);
	setfreq_command[1] = div >> 0;
	setfreq_command[2] = div >> 8;
	write_dumpfile(1, setfreq_command, sizeof(setfreq_command), 0);
	rc = my_ftdi_write_data(u, setfreq_command, sizeof(setfreq_command), 1);
	if (rc != sizeof(setfreq_command)) {
		fprintf(stderr, "IO Error: Set frequency write failed: (rc=%d/%d)\n",
				rc, (int)sizeof(setfreq_command));
		u->error_rc = -1;
	}
	return 0;
}

static void h_report_tapstate(struct libxsvf_host *h)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 2)
		printf("[%s]\n", libxsvf_state2str(h->tap_state));
}

static void h_report_device(struct libxsvf_host *h, unsigned long idcode)
{
	printf("idcode=0x%08lx, revision=0x%01lx, part=0x%04lx, manufactor=0x%03lx\n", idcode,
			(idcode >> 28) & 0xf, (idcode >> 12) & 0xffff, (idcode >> 1) & 0x7ff);
}

static void h_report_status(struct libxsvf_host *h, const char *message)
{
	struct udata_s *u = h->user_data;
	if (u->verbose >= 1)
		printf("[STATUS] %s\n", message);
}

static void h_report_error(struct libxsvf_host *h, const char *file, int line, const char *message)
{
	fprintf(stderr, "\n[%s:%d] %s\n", file, line, message);
}

static void *h_realloc(struct libxsvf_host *h, void *ptr, int size, enum libxsvf_mem which)
{
	return realloc(ptr, size);
}

static struct udata_s u = {
};

static struct libxsvf_host h = {
	.udelay = h_udelay,
	.setup = h_setup,
	.shutdown = h_shutdown,
	.getbyte = h_getbyte,
	.sync = h_sync,
	.pulse_tck = h_pulse_tck,
	.set_frequency = h_set_frequency,
	.report_tapstate = h_report_tapstate,
	.report_device = h_report_device,
	.report_status = h_report_status,
	.report_error = h_report_error,
	.realloc = h_realloc,
	.user_data = &u
};

const char *progname;

static void help()
{
	fprintf(stderr, "\n");
	fprintf(stderr, "A JTAG SVF/XSVF Player based on libxsvf for the FTDI FT232H, FT2232H and\n");
	fprintf(stderr, "FT4232H High Speed USB to Multipurpose UART/FIFO ICs.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "xsvftool-ft2232h, part of Lib(X)SVF (http://www.clifford.at/libxsvf/).\n");
	fprintf(stderr, "Copyright (C) 2009  RIEGL Research ForschungsGmbH\n");
	fprintf(stderr, "Copyright (C) 2009  Clifford Wolf <clifford@clifford.at>\n");
	fprintf(stderr, "Lib(X)SVF is free software licensed under the ISC license.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [ -v[v..] ] [ -d dumpfile ] [ -p ] [ -L | -B ] [ -S ] [ -F ] \\\n", progname);
	fprintf(stderr, "      %*s [ -f freq[k|M] ] { -s svf-file | -x xsvf-file | -c } ...\n", (int)(strlen(progname)+1), "");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -p\n");
	fprintf(stderr, "          Show progress\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -n\n");
	fprintf(stderr, "          Do not try to enable JTAG access\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -v\n");
	fprintf(stderr, "          Enable verbose output (repeat for incrased verbosity)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -P PID\n");
	fprintf(stderr, "          PID value for USB FTDI device\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -U VID\n");
	fprintf(stderr, "          VID value for USB FTDI device\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -J jtagport\n");
	fprintf(stderr, "          JTAG Port Name, by default '%s'\n", jtag_port_name);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -J jtagportnum\n");
	fprintf(stderr, "          JTAG Port position in -l list, overrides '-J' parameter\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -C ctrlport\n");
	fprintf(stderr, "          JTAG Control Port Name, by default '%s'\n", control_port_name);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -M bitmask\n");
	fprintf(stderr, "          JTAG Control bit mask, by default %02X\n", control_mask);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -V enable_value\n");
	fprintf(stderr, "          JTAG Control bit enable, by default %02X\n", control_value);
	fprintf(stderr, "\n");
	fprintf(stderr, "   -l\n");
	fprintf(stderr, "       List FTDI device names\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -d dumpfile\n");
	fprintf(stderr, "          Write a logfile of all MPSSE comunication\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -L, -B\n");
	fprintf(stderr, "          Print RMASK bits as hex value (little or big endian)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -S\n");
	fprintf(stderr, "          Run in synchronous mode (slow but report errors right away)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -F\n");
	fprintf(stderr, "          Force mode (ignore all TDO mismatches)\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -f freq[k|M]\n");
	fprintf(stderr, "          Set maximum frequency in Hz, kHz or MHz\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -s svf-file\n");
	fprintf(stderr, "          Play the specified SVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -x xsvf-file\n");
	fprintf(stderr, "          Play the specified XSVF file\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "   -c\n");
	fprintf(stderr, "          List devices in JTAG chain\n");
	fprintf(stderr, "\n");
	exit(1);
}

int main(int argc, char **argv)
{
	int rc = 0;
	int gotaction = 0;
	int hex_mode = 0;
	int opt, i, j;
    time_t start = time(NULL);

	progname = argc >= 1 ? argv[0] : "xsvftool-play";
	while ((opt = getopt(argc, argv, "npvlP:U:J:j:C:M:V:d:LBSFf:x:s:c")) != -1)
	{
		switch (opt)
		{
        case 'n':
            u.no_enable_jtag = 1;
            break;
		case 'v':
			u.verbose++;
			break;
        case 'l':
            gotaction = 1;
            listFTDI();
            break;
		case 'd':
			if (!strcmp(optarg, "-"))
				dumpfile = stdout;
			else
				dumpfile = fopen(optarg, "w");
			if (!dumpfile) {
				fprintf(stderr, "Can't open dumpfile `%s': %s\n", optarg, strerror(errno));
				rc = 1;
			}
			break;
        case 'J':
            strncpy(jtag_port_name, optarg, 255);
            break;
        case 'j':
            jtag_port_pos = atoi(optarg);
            break;
        case 'C':
            strncpy(control_port_name, optarg, 255);
            break;
        case 'M':
            control_mask = strtol(optarg, NULL, 0);
            break;
        case 'V':
            control_value = strtol(optarg, NULL, 0);
            break;
        case 'p':
            u.progress=1;
            break;
		case 'f':
			u.frequency = strtol(optarg, &optarg, 10);
			while (*optarg != 0) {
				if (*optarg == 'k') {
					u.frequency *= 1000;
					optarg++;
					continue;
				}
				if (*optarg == 'M') {
					u.frequency *= 1000000;
					optarg++;
					continue;
				}
				if (optarg[0] == 'H' && optarg[1] == 'z') {
					optarg += 2;
					continue;
				}
				help();
			}
			break;
		case 'x':
		case 's':
			gotaction = 1;
			if (!strcmp(optarg, "-"))
				u.f = stdin;
			else
				u.f = fopen(optarg, "rb");
			if (u.f == NULL) {
				fprintf(stderr, "Can't open %s file `%s': %s\n", opt == 's' ? "SVF" : "XSVF", optarg, strerror(errno));
				rc = 1;
				break;
			}
            {
                struct stat _s;
                u.filesize = 0;
                if(stat(optarg, &_s) < 0)
                    fprintf(stderr, "Failed to stat file %m\n");
                else
                    u.filesize = _s.st_size;
            }
			if (libxsvf_play(&h, opt == 's' ? LIBXSVF_MODE_SVF : LIBXSVF_MODE_XSVF) < 0) {
				fprintf(stderr, "Error while playing %s file `%s'.\n", opt == 's' ? "SVF" : "XSVF", optarg);
				rc = 1;
			}
			if (strcmp(optarg, "-"))
				fclose(u.f);
			break;
		case 'c':
			gotaction = 1;
			int old_frequency = u.frequency;
			if (u.frequency == 0)
				u.frequency = 10000;
			if (libxsvf_play(&h, LIBXSVF_MODE_SCAN) < 0) {
				fprintf(stderr, "Error while scanning JTAG chain.\n");
				rc = 1;
			}
			u.frequency = old_frequency;
			break;
		case 'L':
			hex_mode = 1;
			break;
		case 'B':
			hex_mode = 2;
			break;
		case 'S':
			if (u.frequency == 0)
				u.frequency = 10000;
			u.syncmode = 1;
			break;
		case 'F':
			u.forcemode = 1;
			break;
		case 'P':
			u.PID = strtol(optarg, NULL, 0);
#ifndef FORWIN
            if(u.PID && u.VID)
            {
                FT_SetVIDPID(u.PID, u.VID);
            }
#endif
			break;
		case 'U':
			u.VID = strtol(optarg, NULL, 0);
#ifndef FORWIN
            if(u.PID && u.VID)
            {
                FT_SetVIDPID(u.PID, u.VID);
            }
#endif
			break;
		default:
			help();
			break;
		}
	}

	if (!gotaction)
		help();

	if (u.retval_i) {
		if (hex_mode) {
			printf("0x");
			for (i=0; i < u.retval_i; i+=4) {
				int val = 0;
				for (j=i; j<i+4; j++)
					val = val << 1 | u.retval[hex_mode > 1 ? j : u.retval_i - j - 1];
				printf("%x", val);
			}
		} else {
			printf("%d rmask bits:", u.retval_i);
			for (i=0; i < u.retval_i; i++)
				printf(" %d", u.retval[i]);
		}
		printf("\n");
	}

    printf("\n\nTime : %li\n", time(NULL)-start);

	return rc;
}

