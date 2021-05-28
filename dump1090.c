// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// dump1090.c: main program & miscellany
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
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

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"
#include "sdr_hackrf.h"
#include "cpu.h"

#include <stdarg.h>

struct _Modes Modes;

//
// ============================= Utility functions ==========================
//

static void sigintHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
}

static void sigtermHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGTERM, SIG_DFL); // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
}

void receiverPositionChanged(float lat, float lon, float alt)
{
    /* nothing */
    (void) lat;
    (void) lon;
    (void) alt;
}

//
// =============================== Initialization ===========================
//
static void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(&Modes, 0, sizeof(Modes));

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.gain                    = MODES_MAX_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.check_crc               = 1;
    Modes.fix_df                  = 1;
    Modes.net_heartbeat_interval  = MODES_NET_HEARTBEAT_INTERVAL;
    Modes.json_interval           = 1000;
    Modes.json_stats_interval     = 60000;
    Modes.json_location_accuracy  = 1;
    Modes.mode_ac_auto            = 1;

    Modes.sdr_type = SDR_HACKRF;
    hackRFInitConfig();
}
//
//=========================================================================
//
static void modesInit(void) {
    int i;

    Modes.sample_rate = 2400000.0;

    // Allocate the various buffers used by Modes
    Modes.trailing_samples = (MODES_PREAMBLE_US + MODES_LONG_MSG_BITS + 16) * 1e-6 * Modes.sample_rate;

    if ( ((Modes.log10lut   = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) )
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    if (!fifo_create(MODES_MAG_BUFFERS, MODES_MAG_BUF_SAMPLES + Modes.trailing_samples, Modes.trailing_samples)) {
        fprintf(stderr, "Out of memory allocating FIFO\n");
        exit(1);
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block
    if (Modes.net_output_flush_size > (MODES_OUT_FLUSH_SIZE))
      {Modes.net_output_flush_size = MODES_OUT_FLUSH_SIZE;}
    if (Modes.net_output_flush_interval > (MODES_OUT_FLUSH_INTERVAL))
      {Modes.net_output_flush_interval = MODES_OUT_FLUSH_INTERVAL;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // Prepare the log10 lookup table: 100log10(x)
    Modes.log10lut[0] = 0; // poorly defined..
    for (i = 1; i <= 65535; i++) {
        Modes.log10lut[i] = (uint16_t) round(100.0 * log10(i));
    }

    // Prepare error correction tables
    modesChecksumInit(Modes.nfix_crc);
    icaoFilterInit();
    modeACInit();

    if (Modes.show_only)
        icaoFilterAdd(Modes.show_only);
}

//
//=========================================================================
//
// We use a thread reading data in background, while the main thread
// handles decoding and visualization of data to the user.
//
// The reading thread calls the RTLSDR API to read data asynchronously, and
// uses a callback to populate the data buffer.
//
// A Mutex is used to avoid races with the decoding thread.
//

//
//=========================================================================
//
// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition
//

static void *readerThreadEntryPoint(void *arg)
{
    MODES_NOTUSED(arg);

    hackRFRun();

    if (!Modes.exit)
        Modes.exit = 2; // unexpected exit

    fifo_halt(); // wakes the main thread, if it's still waiting
    return NULL;
}
//
// ================================ Main ====================================
//

// int main() {
//     // Set sane defaults
//     modesInitConfig();

//     // signal handlers:
//     signal(SIGINT, sigintHandler);
//     signal(SIGTERM, sigtermHandler);

//     if (Modes.nfix_crc > MODES_MAX_BITERRORS)
//         Modes.nfix_crc = MODES_MAX_BITERRORS;

//     // Initialization
//     modesInit();

//     if (!hackRFOpen()) {
//         exit(1);
//     }

//     // Create the thread that will read the data from the device.
//     pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);

//     while (!Modes.exit) {
//         // get the next sample buffer off the FIFO; wait only up to 100ms
//         // this is fairly aggressive as all our network I/O runs out of the background work!
//         struct mag_buf *buf = fifo_dequeue(100 /* milliseconds */);

//         if (buf) {
//             // Process one buffer
//             demodulate2400(buf);
//             if (Modes.mode_ac)
//                 demodulate2400AC(buf);

//             // Return the buffer to the FIFO freelist for reuse
//             fifo_release(buf);
//         }
//     }

//     fifo_halt(); // Reader thread should do this anyway, but just in case..
//     pthread_join(Modes.reader_thread,NULL);     // Wait on reader thread exit

//     hackRFClose();
//     fifo_destroy();

//     if (Modes.exit == 1)
//         return 0;
//     else
//         return 1;
// }
//
//=========================================================================
//

int start_config(void);
int start_receive(void);
int stop_receive(void);

int start_config() {
    // Set sane defaults
    modesInitConfig();

    // signal handlers:
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigtermHandler);

    if (Modes.nfix_crc > MODES_MAX_BITERRORS)
        Modes.nfix_crc = MODES_MAX_BITERRORS;

    // Initialization
    modesInit();

    if (!hackRFOpen())
        return 1;

    // Create the thread that will read the data from the device.
    pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);

    return 0;
}

int start_receive() {
    while (!Modes.exit) {
        // get the next sample buffer off the FIFO; wait only up to 100ms
        // this is fairly aggressive as all our network I/O runs out of the background work!
        struct mag_buf *buf = fifo_dequeue(100 /* milliseconds */);

        if (buf) {
            // Process one buffer
            demodulate2400(buf);
            if (Modes.mode_ac)
                demodulate2400AC(buf);

            // Return the buffer to the FIFO freelist for reuse
            fifo_release(buf);

        }
    }
    return 0;
}

int stop_receive() {
    fifo_halt();                             // Reader thread should do this anyway, but just in case..
    pthread_join(Modes.reader_thread, NULL); // Wait on reader thread exit

    hackRFClose();
    fifo_destroy();

    if (Modes.exit == 1)
        return 0;
    else
        return 1;
}