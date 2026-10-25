// ModBee GSM.
//
// Licensed under the GNU Affero General Public License, Version 3, WITH
// ADDITIONAL TERMS - see LICENSE. Commercial use requires a separate licence.
//
// Deliberately no SPDX-License-Identifier: the additional terms mean this is
// not plain AGPL-3.0, and stamping that identifier on it would tell automated
// tooling the opposite of what LICENSE says.

// The whole library, end to end, on a host.
//
//     ./socket_on_a_host <device> <host> <port>
//     ./socket_on_a_host /dev/ttyUSB0 example.com 80
//
// Brings a modem up, opens a TCP socket to a web server, sends a minimal HTTP
// HEAD request and prints the response headers that come back. The host has
// to be reachable from the internet: the socket is opened by the modem, over
// cellular, not by this machine.
// The device is any byte channel the modem is on - a serial port, a USB modem,
// a pseudo-terminal.
//
// Nothing here names a module: the driver is chosen from what the modem says
// about itself, and the same source runs unchanged on a board.

#include <ModBeeGsm.h>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv) {
    if (argc < 4) {
        std::printf("usage: %s <device> <host> <port>\n", argv[0]);
        return 2;
    }

    const char* device = argv[1];
    const char* host = argv[2];
    const uint16_t port = (uint16_t)std::atoi(argv[3]);

    GsmSerialTransport link;
    link.configure(device, 115200);

    GsmDevice gsm;
    gsm.apn("internet");

    if (!gsm.begin(&link)) {
        std::printf("could not open %s: %s\n", device, link.lastError().c_str());
        return 1;
    }

    // Bring-up. NOTHING BLOCKS: this loop is the application's own, and the
    // library only ever does work inside poll().
    std::printf("bringing up...\n");
    const gsm_time_t deadline = (gsm_time_t)(gsm_millis() + 60000);
    while (!gsm.isReady() && gsm_isFuture(gsm_millis(), deadline)) {
        gsm.poll();
    }
    if (!gsm.isReady()) {
        std::printf("not ready: %s (%s)\n", gsm.stateName(),
                    gsm.lastError().describe());
        return 1;
    }

    // isReady() means the bearer is up. The address and the operator are
    // asked for at that moment and answer a few polls later, so give them a
    // beat before printing - they are diagnostics, not preconditions.
    const gsm_time_t settled = (gsm_time_t)(gsm_millis() + 2000);
    while (gsm_isFuture(gsm_millis(), settled)) { gsm.poll(); }

    std::printf("module   %s\n", gsm.module());
    std::printf("operator %s\n", gsm.operatorName());
    std::printf("address  %s\n", gsm.ipAddress());
    std::printf("sockets  %u of %u free\n",
                (unsigned)gsm.socketsAvailable(), (unsigned)gsm.sockets().slots());

    GsmSocket* socket = gsm.openTcp(host, port);
    if (socket == nullptr) { std::printf("no socket available\n"); return 1; }

    // Starts the open and RETURNS. state() says how it is going.
    socket->begin();
    const gsm_time_t openBy = (gsm_time_t)(gsm_millis() + 30000);
    while (socket->state() == GsmSocketState::Opening &&
           gsm_isFuture(gsm_millis(), openBy)) {
        gsm.poll();
    }
    if (socket->state() != GsmSocketState::Open) {
        std::printf("open failed: %s\n", socket->lastError().describe());
        return 1;
    }
    std::printf("connected to %s:%u\n", host, (unsigned)port);

    // HTTP/1.0 with Connection: close, so the server closes once it has
    // answered and peerClosed() says when the reply is complete.
    const std::string request = std::string("HEAD / HTTP/1.0\r\nHost: ") + host +
                                "\r\nConnection: close\r\n\r\n";
    socket->write((const uint8_t*)request.data(), request.size());

    // Drain the reply until the server closes, or for ten seconds at most.
    const gsm_time_t until = (gsm_time_t)(gsm_millis() + 10000);
    std::string reply;
    while (gsm_isFuture(gsm_millis(), until) && !socket->peerClosed()) {
        gsm.poll();
        uint8_t buffer[256];
        const size_t taken = socket->read(buffer, sizeof buffer);
        if (taken > 0) { reply.append((const char*)buffer, taken); }
    }

    std::printf("received %zu bytes: %s\n", reply.size(), reply.c_str());
    socket->end();
    gsm.end();
    return reply.empty() ? 1 : 0;
}
