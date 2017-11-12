#include "console.h"

#include <usr/type.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <ccrf/ccrf.h>
#include <stdarg.h>

#include <virtual_com.h>
#include <fsl_device_registers.h>
#include <kio/sclk.h>
#include <math.h>
#include <ccrf/phy.h>

/**
 * Still to do:
 * - tx on/off <size> [<count>]
 * - power levels, hgm
 */

void console_printf(char *format, ...);

void command_master(bool enable, bool nosync);
void command_channel(chan_id_t chan);
void command_cw(bool cw);
void command_hgm(bool hgm);
void command_pwr(u8 pwr);
void command_send(u8 size, size_t count);

static struct __packed {
    mac_t mac;
    phy_t phy;

} console;


void console_init(mac_t mac)
{
    console.mac = mac;
    console.phy = mac_phy(mac);
}


void command_master(bool enable, bool nosync)
{
    if (phy_diag_boss(console.phy, enable, nosync)) {
        if (enable)
            console_printf("master: enabled %s\r\n", nosync ? "(no sync)" : "");
        else
            console_printf("master: disabled\r\n");
    }
}


void command_channel(chan_id_t chan)
{
    u32 freq;
    if (phy_diag_chan(console.phy, chan, &freq))
        console_printf("channel %u: %lu Hz\r\n", chan, freq);
}


void command_cw(bool cw)
{
    if (phy_diag_cw(console.phy, cw)) {
        if (cw)
            console_printf("cw enabled\r\n");
        else
            console_printf("cw disabled\r\n");
    }
}


void command_hgm(bool hgm)
{
    if (phy_diag_hgm(console.phy, hgm)) {
        if (hgm)
            console_printf("hgm enabled\r\n");
        else
            console_printf("hgm disabled\r\n");
    }
}


void command_pwr(u8 pwr)
{
    u8 pwr_mapped = (u8) ((PHY_PWR_MAX * (float)pwr + (100>>1)) / 100);
    if (phy_diag_pwr(console.phy, pwr_mapped)) {
        console_printf("power level: %u%%\r\n", pwr);
    }
}


void command_send(u8 size, size_t count)
{
    u8 data[size];
    memset(data, 0xFA, size);

    console_printf("sending %lu bytes in %lu packets...\r\n", size * count, count);

    while (count--)
        mac_send(console.mac, MAC_SEND_STRM, 0xFFFE, size, data, false);

    console_printf("send complete.\r\n");

}


void console_printf(char *format, ...)
{
    va_list va;
    u8 *output;
    int result;

    va_start(va, format);
    result = vasprintf((char **)&output, format, va);

    if (result >= 0) {
        usb_write_raw(1, output, (size_t) result);
        free(output);
    }

    va_end(va);
}


void console_input(char *data)
{
    assert(data);

    char *cmd;
    char *saveptr;
    cmd = strtok_r(data, " ", &saveptr);

    if (!cmd) goto parse_done;

    if (!strcmp(cmd, "master")) {
        char *cmd_boss = strtok_r(NULL, " ", &saveptr);

        if (cmd_boss) {
            if (!strcmp(cmd_boss, "enable")) {
                char *cmd_nosync = strtok_r(NULL, " ", &saveptr);
                bool nosync = false;

                if (cmd_nosync) {
                    if (!strcmp(cmd_nosync, "nosync")) {
                        nosync = true;
                    } else if (strlen(cmd_nosync)) {
                        goto parse_usage_master;
                    }
                }

                command_master(true, nosync);

                goto parse_done;
            }

            if (!strcmp(cmd_boss, "disable")) {
                command_master(false, false);
                goto parse_done;
            }

        }

        goto parse_usage_master;
    }

    if (!strcmp(cmd, "cw")) {
        char *cmd_cw = strtok_r(NULL, " ", &saveptr);

        if (cmd_cw) {
            if (!strcmp(cmd_cw, "on")) {
                command_cw(true);
                goto parse_done;
            }

            if (!strcmp(cmd_cw, "off")) {
                command_cw(false);
                goto parse_done;
            }

        }

        goto parse_usage_cw;
    }

    if (!strcmp(cmd, "chan")) {
        char *cmd_chan = strtok_r(NULL, " ", &saveptr);

        if (cmd_chan) {
            unsigned long chn_input = strtoul(cmd_chan, NULL, 10);

            if (chn_input >= 0 && chn_input < 25) {
                command_channel((chan_id_t) chn_input);
                goto parse_done;
            }
        }

        goto parse_usage_chan;
    }

    if (!strcmp(cmd, "hgm")) {
        char *cmd_hgm = strtok_r(NULL, " ", &saveptr);

        if (cmd_hgm) {
            if (!strcmp(cmd_hgm, "on")) {
                command_hgm(true);
                goto parse_done;
            }

            if (!strcmp(cmd_hgm, "off")) {
                command_hgm(false);
                goto parse_done;
            }

        }

        goto parse_usage_hgm;
    }

    if (!strcmp(cmd, "pwr")) {
        char *cmd_pwr = strtok_r(NULL, " ", &saveptr);

        if (cmd_pwr) {
            unsigned long pwr_input = strtoul(cmd_pwr, NULL, 10);

            if (pwr_input >= 0 && pwr_input <= 100) {
                command_pwr((u8) pwr_input);
                goto parse_done;
            }
        }

        goto parse_usage_pwr;
    }

    if (!strcmp(cmd, "send")) {
        unsigned long size_input, count_input;
        char *cmd_send = strtok_r(NULL, " ", &saveptr);

        if (cmd_send) {
            size_input = strtoul(cmd_send, NULL, 10);

            if (size_input > MAC_PKT_SIZE_MAX) {
                goto parse_usage_send;
            }
        } else {
            goto parse_usage_send;
        }

        cmd_send = strtok_r(NULL, " ", &saveptr);

        if (cmd_send) {
            count_input = strtoul(cmd_send, NULL, 10);

            if (!count_input) {
                goto parse_usage_send;
            }
        } else {
            goto parse_usage_send;
        }

        command_send((u8) size_input, (size_t) count_input);

        goto parse_done;
    }

    if (!strcmp(cmd, "help")) {
        console_printf("available commands:\r\n  master      set master (hop) mode\r\n  chan        select channel\r\n  cw          continuous wave transmit\r\n  hgm         high-gain mode toggle\r\n  pwr         power level set\r\n  send        transmit packets\r\n  info        show params/stats\r\n  reboot      restart system\r\n");
        goto parse_done;
    }

    if (!strcmp(cmd, "info")) {
        char *cmd_info = strtok_r(NULL, " ", &saveptr);

        if (cmd_info) {
            if (!strcmp(cmd_info, "chan")) {
                chan_id_t hop_table[PHY_CHAN_COUNT];
                phy_hops(console.phy, hop_table);

                for (size_t i = 0; i < PHY_CHAN_COUNT; ++i) {
                    console_printf(" %02u    %lu Hz\r\n", hop_table[i], phy_freq(console.phy, hop_table[i]));
                }

                goto parse_done;
            } else if (strlen(cmd_info)) {
                goto parse_usage_info;
            }
        }

        u32 sec = SCLK_MSEC(sclk_time()) / 1000;
        u32 min = sec / 60;
        sec %= 60;

        u8 pwr_pct = (u8) ((100 * (float)phy_pwr(console.phy) + (PHY_PWR_MAX>>1)) / PHY_PWR_MAX);

        console_printf("  address:  %02X.%04X\r\n", phy_cell(console.phy), mac_addr(console.mac));
        console_printf("   uptime:  %lum%lus\r\n", min, sec);

        if (phy_sync(console.phy))
            console_printf("  channel:  (hopping)\r\n");
        else {
            u32 freq;
            chan_id_t chan = phy_chan(console.phy, &freq);
            console_printf("  channel:  %02u %lu Hz\r\n", chan, freq);
        }

        console_printf("    power:  %u%%\r\n", pwr_pct);
        console_printf("      hgm:  %s\r\n", phy_hgm(console.phy) ? "on" : "off");

        phy_stat_t phy_stats;

        phy_stat(console.phy, &phy_stats);

        console_printf(
                "    stats:  rx=%lu/%lu/%lu  tx=%lu/%lu/%lu\r\n",
                phy_stats.rx.count, phy_stats.rx.bytes, phy_stats.rx.errors,
                phy_stats.tx.count, phy_stats.tx.bytes, phy_stats.tx.errors
        );

        goto parse_done;
    }

    if (!strcmp(cmd, "reboot")) {
        console_printf("rebooting...\r\n");
        NVIC_SystemReset();
        goto parse_done;
    }

    console_printf("unknown command: %s\r\n", cmd);

    goto parse_done;

    parse_usage_master:
    console_printf("usage: master <enable [nosync] | disable>\r\n");
    goto parse_done;

    parse_usage_cw:
    console_printf("usage: cw <on | off>\r\n");
    goto parse_done;

    parse_usage_chan:
    console_printf("usage: chan <0-24>\r\n");
    goto parse_done;

    parse_usage_hgm:
    console_printf("usage: hgm <on | off>\r\n");
    goto parse_done;

    parse_usage_pwr:
    console_printf("usage: pwr <0-100>\r\n");
    goto parse_done;

    parse_usage_send:
    console_printf("usage: send <size:1-%u> <count>\r\n", MAC_PKT_SIZE_MAX);
    goto parse_done;

    parse_usage_info:
    console_printf("usage: info [chan]\r\n");
    goto parse_done;


parse_done:
    console_printf("> ");
}
