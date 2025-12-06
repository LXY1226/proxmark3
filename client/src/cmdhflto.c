//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
// LTO-CM commands
// LTO Cartridge memory
//-----------------------------------------------------------------------------
#include "cmdhflto.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "cliparser.h"
#include "cmdhf14a.h"
#include "cmdparser.h"  // command_t
#include "cmdtrace.h"
#include "commonutil.h"  // ARRAYLEN
#include "comms.h"
#include "crc16.h"
#include "fileutils.h"  // saveFile
#include "protocols.h"
#include "ui.h"
/*
  iceman notes
  We can't dump LTO 5 or 6 tags yet since we don't have a datasheet.
  If you have access to datasheet,  le me know!

  CM size  xx field indicate size in units of 1024b

  LTO w Type info 00 01   has 101 blocks.
  LTO w Type info 00 02   has  95 blocks.
  LTO w Type info 00 03   has 255 blocks.
  LTO w Type info 00 xx   has NN blocks.
*/
#define CM_MEM_MAX_SIZE 0x7FE0  // (32byte/block * 1023block = 32736byte)

// todo: vendor mapping table..

// structure and database for uid -> tagtype lookups
typedef struct cm_page_s {
    uint16_t pageid;
    uint16_t len;
    const char *name;
    const char *desc;
} cm_page_t;

static const cm_page_t cm_page_map[] = {
    {0x001, 64,   "Cartridge Manufacture's information",  ""                                                                                               },
    {0x002, 64,   "Media Manufacture's information",      ""                                                                                               },
    {0x101, 64,   "Initialisation Data",                  ""                                                                                               },
    {0x102, 48,   "Tape Write Data",                      ""                                                                                               },
    {0x103, 1552, "Tape Directory",                       ""                                                                                               },
    {0x104, 64,   "EOD Information",                      ""                                                                                               },
    {0x105, 32,   "Cartidge Status and Tape Alert Flags", ""                                                                                               },
    {0x106, 384,  "Mechanism Related",                    ""                                                                                               },
    {0x107, 128,  "Suspended Append Writes",              ""                                                                                               },
    {0x108, 64,   "Usage Information 0",                  ""                                                                                               },
    {0x109, 64,   "Usage Information 1",                  ""                                                                                               },
    {0x10A, 64,   "Usage Information 2",                  ""                                                                                               },
    {0x10B, 64,   "Usage Information 3",                  ""                                                                                               },
    {0x200, 1056, "Application Specific",                 ""                                                                                               },
    {0x701, 0x40, "CM Header",                            ""                                                                                               },
    {0xFFC, 0,    "Pad",                                  "Used to reserve space for future Pages, and to align some Pages to 16-byte / 32-byte boundaries"},
    {0xFFD, 0,    "Defect",                               "Used to indicate that the LTO CM contains defective memory locations in that area"              },
    {0xFFE, 0,    "Empty",                                "Indicates an empty table"                                                                       },
    {0xFFF, 0,    "End Of Page Table",                    "End Of Page Table"                                                                              },
    {0x000, 0,    "unknown page",                         ""                                                                                               }  // must be the last entry
};

/*
static uint16_t get_page_len(uint16_t pageid) {
    for (uint8_t i = 0; i < ARRAYLEN(cm_page_map ); ++i) {
        if (pageid == cm_page_map[i].pageid) {
            return cm_page_map[i].len;
        }
    }
    //No match, return default
    return 0;
}
*/

static const char *get_page_name(uint16_t pageid) {
    for (uint8_t i = 0; i < ARRAYLEN(cm_page_map); ++i) {
        if (pageid == cm_page_map[i].pageid) {
            return cm_page_map[i].name;
        }
    }
    // No match, return default
    return cm_page_map[ARRAYLEN(cm_page_map) - 1].name;
}

static int CmdHelp(const char *Cmd);

static void lto_switch_off_field(void) {
    SetISODEPState(ISODEP_INACTIVE);
    SendCommandMIX(CMD_HF_ISO14443A_READER, 0, 0, 0, NULL, 0);
}

static void lto_switch_on_field(void) {
    SendCommandMIX(CMD_HF_ISO14443A_READER, ISO14A_CONNECT | ISO14A_NO_SELECT | ISO14A_NO_DISCONNECT | ISO14A_NO_RATS, 0, 0, NULL, 0);
}

// send a raw LTO-CM command, returns the length of the response (0 in case of error)
static int lto_send_cmd_raw(uint8_t *cmd, uint8_t len, uint8_t *response, uint16_t *response_len, bool addcrc, bool is7bits, bool verbose) {
    uint64_t arg0 = ISO14A_RAW | ISO14A_NO_DISCONNECT | ISO14A_NO_RATS;
    uint32_t arg1;

    if (addcrc) {
        arg0 |= ISO14A_APPEND_CRC;
    }

    if (is7bits) {
        arg1 = 7 << 16;
    } else {
        arg1 = 0;
    }

    arg1 |= len;

    SendCommandMIX(CMD_HF_ISO14443A_READER, arg0, arg1, 0, cmd, len);
    PacketResponseNG resp;

    if (WaitForResponseTimeout(CMD_ACK, &resp, 1500) == false) {
        if (verbose) PrintAndLogEx(WARNING, "timeout while waiting for reply");
        return PM3_ETIMEOUT;
    }

    if (resp.oldarg[0] == *response_len) {
        *response_len = resp.oldarg[0];
        if (*response_len > 0) {
            memcpy(response, resp.data.asBytes, *response_len);
        }
    } else {
        if (verbose) PrintAndLogEx(WARNING, "Wrong response length (%d != %" PRIu64 ")", *response_len, resp.oldarg[0]);
        return PM3_ESOFT;
    }

    return PM3_SUCCESS;
}

// select a LTO-CM tag. Send WUPA and RID.
static int lto_select(uint8_t *id_response, uint8_t id_len, uint8_t *type_response, bool verbose) {
    // Todo: implement anticollision

    uint8_t resp[] = {0, 0};
    uint16_t resp_len;
    uint8_t wupa_cmd[] = {LTO_REQ_STANDARD};
    uint8_t select_sn_cmd[] = {LTO_SELECT, 0x20};
    uint8_t select_cmd[] = {LTO_SELECT, 0x70, 0, 0, 0, 0, 0};

    resp_len = 2;
    int status = lto_send_cmd_raw(wupa_cmd, sizeof(wupa_cmd), type_response, &resp_len, false, true, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_ESOFT;  // WUPA failed
    }

    resp_len = id_len;
    status = lto_send_cmd_raw(select_sn_cmd, sizeof(select_sn_cmd), id_response, &resp_len, false, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_EWRONGANSWER;  // REQUEST SERIAL NUMBER failed
    }

    memcpy(select_cmd + 2, id_response, sizeof(select_cmd) - 2);
    resp_len = 1;
    status = lto_send_cmd_raw(select_cmd, sizeof(select_cmd), resp, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT || resp[0] != 0x0A) {
        return PM3_EWRONGANSWER;  // SELECT failed
    }

    // tag is now INIT and SELECTED.
    return PM3_SUCCESS;
}

static int lto_rdbl(uint8_t blk, uint8_t *block_response, uint8_t *block_cnt_response, bool verbose) {
    uint16_t resp_len = 18;
    uint8_t rdbl_cmd[] = {0x30, blk};
    uint8_t rdbl_cnt_cmd[] = {0x80};

    int status = lto_send_cmd_raw(rdbl_cmd, sizeof(rdbl_cmd), block_response, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_EWRONGANSWER;  // READ BLOCK failed
    }

    status = lto_send_cmd_raw(rdbl_cnt_cmd, sizeof(rdbl_cnt_cmd), block_cnt_response, &resp_len, false, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_EWRONGANSWER;  // READ BLOCK CONTINUE failed
    }

    return PM3_SUCCESS;
}

static int lto_rdbl_ext(uint16_t blk, uint8_t *block_response, uint8_t *block_cnt_response, bool verbose) {
    uint16_t resp_len = 18;
    uint8_t rdbl_ext_cmd[] = {0x21, blk & 0xFF, (blk >> 8) & 0xFF};
    uint8_t rdbl_cnt_cmd[] = {0x80};

    int status = lto_send_cmd_raw(rdbl_ext_cmd, sizeof(rdbl_ext_cmd), block_response, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_EWRONGANSWER;  // READ BLOCK failed
    }

    status = lto_send_cmd_raw(rdbl_cnt_cmd, sizeof(rdbl_cnt_cmd), block_cnt_response, &resp_len, false, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT) {
        return PM3_EWRONGANSWER;  // READ BLOCK CONTINUE failed
    }

    return PM3_SUCCESS;
}

static int CmdHfLTOInfo(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto info",
                  "Get info from LTO tags",
                  "hf lto info");

    void *argtable[] = {
        arg_param_begin,
        arg_lit0("@", NULL, "optional - continuous reader mode"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, true);
    bool cm = arg_get_lit(ctx, 1);
    CLIParserFree(ctx);
    if (!cm) return infoLTO(true);

    int ret = PM3_SUCCESS;
    uint8_t last_serial[5] = {0};
    do {
        uint8_t serial[5] = {0};
        uint8_t serial_len = sizeof(serial);
        uint8_t type_info[2] = {0};

        lto_switch_off_field();
        lto_switch_on_field();
        clearCommandBuffer();

        ret = lto_select(serial, serial_len, type_info, false);
        if (ret != PM3_SUCCESS) {
            continue;
        }

        if (ret == PM3_SUCCESS &&
            serial[0] != last_serial[0] &&
            serial[1] != last_serial[1] &&
            serial[2] != last_serial[2] &&
            serial[3] != last_serial[3] &&
            serial[4] != last_serial[4]) {
            lto_switch_off_field();
            ret = infoLTO(true);
            if (ret == PM3_SUCCESS) {
                last_serial[0] = serial[0];
                last_serial[1] = serial[1];
                last_serial[2] = serial[2];
                last_serial[3] = serial[3];
                last_serial[4] = serial[4];
            }
        }

    } while (kbd_enter_pressed() == false);
    return PM3_SUCCESS;
}

// common pages
//  - pad page
//  - defect page
// Unprotected pages
//  - initialisation data (64b)
//  - Cartridge Status and Tape Alert Flags  (64b)
//  - Usage Information (4*64b , 256b)
//  - Tape Write Pass (48b)
//  - Tape Directory (16*xx) (max 1536b)
//  - EOD Information (64b)
//  - Mechanism Related (384b)
//  - Application Specific Data (1056b)
//  - Suspended Append Writes (128b)

static int CmdHFLTOReader(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto reader",
                  "Act as a LTO-CM reader. Look for LTO-CM tags until Enter or the pm3 button is pressed",
                  "hf lto reader -@   -> continuous reader mode");

    void *argtable[] = {
        arg_param_begin,
        arg_lit0("@", NULL, "optional - continuous reader mode"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, true);
    bool cm = arg_get_lit(ctx, 1);
    CLIParserFree(ctx);

    if (cm) {
        PrintAndLogEx(INFO, "Press " _GREEN_("<Enter>") " to exit");
    }

    return reader_lto(cm, true);
}

int reader_lto(bool loop, bool verbose) {
    int ret = PM3_SUCCESS;

    do {
        uint8_t serial[5] = {0};
        uint8_t serial_len = sizeof(serial);
        uint8_t type_info[2] = {0};

        lto_switch_off_field();
        lto_switch_on_field();
        clearCommandBuffer();

        ret = lto_select(serial, serial_len, type_info, verbose);
        if (loop) {
            if (ret != PM3_SUCCESS) {
                continue;
            }
        }

        if (ret == PM3_SUCCESS) {
            if (loop == false) {
                PrintAndLogEx(NORMAL, "");
            }

            PrintAndLogEx(INFO, "UID......... " _GREEN_("%s"), sprint_hex_inrow(serial, sizeof(serial)));
        }

    } while (loop && (kbd_enter_pressed() == false));

    lto_switch_off_field();
    return ret;
}

typedef struct lto_info_s {
    const char *Format;
    uint32_t nWraps, setsPerWrap, tapeDirLength, kBPerDataSet, TapeLife;
} lto_info_t;

static lto_info_t lto_info_table[] = {
    {"UNKNOWN",       -1,  -1,    -1, -1,   -1 },
    /* at_Offset = {24, 28, 36, 44, 48, 52, 54, 56, 58} */
    {"LTO-1",         48,  5500,  32, 404,  260},  // LTO-1: 16 + 16 = 32 bytes per entry (even+odd)
    {"LTO-2",         64,  8200,  28, 404,  260},  // LTO-2: 7*4 = 28 bytes per entry
    {"LTO-3",         44,  6000,  32, 1617, 260},  // LTO-3: 8*4 = 32 bytes per entry
    {"LTO-4",         56,  9500,  32, 1590, 260},  // LTO-4: 8*4 = 32 bytes per entry
    /* at_Offset = {32, 36, 44, 52, 56, 60, 62, 64, 66, 80} */
    {"LTO-5",         80,  7800,  32, 2473, 260},  // LTO-5: 8*4 = 32 bytes per entry
    {"LTO-6",         136, 7805,  32, 2473, 130},  // LTO-6: 8*4 = 32 bytes per entry
    {"LTO-7",         112, 10950, 32, 5032, 130},  // LTO-7: 8*4 = 32 bytes per entry
    {"LTO-8",         208, 11660, 32, 5032, 75 },  // LTO-8: 8*4 = 32 bytes per entry
    {"LTO-9",         280, 6770,  32, 9806, 55 },  // LTO-9: 8*4 = 32 bytes per entry
    {"Cleaning Tape", -1,  -1,    -1, -1,   -1 }
};

static uint8_t lto_get_info_by_type(uint16_t type) {
    if ((type >> 15 & 1) == 1) {
        return ARRAYLEN(lto_info_table) - 1;  // Cleaning Tape
    }
    switch (type) {
        case 1:
            return 1;  // LTO-1
        case 2:
            return 2;  // LTO-2
        case 0b100:
            return 3;  // LTO-3
        case 0b1000:
            return 4;  // LTO-4
        case 0b10000:
            return 5;  // LTO-5
        case 0b100000:
            return 6;  // LTO-6
        case 0b1000000:
            return 7;  // LTO-7
            // TODO LTO-7 Type-M
        case 0b10000000:
            return 8;  // LTO-8
        case 0b10000001:
            return 9;  // LTO-9
        default:
            return 10;
    }
}

// 读取CM数据，支持字节级offset和length（非对齐访问）
// offset: 字节偏移量
// length: 要读取的字节数
static int read_cm(uint8_t *data, uint8_t *read_status, uint16_t offset, uint16_t length) {
    if (offset + length > CM_MEM_MAX_SIZE)
        return PM3_EINVARG;
    // PrintAndLogEx(NORMAL, "READ CM offset=%x length=%u", offset, length);
    
    // 计算需要读取的块范围
    uint16_t blk_start = offset / 32;
    uint16_t blk_end = (offset + length + 31) / 32;
    
    for (uint16_t blk = blk_start; blk < blk_end; blk++) {
        if (read_status[blk] == 0) {
            int ret;
            if (blk < 255) {
                ret = lto_rdbl(blk, data + (blk * 32), data + (blk * 32) + 16, false);
            } else {
                ret = lto_rdbl_ext(blk, data + (blk * 32), data + (blk * 32) + 16, false);
            }
            if (ret != PM3_SUCCESS) {
                return ret;
            }
            read_status[blk] = 1;
        }
    }
    return PM3_SUCCESS;
}

// 宏：读取CM数据并自动处理错误
// 使用方式: READ_CM(offset_bytes, length_bytes)
// 失败时会打印错误信息并跳转到 error_exit 标签
#define READ_CM(offset, length) do { \
    if (read_cm(data, read_status, (offset), (length)) != PM3_SUCCESS) { \
        PrintAndLogEx(ERR, "Failed to read CM data at offset %u, length %u", (offset), (length)); \
        ret_val = PM3_ESOFT; \
        goto error_exit; \
    } \
} while(0)

int infoLTO(bool verbose) {
    clearCommandBuffer();
    lto_switch_on_field();

    uint8_t serial_number[5];
    uint8_t serial_len = sizeof(serial_number);
    uint8_t type_info[2];

    int ret_val = lto_select(serial_number, serial_len, type_info, verbose);
    if (verbose == false) {
        if (ret_val == PM3_SUCCESS) {
            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "UID......... " _YELLOW_("%s"), sprint_hex_inrow(serial_number, sizeof(serial_number)));
        }
        lto_switch_off_field();
        return ret_val;
    }

    if (ret_val == PM3_SUCCESS) {
        PrintAndLogEx(NORMAL, "");
        PrintAndLogEx(INFO, "--- " _CYAN_("Tag Information") " -------------------------------------");
        PrintAndLogEx(INFO, "UID......... " _YELLOW_("%s"), sprint_hex_inrow(serial_number, sizeof(serial_number)));
        PrintAndLogEx(INFO, "Type info... " _YELLOW_("%s"), sprint_hex_inrow(type_info, sizeof(type_info)));
        
        // 分配内存用于缓存所有CM数据
        uint8_t data[CM_MEM_MAX_SIZE] = {0};
        uint8_t read_status[CM_MEM_MAX_SIZE / 32] = {0};
        
        // 读取块 0-1 (LTO Cartridge Information) - 64字节
        READ_CM(0, 64);
        
        {
            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("LTO Cartridge Information") " ---------------------------");
            PrintAndLogEx(INFO, "CM Serial number... " _YELLOW_("%u"), (bytes_to_num(data, 4) & 0x0FFFFFFF));
            PrintAndLogEx(INFO, "Manufacture Id..... " _YELLOW_("%u"), (data[3] >> 4));
            PrintAndLogEx(INFO, "CM Size............ " _YELLOW_("%u") " ( 1024 x %u bytes )", data[5], data[5]);
            PrintAndLogEx(INFO, "Type............... " _YELLOW_("%s"), sprint_hex_inrow(data + 6, 2));
            PrintAndLogEx(INFO, "Manufacture info... " _YELLOW_("%s"), sprint_hex_inrow(data + 8, 24));
        }
        
        uint16_t page_sa[0x101] = {0};
#define _PageOffset(p) (p - 0x100)
        enum LTO_WELLKNOWN_PAGES {
            PageTapeDir = _PageOffset(0x103),
            PageEOD = _PageOffset(0x104),
            PageMediaUsage = _PageOffset(0x105),
            PageUsage0 = _PageOffset(0x108),
            PageUsage1 = _PageOffset(0x109),
            PageUsage2 = _PageOffset(0x10A),
            PageUsage3 = _PageOffset(0x10B),
            PageAppInfo = _PageOffset(0x200)
        };
        
        // 读取块 1 (LTO CM Protected Info) - 32字节
        READ_CM(32, 32);
        
        {
            uint8_t *d = data + 32;  // 块1从偏移32开始
            uint16_t page_len = (d[2] << 8) | d[3];

            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("LTO CM Protected Info") " -------------------------------");
            PrintAndLogEx(INFO, "Raw (32 of %u)", page_len);
            PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d, 32));
            PrintAndLogEx(INFO, "                                                           ^^^^^^^^ CRC-32");
            PrintAndLogEx(INFO, "Last write-inhibited block#... " _YELLOW_("%u"), d[0]);
            PrintAndLogEx(INFO, "Block 1 protected flag........ %s", (d[1] == 0) ? _GREEN_("uninitialised cartridge") : (d[1] == 1) ? "initialised cartridge"
                                                                                                                                    : "n/a");
            PrintAndLogEx(INFO, "Reserved for future use....... " _YELLOW_("%s"), sprint_hex_inrow(d + 2, 2));

            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("Protected Page Descriptor Table") " ---------------------");
            PrintAndLogEx(INFO, "------------------------------------------------------------------");
            PrintAndLogEx(INFO, "                    start ");
            PrintAndLogEx(INFO, "  # | ver | id  | address | name ");
            PrintAndLogEx(INFO, "----+-----+-----+---------+----------------------------------------");
            uint8_t p = 0;
            uint16_t unprot_blk_offset = 0x100;
            for (uint8_t i = 0; i < 28; i += 4) {
                uint8_t page_vs = d[i] >> 4;
                uint16_t page_id = ((d[i] & 0x0F) << 8) | d[i + 1];
                uint16_t sa = (d[i + 2] << 8 | d[i + 3]);
                PrintAndLogEx(INFO, " %02u |  %u  | %03x | 0x%04X  | %s", p, page_vs, page_id, sa, get_page_name(page_id));

                p++;
                if (page_id >= 0x100 && page_id <= 0x200)
                    page_sa[page_id - 0x100] = sa;
                if (page_id == 0xFFF) {
                    PrintAndLogEx(INFO, "---+-----+-----+---------+----------------------------------------");
                    PrintAndLogEx(INFO, "# Protected Pages found...  %u", p);
                    unprot_blk_offset = sa;
                    break;
                }
            }

            // 读取 Unprotected Page Descriptor Table
            READ_CM(unprot_blk_offset, 32);
            d = data + (unprot_blk_offset);
            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("Unprotected Page Descriptor Table") " ---------------------");
            uint16_t page_id = (d[0] << 8) | d[1];
            page_len = (d[2] << 8) | d[3];
            PrintAndLogEx(INFO, "Page id..................... " _YELLOW_("0x%04x"), page_id);
            PrintAndLogEx(INFO, "Page len.................... " _YELLOW_("%u"), page_len);
            uint16_t unprot_blk_end = unprot_blk_offset + ((d[2] << 8) | d[3]);
            PrintAndLogEx(INFO, "------------------------------------------------------------------");
            PrintAndLogEx(INFO, "                    start ");
            PrintAndLogEx(INFO, "  # | ver | id  | address | name ");
            PrintAndLogEx(INFO, "-----+-----+-----+---------+----------------------------------------");
            p = 0;
            for (uint16_t unprot_blk = unprot_blk_offset; unprot_blk < unprot_blk_end; unprot_blk+=4) {
                READ_CM(unprot_blk, 4);
                uint8_t page_vs = data[unprot_blk] >> 4;
                page_id = ((data[unprot_blk] & 0x0F) << 8) | data[unprot_blk + 1];
                uint16_t sa = (data[unprot_blk + 2] << 8 | data[unprot_blk + 3]);
                PrintAndLogEx(INFO, " %02u |  %u  | %03x | 0x%04X  | %s", p, page_vs, page_id, sa, get_page_name(page_id));

                p++;
                if (page_id >= 0x100 && page_id <= 0x200)
                    page_sa[page_id - 0x100] = sa;
                if (page_id == 0xFFF) {
                    PrintAndLogEx(INFO, "---+-----+-----+---------+----------------------------------------");
                    PrintAndLogEx(INFO, "# Unprotected Pages found...  %u", p);
                    break;
                }
            }
        }

        // 读取块 2-3 (Cartridge Manufacturer's information) - 64字节
        READ_CM(64, 64);
        
        uint8_t lto_info_idx = 0;
        {
            uint8_t *d = data + 64;  // 块2从偏移64开始
            uint16_t page_len = (d[2] << 8) | d[3];

            char man[8 + 1];
            memcpy(man, (char *)d + 4, 8);
            man[8] = 0;

            char serial[10 + 1];
            memcpy(serial, (char *)d + 12, 10);
            serial[10] = 0;

            uint16_t cart_type = (d[22] << 8) | d[23];
            lto_info_idx = lto_get_info_by_type(cart_type);
            char dom[8 + 1];
            memcpy(dom, (char *)d + 24, 8);
            dom[8] = 0;

            uint16_t tape_len = (d[32] << 8) | d[33];
            uint16_t tape_thick = (d[34] << 8) | d[35];
            uint16_t empty_reel = (d[36] << 8) | d[37];
            uint16_t hub_radius = (d[38] << 8) | d[39];
            uint16_t full_reel = (d[40] << 8) | d[41];
            uint16_t max_media_speed = (d[42] << 8) | d[43];
            char lic[4 + 1];
            memcpy(lic, (char *)d + 44, 4);
            lic[4] = 0;

            char cmuse[12 + 1];
            memcpy(cmuse, (char *)d + 48, 12);
            cmuse[12] = 0;

            //    uint32_t crc = bytes_to_num(d+60, 4);

            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("Cartridge Manufacturer's information") " --------------------------");
            PrintAndLogEx(INFO, "Raw (64 of %u)", page_len);
            PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d, 32));
            PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d + 32, 32));
            if (page_len == 64)
                PrintAndLogEx(INFO, "                                                           ^^^^^^^^ CRC-32");
            PrintAndLogEx(INFO, "Cartridge Manufacturer...... " _YELLOW_("%s"), man);
            PrintAndLogEx(INFO, "Serial number............... " _YELLOW_("%s"), serial);
            PrintAndLogEx(INFO, "Cartridge type.............. " _YELLOW_("%s%s (0x%02x)"), lto_info_table[lto_info_idx].Format, (cart_type >> 13 & 1) == 1 ? " WORM" : "", cart_type);
            PrintAndLogEx(INFO, "Date of manufacture......... " _YELLOW_("%.8s"), dom);
            PrintAndLogEx(INFO, "Tape len.................... " _YELLOW_("%um (%u)"), tape_len / 4, tape_len);
            PrintAndLogEx(INFO, "Tape thickness.............. " _YELLOW_("%u"), tape_thick);
            PrintAndLogEx(INFO, "Empty reel inertia.......... " _YELLOW_("%u") " %s", empty_reel, (empty_reel == 7270) ? "( def )" : "");
            PrintAndLogEx(INFO, "Hub radius.................. " _YELLOW_("%u") " %s", hub_radius, (hub_radius == 22528) ? "( def )" : "");
            PrintAndLogEx(INFO, "Full reel pack radius....... " _YELLOW_("%u") " / 0x%04x", full_reel, full_reel);
            PrintAndLogEx(INFO, "Maximum media speed......... " _YELLOW_("%u"), max_media_speed);
            PrintAndLogEx(INFO, "License code................ " _YELLOW_("%s"), lic);
            PrintAndLogEx(INFO, "Cartridge manufacture use... " _YELLOW_("%s"), cmuse);
        }

        // 读取块 4-5 (Media Manufacturer's information) - 64字节
        READ_CM(128, 64);
        
        {
            uint8_t *d = data + 128;  // 块4从偏移128开始
            uint16_t page_len = (d[2] << 8) | d[3];

            PrintAndLogEx(NORMAL, "");
            PrintAndLogEx(INFO, "--- " _CYAN_("Media Manufacturer's information") " --------------------------");
            PrintAndLogEx(INFO, "Raw (64 of %u)", page_len);
            PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d, 32));
            PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d + 32, 32));
            if (page_len == 64)
                PrintAndLogEx(INFO, "                                                           ^^^^^^^^ CRC-32");
            PrintAndLogEx(INFO, "Servowriter Manufacturer... " _YELLOW_("%.48s"), d + 4);
        }

        if (lto_info_idx > 0 && lto_info_idx < ARRAYLEN(lto_info_table) - 1) {
            if (page_sa[PageTapeDir] != 0) {
                uint16_t hdr_length = lto_info_idx >= 6 ? 48 : 16;  // LTO-1 to LTO-5
                // 读取 Tape Directory header - 64字节
                READ_CM(page_sa[PageTapeDir], hdr_length);

                PrintAndLogEx(NORMAL, "");
                // Determine header length based on LTO generation
                PrintAndLogEx(INFO, "--- " _CYAN_("Tape Directory") " ----------------------- %dBytes", hdr_length);
                
                uint8_t *d = data + page_sa[PageTapeDir];
                
                // Display Write Pass information for partitioned tapes (LTO-4+)
            
                if (lto_info_idx >= 6) {
                    uint32_t wp_p0 = bytes_to_num(d + 4, 4);
                    uint32_t wp_p1 = bytes_to_num(d + 8, 4);
                    uint32_t wp_p2 = bytes_to_num(d + 12, 4);
                    uint32_t wp_p3 = bytes_to_num(d + 16, 4);
                    PrintAndLogEx(INFO, "Write Pass Partition 0...... " _YELLOW_("%u"), wp_p0);
                    if (wp_p1 > 0) PrintAndLogEx(INFO, "Write Pass Partition 1...... " _YELLOW_("%u"), wp_p1);
                    if (wp_p2 > 0) PrintAndLogEx(INFO, "Write Pass Partition 2...... " _YELLOW_("%u"), wp_p2);
                    if (wp_p3 > 0) PrintAndLogEx(INFO, "Write Pass Partition 3...... " _YELLOW_("%u"), wp_p3);
                } else if (lto_info_idx >= 4) {
                    uint32_t wp_p0 = bytes_to_num(d + 4, 4);
                    uint32_t wp_p1 = bytes_to_num(d + 8, 4);
                    PrintAndLogEx(INFO, "Write Pass Partition 0...... " _YELLOW_("%u"), wp_p0);
                    if (wp_p1 > 0) PrintAndLogEx(INFO, "Write Pass Partition 1...... " _YELLOW_("%u"), wp_p1);
                }
                
                // Parse wrap entries
                uint32_t n_wraps = lto_info_table[lto_info_idx].nWraps;
                uint32_t sets_per_wrap = lto_info_table[lto_info_idx].setsPerWrap;
                // uint16_t tape_dir_entry_len = lto_info_table[lto_info_idx].tapeDirLength; // hard encoded 32

                // uint64_t dataset_written = 0;
                if (lto_info_idx > 2) {
                    PrintAndLogEx(INFO, "   # | Datasets |  FMs | Capacity ");
                    uint16_t base_offset = page_sa[PageTapeDir] + hdr_length;
                    uint32_t last_dateset_id = 0;
                    uint32_t dateset_delta = 0;
                    uint32_t full_written_sets = 0;
                    for (uint32_t wrap_idx = 0; wrap_idx < n_wraps; wrap_idx++) {
                        // 读取当前wrap的两个块 - 64字节
                        READ_CM(base_offset + (wrap_idx * 32), 32);
                        d = data + base_offset + (wrap_idx * 32);
                        // uint32_t start_block = wrap_idx * sets_per_wrap * lto_info_table[lto_info_idx].kBPerDataSet / 32;
                        // uint32_t end_block = start_block + (sets_per_wrap * lto_info_table[lto_info_idx].kBPerDataSet / 32) - 1;
                        uint32_t dataset_id = bytes_to_num(d + 4, 4);
                        if (dataset_id == 0xFFFFFFFF) {
                            continue;  // Empty Wrap
                        }
                        if (dataset_id == 0xFFFFFFFE) {
                            full_written_sets += dateset_delta;
                            continue;  // Guard Wrap
                        }
                        dateset_delta = dataset_id - last_dateset_id;
                        last_dateset_id = dataset_id;

                        PrintAndLogEx(INFO, " %3u | %8u | %4u | %6.02f%%",
                                        wrap_idx,
                                        dateset_delta, // DatasetID
                                        bytes_to_num(d + 16, 4)+bytes_to_num(d + 20, 4), // EOW_FMCnt + HOW_FMCnt
                                        (double)(dateset_delta)/(double)(sets_per_wrap)*100 // DatasetID
                                       );
                    }
                } else
                    PrintAndLogEx(NORMAL, "Wrap analysis not supported for LTO-1 and LTO-2, PR welcomed");
            }

            if (page_sa[PageEOD] != 0) {
                PrintAndLogEx(NORMAL, "");
                // 读取 EOD Information - 64字节
                READ_CM(page_sa[PageEOD], 64);
                
                uint8_t *d = data + page_sa[PageEOD];
                uint16_t page_len = (d[2] << 8) | d[3];

                PrintAndLogEx(NORMAL, "");
                PrintAndLogEx(INFO, "--- " _CYAN_("EOD Information") " ----------------------- %dBytes", page_len);
                PrintAndLogEx(INFO, "Raw (64 of %u)", page_len);
                PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d, 32));
                PrintAndLogEx(INFO, "   " _YELLOW_("%s"), sprint_hex_inrow(d + 32, 32));
                PrintAndLogEx(INFO, "Load Count.................. " _YELLOW_("%d"), bytes_to_num(d + 24, 4));
                PrintAndLogEx(INFO, "Load Count.................. " _YELLOW_("%d"), bytes_to_num(d + 28, 4));
                PrintAndLogEx(INFO, "Load Count.................. " _YELLOW_("%d"), bytes_to_num(d + 32, 4));
                PrintAndLogEx(INFO, "Load Count.................. " _YELLOW_("%d"), bytes_to_num(d + 36, 4));
            }
            
            {
                PrintAndLogEx(INFO, "--- " _CYAN_("Usage Information") " -----------------------");
                uint64_t total_write_sets, total_read_sets;
                const uint8_t start = lto_info_idx >= 5 ? 32 : 24;
                // uint64_t total_write_sets = bytes_to_num(d + start + 4, 8);
                // uint64_t total_read_sets = bytes_to_num(d + start + 12, 8);
                uint8_t latestPage = 0;
                int32_t maxLoadCount = -1;
                if (lto_info_idx >= 5) {
                    for (uint16_t o = 0; o < 4; o++) {
                        READ_CM(page_sa[PageUsage0 + o], 64);
                        uint8_t *d = data + page_sa[PageUsage0 + o];
                        int32_t thisLoadCount = bytes_to_num(d + start, 4);
                        if (thisLoadCount >= maxLoadCount) {
                            latestPage = o;
                            maxLoadCount = thisLoadCount;
                        }
                    }
                    // 确保读取最新的页面 - 64字节
                    READ_CM(page_sa[PageUsage0 + latestPage], 64);
                    uint8_t *d = data + page_sa[PageUsage0 + latestPage];
                    total_write_sets = bytes_to_num(d + start + 4, 8);
                    total_read_sets = bytes_to_num(d + start + 12, 8);
                } else {
                    for (uint16_t o = 0; o < 4; o++) {
                        READ_CM(page_sa[PageUsage0 + o], 32);
                        uint8_t *d = data + page_sa[PageUsage0 + o];
                        int32_t thisLoadCount = bytes_to_num(d + start, 4);
                        PrintAndLogEx(INFO, "#%d " _YELLOW_("%s"), o, sprint_hex_inrow(d, 32));
                        if (thisLoadCount >= maxLoadCount) {
                            latestPage = o;
                            maxLoadCount = thisLoadCount;
                        }
                    }
                    // 确保读取最新的页面 - 32字节
                    READ_CM(page_sa[PageUsage0 + latestPage], 32);
                    uint8_t *d = data + page_sa[PageUsage0 + latestPage];
                    total_write_sets = bytes_to_num(d + start + 4, 8);
                    total_read_sets = bytes_to_num(d + start + 12, 8);
                }
                uint8_t *d = data + page_sa[PageUsage0 + latestPage];
                PrintAndLogEx(INFO, "Load Count.................. " _YELLOW_("%d"), bytes_to_num(d + start, 4));
                PrintAndLogEx(INFO, "Total Write................. " _YELLOW_("%.3f TiB (%u sets)"), (double)(total_write_sets * lto_info_table[lto_info_idx].kBPerDataSet) / 1024 / 1024 / 1024, total_write_sets);
                PrintAndLogEx(INFO, "Total Read.................. " _YELLOW_("%.3f TiB (%u sets)"), (double)(total_read_sets * lto_info_table[lto_info_idx].kBPerDataSet) / 1024 / 1024 / 1024, total_read_sets);
                double fve = (double)(total_write_sets + total_read_sets) / (double)(lto_info_table[lto_info_idx].nWraps * lto_info_table[lto_info_idx].setsPerWrap);
                PrintAndLogEx(INFO, "Full volume equivalents..... " _YELLOW_("%.3f / %u FVE (%.3f%%)"), fve, lto_info_table[lto_info_idx].TapeLife, fve * 100 / lto_info_table[lto_info_idx].TapeLife);
            }
            
            if (page_sa[PageAppInfo] != 0) {
                // 读取第一个块以获取page长度 - 32字节
                READ_CM(page_sa[PageAppInfo], 32);
                
                uint8_t *app_data = data + page_sa[PageAppInfo];
                uint16_t page_len = (app_data[2] << 8) | app_data[3];
                uint16_t max_blocks_needed = (page_len + 31) / 32;

                PrintAndLogEx(NORMAL, "");
                PrintAndLogEx(INFO, "--- " _CYAN_("Application Specific") " -------------------- %dBytes", page_len);
                
                // 读取所有需要的块
                if (max_blocks_needed > 1) {
                    READ_CM(page_sa[PageAppInfo] + 32, (max_blocks_needed - 1) * 32);
                }
                
                // 解析TLV字段
                uint16_t i = 10; // TLV数据从偏移10开始
                while (i < page_len && i < (max_blocks_needed * 32 - 4)) {
                    
                    // 读取TLV头部
                    uint16_t attr_id = (app_data[i] << 8) | app_data[i + 1];
                    uint16_t attr_len = (app_data[i + 2] << 8) | app_data[i + 3];
                    
                    if (attr_id == 0xFFF || attr_len == 0) {
                        break;
                    }
                    
                    // 解析并显示TLV字段
                    uint16_t value_start = i + 4;
                    switch (attr_id) {
                        case 0x800:
                            // Application Vendor (8 bytes ASCII)
                            PrintAndLogEx(INFO, "Application Vendor.......... " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x801:
                            // Application Name (32 bytes ASCII)
                            PrintAndLogEx(INFO, "Application Name............ " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x802:
                            // Application Version (8 bytes ASCII)
                            PrintAndLogEx(INFO, "Application Version......... " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x803:
                            // User Medium Text Label (160 bytes text)
                            PrintAndLogEx(INFO, "User Medium Text Label...... " _YELLOW_("%.*s"), attr_len > 40 ? 40 : attr_len, app_data + value_start);
                            break;
                        case 0x805:
                            // Text Localization Identifier (1 byte binary)
                            PrintAndLogEx(INFO, "Text Localization ID........ " _YELLOW_("0x%02X"), app_data[value_start]);
                            break;
                        case 0x806:
                            // Barcode (32 bytes ASCII)
                            PrintAndLogEx(INFO, "Barcode..................... " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x80b:
                            // Application Format Version (16 bytes ASCII)
                            PrintAndLogEx(INFO, "Application Format Version.. " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x808:
                            // Media Pool (160 bytes text)
                            PrintAndLogEx(INFO, "Media Pool.................. " _YELLOW_("%.*s"), attr_len > 40 ? 40 : attr_len, app_data + value_start);
                            break;
                        case 0x820:
                            // Medium Globally Unique Identifier (36 bytes binary)
                            PrintAndLogEx(INFO, "Medium UUID................. " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        case 0x821:
                            // Media Pool Globally Unique Identifier (36 bytes binary)
                            PrintAndLogEx(INFO, "Media Pool UUID............. " _YELLOW_("%.*s"), attr_len, app_data + value_start);
                            break;
                        default:
                            // 未知属性ID，可选择性显示
                            // PrintAndLogEx(INFO, "Unknown attr 0x%04x...... " _YELLOW_("%.*s"), attr_id, attr_len > 32 ? 32 : attr_len, app_data + value_start);
                            break;
                    }
                    
                    i += 4 + attr_len;
                }
            }
        }

    }
    
error_exit:
    PrintAndLogEx(NORMAL, "Tag Read finished");
    lto_switch_off_field();
    #undef READ_CM
    return ret_val;
}
static int CmdHfLTOList(const char *Cmd) {
    return CmdTraceListAlias(Cmd, "hf lto", "lto -c");
}

int rdblLTO(uint16_t st_blk, uint16_t end_blk, bool verbose) {
    clearCommandBuffer();
    lto_switch_on_field();

    uint8_t serial_number[5];
    uint8_t serial_len = sizeof(serial_number);
    uint8_t type_info[2];
    int ret_val = lto_select(serial_number, serial_len, type_info, verbose);

    if (ret_val != PM3_SUCCESS) {
        lto_switch_off_field();
        return ret_val;
    }

    uint8_t block_data_d00_d15[18];
    uint8_t block_data_d16_d31[18];
    uint8_t block_data[32];

    for (uint16_t i = st_blk; i < end_blk + 1; i++) {
        if (i < 255) {
            ret_val = lto_rdbl(i, block_data_d00_d15, block_data_d16_d31, verbose);
        } else {
            ret_val = lto_rdbl_ext(i, block_data_d00_d15, block_data_d16_d31, verbose);
        }
        if (ret_val == PM3_SUCCESS) {
            memcpy(block_data, block_data_d00_d15, 16);
            memcpy(block_data + 16, block_data_d16_d31, 16);
            PrintAndLogEx(SUCCESS, "BLK %03d: " _YELLOW_("%s"), i, sprint_hex_inrow(block_data, sizeof(block_data)));
        } else {
            lto_switch_off_field();
            return ret_val;
        }
    }

    lto_switch_off_field();
    return ret_val;
}

static int CmdHfLTOReadBlock(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto rdbl",
                  "Reead blocks from LTO tag",
                  "hf lto rdbl --first 0 --last 1022");

    void *argtable[] = {
        arg_param_begin,
        arg_int0(NULL, "first", "<dec>", "The first block number to read as an integer"),
        arg_int0(NULL, "last", "<dec>", "The last block number to read as an integer"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, true);

    int startblock = arg_get_int_def(ctx, 1, 0);
    int endblock = arg_get_int_def(ctx, 2, 1022);

    CLIParserFree(ctx);

    // Validations
    if (endblock < startblock) {
        PrintAndLogEx(ERR, "First block must be less than last block");
        return PM3_EINVARG;
    }

    return rdblLTO(startblock, endblock, true);
}

static int lto_wrbl(uint8_t blk, uint8_t *data, bool verbose) {
    uint8_t resp[] = {0, 0};
    uint16_t resp_len = 1;
    uint8_t wrbl_cmd[] = {0xA0, blk};
    uint8_t wrbl_d00_d15[16];
    uint8_t wrbl_d16_d31[16];

    memcpy(wrbl_d00_d15, data, 16);
    memcpy(wrbl_d16_d31, data + 16, 16);

    int status = lto_send_cmd_raw(wrbl_cmd, sizeof(wrbl_cmd), resp, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT || resp[0] != 0x0A) {
        return PM3_EWRONGANSWER;  // WRITE BLOCK failed
    }

    status = lto_send_cmd_raw(wrbl_d00_d15, sizeof(wrbl_d00_d15), resp, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT || resp[0] != 0x0A) {
        return PM3_EWRONGANSWER;  // WRITE BLOCK failed
    }

    status = lto_send_cmd_raw(wrbl_d16_d31, sizeof(wrbl_d16_d31), resp, &resp_len, true, false, verbose);
    if (status == PM3_ETIMEOUT || status == PM3_ESOFT || resp[0] != 0x0A) {
        return PM3_EWRONGANSWER;  // WRITE BLOCK failed
    }

    return PM3_SUCCESS;
}

int wrblLTO(uint8_t blk, uint8_t *data, bool verbose) {
    clearCommandBuffer();
    lto_switch_on_field();

    uint8_t serial_number[5];
    uint8_t serial_len = sizeof(serial_number);
    uint8_t type_info[2];
    int ret_val = lto_select(serial_number, serial_len, type_info, verbose);

    if (ret_val != PM3_SUCCESS) {
        lto_switch_off_field();
        return ret_val;
    }

    ret_val = lto_wrbl(blk, data, verbose);
    lto_switch_off_field();

    if (ret_val == PM3_SUCCESS) {
        PrintAndLogEx(SUCCESS, "BLK %03d: " _YELLOW_("write success"), blk);
    } else {
        PrintAndLogEx(WARNING, "BLK %03d: write error. Maybe this is a read-only block address.", blk);
    }

    return ret_val;
}

static int CmdHfLTOWriteBlock(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto wrbl",
                  "Write data to block on LTO tag",
                  "hf lto wrbl --blk 128 -d 0001020304050607080910111213141516171819202122232425262728293031");

    void *argtable[] = {
        arg_param_begin,
        arg_str1("d", "data", "<hex>", "32 bytes of data to write (64 hex symbols, no spaces)"),
        arg_int1(NULL, "blk", "<dec>", "The  block number to write to as an integer"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, false);

    int block_data_len = 0;
    uint8_t block_data[32] = {0};

    CLIGetHexWithReturn(ctx, 1, block_data, &block_data_len);

    if (block_data_len != 32) {
        PrintAndLogEx(ERR, "Block data is incorrect length");
        CLIParserFree(ctx);
        return PM3_EINVARG;
    }

    int blk = arg_get_int_def(ctx, 2, 0);

    CLIParserFree(ctx);

    int res = wrblLTO(blk, block_data, true);
    if (res == PM3_SUCCESS)
        PrintAndLogEx(HINT, "Hint: Try `" _YELLOW_("hf lto rdbl") "` to verify");

    return res;
}

int dumpLTO(uint8_t *dump, uint16_t *blocks_read, bool verbose) {
    clearCommandBuffer();
    lto_switch_on_field();

    uint8_t serial_number[5];
    uint8_t serial_len = sizeof(serial_number);
    uint8_t type_info[2];
    int ret_val = lto_select(serial_number, serial_len, type_info, verbose);

    if (ret_val != PM3_SUCCESS) {
        lto_switch_off_field();
        return ret_val;
    }

    // read block 0
    uint8_t block_data_d00_d15[18];
    uint8_t block_data_d16_d31[18];
    ret_val = lto_rdbl(0, block_data_d00_d15, block_data_d16_d31, verbose);
    if (ret_val != PM3_SUCCESS) {
        lto_switch_off_field();
        return ret_val;
    }

    uint16_t blocks = block_data_d00_d15[5] * 32;
    if (blocks > 128) blocks--;
    PrintAndLogEx(SUCCESS, "Total %02d blocks", blocks);
    PrintAndLogEx(SUCCESS, "Found LTO tag w " _YELLOW_("%d bytes") " memory", blocks * 32);

    for (uint16_t i = 0; i < blocks; i++) {
        PrintAndLogEx(INPLACE, "...reading block %d", i);
        if (i < 255) {
            ret_val = lto_rdbl(i, block_data_d00_d15, block_data_d16_d31, verbose);
        } else {
            ret_val = lto_rdbl_ext(i, block_data_d00_d15, block_data_d16_d31, verbose);
        }
        if (ret_val == PM3_SUCCESS) {
            // remove CRCs
            memcpy(dump + i * 32, block_data_d00_d15, 16);
            memcpy(dump + (i * 32) + 16, block_data_d16_d31, 16);
        } else {
            lto_switch_off_field();
            return ret_val;
        }
        fflush(stdout);
    }
    *blocks_read = blocks;
    lto_switch_off_field();
    return ret_val;
}

static int CmdHfLTODump(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto dump",
                  "Dump data from LTO tag",
                  "hf lto dump -f myfile");

    void *argtable[] = {
        arg_param_begin,
        arg_str0("f", "file", "<fn>", "specify a filename for dumpfile"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, true);

    int fnlen = 0;
    char filename[FILE_PATH_SIZE] = {0};
    CLIParamStrToBuf(arg_get_str(ctx, 1), (uint8_t *)filename, FILE_PATH_SIZE, &fnlen);
    CLIParserFree(ctx);

    uint32_t dump_len = CM_MEM_MAX_SIZE;
    uint8_t *dump = calloc(dump_len, sizeof(uint8_t));
    if (dump == NULL) {
        PrintAndLogEx(WARNING, "Failed to allocate memory");
        return PM3_EMALLOC;
    }

    uint16_t blocks_read = 0;
    int ret_val = dumpLTO(dump, &blocks_read, true);
    PrintAndLogEx(NORMAL, "");
    if (ret_val != PM3_SUCCESS) {
        free(dump);
        return ret_val;
    }

    if (strlen(filename) == 0) {
        char *fptr = filename + snprintf(filename, sizeof(filename), "hf-lto-");
        FillFileNameByUID(fptr, dump, "-dump", 5);
    }

    pm3_save_dump(filename, dump, blocks_read * 32, jsfLto);
    free(dump);
    return PM3_SUCCESS;
}

int restoreLTO(uint8_t *dump, bool verbose) {
    clearCommandBuffer();
    lto_switch_on_field();

    uint8_t type_info[2];
    uint8_t serial_number[5];
    uint8_t serial_len = sizeof(serial_number);
    int ret_val = lto_select(serial_number, serial_len, type_info, verbose);

    if (ret_val != PM3_SUCCESS) {
        lto_switch_off_field();
        return ret_val;
    }

    uint8_t block_data[32] = {0};

    // Block address 0 and 1 are read-only
    for (uint8_t blk = 2; blk < 255; blk++) {
        memcpy(block_data, dump + (blk * 32), 32);

        ret_val = lto_wrbl(blk, block_data, verbose);

        if (ret_val == PM3_SUCCESS) {
            PrintAndLogEx(SUCCESS, "Block %03d - " _YELLOW_("write success"), blk);
        } else {
            lto_switch_off_field();
            return ret_val;
        }
    }

    lto_switch_off_field();
    return ret_val;
}

static int CmdHfLTRestore(const char *Cmd) {
    CLIParserContext *ctx;
    CLIParserInit(&ctx, "hf lto restore",
                  "Restore data from dumpfile to LTO tag",
                  "hf lto restore -f hf-lto-92C7842CFF.bin|.eml");

    void *argtable[] = {
        arg_param_begin,
        arg_str1("f", "file", "<fn>", "specify a filename for dumpfile"),
        arg_param_end};
    CLIExecWithReturn(ctx, Cmd, argtable, false);

    int fnlen = 0;
    char filename[FILE_PATH_SIZE] = {0};
    CLIParamStrToBuf(arg_get_str(ctx, 1), (uint8_t *)filename, FILE_PATH_SIZE, &fnlen);

    CLIParserFree(ctx);

    size_t dump_len = 0;
    char *lowstr = str_dup(filename);
    str_lower(lowstr);

    if (str_endswith(lowstr, ".bin")) {
        uint8_t *dump = NULL;
        if (loadFile_safe(filename, "", (void **)&dump, &dump_len) == PM3_SUCCESS) {
            restoreLTO(dump, true);
        }
        free(dump);

    } else if (str_endswith(lowstr, ".eml")) {
        uint8_t *dump = NULL;
        if (loadFileEML_safe(filename, (void **)&dump, &dump_len) == PM3_SUCCESS) {
            restoreLTO(dump, true);
        }
        free(dump);

    } else {
        PrintAndLogEx(WARNING, "Warning: invalid dump filename " _YELLOW_("%s") " to restore", filename);
    }
    free(lowstr);
    return PM3_SUCCESS;
}

static command_t CommandTable[] = {
    {"help",    CmdHelp,            AlwaysAvailable, "This help"                      },
    {"dump",    CmdHfLTODump,       IfPm3Iso14443a,  "Dump LTO-CM tag to file"        },
    {"info",    CmdHfLTOInfo,       IfPm3Iso14443a,  "Tag information"                },
    {"list",    CmdHfLTOList,       AlwaysAvailable, "List LTO-CM history"            },
    {"rdbl",    CmdHfLTOReadBlock,  IfPm3Iso14443a,  "Read block"                     },
    {"reader",  CmdHFLTOReader,     IfPm3Iso14443a,  "Act like a LTO-CM reader"       },
    {"restore", CmdHfLTRestore,     IfPm3Iso14443a,  "Restore dump file to LTO-CM tag"},
    {"wrbl",    CmdHfLTOWriteBlock, IfPm3Iso14443a,  "Write block"                    },
    {NULL,      NULL,               NULL,            NULL                             }
};

static int CmdHelp(const char *Cmd) {
    (void)Cmd;  // Cmd is not used so far
    CmdsHelp(CommandTable);
    return PM3_SUCCESS;
}

int CmdHFLTO(const char *Cmd) {
    clearCommandBuffer();
    return CmdsParse(CommandTable, Cmd);
}
