/*
 * fw_diag.c — FireWire Audio Stream Diagnostic Tool
 *
 * Captures the complete stream configuration from the Prism Sound Orpheus
 * on a WORKING setup (old laptop, macOS 11). Run while audio is playing
 * through the Orpheus to capture the live connection state.
 *
 * Build on macOS 11 (Big Sur):
 *   clang -o fw_diag fw_diag.c -framework IOKit -framework CoreFoundation
 *
 * Run:
 *   sudo ./fw_diag                      # print to terminal
 *   sudo ./fw_diag 2>&1 | tee report.txt  # save + display
 *   sudo ./fw_diag -v                   # verbose (show all devices)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/firewire/IOFireWireLib.h>
#include <IOKit/firewire/IOFireWireFamilyCommon.h>
#include <IOKit/avc/IOFireWireAVCLib.h>
#include <CoreFoundation/CoreFoundation.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define ORPHEUS_VENDOR_ID  0x001198
#define ORPHEUS_MODEL_ID   0x010048

/* CSR register offsets (IEC 61883-1) */
#define CSR_oMPR   0xF0000900u
#define CSR_oPCR0  0xF0000904u
#define CSR_oPCR1  0xF0000908u
#define CSR_iMPR   0xF0000980u
#define CSR_iPCR0  0xF0000984u
#define CSR_iPCR1  0xF0000988u

/* Prism Sound OUI */
static const UInt8 kPrismOUI[3] = { 0x00, 0x11, 0x98 };

static int g_verbose = 0;

/* ------------------------------------------------------------------ */
/*  Utility helpers                                                    */
/* ------------------------------------------------------------------ */

static void hexdump(const char *label, const UInt8 *data, UInt32 len)
{
    printf("  %s (%u bytes):", label, (unsigned)len);
    for (UInt32 i = 0; i < len; i++)
        printf(" %02X", data[i]);
    printf("\n");
}

static int get_int_prop(io_object_t obj, const char *key, UInt64 *out)
{
    CFStringRef cfkey = CFStringCreateWithCString(NULL, key, kCFStringEncodingUTF8);
    if (!cfkey) return -1;
    CFTypeRef prop = IORegistryEntryCreateCFProperty(obj, cfkey, NULL, 0);
    CFRelease(cfkey);
    if (!prop) return -1;
    int rc = -1;
    if (CFGetTypeID(prop) == CFNumberGetTypeID()) {
        CFNumberGetValue(prop, kCFNumberSInt64Type, out);
        rc = 0;
    }
    CFRelease(prop);
    return rc;
}

static void get_str_prop(io_object_t obj, const char *key, char *buf, size_t len)
{
    buf[0] = 0;
    CFStringRef cfkey = CFStringCreateWithCString(NULL, key, kCFStringEncodingUTF8);
    if (!cfkey) return;
    CFTypeRef prop = IORegistryEntryCreateCFProperty(obj, cfkey, NULL, 0);
    CFRelease(cfkey);
    if (!prop) return;
    if (CFGetTypeID(prop) == CFStringGetTypeID())
        CFStringGetCString(prop, buf, (CFIndex)len, kCFStringEncodingUTF8);
    CFRelease(prop);
}

/* ------------------------------------------------------------------ */
/*  PCR register decoders                                              */
/* ------------------------------------------------------------------ */

static void decode_oMPR(UInt32 v)
{
    printf("  oMPR raw = 0x%08X\n", v);
    printf("    data_rate        = %u  (0=S100 1=S200 2=S400)\n", (v >> 30) & 3);
    printf("    bcast_channel    = %u\n", (v >> 24) & 0x3F);
    printf("    num_output_plugs = %u\n", v & 0x1F);
}

static void decode_oPCR(UInt32 v, int idx)
{
    unsigned online  = (v >> 31) & 1;
    unsigned bcast   = (v >> 30) & 1;
    unsigned p2p     = (v >> 24) & 0x3F;
    unsigned ch      = (v >> 16) & 0x3F;
    unsigned rate    = (v >> 14) & 3;
    unsigned ovhd    = (v >> 10) & 0xF;
    unsigned payload = v & 0x3FF;

    printf("  oPCR[%d] raw = 0x%08X\n", idx, v);
    printf("    online      = %u\n", online);
    printf("    bcast_conn  = %u\n", bcast);
    printf("    p2p_conn    = %u\n", p2p);
    printf("    channel     = %u\n", ch);
    printf("    data_rate   = %u  (0=S100 1=S200 2=S400)\n", rate);
    printf("    overhead_ID = %u\n", ovhd);
    printf("    payload     = %u quadlets (%u bytes)\n", payload, payload * 4);

    /* Infer DBS from payload. payload = 2 (CIP hdr) + blocks * DBS */
    if (payload > 2) {
        unsigned data_q = payload - 2;
        printf("    --- inferred (payload - 2 CIP quadlets = %u) ---\n", data_q);
        for (int bf = 1; bf <= 16; bf++) {
            if (data_q % bf == 0)
                printf("      blocking=%d  =>  DBS=%u  (%u channels)\n",
                       bf, data_q / bf, data_q / bf);
        }
    }
}

static void decode_iMPR(UInt32 v)
{
    printf("  iMPR raw = 0x%08X\n", v);
    printf("    data_rate       = %u  (0=S100 1=S200 2=S400)\n", (v >> 30) & 3);
    printf("    num_input_plugs = %u\n", v & 0x1F);
}

static void decode_iPCR(UInt32 v, int idx)
{
    unsigned online = (v >> 31) & 1;
    unsigned bcast  = (v >> 30) & 1;
    unsigned p2p    = (v >> 24) & 0x3F;
    unsigned ch     = (v >> 16) & 0x3F;

    printf("  iPCR[%d] raw = 0x%08X\n", idx, v);
    printf("    online      = %u\n", online);
    printf("    bcast_conn  = %u\n", bcast);
    printf("    p2p_conn    = %u\n", p2p);
    printf("    channel     = %u\n", ch);
}

/* ------------------------------------------------------------------ */
/*  Phase 1 — Device discovery                                         */
/* ------------------------------------------------------------------ */

static io_service_t find_orpheus_device(void)
{
    CFMutableDictionaryRef match = IOServiceMatching("IOFireWireDevice");
    if (!match) return 0;

    io_iterator_t iter;
    if (IOServiceGetMatchingServices(kIOMasterPortDefault, match, &iter))
        return 0;

    io_service_t svc, found = 0;
    while ((svc = IOIteratorNext(iter))) {
        UInt64 vid = 0, mid = 0, guid = 0;
        char name[256] = {0};
        get_int_prop(svc, "Vendor_ID", &vid);
        get_int_prop(svc, "Model_ID", &mid);
        get_int_prop(svc, "GUID", &guid);
        get_str_prop(svc, "FireWire Product Name", name, sizeof(name));

        printf("  device: vendor=0x%06llX model=0x%06llX GUID=0x%016llX \"%s\"\n",
               vid, mid, guid, name);

        if (vid == ORPHEUS_VENDOR_ID && mid == ORPHEUS_MODEL_ID && !found)
            found = svc;
        else
            IOObjectRelease(svc);
    }
    IOObjectRelease(iter);
    return found;
}

/* ------------------------------------------------------------------ */
/*  Phase 2 — PCR register reads via IOFireWireLib                     */
/* ------------------------------------------------------------------ */

static IOFireWireLibDeviceRef open_fw_device(io_service_t svc)
{
    IOCFPlugInInterface **plug = NULL;
    SInt32 score;

    kern_return_t kr = IOCreatePlugInInterfaceForService(
        svc, kIOFireWireLibTypeID, kIOCFPlugInInterfaceID, &plug, &score);
    if (kr || !plug) {
        fprintf(stderr, "  WARN: IOCreatePlugIn for FW device failed (0x%x)\n", kr);
        return NULL;
    }

    IOFireWireLibDeviceRef dev = NULL;
    (*plug)->QueryInterface(plug,
        CFUUIDGetUUIDBytes(kIOFireWireDeviceInterfaceID), (LPVOID *)&dev);
    (*plug)->Release(plug);

    if (!dev) {
        fprintf(stderr, "  WARN: QueryInterface for FW device failed\n");
        return NULL;
    }

    IOReturn rc = (*dev)->Open(dev);
    if (rc != kIOReturnSuccess) {
        fprintf(stderr, "  WARN: Open failed (0x%x) — audio driver may hold exclusive access\n", rc);
        fprintf(stderr, "        PCR reads will be attempted anyway...\n");
    }
    return dev;
}

static int read_quadlet(IOFireWireLibDeviceRef dev, UInt32 addrLo, UInt32 *val)
{
    FWAddress addr;
    addr.nodeID    = 0;
    addr.addressHi = 0xFFFF;
    addr.addressLo = addrLo;

    IOReturn rc = (*dev)->ReadQuadlet(dev,
        (*dev)->GetDevice(dev), &addr, val, false, 0);
    return (rc == kIOReturnSuccess) ? 0 : -1;
}

static void read_all_pcr(IOFireWireLibDeviceRef dev)
{
    UInt32 v;

    printf("\n-- Output Plug Registers (Orpheus -> Mac) --\n");
    if (read_quadlet(dev, CSR_oMPR,  &v) == 0) decode_oMPR(v);
    else printf("  oMPR: read FAILED\n");
    if (read_quadlet(dev, CSR_oPCR0, &v) == 0) decode_oPCR(v, 0);
    else printf("  oPCR[0]: read FAILED\n");
    if (read_quadlet(dev, CSR_oPCR1, &v) == 0) decode_oPCR(v, 1);

    printf("\n-- Input Plug Registers (Mac -> Orpheus) --\n");
    if (read_quadlet(dev, CSR_iMPR,  &v) == 0) decode_iMPR(v);
    else printf("  iMPR: read FAILED\n");
    if (read_quadlet(dev, CSR_iPCR0, &v) == 0) decode_iPCR(v, 0);
    else printf("  iPCR[0]: read FAILED\n");
    if (read_quadlet(dev, CSR_iPCR1, &v) == 0) decode_iPCR(v, 1);
}

/* ------------------------------------------------------------------ */
/*  Phase 3 — AV/C queries via IOFireWireAVCLib                        */
/* ------------------------------------------------------------------ */

static IOFireWireAVCLibUnitInterface **open_avc_unit(io_service_t fwDev)
{
    /* Walk children of IOFireWireDevice to find IOFireWireAVCUnit */
    io_iterator_t iter;
    kern_return_t kr = IORegistryEntryCreateIterator(
        fwDev, kIOServicePlane, kIORegistryIterateRecursively, &iter);
    if (kr) return NULL;

    io_service_t child, avcSvc = 0;
    while ((child = IOIteratorNext(iter))) {
        io_name_t cls;
        IOObjectGetClass(child, cls);
        if (strcmp(cls, "IOFireWireAVCUnit") == 0 && !avcSvc)
            avcSvc = child;
        else
            IOObjectRelease(child);
    }
    IOObjectRelease(iter);

    /* Fallback: iterate ALL AVC units */
    if (!avcSvc) {
        CFMutableDictionaryRef m = IOServiceMatching("IOFireWireAVCUnit");
        if (m && IOServiceGetMatchingServices(kIOMasterPortDefault, m, &iter) == KERN_SUCCESS) {
            avcSvc = IOIteratorNext(iter);
            IOObjectRelease(iter);
        }
    }
    if (!avcSvc) {
        fprintf(stderr, "  WARN: No IOFireWireAVCUnit found\n");
        return NULL;
    }

    IOCFPlugInInterface **plug = NULL;
    SInt32 score;
    kr = IOCreatePlugInInterfaceForService(
        avcSvc, kIOFireWireAVCLibUnitTypeID, kIOCFPlugInInterfaceID, &plug, &score);
    IOObjectRelease(avcSvc);
    if (kr || !plug) {
        fprintf(stderr, "  WARN: IOCreatePlugIn for AVC failed (0x%x)\n", kr);
        return NULL;
    }

    IOFireWireAVCLibUnitInterface **avc = NULL;
    (*plug)->QueryInterface(plug,
        CFUUIDGetUUIDBytes(kIOFireWireAVCLibUnitInterfaceID), (LPVOID *)&avc);
    (*plug)->Release(plug);
    if (!avc) {
        fprintf(stderr, "  WARN: QueryInterface for AVC failed\n");
        return NULL;
    }

    IOReturn rc = (*avc)->open(avc);
    if (rc != kIOReturnSuccess)
        fprintf(stderr, "  WARN: AVC open failed (0x%x) — commands may still work\n", rc);

    return avc;
}

static int avc_cmd(IOFireWireAVCLibUnitInterface **avc,
                   const UInt8 *cmd, UInt32 cmdLen,
                   UInt8 *resp, UInt32 *respLen)
{
    IOReturn rc = (*avc)->AVCCommand(avc, cmd, cmdLen, resp, respLen);
    return (rc == kIOReturnSuccess) ? 0 : -1;
}

/* --- Standard AV/C queries --- */

static void query_plug_info(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n-- AV/C PLUG INFO --\n");
    UInt8 cmd[] = { 0x01, 0xFF, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };
    UInt8 resp[64];
    UInt32 rlen = sizeof(resp);

    if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
        hexdump("response", resp, rlen);
        if (rlen >= 8 && resp[0] == 0x0C) {
            printf("    isoch_output_plugs  = %u\n", resp[4]);
            printf("    isoch_input_plugs   = %u\n", resp[5]);
            printf("    external_out_plugs  = %u\n", resp[6]);
            printf("    external_in_plugs   = %u\n", resp[7]);
        }
    } else {
        printf("  (query failed)\n");
    }
}

static void query_subunit_info(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n-- AV/C SUBUNIT INFO --\n");
    UInt8 cmd[] = { 0x01, 0xFF, 0x31, 0x07, 0xFF, 0xFF, 0xFF, 0xFF };
    UInt8 resp[64];
    UInt32 rlen = sizeof(resp);

    if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
        hexdump("response", resp, rlen);
    } else {
        printf("  (query failed)\n");
    }
}

static void decode_plug_signal(const UInt8 *resp, UInt32 rlen)
{
    if (rlen < 8 || resp[0] != 0x0C) return;

    UInt8 b4 = resp[4], b5 = resp[5], b6 = resp[6], b7 = resp[7];
    printf("    raw signal_format: %02X %02X %02X %02X\n", b4, b5, b6, b7);

    /* AM824: FMT=0x10 → byte4 & 0xC0 should be 0x40 (FMT<<2 with evt=0)
       Actually the encoding is: byte4[7:2]=FMT, byte4[1:0]+byte5=FDF */
    UInt8 fmt = b4 >> 2;  /* top 6 bits */
    printf("    FMT = 0x%02X", fmt);
    if (fmt == 0x10) printf("  (AM824 / IEC 61883-6)");
    else if (fmt == 0x20) printf("  (IEC 61883-4 MPEG)");
    else if (fmt == 0x00) printf("  (IEC 61883-2 DV)");
    printf("\n");

    if (fmt == 0x10) {
        /* FDF for AM824: byte5 bits [3:1] = SFC */
        int sfc = (b5 >> 1) & 0x7;
        const char *rates[] = {
            "32kHz","44.1kHz","48kHz","88.2kHz",
            "96kHz","176.4kHz","192kHz","reserved"
        };
        printf("    SFC = %d (%s)\n", sfc, rates[sfc]);
        printf("    FDF raw = %02X %02X %02X\n", b5, b6, b7);
    }
}

static void query_plug_signal_format(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n-- AV/C PLUG SIGNAL FORMAT --\n");

    /* Output plugs (Orpheus -> Mac): opcode 0x18 */
    printf("Output plugs (Orpheus -> Mac):\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[] = { 0x01, 0xFF, 0x18, (UInt8)p, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            decode_plug_signal(resp, rlen);
        } else {
            printf("(failed — plug may not exist)\n");
        }
    }

    /* Input plugs (Mac -> Orpheus): opcode 0x19 */
    printf("Input plugs (Mac -> Orpheus):\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[] = { 0x01, 0xFF, 0x19, (UInt8)p, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            decode_plug_signal(resp, rlen);
        } else {
            printf("(failed — plug may not exist)\n");
        }
    }
}

/* AV/C EXTENDED STREAM FORMAT INFORMATION (opcode 0x2F) */
static void query_stream_format_single(IOFireWireAVCLibUnitInterface **avc,
                                       int direction, int plug)
{
    /*  direction: 0=input (Mac->device), 1=output (device->Mac)
     *  Subfunction 0xC0 = SINGLE (current format)
     *  [01] [FF] [2F] [C0] [dir] [00=unit_plug] [plug_id] [FF...] */
    UInt8 cmd[16];
    memset(cmd, 0xFF, sizeof(cmd));
    cmd[0] = 0x01;     /* STATUS */
    cmd[1] = 0xFF;     /* UNIT */
    cmd[2] = 0x2F;     /* STREAM FORMAT */
    cmd[3] = 0xC0;     /* SINGLE */
    cmd[4] = (UInt8)direction;
    cmd[5] = 0x00;     /* unit plug address mode */
    cmd[6] = (UInt8)plug;

    UInt8 resp[64]; UInt32 rlen = sizeof(resp);
    const char *dir = direction ? "Output" : "Input";

    printf("  %s plug %d (SINGLE): ", dir, plug);
    if (avc_cmd(avc, cmd, 16, resp, &rlen) != 0) {
        printf("(not supported or failed)\n");
        return;
    }
    hexdump("", resp, rlen);

    if (rlen < 11 || resp[0] != 0x0C) return;

    UInt8 hierRoot = resp[7];
    UInt8 hierL1   = resp[8];
    printf("    hierarchy_root = 0x%02X\n", hierRoot);
    printf("    hierarchy_l1   = 0x%02X\n", hierL1);

    /* AM824 compound: root=0x90, l1=0x40 */
    if (hierRoot == 0x90 && hierL1 == 0x40 && rlen >= 12) {
        UInt8 sfc       = resp[9];
        UInt8 rateCtl   = resp[10];
        UInt8 nEntries  = resp[11];
        const char *rates[] = {
            "32kHz","44.1kHz","48kHz","88.2kHz",
            "96kHz","176.4kHz","192kHz","reserved"
        };
        printf("    AM824 compound format\n");
        printf("    SFC          = %u (%s)\n", sfc, sfc < 7 ? rates[sfc] : "?");
        printf("    rate_control = 0x%02X\n", rateCtl);
        printf("    num_entries  = %u\n", nEntries);

        for (int i = 0; i < nEntries && (12 + i*2 + 1) < (int)rlen; i++) {
            UInt8 nch  = resp[12 + i*2];
            UInt8 fmtc = resp[12 + i*2 + 1];
            const char *fn = "unknown";
            switch (fmtc) {
                case 0x00: fn = "IEC60958-3 (AC3)"; break;
                case 0x01: fn = "IEC60958-3"; break;
                case 0x02: fn = "raw IEC60958"; break;
                case 0x06: fn = "MBLA (multi-bit linear audio)"; break;
                case 0x0D: fn = "MIDI conformant"; break;
                case 0x40: fn = "AM824 raw/compound"; break;
                case 0xFF: fn = "DONT_CARE"; break;
            }
            printf("    entry[%d]: %u ch, format=0x%02X (%s)\n", i, nch, fmtc, fn);
        }
    }
}

static void query_stream_format_list(IOFireWireAVCLibUnitInterface **avc,
                                     int direction, int plug)
{
    const char *dir = direction ? "Output" : "Input";
    printf("  %s plug %d supported formats:\n", dir, plug);

    for (int idx = 0; idx < 16; idx++) {
        UInt8 cmd[16];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01;
        cmd[1] = 0xFF;
        cmd[2] = 0x2F;
        cmd[3] = 0xC1;  /* LIST */
        cmd[4] = (UInt8)direction;
        cmd[5] = 0x00;
        cmd[6] = (UInt8)plug;
        cmd[7] = (UInt8)idx;

        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        if (avc_cmd(avc, cmd, 16, resp, &rlen) != 0)
            break;
        if (rlen < 1 || resp[0] != 0x0C)
            break;

        printf("    [%d] ", idx);
        hexdump("", resp, rlen);

        /* Decode AM824 compound entries */
        if (rlen >= 12 && resp[7] == 0x90 && resp[8] == 0x40) {
            UInt8 sfc = resp[9];
            UInt8 nEntries = resp[11];
            const char *rates[] = {
                "32kHz","44.1kHz","48kHz","88.2kHz",
                "96kHz","176.4kHz","192kHz","reserved"
            };
            printf("         AM824 SFC=%u (%s) entries=%u:",
                   sfc, sfc < 7 ? rates[sfc] : "?", nEntries);
            for (int i = 0; i < nEntries && (12 + i*2 + 1) < (int)rlen; i++)
                printf(" [%uch/0x%02X]", resp[12 + i*2], resp[12 + i*2 + 1]);
            printf("\n");
        }
    }
}

/* AV/C SIGNAL SOURCE (opcode 0x1A) — clock routing */
static void query_signal_source(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n-- AV/C SIGNAL SOURCE (clock routing) --\n");

    /* STATUS for dest plug = isoch serial bus input plug 0 */
    UInt8 cmd[] = { 0x01, 0xFF, 0x1A, 0x0F, 0xFF, 0xFE, 0x60, 0xFF };
    UInt8 resp[64]; UInt32 rlen = sizeof(resp);

    printf("  SIGNAL SOURCE STATUS: ");
    if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
        hexdump("", resp, rlen);
        if (rlen >= 8 && resp[0] == 0x0C) {
            printf("    status    = 0x%02X\n", resp[3]);
            printf("    src_plug  = 0x%02X\n", resp[4]);
            printf("    dest      = 0x%02X 0x%02X 0x%02X\n", resp[5], resp[6], resp[7]);
        }
    } else {
        printf("(failed)\n");
    }
}

/* --- Prism Sound vendor-specific queries --- */

static void query_vendor_state(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n-- PRISM SOUND VENDOR-SPECIFIC STATE --\n");

    /* Analog channel bulk reads (cmd 0xCF) */
    printf("\nAnalog channels (bulk 0xCF):\n");
    for (int ch = 0; ch < 8; ch++) {
        UInt8 cmd[15];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01; /* STATUS */
        cmd[1] = 0x08; /* subunit */
        cmd[2] = 0x00; /* VENDOR_DEPENDENT */
        cmd[3] = kPrismOUI[0]; cmd[4] = kPrismOUI[1]; cmd[5] = kPrismOUI[2];
        cmd[6] = 0xCF; /* bulk analog */
        cmd[7] = (UInt8)ch;

        UInt8 resp[32]; UInt32 rlen = sizeof(resp);
        printf("  ch%d: ", ch);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 10 && resp[0] == 0x0C) {
                UInt8 all = resp[9];
                printf("    type=0x%02X  all=0x%02X"
                       " (lineOut=%u lineIn=%u ovk=%u ms=%u phant=%u phase=%u)",
                       resp[8], all,
                       all & 1, (all>>1)&1, (all>>2)&1,
                       (all>>3)&1, (all>>5)&1, (all>>6)&1);
                if (ch <= 3 && rlen >= 12)
                    printf(" filter=%u mic=%u", resp[10], resp[11]);
                if (ch <= 1 && rlen >= 13)
                    printf(" imp=%u", resp[12]);
                printf("\n");
            }
        } else {
            printf("(failed)\n");
        }
    }

    /* Digital bulk read (cmd 0xDF) */
    printf("\nDigital state (bulk 0xDF):\n");
    {
        UInt8 cmd[15];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01; cmd[1] = 0x08; cmd[2] = 0x00;
        cmd[3] = kPrismOUI[0]; cmd[4] = kPrismOUI[1]; cmd[5] = kPrismOUI[2];
        cmd[6] = 0xDF;
        UInt8 resp[32]; UInt32 rlen = sizeof(resp);

        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("response", resp, rlen);
            if (rlen >= 12 && resp[0] == 0x0C) {
                printf("  byte7  (digitalInputType) = %u\n", resp[7]);
                printf("  byte8  (channelStatus)    = %u\n", resp[8]);
                printf("  byte9  (bitDepth)         = %u\n", resp[9]);
                printf("  byte10 (sampleRate)       = %u\n", resp[10]);
                printf("  byte11 (syncSource)       = %u\n", resp[11]);
            }
        } else {
            printf("  (failed)\n");
        }
    }

    /* Device-level settings: meter, source, bulk */
    printf("\nDevice-level settings:\n");
    static const struct { UInt8 cmd; const char *name; } devCmds[] = {
        { 0xB0, "Meter mode" },
        { 0xB1, "Source"     },
        { 0xB2, "Wordclock"  },
        { 0xB3, "ADAT"       },
        { 0xB7, "Meter brightness" },
        { 0xBF, "Bulk device state" },
    };
    for (int i = 0; i < (int)(sizeof(devCmds)/sizeof(devCmds[0])); i++) {
        UInt8 cmd[15];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01; cmd[1] = 0x08; cmd[2] = 0x00;
        cmd[3] = kPrismOUI[0]; cmd[4] = kPrismOUI[1]; cmd[5] = kPrismOUI[2];
        cmd[6] = devCmds[i].cmd;
        UInt8 resp[32]; UInt32 rlen = sizeof(resp);

        printf("  %s (0x%02X): ", devCmds[i].name, devCmds[i].cmd);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0)
            hexdump("", resp, rlen);
        else
            printf("(failed)\n");
    }
}

/* ------------------------------------------------------------------ */
/*  Phase 3b — Music Subunit Stream Format Discovery                   */
/* ------------------------------------------------------------------ */

/* The Orpheus has a Music subunit (type 0x0C, address byte 0x60).
 * BeBoB devices store their definitive stream format descriptor in the
 * Music subunit.  If unit-level Extended Stream Format returns
 * NOT_IMPLEMENTED, the Music subunit's plugs may still respond.        */

static void query_music_subunit_plugs(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n=== PHASE 3b: Music Subunit Probes ===\n");

    /* --- Music Subunit PLUG INFO --- */
    printf("\n-- Music Subunit PLUG INFO (addr 0x60) --\n");
    {
        UInt8 cmd[] = { 0x01, 0x60, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("response", resp, rlen);
            if (rlen >= 8) {
                if (resp[0] == 0x0C) {
                    printf("    source_plugs      = %u  (subunit output, feeds oPCR)\n", resp[4]);
                    printf("    dest_plugs        = %u  (subunit input, fed by iPCR)\n", resp[5]);
                    printf("    ext_source_plugs  = %u\n", resp[6]);
                    printf("    ext_dest_plugs    = %u\n", resp[7]);
                } else if (resp[0] == 0x08) {
                    printf("    NOT_IMPLEMENTED at Music subunit level\n");
                }
            }
        } else {
            printf("  (query failed)\n");
        }
    }

    /* --- Extended Stream Format for Music Subunit plugs --- */
    /* Try both source (direction 0) and dest (direction 1) plugs at
     * subunit plug address mode (0x01) */
    printf("\n-- Extended Stream Format: Music Subunit Source Plugs --\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[16];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01;  /* STATUS */
        cmd[1] = 0x60;  /* Music subunit */
        cmd[2] = 0x2F;  /* STREAM FORMAT */
        cmd[3] = 0xC0;  /* SINGLE */
        cmd[4] = 0x01;  /* addr_type: subunit plug */
        cmd[5] = 0x00;  /* direction: source (output from subunit) */
        cmd[6] = (UInt8)p; /* plug num */

        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Source plug %d: ", p);
        if (avc_cmd(avc, cmd, 16, resp, &rlen) != 0) {
            printf("(failed)\n");
            if (p == 0) break;
            continue;
        }
        hexdump("", resp, rlen);
        if (rlen >= 1 && resp[0] == 0x08)
            printf("    NOT_IMPLEMENTED\n");
        else if (rlen >= 12 && resp[0] == 0x0C && resp[7] == 0x90 && resp[8] == 0x40) {
            UInt8 sfc = resp[9], nEntries = resp[11];
            const char *rates[] = {"32k","44.1k","48k","88.2k","96k","176.4k","192k","?"};
            printf("    AM824 compound: SFC=%u (%s) entries=%u\n",
                   sfc, sfc < 7 ? rates[sfc] : "?", nEntries);
            for (int i = 0; i < nEntries && (12 + i*2 + 1) < (int)rlen; i++) {
                UInt8 nch = resp[12 + i*2], fmtc = resp[12 + i*2 + 1];
                const char *fn = (fmtc == 0x06) ? "MBLA" :
                                 (fmtc == 0x0D) ? "MIDI" : "other";
                printf("    entry[%d]: %u ch, format=0x%02X (%s)\n", i, nch, fmtc, fn);
            }
        }
    }

    printf("\n-- Extended Stream Format: Music Subunit Dest Plugs --\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[16];
        memset(cmd, 0xFF, sizeof(cmd));
        cmd[0] = 0x01;
        cmd[1] = 0x60;
        cmd[2] = 0x2F;
        cmd[3] = 0xC0;
        cmd[4] = 0x01;  /* subunit plug */
        cmd[5] = 0x01;  /* direction: dest (input to subunit) */
        cmd[6] = (UInt8)p;

        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Dest plug %d: ", p);
        if (avc_cmd(avc, cmd, 16, resp, &rlen) != 0) {
            printf("(failed)\n");
            if (p == 0) break;
            continue;
        }
        hexdump("", resp, rlen);
        if (rlen >= 1 && resp[0] == 0x08)
            printf("    NOT_IMPLEMENTED\n");
        else if (rlen >= 12 && resp[0] == 0x0C && resp[7] == 0x90 && resp[8] == 0x40) {
            UInt8 sfc = resp[9], nEntries = resp[11];
            const char *rates[] = {"32k","44.1k","48k","88.2k","96k","176.4k","192k","?"};
            printf("    AM824 compound: SFC=%u (%s) entries=%u\n",
                   sfc, sfc < 7 ? rates[sfc] : "?", nEntries);
            for (int i = 0; i < nEntries && (12 + i*2 + 1) < (int)rlen; i++) {
                UInt8 nch = resp[12 + i*2], fmtc = resp[12 + i*2 + 1];
                const char *fn = (fmtc == 0x06) ? "MBLA" :
                                 (fmtc == 0x0D) ? "MIDI" : "other";
                printf("    entry[%d]: %u ch, format=0x%02X (%s)\n", i, nch, fmtc, fn);
            }
        }
    }

    /* --- Plug Signal Format for Music subunit plugs --- */
    printf("\n-- Plug Signal Format: Music Subunit Source Plugs --\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[] = { 0x01, 0x60, 0x18, (UInt8)p, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Source plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) != 0) {
            printf("(failed)\n");
            if (p == 0) break;
            continue;
        }
        hexdump("", resp, rlen);
        if (rlen >= 1 && resp[0] == 0x08) printf("    NOT_IMPLEMENTED\n");
        else decode_plug_signal(resp, rlen);
    }

    printf("\n-- Plug Signal Format: Music Subunit Dest Plugs --\n");
    for (int p = 0; p < 4; p++) {
        UInt8 cmd[] = { 0x01, 0x60, 0x19, (UInt8)p, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Dest plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) != 0) {
            printf("(failed)\n");
            if (p == 0) break;
            continue;
        }
        hexdump("", resp, rlen);
        if (rlen >= 1 && resp[0] == 0x08) printf("    NOT_IMPLEMENTED\n");
        else decode_plug_signal(resp, rlen);
    }

    /* --- SIGNAL SOURCE for Music Subunit dest plugs --- */
    printf("\n-- Signal Source: Music Subunit Dest Plugs --\n");
    for (int p = 0; p < 4; p++) {
        /* STATUS SIGNAL_SOURCE at Music subunit, querying dest plug p */
        UInt8 cmd[] = { 0x01, 0x60, 0x1A, 0xFF, 0xFF, 0xFF, 0x00, (UInt8)p };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Dest plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) != 0) {
            printf("(failed)\n");
            if (p == 0) break;
            continue;
        }
        hexdump("", resp, rlen);
        if (rlen >= 8 && resp[0] == 0x0C) {
            printf("    status=0x%02X src_plug=0x%02X src_subunit=0x%02X"
                   " dest_plug_type=0x%02X dest_plug=0x%02X\n",
                   resp[3], resp[4], resp[5], resp[6], resp[7]);
        } else if (rlen >= 1 && resp[0] == 0x08) {
            printf("    NOT_IMPLEMENTED\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Phase 3c — SetFormat CONTROL Response Test                         */
/* ------------------------------------------------------------------ */

/* Send the EXACT same AV/C Extended Stream Format CONTROL command that
 * our BeBoB driver sends in StartDuplex48k(), and capture the response.
 * This confirms whether the Orpheus accepts or rejects the command.     */

static void test_setformat_control(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n=== PHASE 3c: SetFormat CONTROL Response Test ===\n");
    printf("(Sending the same 0x2F CONTROL our driver sends)\n");

    /* --- Test 1: iPCR[0] with 12 audio + 1 MIDI (what our driver sends) --- */
    {
        UInt8 cmd[16] = {
            0x00,       /* CONTROL */
            0xFF,       /* UNIT */
            0x2F,       /* EXTENDED STREAM FORMAT INFORMATION */
            0xC0,       /* SINGLE (set current) */
            0x00,       /* addr_type: unit PCR */
            0x01,       /* direction: input (iPCR) */
            0xFF,       /* reserved */
            0x00,       /* plug 0 */
            0x90,       /* AM824 compound */
            0x02,       /* SFC: 48kHz */
            0x00,       /* reserved */
            0x02,       /* 2 entries */
            0x06,       /* MBLA */
            0x0C,       /* 12 audio channels */
            0x0D,       /* MIDI */
            0x01,       /* 1 MIDI channel */
        };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);

        printf("\n  iPCR[0] CONTROL (12 audio + 1 MIDI): ");
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 1) {
                if (resp[0] == 0x09) printf("    >> ACCEPTED — device supports SetFormat\n");
                else if (resp[0] == 0x08) printf("    >> NOT_IMPLEMENTED — device IGNORES SetFormat!\n");
                else if (resp[0] == 0x0A) printf("    >> REJECTED — device refuses this format\n");
                else printf("    >> response_type = 0x%02X\n", resp[0]);
            }
        } else {
            printf("(AVC command failed at transport level)\n");
        }
    }

    /* --- Test 2: oPCR[0] with 10 audio + 1 MIDI --- */
    {
        UInt8 cmd[16] = {
            0x00, 0xFF, 0x2F, 0xC0, 0x00,
            0x00,       /* direction: output (oPCR) */
            0xFF, 0x00,
            0x90, 0x02, 0x00, 0x02,
            0x06, 0x0A, /* 10 audio channels */
            0x0D, 0x01,
        };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);

        printf("  oPCR[0] CONTROL (10 audio + 1 MIDI): ");
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 1) {
                if (resp[0] == 0x09) printf("    >> ACCEPTED\n");
                else if (resp[0] == 0x08) printf("    >> NOT_IMPLEMENTED\n");
                else if (resp[0] == 0x0A) printf("    >> REJECTED\n");
                else printf("    >> response_type = 0x%02X\n", resp[0]);
            }
        } else {
            printf("(AVC command failed at transport level)\n");
        }
    }

    /* --- Test 3: iPCR[0] with just 2 audio, no MIDI (minimal stereo) --- */
    {
        UInt8 cmd[14] = {
            0x00, 0xFF, 0x2F, 0xC0, 0x00,
            0x01, 0xFF, 0x00,
            0x90, 0x02, 0x00,
            0x01,       /* 1 entry */
            0x06, 0x02, /* 2 audio channels (stereo) */
        };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);

        printf("  iPCR[0] CONTROL (2 audio, no MIDI): ");
        if (avc_cmd(avc, cmd, 14, resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 1) {
                if (resp[0] == 0x09) printf("    >> ACCEPTED\n");
                else if (resp[0] == 0x08) printf("    >> NOT_IMPLEMENTED\n");
                else if (resp[0] == 0x0A) printf("    >> REJECTED\n");
                else printf("    >> response_type = 0x%02X\n", resp[0]);
            }
        } else {
            printf("(AVC command failed at transport level)\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Phase 3d — Additional Signal Source Routing Queries                 */
/* ------------------------------------------------------------------ */

static void query_signal_source_extended(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n=== PHASE 3d: Extended Signal Source Queries ===\n");

    /* Query SIGNAL SOURCE for Unit isoch input plugs 0 and 1 */
    printf("\n-- Unit-level SIGNAL SOURCE for isoch input plugs --\n");
    for (int p = 0; p < 2; p++) {
        /* The dest in SIGNAL SOURCE is [plug_type, plug_id]:
         * For unit isoch serial bus: plug_type = 0x00, plug_id = p
         * Status_info=0xFF, conv=0xFF 0xFF, src_plug=0xFF, src_subunit=0xFF */
        UInt8 cmd[] = { 0x01, 0xFF, 0x1A, 0xFF, 0xFF, 0xFF, 0x00, (UInt8)p };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Isoch input plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 8 && resp[0] == 0x0C)
                printf("    status=0x%02X src_plug=0x%02X src_sub=0x%02X"
                       " dst_type=0x%02X dst_id=0x%02X\n",
                       resp[3], resp[4], resp[5], resp[6], resp[7]);
            else if (rlen >= 1 && resp[0] == 0x08)
                printf("    NOT_IMPLEMENTED\n");
        } else {
            printf("(failed)\n");
        }
    }

    /* Query SIGNAL SOURCE for unit external output plugs (DAC outputs) */
    printf("\n-- Unit-level SIGNAL SOURCE for external output plugs --\n");
    for (int p = 0; p < 8; p++) {
        /* dest = external output plug: plug_type = 0x01, plug_id = p */
        UInt8 cmd[] = { 0x01, 0xFF, 0x1A, 0xFF, 0xFF, 0xFF, 0x01, (UInt8)p };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Ext output plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 8 && resp[0] == 0x0C)
                printf("    status=0x%02X src_plug=0x%02X src_sub=0x%02X"
                       " dst_type=0x%02X dst_id=0x%02X\n",
                       resp[3], resp[4], resp[5], resp[6], resp[7]);
            else if (rlen >= 1 && resp[0] == 0x08)
                printf("    NOT_IMPLEMENTED\n");
        } else {
            printf("(failed)\n");
            break;
        }
    }

    /* Query SIGNAL SOURCE for unit isoch output plugs (recording) */
    printf("\n-- Unit-level SIGNAL SOURCE for isoch output plugs --\n");
    for (int p = 0; p < 2; p++) {
        /* dest = isoch serial bus output: plug_type = 0x00, plug_id = p
         * BUT direction is reversed: for output plugs, the signal
         * destination is the oPCR, source is internal. Use subunit=0xFF. */
        UInt8 cmd[] = { 0x01, 0xFF, 0x1A, 0xFF, 0xFF, 0xFF, 0x00, (UInt8)p };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);
        printf("  Isoch output plug %d: ", p);
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
        } else {
            printf("(failed)\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Phase 3e — AV/C Descriptor Read (Music Subunit)                    */
/* ------------------------------------------------------------------ */

/* BeBoB devices store their complete stream format descriptors in the
 * Music subunit's AV/C descriptor.  The access sequence is:
 *   1. OPEN DESCRIPTOR (opcode 0x08) — open for read
 *   2. READ DESCRIPTOR (opcode 0x09) — read content
 *   3. Optionally CLOSE DESCRIPTOR
 *
 * The descriptor contains the definitive channel layout for all
 * supported sample rates.                                              */

static void read_music_subunit_descriptor(IOFireWireAVCLibUnitInterface **avc)
{
    printf("\n=== PHASE 3e: Music Subunit Descriptor Read ===\n");

    /* --- Step 1: OPEN DESCRIPTOR for read --- */
    /* AV/C OPEN DESCRIPTOR (opcode 0x08)
     * [0] 0x00  CONTROL
     * [1] 0x60  Music subunit
     * [2] 0x08  OPEN DESCRIPTOR
     * [3] 0x01  read mode (0x01 = read open)
     * [4] 0x00  descriptor_type (0x00 = general music subunit status descriptor)
     * [5-7] reserved (0xFF)
     */
    printf("\n-- OPEN DESCRIPTOR (read) --\n");
    {
        UInt8 cmd[] = { 0x00, 0x60, 0x08, 0x01, 0x00, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);

        printf("  Open: ");
        if (avc_cmd(avc, cmd, sizeof(cmd), resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 1) {
                if (resp[0] == 0x09) printf("    >> ACCEPTED (descriptor opened)\n");
                else if (resp[0] == 0x08) printf("    >> NOT_IMPLEMENTED\n");
                else if (resp[0] == 0x0A) printf("    >> REJECTED\n");
                else printf("    >> response_type = 0x%02X\n", resp[0]);
            }
        } else {
            printf("(command failed)\n");
            return;
        }
    }

    /* --- Step 2: READ DESCRIPTOR --- */
    /* AV/C READ DESCRIPTOR (opcode 0x09)
     * [0] 0x01  STATUS
     * [1] 0x60  Music subunit
     * [2] 0x09  READ DESCRIPTOR
     * [3] 0xFF  descriptor_type
     * [4-5] address (offset into descriptor, big-endian: 0x0000)
     * [6-7] read_length (0xFFFF = max available)
     */
    printf("\n-- READ DESCRIPTOR --\n");
    {
        UInt8 cmd[] = { 0x01, 0x60, 0x09, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
                        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        UInt8 resp[512]; UInt32 rlen = sizeof(resp);

        printf("  Read: ");
        if (avc_cmd(avc, cmd, 16, resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (rlen >= 1) {
                if (resp[0] == 0x0C) {
                    printf("    >> Got descriptor (%u bytes)\n", (unsigned)rlen);
                    /* Print raw bytes in rows of 16 for analysis */
                    printf("    Descriptor content:\n");
                    for (UInt32 i = 0; i < rlen; i++) {
                        if (i % 16 == 0) printf("      %04X: ", (unsigned)i);
                        printf("%02X ", resp[i]);
                        if (i % 16 == 15 || i == rlen - 1) printf("\n");
                    }
                } else if (resp[0] == 0x08) {
                    printf("    >> NOT_IMPLEMENTED\n");
                } else if (resp[0] == 0x0A) {
                    printf("    >> REJECTED\n");
                }
            }
        } else {
            printf("(command failed)\n");
        }
    }

    /* --- Alternative: Try BridgeCo-style descriptor read (subunit_type 0x80) --- */
    printf("\n-- BridgeCo-style READ DESCRIPTOR (subunit_type identifier list) --\n");
    {
        /* Some BeBoB devices use a different descriptor format with extended
         * addressing. Try the BridgeCo-specific descriptor read:
         * [0] 0x01  STATUS
         * [1] 0x60  Music subunit
         * [2] 0x09  READ DESCRIPTOR
         * [3] descriptor_specifier_type (0x80 = entry by position in ID list)
         * [4] 0x00  subfunction (read entry)
         * [5-6] entry address (big-endian: 0x0000 = first)
         * [7-8] read result length placeholder
         * [9-10] read length (big-endian: 0x00FF = 255 bytes)
         */
        UInt8 cmd[12] = { 0x01, 0x60, 0x09,
                          0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF };
        UInt8 resp[512]; UInt32 rlen = sizeof(resp);

        printf("  Read (0x80): ");
        if (avc_cmd(avc, cmd, 12, resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (resp[0] == 0x0C && rlen > 8) {
                printf("    Descriptor content:\n");
                for (UInt32 i = 0; i < rlen; i++) {
                    if (i % 16 == 0) printf("      %04X: ", (unsigned)i);
                    printf("%02X ", resp[i]);
                    if (i % 16 == 15 || i == rlen - 1) printf("\n");
                }
            } else if (rlen >= 1 && resp[0] == 0x08) {
                printf("    NOT_IMPLEMENTED\n");
            }
        } else {
            printf("(command failed)\n");
        }
    }

    /* --- Try BeBoB-specific: OPEN DESCRIPTOR with specifier type 0x00 --- */
    printf("\n-- BeBoB-style OPEN + READ (type 0x00 = info block) --\n");
    {
        /* BeBoB open descriptor: specifier=0x00, subfunction=0x01 (read) */
        UInt8 openCmd[10] = { 0x00, 0x60, 0x08, 0x00, 0x01,
                              0x00, 0x01, 0xFF, 0xFF, 0xFF };
        UInt8 resp[64]; UInt32 rlen = sizeof(resp);

        printf("  Open (0x00): ");
        if (avc_cmd(avc, openCmd, 10, resp, &rlen) == 0) {
            hexdump("", resp, rlen);
            if (resp[0] == 0x09) {
                /* Read the descriptor */
                UInt8 readCmd[12] = { 0x01, 0x60, 0x09, 0x00, 0x01,
                                      0x00, 0x00, 0xFF, 0xFF, 0x01, 0x00, 0xFF };
                rlen = sizeof(resp);
                printf("  Read (0x00): ");
                if (avc_cmd(avc, readCmd, 12, resp, &rlen) == 0) {
                    hexdump("", resp, rlen);
                    if (resp[0] == 0x0C) {
                        printf("    Descriptor content:\n");
                        for (UInt32 i = 0; i < rlen; i++) {
                            if (i % 16 == 0) printf("      %04X: ", (unsigned)i);
                            printf("%02X ", resp[i]);
                            if (i % 16 == 15 || i == rlen - 1) printf("\n");
                        }
                    }
                } else {
                    printf("(read failed)\n");
                }
            } else if (rlen >= 1 && resp[0] == 0x08) {
                printf("    NOT_IMPLEMENTED\n");
            }
        } else {
            printf("(open failed)\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Phase 3f — PCR Read via IOFireWireUnit (alternate path)             */
/* ------------------------------------------------------------------ */

/* The primary FW device may be locked by the audio kext.  Try reading
 * PCR registers through an IOFireWireUnit child service, which may have
 * separate access permissions.                                         */

static void try_pcr_via_unit(io_service_t fwDev)
{
    printf("\n=== PHASE 3f: PCR Read via IOFireWireUnit ===\n");

    /* Find IOFireWireUnit child of the device */
    io_iterator_t iter;
    kern_return_t kr = IORegistryEntryCreateIterator(
        fwDev, kIOServicePlane, kIORegistryIterateRecursively, &iter);
    if (kr) {
        printf("  Cannot iterate children\n");
        return;
    }

    io_service_t child, unitSvc = 0;
    while ((child = IOIteratorNext(iter))) {
        io_name_t cls;
        IOObjectGetClass(child, cls);
        if (strcmp(cls, "IOFireWireUnit") == 0 && !unitSvc)
            unitSvc = child;
        else
            IOObjectRelease(child);
    }
    IOObjectRelease(iter);

    if (!unitSvc) {
        printf("  No IOFireWireUnit found\n");
        return;
    }

    /* Try opening it as a FW device */
    IOCFPlugInInterface **plug = NULL;
    SInt32 score;
    kr = IOCreatePlugInInterfaceForService(
        unitSvc, kIOFireWireLibTypeID, kIOCFPlugInInterfaceID, &plug, &score);
    IOObjectRelease(unitSvc);

    if (kr || !plug) {
        printf("  IOCreatePlugIn for FW unit failed (0x%x)\n", kr);
        return;
    }

    IOFireWireLibDeviceRef dev = NULL;
    (*plug)->QueryInterface(plug,
        CFUUIDGetUUIDBytes(kIOFireWireDeviceInterfaceID), (LPVOID *)&dev);
    (*plug)->Release(plug);

    if (!dev) {
        printf("  QueryInterface failed\n");
        return;
    }

    IOReturn rc = (*dev)->Open(dev);
    if (rc != kIOReturnSuccess)
        printf("  WARN: Open via IOFireWireUnit also failed (0x%x), trying reads anyway\n", rc);
    else
        printf("  Opened via IOFireWireUnit — reading PCR registers:\n");

    UInt32 v;
    printf("\n-- Output Plug Registers (Orpheus -> Mac) --\n");
    if (read_quadlet(dev, CSR_oMPR, &v) == 0) decode_oMPR(v);
    else printf("  oMPR: read FAILED\n");
    if (read_quadlet(dev, CSR_oPCR0, &v) == 0) decode_oPCR(v, 0);
    else printf("  oPCR[0]: read FAILED\n");

    printf("\n-- Input Plug Registers (Mac -> Orpheus) --\n");
    if (read_quadlet(dev, CSR_iMPR, &v) == 0) decode_iMPR(v);
    else printf("  iMPR: read FAILED\n");
    if (read_quadlet(dev, CSR_iPCR0, &v) == 0) decode_iPCR(v, 0);
    else printf("  iPCR[0]: read FAILED\n");

    (*dev)->Close(dev);
    (*dev)->Release(dev);
}

/* ------------------------------------------------------------------ */
/*  Phase 4 — IORegistry audio info                                    */
/* ------------------------------------------------------------------ */

static void dump_ioreg_audio(void)
{
    printf("\n=== PHASE 4: IORegistry Audio Device Info ===\n");

    const char *classes[] = {
        "AppleFWAudioDevice", "IOAudioEngine", "IOAudioStream", NULL
    };
    static const char *keys[] = {
        "IOAudioEngineState",
        "IOAudioEngineSampleRate",
        "IOAudioEngineNumSampleFramesPerBuffer",
        "IOAudioEngineInputSampleLatency",
        "IOAudioEngineOutputSampleLatency",
        "IOAudioStreamNumClients",
        "IOAudioStreamBitDepth",
        "IOAudioStreamBitWidth",
        "IOAudioStreamNumChannels",
        "IOAudioStreamSampleFormatByteOrder",
        "IOAudioStreamAlignment",
        "IOAudioStreamIsMixable",
        "IOAudioStreamDirection",
        "IOAudioStreamStartingChannelID",
        NULL
    };

    for (int c = 0; classes[c]; c++) {
        io_iterator_t iter;
        CFMutableDictionaryRef m = IOServiceMatching(classes[c]);
        if (!m) continue;
        if (IOServiceGetMatchingServices(kIOMasterPortDefault, m, &iter)) continue;

        io_service_t svc;
        while ((svc = IOIteratorNext(iter))) {
            io_name_t name;
            IORegistryEntryGetName(svc, name);
            printf("\n  [%s] \"%s\"\n", classes[c], name);

            for (int k = 0; keys[k]; k++) {
                UInt64 val;
                if (get_int_prop(svc, keys[k], &val) == 0)
                    printf("    %s = %lld\n", keys[k], val);
            }
            IOObjectRelease(svc);
        }
        IOObjectRelease(iter);
    }
}

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) g_verbose = 1;
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: sudo %s [-v]\n"
                   "  -v  verbose output\n"
                   "\nRun on the OLD laptop while audio is playing through the Orpheus.\n"
                   "Redirect output to a file: sudo %s 2>&1 | tee report.txt\n",
                   argv[0], argv[0]);
            return 0;
        }
    }

    printf("========================================================\n");
    printf("  FireWire Audio Stream Diagnostic Tool\n");
    printf("  Target: Prism Sound Orpheus\n");
    printf("========================================================\n\n");

    if (geteuid() != 0) {
        fprintf(stderr, "WARNING: Not running as root. Some operations may fail.\n");
        fprintf(stderr, "Run with: sudo %s\n\n", argv[0]);
    }

    /* ---- Phase 1: Discovery ---- */
    printf("=== PHASE 1: Device Discovery ===\n");
    io_service_t fwDev = find_orpheus_device();
    if (!fwDev) {
        fprintf(stderr, "\nERROR: Prism Sound Orpheus not found on FireWire bus.\n");
        return 1;
    }
    UInt64 guid = 0;
    get_int_prop(fwDev, "GUID", &guid);
    printf("\n  >> Orpheus found, GUID = 0x%016llX\n", guid);

    /* ---- Phase 2: PCR Registers ---- */
    printf("\n=== PHASE 2: PCR Registers ===\n");
    IOFireWireLibDeviceRef dev = open_fw_device(fwDev);
    if (dev) {
        read_all_pcr(dev);
        (*dev)->Close(dev);
        (*dev)->Release(dev);
    } else {
        printf("  Skipping PCR reads (could not open device interface).\n");
    }

    /* ---- Phase 3: AV/C Queries ---- */
    printf("\n=== PHASE 3: AV/C Queries ===\n");
    IOFireWireAVCLibUnitInterface **avc = open_avc_unit(fwDev);
    if (avc) {
        query_plug_info(avc);
        query_subunit_info(avc);
        query_plug_signal_format(avc);

        printf("\n-- AV/C EXTENDED STREAM FORMAT (current) --\n");
        query_stream_format_single(avc, 0, 0);   /* input plug 0 */
        query_stream_format_single(avc, 1, 0);   /* output plug 0 */

        printf("\n-- AV/C EXTENDED STREAM FORMAT (supported list) --\n");
        query_stream_format_list(avc, 0, 0);
        query_stream_format_list(avc, 1, 0);

        query_signal_source(avc);
        query_vendor_state(avc);

        /* ---- Phase 3b: Music Subunit ---- */
        query_music_subunit_plugs(avc);

        /* ---- Phase 3c: SetFormat CONTROL test ---- */
        test_setformat_control(avc);

        /* ---- Phase 3d: Extended signal routing ---- */
        query_signal_source_extended(avc);

        /* ---- Phase 3e: Music Subunit Descriptor ---- */
        read_music_subunit_descriptor(avc);

        (*avc)->close(avc);
        (*avc)->Release(avc);
    } else {
        printf("  Skipping AV/C queries (could not open AVC unit).\n");
    }

    /* ---- Phase 3f: Alternate PCR read ---- */
    try_pcr_via_unit(fwDev);

    /* ---- Phase 4: IORegistry ---- */
    dump_ioreg_audio();

    /* ---- Summary ---- */
    printf("\n\n========================================================\n");
    printf("  KEY VALUES TO COMPARE WITH NEW DRIVER\n");
    printf("========================================================\n");
    printf("  1. iPCR[0] — channel + p2p count when streaming\n");
    printf("  2. oPCR[0] — payload size => DBS (data block size)\n");
    printf("  3. Input plug signal format — FMT + FDF + SFC\n");
    printf("  4. Extended stream format — exact channel map & types\n");
    printf("     (THIS IS LIKELY THE KEY: how many MBLA vs MIDI ch)\n");
    printf("  5. Signal source — clock routing on working setup\n");
    printf("  6. Digital state — sample rate byte encoding\n");
    printf("  7. IOAudioStream — channel count, bit depth, direction\n");
    printf("  8. SetFormat CONTROL response — does Orpheus accept/reject?\n");
    printf("  9. Music subunit stream format — definitive channel layout\n");
    printf(" 10. Music subunit descriptor — complete format descriptor\n");
    printf("========================================================\n");
    printf("\nDone. Copy this output and compare with the new driver.\n");

    IOObjectRelease(fwDev);
    return 0;
}
