#include <kio/flsh.h>
#include <board/trace.h>
#include <fsl_flash.h>
#include <fsl_flash.h>
#include <FreeRTOS.h>


static_assert(FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP, "FSL_FEATURE_SIM_FCFG_HAS_PFLSH_SWAP");


#define OFFSET(x)           ((u32)(0x100000 + (u8 *)(x)))
#define USER_FLASH_SIZE     FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE          FSL_FEATURE_FLASH_PFLASH_BLOCK_SECTOR_SIZE
#define BLOCK_SIZE_32       (BLOCK_SIZE / sizeof(u32))
#define FLASH_SWAP_ADDR     (((FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT) >> 1u) - BLOCK_SIZE)


static_assert(BLOCK_SIZE >= USER_FLASH_SIZE, "user flash must fit in one block");


typedef struct {
    u32 sanity;
    u32 version;

    u8 data[];

} user_flash_t;


extern u32 __user_flash_init_base[];
extern u32 __user_flash_init_end[];
extern u32 __user_flash_base[];
extern u32 __user_flash_end[];
extern u32 __user_flash_size[];
extern u32 __user_ram_base[];
extern u32 __user_ram_end[];

extern u32 __fast_text_begin[];
extern u32 __fast_text_end[];
extern u32 __fast_code_begin[];

extern u32 __text_begin[];
extern u32 __text_end[];

extern u32 __DATA_ROM[];
extern u32 __DATA_END[];

extern u32 __VECTOR_TABLE[];
extern u32 __VECTOR_RAM[];
extern u32 __RAM_VECTOR_TABLE_SIZE_BYTES[];

extern u32 __flash_header_begin[];
extern u32 __flash_header_end[];


static user_flash_t *user_flsh;
static user_flash_t *user_flsh_rom;
static user_flash_t *user_flsh_othr;
static user_flash_t user_flsh_ram __section(".user.base") = {
        .sanity = FLASH_SANITY,
        .version = FLASH_VERSION,
};

static flash_config_t flash_config;
static ftfx_config_t *ftfx_config;

static status_t flsh_bcpy(u32 *src, u32 *end, u32 *dst);
static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)__used;


status_t flsh_init(void)
{
    status_t status = 0;

    FLASH_Init(&flash_config);

    ftfx_config = flash_config.ftfxConfig;

    user_flsh_rom = (user_flash_t *) __user_flash_init_base;
    user_flsh = (user_flash_t *) __user_flash_base;
    user_flsh_othr = (user_flash_t *) OFFSET(__user_flash_base);

    wcpy(__user_flash_base, __user_flash_end, __user_ram_base);

    board_trace_f(
            "user: addr=0x%X  s=0x%08X v=0x%08X ssz=0x%02X dsz=0x%02X ",
            (u32)__user_flash_base,
            user_flsh->sanity,
            user_flsh->version,
            USER_FLASH_SIZE,
            (u32)__user_flash_end - (u32)__user_flash_base
    );


    if (user_flsh_ram.sanity != user_flsh_rom->sanity || user_flsh_ram.version != user_flsh_rom->version) {
        board_trace_fr("update to s=0x%08X v=0x%08X ... ", user_flsh_rom->sanity, user_flsh_rom->version);

        wcpy(__user_flash_init_base, __user_flash_init_end, __user_ram_base);


        if ((status = flsh_write((u32) __user_flash_base, (u32) __user_flash_end - (u32) __user_flash_base, __user_ram_base))) {
            return status;
        } else {
            board_trace("done.");
        }
    }

    board_trace_fr(
            "othr: addr=0x%X s=0x%08X v=0x%08X ",
            user_flsh_othr, user_flsh_othr->sanity, user_flsh_othr->version
    );

    if (user_flsh_othr->sanity == user_flsh_ram.sanity) {

        board_trace("<valid>");

    } else {


        board_trace("<invalid>");
        board_trace_fr("copy flash header s=0x%X ... ", (u32)__flash_header_end - (u32)__flash_header_begin);
        
        if ((status = flsh_write(OFFSET(__flash_header_begin), (u32)__flash_header_end - (u32)__flash_header_begin, __flash_header_begin))) {
            return status;
        } else {
            board_trace("done.");
        }


        board_trace_fr("copy text/data s=%u ... ",
                       (u32) __DATA_END - (u32) __user_flash_base);

        if ((status = flsh_bcpy(__user_flash_base, __DATA_END, (u32 *) OFFSET(__user_flash_base)))) {
            return status;
        } else {
            board_trace("done.");
        }
        
        board_trace_r("swapping... ");

        if (user_flsh_othr->sanity == user_flsh_ram.sanity) {
            if ((status = FLASH_Swap(&flash_config, FLASH_SWAP_ADDR, true))) {
                board_trace_f("swap fail: s=%u", status);
            } else {
                //board_trace("success.")
                board_trace("success.\nreboot....\r\n\r\n\r\n");
                //__BKPT(0);
                NVIC_SystemReset();
            }
        } else {
            board_trace("error: mark not set!");
            __BKPT(0);
        }
    }

    return status;
}


static status_t flsh_bcpy(u32 *src, u32 *end, u32 *dst)
{
    status_t status = 1;
    u32 size;
    
    while (src < end) {
        size = (u8 *) end - (u8 *) src;

        // Disable this to allow errors that prevent copying garbage.
        if (size > BLOCK_SIZE) {
            size = BLOCK_SIZE;
        } else {
            if (size % 16) size += 16 - (size % 16);
        }

        /*board_trace_fr("copying %u: [0x%X:0x%X]->0x%X... ", size, src, (u8 *) src + size, dst)*/

        if ((status = flsh_write((u32)dst, size, src))) {
            return status;
        } else {
            //board_trace("done.");
        }
        
        src += size >> 2;
        dst += size >> 2;
    }
    
    return status;
}

status_t flsh_write(u32 begin, size_t size, u32 *data)
{
    status_t status;

    if (size % 4) return 1;

    portDISABLE_INTERRUPTS();

    status = FLASH_Erase(&flash_config, begin, size, kFLASH_ApiEraseKey);

    if (status) {
        board_trace_f("erase fail: s=%li", status);

    } else {
        status = FLASH_Program(&flash_config, begin, (u8 *) data, size);

        if (status != kStatus_FLASH_Success) {
            board_trace_f("write fail: %li", status);
        } else {
            u32 *dst = (u32 *) begin;
            u32 *end = (u32 *) (begin + size);

            while (dst < end) {
                if (*dst != *data) {
                    board_trace_f("verify fail: wrote=0x%X@0x%X != read=0x%X@0x%X", *dst, dst, *data, data);
                    return 2;
                }

                ++dst;
                ++data;
            }
        }
    }

    portENABLE_INTERRUPTS();
    return status;
}


status_t flsh_user_cmit(void)
{
    return flsh_bcpy(__user_ram_base, __user_ram_end, __user_flash_base);
}


static status_t get_swap_state_config(ftfx_swap_state_config_t *swap_state_config)
{
    status_t status = FTFx_CMD_SwapControl(ftfx_config, FLASH_SWAP_ADDR, kFTFx_SwapControlOptionReportStatus, swap_state_config);

    if (status) {
        board_trace_f("swap: read config fail. s=%u", status);
    }

    return status;
}
