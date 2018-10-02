#pragma once

#include <usr/type.h>
#include <fsl_sim.h>


static inline u64 uid(void)
{
    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    //TODO: Determine whether H is useful
    //sim_uid.L ^= sim_uid.H;
    sim_uid.ML ^= ~sim_uid.MH;

    return sim_uid.L | ((u64)sim_uid.ML << 32);
}


static inline void uid32(u32 *h, u32 *l)
{
    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    //sim_uid.L ^= sim_uid.H;
    sim_uid.ML ^= ~sim_uid.MH;

    *h = sim_uid.ML;
    *l = sim_uid.L;
}


static inline u16 uid_short(void)
{
    sim_uid_t sim_uid;
    SIM_GetUniqueId(&sim_uid);

    //sim_uid.L ^= sim_uid.H;
    sim_uid.ML ^= ~sim_uid.MH;

    u32 word = sim_uid.L ^ sim_uid.ML;

    return ((u16) word) ^ ((u16) (word >> 16));
}
