/**
 * @brief g233自定义 dma指令的helper函数
 * @date 2025-11-9
 */
#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"


/**
 * @note  DEF_HELPER_4(dma, void, env, tl, tl, tl)
 * @param env:cpustate 
 * @param dst： 目标矩阵地址
 * @param src: 源矩阵地址
 * @param grain： 矩阵粒度 0->8 1->16 2->32
 */
void helper_dma(CPURISCVState *env, target_ulong dst,target_ulong src, target_ulong grain){
    size_t size;
    switch (grain)
    {
    case 0:
        size=8;
        break;
    case 1:
        size=16;
        break;
    case 2:
        size=32;
        break;
    default:
        return;//错误了
        break;
    }

    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < i; j++)
        {
            //行排序的
            target_ulong src_addr = src + (j * size + i) * 4;
            uint32_t value = cpu_ldl_data(env, src_addr);
            
            target_ulong dst_addr = dst + (i * size + j) * 4;
            cpu_stl_data(env, dst_addr, value);
        }
    }
    

}