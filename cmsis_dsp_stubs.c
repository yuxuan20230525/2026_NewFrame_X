/**
 * cmsis_dsp_stubs.c — CMSIS-DSP 函数的最小实现
 * 替代 Keil 预编译的 arm_cortexM4lf_math.lib
 * 仅实现项目实际用到的函数
 */
#include "arm_math.h"
#include <math.h>

void arm_mat_init_f32(arm_matrix_instance_f32 *S,
                      uint16_t nRows, uint16_t nColumns,
                      float32_t *pData)
{
    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData   = pData;
}

arm_status arm_mat_scale_f32(const arm_matrix_instance_f32 *pSrc,
                             float32_t scale,
                             arm_matrix_instance_f32 *pDst)
{
    uint32_t numSamples = pSrc->numRows * pSrc->numCols;
    const float32_t *pIn  = pSrc->pData;
    float32_t       *pOut = pDst->pData;

    for (uint32_t i = 0; i < numSamples; i++)
    {
        pOut[i] = pIn[i] * scale;
    }

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    uint16_t rA = pSrcA->numRows;
    uint16_t cA = pSrcA->numCols;
    uint16_t cB = pSrcB->numCols;

    for (uint16_t i = 0; i < rA; i++)
    {
        for (uint16_t j = 0; j < cB; j++)
        {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < cA; k++)
            {
                sum += pSrcA->pData[i * cA + k] * pSrcB->pData[k * cB + j];
            }
            pDst->pData[i * cB + j] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    uint16_t r = pSrc->numRows;
    uint16_t c = pSrc->numCols;

    for (uint16_t i = 0; i < r; i++)
    {
        for (uint16_t j = 0; j < c; j++)
        {
            pDst->pData[j * r + i] = pSrc->pData[i * c + j];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    uint32_t n = pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < n; i++)
    {
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    uint32_t n = pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < n; i++)
    {
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *src,
                               arm_matrix_instance_f32 *dst)
{
    uint16_t n = src->numRows;
    float32_t aug[n * 2 * n];

    /* build augmented matrix [src | I] */
    for (uint16_t i = 0; i < n; i++)
    {
        for (uint16_t j = 0; j < n; j++)
        {
            aug[i * 2 * n + j]     = src->pData[i * n + j];
            aug[i * 2 * n + n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* Gauss-Jordan elimination */
    for (uint16_t i = 0; i < n; i++)
    {
        /* find pivot */
        float32_t maxVal = fabsf(aug[i * 2 * n + i]);
        uint16_t maxRow = i;
        for (uint16_t k = i + 1; k < n; k++)
        {
            float32_t v = fabsf(aug[k * 2 * n + i]);
            if (v > maxVal) { maxVal = v; maxRow = k; }
        }
        if (maxVal < 1e-12f) return ARM_MATH_SINGULAR;

        /* swap rows */
        if (maxRow != i)
        {
            for (uint16_t j = 0; j < 2 * n; j++)
            {
                float32_t tmp = aug[i * 2 * n + j];
                aug[i * 2 * n + j] = aug[maxRow * 2 * n + j];
                aug[maxRow * 2 * n + j] = tmp;
            }
        }

        /* scale pivot row */
        float32_t pivot = aug[i * 2 * n + i];
        for (uint16_t j = 0; j < 2 * n; j++)
            aug[i * 2 * n + j] /= pivot;

        /* eliminate column */
        for (uint16_t k = 0; k < n; k++)
        {
            if (k == i) continue;
            float32_t factor = aug[k * 2 * n + i];
            for (uint16_t j = 0; j < 2 * n; j++)
                aug[k * 2 * n + j] -= factor * aug[i * 2 * n + j];
        }
    }

    /* extract inverse */
    for (uint16_t i = 0; i < n; i++)
        for (uint16_t j = 0; j < n; j++)
            dst->pData[i * n + j] = aug[i * 2 * n + n + j];

    return ARM_MATH_SUCCESS;
}

float32_t arm_sin_f32(float32_t x)
{
    return sinf(x);
}

float32_t arm_cos_f32(float32_t x)
{
    return cosf(x);
}
