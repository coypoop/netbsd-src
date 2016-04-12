/*
 * Copyright (c) 2010 Broadcom Corporation
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _BRCM_QMATH_H_
#define _BRCM_QMATH_H_

#include <bcm_types.h>

uint16_t qm_muluint16_t(uint16_t op1, uint16_t op2);

int16_t qm_mulint16_t(int16_t op1, int16_t op2);

int32_t qm_add32(int32_t op1, int32_t op2);

int16_t qm_add16(int16_t op1, int16_t op2);

int16_t qm_sub16(int16_t op1, int16_t op2);

int32_t qm_shl32(int32_t op, int shift);

int16_t qm_shl16(int16_t op, int shift);

int16_t qm_shr16(int16_t op, int shift);

int16_t qm_norm32(int32_t op);

void qm_log10(int32_t N, int16_t qN, int16_t *log10N, int16_t *qLog10N);

#endif				/* #ifndef _BRCM_QMATH_H_ */
