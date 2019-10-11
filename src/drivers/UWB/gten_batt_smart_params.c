/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file batt_smart_params.c
 */

/**
 * Horizontal flight average current
 *
 *
 *
 * @unit a
 * @min 0.01
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_H_CURR, 13.0f);


/**
 * vertical flight average current
 *
 *
 *
 * @unit a
 * @min 0.01
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_CURR, 14.0f);


/**
 * safely capacity remain for return
 *
 *
 *
 * @unit mah
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @increment 0.1
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_C_SAFE, 700.0f);


/**
 * safely capacity remain in unbalance for return
 *
 *
 *
 * @unit mah
 * @min 0.0
 * @max 10000.0
 * @decimal 1
 * @increment 0.1
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_C_SAFE_UB, 1500.0f);

