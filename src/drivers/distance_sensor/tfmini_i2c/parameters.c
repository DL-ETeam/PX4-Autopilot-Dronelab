/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * Enable TFmini Plus I2C rangefinder (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Autodetect
 */
PARAM_DEFINE_INT32(SENS_EN_TFMI2C, 0);

/**
 * TFmini Plus I2C Sensor 0 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_0_ORT, 0);

/**
 * TFmini Plus I2C Sensor 1 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_1_ORT, 0);

/**
 * TFmini Plus I2C Sensor 2 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_2_ORT, 0);

/**
 * TFmini Plus I2C Sensor 3 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_3_ORT, 0);

/**
 * TFmini Plus I2C Sensor 4 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_4_ORT, 0);

/**
 * TFmini Plus I2C Sensor 5 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_5_ORT, 0);

/**
 * TFmini Plus I2C Sensor 6 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_6_ORT, 0);

/**
 * TFmini Plus I2C Sensor 7 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_7_ORT, 0);

/**
 * TFmini Plus I2C Sensor 8 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_8_ORT, 0);

/**
 * TFmini Plus I2C Sensor 9 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_9_ORT, 0);

/**
 * TFmini Plus I2C Sensor 10 Rotation
 *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_10_ORT, 0);

/**
 * TFmini Plus I2C Sensor 12 Rotation
  *
 * This parameter defines the rotation of the TFmini Plus I2C sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 25
 * @group Sensors
 *
 * 
 *@value   0 No rotation
 *@value   1 Yaw 45°
 *@value   2 Yaw 90°
 *@value   3 Yaw 135°
 *@value   4 Yaw 180°
 *@value   5 Yaw 225°
 *@value   6 Yaw 270°
 *@value   7 Yaw 315°
 *@value   8 Roll 180°
 *@value   9 Roll 180°, Yaw 45°
 *@value   10 Roll 180°, Yaw 90°
 *@value   11 Roll 180°, Yaw 135°
 *@value   12 Pitch 180°
 *@value   13 Roll 180°, Yaw 225°
 *@value   14 Roll 180°, Yaw 270°
 *@value   15 Roll 180°, Yaw 315°
 *@value   16 Roll 90°
 *@value   17 Roll 90°, Yaw 45°
 *@value   18 Roll 90°, Yaw 90°
 *@value   19 Roll 90°, Yaw 135°
 *@value   20 Roll 270°
 *@value   21 Roll 270°, Yaw 45°
 *@value   22 Roll 270°, Yaw 90°
 *@value   23 Roll 270°, Yaw 135°
 *@value   24 Pitch 90°
 *@value   25 Pitch 270°
 */
PARAM_DEFINE_INT32(SENS_TFM_11_ORT, 0);
