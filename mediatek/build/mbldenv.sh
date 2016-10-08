#!/bin/bash
# ##########################################################
# ALPS(Android4.1 based) build environment profile setting
# ##########################################################

TARGET_BUILD_VARIANT=user
#TARGET_BUILD_VARIANT=eng
export TARGET_BUILD_VARIANT

# Overwrite JAVA_HOME environment variable setting if already exists
JAVA_HOME=/usr/lib/jvm/java-6-oracle
export JAVA_HOME

# Overwrite ANDROID_JAVA_HOME environment variable setting if already exists
ANDROID_JAVA_HOME=/usr/lib/jvm/java-6-oracle
export ANDROID_JAVA_HOME

export TARGET_ARCH_VARIANT=armv7-a-neon

# Overwrite PATH environment setting for JDK & arm-eabi if already exists
PATH=/usr/lib/jvm/java-6-oracle/bin:$PWD/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.7/bin:$PWD/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin:$PWD/prebuilts/misc/linux-x86/make:$PATH
export PATH

# Add MediaTek developed Python libraries path into PYTHONPATH
if [ -z "$PYTHONPATH" ]; then
  PYTHONPATH=$PWD/mediatek/build/tools
else
  PYTHONPATH=$PWD/mediatek/build/tools:$PYTHONPATH
fi
export PYTHONPATH

