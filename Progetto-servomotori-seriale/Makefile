##############################################################################
# Build global options
#
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
endif

ifeq ($(USE_COPT),)
  USE_COPT = 
endif

ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

##############################################################################
# Architecture or project specific options
#
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

ifeq ($(USE_FPU),)
  USE_FPU = no
endif

ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

##############################################################################
# Project, target, sources and paths
#
PROJECT = Progetto-servomotori-seriale

# Impostazioni Target
PROCESSOR = Cortex-M4
MCU       = cortex-m4
BOARD     = ST_NUCLEO64_G474RE

# Percorsi di ChibiOS e Configurazione
CHIBIOS  := ../../chibios2111
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Licensing
include $(CHIBIOS)/os/license/license.mk

# Startup e Port
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32g4xx.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M-ALT/compilers/GCC/mk/port.mk

# RTOS ChibiOS/RT e OSAL standard
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk

# HAL e Board
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32G4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/$(BOARD)/board.mk

# Linker script
LDSCRIPT = $(STARTUPLD)/STM32G474xE.ld

# Sorgenti C del progetto e delle librerie
CSRC = $(ALLCSRC) \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       main.c

# Sorgenti C++
CPPSRC = $(ALLCPPSRC)

# Sorgenti Assembly
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

# Directory di inclusione
INCDIR = $(CONFDIR) \
         $(ALLINC) \
         $(CHIBIOS)/os/hal/lib/streams \
         $(CHIBIOS)/os/license

# Warning opzioni
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes -Wcast-align=strict
CPPWARN = -Wall -Wextra -Wundef

##############################################################################
# User section
#
UDEFS =
UADEFS =
UINCDIR =
ULIBDIR =
ULIBS =

##############################################################################
# Common rules
#
RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
