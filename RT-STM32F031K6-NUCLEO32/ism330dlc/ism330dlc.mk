# ism330dlc PATH
ISM330DLCPATH := ./ism330dlc

# List of all the ISM330DLC device files.
ISM330DLCSRC := $(ISM330DLCPATH)/ism330dlc_reg.c \
				$(ISM330DLCPATH)/platform.c

# Required include directories
ISM330DLCINC := $(ISM330DLCPATH)

# Shared variables
ALLCSRC += $(ISM330DLCSRC)
ALLINC += $(ISM330DLCINC)