ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/hal_usb_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/hal_usb_lld.c
endif

PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/hidram.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/hiduser.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbdesc.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbepfunc.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbhw.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbram.c
PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbsystem.c
# PLATFORMSRC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB/usbuser.c

PLATFORMINC += $(CHIBIOS)/os/hal/ports/SN32/LLD/USB
