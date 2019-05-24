
#	Meson DVB drivers

obj-$(CONFIG_MESON_DVB) += dvb_meson.o aml_fe.o wetekdvb.o

dvb_meson-objs = meson_fe.o avl6211.o mn88436.o cxd2841er_wetek.o ascot3.o mxl603.o mxl608.o avl6862.o r912.o tuner_ftm4862.o rda5815m.o
aml_fe-objs    = aml_dvb.o aml_dmx.o 
wetekdvb-objs  = wetek_dvb.o wetek_dmx.o 

EXTRA_CFLAGS += -I. -DDUAL_TUNER
EXTRA_CFLAGS += -Idrivers/media/dvb-core -Idrivers/media/usb/dvb-usb -Idrivers/media/dvb-frontends -Idrivers/media/tuners

