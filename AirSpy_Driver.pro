#==========================================================================================
# + + +   This Software is released under the "Simplified BSD License"  + + +
# Copyright 2014 F4GKR Sylvain AZARIAN . All rights reserved.
#
#Redistribution and use in source and binary forms, with or without modification, are
#permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright notice, this list of
#	  conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright notice, this list
#	  of conditions and the following disclaimer in the documentation and/or other materials
#	  provided with the distribution.
#
#THIS SOFTWARE IS PROVIDED BY Sylvain AZARIAN F4GKR ``AS IS'' AND ANY EXPRESS OR IMPLIED
#WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
#FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Sylvain AZARIAN OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#The views and conclusions contained in the software and documentation are those of the
#authors and should not be interpreted as representing official policies, either expressed
#or implied, of Sylvain AZARIAN F4GKR.
#
# Adds AirSpy capability to SDRNode
#==========================================================================================


QT       -= core gui

TARGET = CloudSDR_AirSpy
TEMPLATE = lib
 
LIBS += -lpthread  
win32 {
    DESTDIR = C:/SDRNode/addons
    LIBS += -lusb-1.0
    RC_FILE = resources.rc
}

unix {
    DESTDIR = /opt/sdrnode/addons
}

SOURCES += \
    entrypoint.cpp \
    jansson/dump.c \
    jansson/error.c \
    jansson/hashtable.c \
    jansson/hashtable_seed.c \
    jansson/load.c \
    jansson/memory.c \
    jansson/pack_unpack.c \
    jansson/strbuffer.c \
    jansson/strconv.c \
    jansson/utf.c \
    jansson/value.c \
    AirSpy.c \
    iqconverter_float.c \
    iqconverter_int16.c

HEADERS +=\
    external_hardware_def.h \
    entrypoint.h \
    jansson/hashtable.h \
    jansson/jansson.h \
    jansson/jansson_config.h \
    jansson/jansson_private.h \
    jansson/lookup3.h \
    jansson/strbuffer.h \
    jansson/utf.h \
    AirSpy.h \
    airspy_commands.h \
    filters.h \
    iqconverter_float.h \
    iqconverter_int16.h \
    driver_version.h

OTHER_FILES += \
    resources.rc

unix {
    #target.path = /usr/lib
    #INSTALLS += target
}
