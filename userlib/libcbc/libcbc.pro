######################################################################
# Automatically generated by qmake (2.01a) Sat Jan 10 06:32:45 2009
######################################################################

TEMPLATE = lib
TARGET = cbc
DEPENDPATH += . src
INCLUDEPATH += . src

INCLUDEPATH += ../../utils/shared_mem

INCLUDEPATH += ../libcommonc++

CONFIG += staticlib thread
CONFIG -= qt debug debug_and_release debug_and_release_target

# Input
HEADERS += src/cbc.h \
           src/cbc_data.h \
           src/compat.h \
           src/process.h \
           src/cbcserial.h \
           src/create.h \
           src/ARDrone.h \
           src/cbc2cxx.h \
           ../shared_mem/shared_mem.h
SOURCES += src/botball.c \ 
           src/cbc.c \
           src/compat.c \
           src/process.c \
           src/cbcserial.c \
           src/create.c \
           src/ARDrone.cpp
