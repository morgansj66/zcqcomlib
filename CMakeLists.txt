cmake_minimum_required(VERSION 3.11)
project(zcqcomlib VERSION 1.0.0 DESCRIPTION "Zeno CANquatro communication library" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(SOURCES
  src/canlib.cpp
  src/linlib.cpp
  src/zdebug.cpp
  src/zcqcore.cpp
  src/PCANBasic.cpp
  src/zusbcontext.cpp
  src/zusbeventthread.cpp
  src/zzenocanchannel.cpp
  src/zzenocandriver.cpp
  src/zzenolinchannel.cpp
  src/zzenolindriver.cpp
  src/zzenousbdevice.cpp
)

set(HEADERS
  include/canlib.h
  include/canstat.h
  include/obsolete.h
  include/linlib.h
  include/vcanevt.h
  include/zpushpack.h
  include/zpoppack.h
  include/PCANBasic.h
  src/zglobal.h
  src/zdebug.h
  src/zring.h
  src/zcqcore.h  
  src/zrefcountingobjbase.h
  src/zcanchannel.h
  src/zcandriver.h
  src/zlinchannel.h
  src/zlindriver.h
  src/zenocan.h
  src/zusbeventthread.h
  src/zzenocanchannel.h
  src/zzenolinchannel.h
  src/zzenousbdevice.h
  src/zcandriverfactory.h
  src/zcanflags.h
  src/zusbcontext.h
  src/zusb.h
  src/zzenocandriver.h
  src/zzenolindriver.h
)

set(INLCUDE_DIRS
    ${LIBUSB_INCLUDE_DIRS}
    include
)

add_library(zcqcomlib SHARED ${SOURCES} ${HEADERS} )

target_link_libraries(zcqcomlib ${LIBUSB_LIBRARIES})
target_include_directories(zcqcomlib PUBLIC ${INLCUDE_DIRS})
target_compile_options(zcqcomlib PUBLIC ${LIBUSB_CFLAGS_OTHER})

add_subdirectory(examples)
