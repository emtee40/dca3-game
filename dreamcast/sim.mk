TARGET ?= dca3-sim.elf

IS_MAC := $(shell uname -s | grep -i "darwin" > /dev/null && echo "yes" || echo "no")


include common.mk

OBJS = $(RE3_OBJS) $(RW_OBJS) \
	../src/audio/sampman_null.o

OBJS_SIM=$(OBJS:.o=.sim.o) \
	../vendor/koshle/hlekos.sim.o \
	../vendor/koshle/hlematrix3d.sim.o \
	../vendor/koshle/hlepvr_mem.sim.o \
	../vendor/koshle/hlepvr_prim.sim.o \
	../vendor/koshle/hlepvr_scene.sim.o \
	../vendor/koshle/hlepvr_misc.sim.o \
	../vendor/koshle/hlepvr_init_term.sim.o \
	../vendor/koshle/hlepvr_buffers.sim.o \
	../vendor/koshle/hlepvr_irq.sim.o \
	../vendor/koshle/hlepvr_fog.sim.o \
	\
	../vendor/emu/emu/window.sim.o \
	\
	../vendor/emu/lxdream/tacore.sim.o3 \
	\
	../vendor/emu/refsw/pvr_mem.sim.o3 \
	../vendor/emu/refsw/pvr_regs.sim.o3 \
	../vendor/emu/refsw/refsw_lists.sim.o3 \
	../vendor/emu/refsw/refsw_tile.sim.o3 \
	../vendor/emu/refsw/TexUtils.sim.o3 \
	
DEPS_SIM1=$(OBJS_SIM:.o=.d)
DEPS_SIM=$(DEPS_SIM1:.o3=.d)

CXXFLAGS+= -MMD -MP

ifeq ($(IS_MAC), yes)
%.sim.o: %.c
	$(CC) -c -O0 -g -fno-pic -no-pie -o $@ $(CFLAGS) -I../vendor/koshle -I../vendor/emu -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 -DMACOS64 $<
%.sim.o: %.cpp
	$(CXX) -c -O0 -g -fno-pic -no-pie -o $@ $(CXXFLAGS) -I../vendor/koshle -I../vendor/emu -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 -DMACOS64 $<
%.sim.o3: %.cpp
	$(CXX) -c -O3 -g -fno-pic -no-pie -o $@ $(CXXFLAGS) -I../vendor/koshle -I../vendor/emu -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 -DMACOS64 $<
else
# Using sse2 here for valgrind compatibility
%.sim.o: %.c
	$(CC) -msse2 -mfpmath=sse -c -O0 -g -fno-pic -no-pie -o $@ $(CFLAGS) -I../vendor/koshle -I../vendor/emu -m32 -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 $<
%.sim.o: %.cpp
	$(CXX) -msse2 -mfpmath=sse -c -O0 -g -fno-pic -no-pie -o $@ $(CXXFLAGS) -I../vendor/koshle -I../vendor/emu -m32 -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 $<
%.sim.o3: %.cpp
	$(CXX) -msse2 -mfpmath=sse -c -O3 -g -fno-pic -no-pie -o $@ $(CXXFLAGS) -I../vendor/koshle -I../vendor/emu -m32 -U_WIN32 -UWIN32 -UWINNT -Ui386 -DDC_SIM -D_FILE_OFFSET_BITS=64 $<	
endif

all: $(TARGET)

clean:
	-rm -f $(OBJS_SIM) $(TARGET)

ifeq ($(IS_MAC), yes)
$(TARGET): $(OBJS_SIM)
	$(CXX) -fno-pic -no-pie -o $(TARGET) $(OBJS_SIM) -lX11
else
$(TARGET): $(OBJS_SIM)
	$(CXX) -m32 -fno-pic -no-pie -o $(TARGET) $(OBJS_SIM) -lX11
endif
-include $(DEPS_SIM)