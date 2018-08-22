# target
TARGET = app

# shell command
CXX = clang++
NVCC = nvcc
RM = rm -fr

# custom DIR, modified according to your own
CAFFE_DIR = ./3rdparty/caffe/distribute

# directory
OUTDIR = build
SRCDIR = src
SUBDIRS = $(shell find $(SRCDIR) -type d)

INCDIR = -Isrc \
	-I/usr/local/cuda/include \
	-I$(CAFFE_DIR)/include \
	`pkg-config --cflags opencv32` \
	`pkg-config --cflags libavcodec` \
	`pkg-config --cflags libavutil` \

LIBDIR = -L/usr/local/cuda/lib64 \
	-L$(CAFFE_DIR)/lib \
	`pkg-config --libs-only-L opencv32` \
	`pkg-config --libs-only-L libavcodec` \
	`pkg-config --libs-only-L libavutil` \

LIBS = -lcaffe \
	-lcudart \
	-lboost_system \
	-lboost_filesystem \
	-pthread \
	-lglog \
	-lgflags \
	`pkg-config --libs-only-l opencv32` \
	`pkg-config --libs-only-l libavcodec` \
	`pkg-config --libs-only-l libavutil` \

# cpp source file
SRCCPP = $(foreach dir,$(SUBDIRS),$(wildcard $(dir)/*.cpp))
CPPOBJS = $(foreach file, $(SRCCPP:.cpp=.cpp.o), $(OUTDIR)/$(file))
CPPDEP = $(foreach file, $(SRCCPP:.cpp=.cpp.d), $(OUTDIR)/$(file))

# cuda source file
SRCCU = $(foreach dir,$(SUBDIRS),$(wildcard $(dir)/*.cu))
CUOBJS = $(foreach file, $(SRCCU:.cu=.cu.o), $(OUTDIR)/$(file))
CUDEP = $(foreach file, $(SRCCU:.cu=.cu.d), $(OUTDIR)/$(file))

# object file
OBJS = $(CPPOBJS) $(CUOBJS)
DEPENDFILES = $(CPPDEP) $(CUDEP)

# Gencode arguments
SMS ?= 50 53 60 61

ifeq ($(SMS),)
$(info >>> WARNING - no SM architectures have been specified - waiving sample <<<)
endif

ifeq ($(GENCODE_FLAGS),)
# Generate SASS code for each SM architecture listed in $(SMS)
$(foreach sm,$(SMS),$(eval GENCODE_FLAGS += -gencode arch=compute_$(sm),code=sm_$(sm)))

# Generate PTX code from the highest SM architecture in $(SMS) to guarantee forward-compatibility
HIGHEST_SM := $(lastword $(sort $(SMS)))
ifneq ($(HIGHEST_SM),)
GENCODE_FLAGS += -gencode arch=compute_$(HIGHEST_SM),code=compute_$(HIGHEST_SM)
endif
endif

#CXXFLAGS = $(INCDIR) -g -fPIC-fpermissive -std=c++11
CXXFLAGS = $(INCDIR) -O3 -fPIC -fpermissive -std=c++14

NVCCFLAGS = $(GENCODE_FLAGS) $(INCDIR) -O4 --use_fast_math -std=c++11

LDFLAGS = $(LIBDIR) $(LIBS) -Wl,-rpath=.:..:lib:$(CAFFE_DIR)/lib

# defination
.SUFFIXES: .cpp .h .d
.PHONY: mk_dir clean all echo

# rules
all: $(OUTDIR)/$(TARGET)

mk_dir:
	@[ -d $(OUTDIR) ] || mkdir -p $(OUTDIR); \
	for val in $(SUBDIRS); do \
		[ -d $(OUTDIR)/$${val} ] || mkdir -p  $(OUTDIR)/$${val};\
	done;

echo:
	@echo 'SUBDIRS:$(SUBDIRS)'
	@echo 'CXXFLAGS:$(CXXFLAGS)'
	@echo 'OBJS:$(OBJS)'
	@echo 'DEPENDFILES:$(DEPENDFILES)'
	@echo 'LDFLAGS:$(LDFLAGS)'

$(OUTDIR)/%.cpp.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OUTDIR)/%.cu.o:%.cu
	$(NVCC) $(NVCCFLAGS) -c $< -o $@

$(OUTDIR)/$(TARGET):$(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	-@ $(RM) $(OUTDIR)/* $(OUTDIR)/$(TARGET)

autoremove:
	-@ $(RM) archive log

# source and header file dependent
-include $(DEPENDFILES)
$(OUTDIR)/%.cpp.d:%.cpp | mk_dir
	@set -e; rm -f $@; \
	$(CXX) -MM $(CXXFLAGS) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	$(RM) $@.$$$$
