SOURCE_DIR = src
SOURCE_SDK = SDK
XNZ_XP_DLL = x-nullzones.xpl
XN_INCLUDE = -I$(SOURCE_DIR)
XP_INCLUDE = -I$(SOURCE_SDK)/CHeaders
XP_LD_LIBS = -F$(SOURCE_SDK)/Libraries/Mac -framework XPLM -framework XPWidgets
XPCPPFLAGS = -DXPLM200 -DXPLM210 -DAPL=1 -DIBM=0 -DLIN=0
CFLAGS     = -O3 -std=c99 -mmacosx-version-min=10.10
TARGETARCH = -arch x86_64
CC         = clang

XNZ_LDFLAGS = -dynamiclib -fvisibility=hidden
XNZ_HEADERS = $(wildcard $(SOURCE_DIR)/*.h)
XNZ_SOURCES = $(wildcard $(SOURCE_DIR)/*.c)
XNZ_OBJECTS = $(addsuffix .o,$(basename $(notdir $(XNZ_SOURCES))))

all: xnz

xnz: xnzobj
	$(CC) $(XNZ_INCLUDE) $(XP_INCLUDE) $(XPCPPFLAGS) $(CFLAGS) $(XNZ_LDFLAGS) $(TARGETARCH) -o $(XNZ_XP_DLL) $(XNZ_OBJECTS) $(XP_LD_LIBS)

xnzobj: $(XNZ_SOURCES) $(XNZ_HEADERS)
	$(CC) $(XNZ_INCLUDE) $(XP_INCLUDE) $(XPCPPFLAGS) $(CFLAGS) $(XNZCPPFLAGS) $(TARGETARCH) -c $(XNZ_SOURCES)

public:
	$(MAKE) XNZ_XP_DLL="quadrant.314.mac.xpl" CFLAGS="$(CFLAGS) -DPUBLIC_RELEASE_BUILD" all

.PHONY: clean
clean:
	$(RM) quadrant.314.mac.xpl $(XNZ_XP_DLL) $(XNZ_OBJECTS)
