# get the system and architecture
ifeq ($(OS),Windows_NT)
    SYSTEM := WIN32
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64) OR ($(PROCESSOR_ARCHITEW6432),AMD64)
        ARCH := AMD64
    endif
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
        ARCH := IA32
    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        SYSTEM := LINUX
    endif
    ifeq ($(UNAME_S),Darwin)
        SYSTEM := OSX
    endif
    UNAME_P := $(shell uname -p)
    ifeq ($(UNAME_P),x86_64)
        ARCH := AMD64
    endif
    ifneq ($(filter %86,$(UNAME_P)),)
        ARCH := IA32
    endif
    ifneq ($(filter arm%,$(UNAME_P)),)
        ARCH := ARM
    endif
    ifneq ($(filter ppc%,$(UNAME_P)),)
        ARCH := PPC
    endif
endif
HOST := $(shell hostname)

ifeq ($(SYSTEM),OSX)
    OPT_FLAGS = -g -O0 -DdDOUBLE 
    #OPT_FLAGS = -O3 -ffast-math -fast -DEXPERIMENTAL
    CXXFLAGS = -Wall -fexceptions $(OPT_FLAGS) -DdDOUBLE
    CFLAGS = -Wall $(OPT_FLAGS) -DdDOUBLE
    # suggested by linker
    # LDFLAGS = -Xlinker -bind_at_load $(OPT_FLAGS) 
    LDFLAGS = $(OPT_FLAGS) 
    LIBS = $(HOME)/Unix/lib/libode.a ${HOME}/Unix/lib/libANN.a -lxml2 -lpthread -lm -lz -framework CoreServices
    OPENGL_LIBS = $(HOME)/Unix/lib/libglui.a -framework GLUT -framework OpenGL
    INC_DIRS = -I$(HOME)/Unix/include -I/usr/include/libxml2
    OPENGL_INC = -I/System/Library/Frameworks/GLUT.framework/Versions/A/Headers -I/System/Library/Frameworks/OpenGL.framework/Versions/A/Headers/
endif

ifeq ($(SYSTEM),LINUX)
    ifeq ($(ARCH),PPC) 
        CXXFLAGS = -O3 -DdDOUBLE 
        LDFLAGS  =  
        CXX      = mpic++
        CC       = mpicc
        LIBS = -L"$(HOME)/Unix/lib" -lode -lxml2 -lpthread -lm  
        INC_DIRS = -I"$(HOME)/Unix/include" -I"$(HOME)/Unix/include/libxml2"
    endif
    
    ifeq ($(ARCH),AMD64)
        #CXXFLAGS = -static -ffast-math -O3 -DdDOUBLE -DEXPERIMENTAL
        CXXFLAGS = -static -O3 -DdDOUBLE -DEXPERIMENTAL -std=c++11
        LDFLAGS  = -static 
        CXX      = CC
        CC       = cc
        LIBS = -L"$(HOME)/Unix/lib" -lode -lANN -lxml2 -lpthread -lm -lz 
        INC_DIRS = -I"$(HOME)/Unix/include" -I/usr/include/libxml2 
    endif
endif


# vpath %.cpp src
# vpath %.c src 

GAITSYMSRC = \
AMotorJoint.cpp                 Driver.cpp                      FacetedSphere.cpp               NPointStrap.cpp                 TIFFWrite.cpp\
BallJoint.cpp                   Environment.cpp                 FixedJoint.cpp                  NamedObject.cpp                 ThreePointStrap.cpp\
Body.cpp                        ErrorHandler.cpp                FloatingHingeJoint.cpp          ObjectiveMain.cpp               TorqueReporter.cpp\
CappedCylinderGeom.cpp          ExpressionFolder.cpp            OpenCLRoutines.cpp              TrimeshGeom.cpp\
Contact.cpp                     ExpressionFunTransform3D.cpp    GLUtils.cpp                     PIDMuscleLength.cpp             TwoPointStrap.cpp\
Controller.cpp                  ExpressionMat.cpp               Geom.cpp                        PlaneGeom.cpp                   UDP.cpp\
CyclicDriver.cpp                ExpressionParser.cpp            HingeJoint.cpp                  PositionReporter.cpp            UGMMuscle.cpp\
CylinderWrapStrap.cpp           ExpressionRef.cpp               RayGeom.cpp                     UniversalJoint.cpp\
DampedSpringMuscle.cpp          ExpressionVar.cpp               Joint.cpp                       Reporter.cpp                    Util.cpp\
DataFile.cpp                    ExpressionVec.cpp               MAMuscle.cpp                    Simulation.cpp                  XMLConverter.cpp\
DataTarget.cpp                  Face.cpp                        MAMuscleComplete.cpp            SphereGeom.cpp                  fec.cpp\
DataTargetQuaternion.cpp        FacetedCappedCylinder.cpp       MAMuscleExtended.cpp            StepDriver.cpp                  SwingClearanceAbortReporter.cpp\
DataTargetScalar.cpp            FacetedConicSegment.cpp         Strap.cpp                       TwoCylinderWrapStrap.cpp        SliderJoint.cpp\
DataTargetVector.cpp            FacetedObject.cpp               Marker.cpp                      StrokeFont.cpp                  BoxGeom.cpp\
Drivable.cpp                    FacetedPolyline.cpp             Muscle.cpp                      TCP.cpp                         FacetedBox.cpp\
BoxCarDriver.cpp                StackedBoxCarDriver.cpp         PIDTargetMatch.cpp              Warehouse.cpp                   FixedDriver.cpp\
PCA.cpp

GAITSYMOBJ = $(addsuffix .o, $(basename $(GAITSYMSRC) ) )
GAITSYMHEADER = $(addsuffix .h, $(basename $(GAITSYMSRC) ) ) PGDMath.h DebugControl.h SimpleStrap.h

BINARIES = bin/gaitsym bin/gaitsym_opengl bin/gaitsym_udp bin/gaitsym_opengl_udp bin/gaitsym_tcp bin/gaitsym_opengl_tcp

BINARIES_NO_OPENGL = bin/gaitsym bin/gaitsym_udp bin/gaitsym_tcp

all: directories binaries 

no_opengl: directories binaries_no_opengl

directories: bin obj 

binaries: $(BINARIES)

binaries_no_opengl: $(BINARIES_NO_OPENGL)

obj: 
	-mkdir obj
	-mkdir obj/no_opengl
	-mkdir obj/opengl
	-mkdir obj/no_opengl_udp
	-mkdir obj/opengl_udp
	-mkdir obj/no_opengl_tcp
	-mkdir obj/opengl_tcp

bin:
	-mkdir bin
	 
obj/no_opengl/%.o : src/%.cpp
	$(CXX) $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym: $(addprefix obj/no_opengl/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(LIBS)

obj/opengl/%.o : src/%.cpp
	$(CXX) -DUSE_OPENGL $(CXXFLAGS) $(INC_DIRS) $(OPENGL_INC)  -c $< -o $@

bin/gaitsym_opengl: $(addprefix obj/opengl/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(OPENGL_LIBS) $(LIBS) 

obj/no_opengl_socket/%.o : src/%.cpp
	$(CXX) -DUSE_SOCKETS $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym_socket: $(addprefix obj/no_opengl_socket/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(SOCKET_LIBS) $(LIBS)

obj/opengl_socket/%.o : src/%.cpp
	$(CXX) -DUSE_OPENGL -DUSE_SOCKETS $(CXXFLAGS) $(INC_DIRS) $(OPENGL_INC)  -c $< -o $@

bin/gaitsym_opengl_socket: $(addprefix obj/opengl_socket/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(SOCKET_LIBS) $(OPENGL_LIBS) $(LIBS) 

obj/no_opengl_udp/%.o : src/%.cpp
	$(CXX) -DUSE_UDP $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym_udp: $(addprefix obj/no_opengl_udp/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(UDP_LIBS) $(LIBS)

obj/opengl_udp/%.o : src/%.cpp
	$(CXX) -DUSE_OPENGL -DUSE_UDP $(CXXFLAGS) $(INC_DIRS) $(OPENGL_INC)  -c $< -o $@

bin/gaitsym_opengl_udp: $(addprefix obj/opengl_udp/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(UDP_LIBS) $(OPENGL_LIBS) $(LIBS) 

obj/no_opengl_tcp/%.o : src/%.cpp
	$(CXX) -DUSE_TCP $(CXXFLAGS) $(INC_DIRS)  -c $< -o $@

bin/gaitsym_tcp: $(addprefix obj/no_opengl_tcp/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(TCP_LIBS) $(LIBS)

obj/opengl_tcp/%.o : src/%.cpp
	$(CXX) -DUSE_OPENGL -DUSE_TCP $(CXXFLAGS) $(INC_DIRS) $(OPENGL_INC)  -c $< -o $@

bin/gaitsym_opengl_tcp: $(addprefix obj/opengl_tcp/, $(GAITSYMOBJ) ) 
	$(CXX) $(LDFLAGS) -o $@ $^ $(TCP_LIBS) $(OPENGL_LIBS) $(LIBS) 


clean:
	rm -rf obj bin
	rm -rf distribution
	rm -rf build*

superclean:
	rm -rf obj bin
	rm -rf distribution
	rm -rf build*
	rm -rf GaitSymQt/GaitSymQt.pro.user.*
	find . -name '.DS_Store' -exec rm -f {} \;
	find . -name '.gdb_history' -exec rm -f {} \;
	find . -name '.#*' -exec rm -f {} \;
	find . -name '*~' -exec rm -f {} \;
	find . -name '#*' -exec rm -f {} \;
	find . -name '*.bak' -exec rm -f {} \;
	find . -name '*.bck' -exec rm -f {} \;
	find . -name '*.tmp' -exec rm -f {} \;
	find . -name '*.o' -exec rm -f {} \;

distribution: distribution_dirs gaitsym_distribution gaitsym_distribution_extras

distribution_dirs:
	rm -rf distribution
	-mkdir distribution
	-mkdir distribution/src

gaitsym_distribution: $(addprefix distribution/src/, $(GAITSYMSRC)) $(addprefix distribution/src/, $(GAITSYMHEADER))

$(addprefix distribution/src/, $(GAITSYMSRC)):
	scripts/strip_ifdef.py EXPERIMENTAL $(addprefix src/, $(notdir $@)) $@ 

$(addprefix distribution/src/, $(GAITSYMHEADER)):
	scripts/strip_ifdef.py EXPERIMENTAL $(addprefix src/, $(notdir $@)) $@ 

gaitsym_distribution_extras:
	cp -rf GaitSymQt distribution/
	cp makefile distribution/
	rm -rf distribution/CVS distribution/*/CVS distribution/*/*/CVS distribution/*/*/*/CVS distribution/*/*/*/*/CVS
	rm -rf distribution/GaitSymQt/GaitSymQt.pro.*


