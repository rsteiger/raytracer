all:
	nvcc kernelPBO.cu callbacksPBO.cpp simpleGLmain.cpp simplePBO.cpp -DGL_GLEXT_PROTOTYPES -I/opt/local/include/ -L/Developer/SDKs/MacOSX10.6.sdk/usr/X11/lib/ -lglut -lGL -O3
