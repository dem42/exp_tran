CC      = c++
LD      = c++
CFLAGS  = -I/usr/X11R6/include -I. -c
LDFLAGS = -L/usr/X11R6/lib -lglut -lGLU -lGL -lXi -lXmu -lXt -lXext -lX11 -lSM -lICE -lm

INCS = 

OBJS = cgRender.o Face.o Vector3.o

all: cgRender

clean:
	rm -rf $(OBJS) cgRender

cgRender.o: $(INCS) cgRender.cpp
	$(CC) $(CFLAGS) cgRender.cpp -o cgRender.o

Face.o: $(INCS) Face.cpp
	$(CC) $(CFLAGS) Face.cpp -o Face.o

cgRender: $(OBJS)
	$(LD) $(OBJS) -o cgRender $(LDFLAGS)
