CFLAGS=-ggdb -g3
LIB_FLAGS=-L. -lrobot_if
LIB_LINK=-lhighgui -lcv -lcxcore -lm

all: robot

robot: robot_assign1.o northstar.o wheel_encoder.o filter.o position.o
	gcc ${CFLAGS} -o robot robot_assign1.o position.o northstar.o wheel_encoder.o filter.o matvec.o ${LIB_FLAGS} ${LIB_LINK}

robot_assign1.o: robot_assign1.c position.o
	gcc ${CFLAGS} -c robot_assign1.c

position.o: position.c northstar.o wheel_encoder.o matvec.o
	gcc ${CFLAGS} -c position.c

northstar.o: northstar.c filter.o
	gcc ${CFLAGS} -c northstar.c

wheel_encoder.o: wheel_encoder.c
	gcc ${CFLAGS} -c wheel_encoder.c

filter.o: filter.c
	gcc ${CFLAGS} -c filter.c

matvec.o: matvec.c
	gcc ${CFLAGS} -c matvec.c

clean:
	rm -rf *.o
	rm -rf *~
	rm -rf robot