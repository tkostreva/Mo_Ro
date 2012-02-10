CFLAGS=-ggdb -g3 -Wall
LIB_FLAGS=-L. -lrobot_if
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack

all: robot

robot: robot_assign1.o northstar.o wheel_encoder.o filter.o position.o
	gcc ${CFLAGS} -o robot robot_assign1.o position.o northstar.o wheel_encoder.o filter.o matvec.o rovioKalmanFilter.o  ${LIB_FLAGS} ${LIB_LINK}

robot_assign1.o: robot_assign1.c position.o matvec.o
	gcc ${CFLAGS} -c robot_assign1.c

position.o: position.c northstar.o wheel_encoder.o matvec.o rovioKalmanFilter.o
	gcc ${CFLAGS} -c position.c

northstar.o: northstar.c matvec.o
	gcc ${CFLAGS} -c northstar.c

wheel_encoder.o: wheel_encoder.c matvec.o
	gcc ${CFLAGS} -c wheel_encoder.c

filter.o: filter.c
	gcc ${CFLAGS} -c filter.c

matvec.o: matvec.c
	gcc ${CFLAGS} -c matvec.c

rovioKalmanFilter.o: rovioKalmanFilter.c
	gcc ${CFLAGS} -c rovioKalmanFilter.c


clean:
	rm -rf *.o
	rm -rf *~
	rm -rf robot