CFLAGS=-ggdb -g3 -Wall
LIB_FLAGS=-L. -lrobot_if
LIB_LINK=-lhighgui -lcv -lcxcore -lm -lgslcblas -L/usr/lib64/atlas -lclapack -lrt

PROGS=robot cal

all: ${PROGS}

cal: theta_cal.o northstar.o wheel_encoder.o filter.o position.o
	gcc ${CFLAGS} -o cal theta_cal.o position.o northstar.o wheel_encoder.o filter.o matvec.o rovioKalmanFilter.o ${LIB_FLAGS} ${LIB_LINK}

robot: robot_assign1.o northstar.o wheel_encoder.o filter.o position.o rovioKalmanFilter.o PID_Control.o robot_vision.o
	gcc ${CFLAGS} -o robot robot_assign1.o position.o northstar.o wheel_encoder.o filter.o matvec.o rovioKalmanFilter.o PID_Control.o robot_vision.o  ${LIB_FLAGS} ${LIB_LINK}

theta_cal.o: theta_cal.c position.o matvec.o rovioKalmanFilter.o
	gcc ${CFLAGS} -c theta_cal.c

robot_assign1.o: robot_assign1.c position.o matvec.o
	gcc ${CFLAGS} -c robot_assign1.c -lrt

position.o: position.c northstar.o wheel_encoder.o matvec.o rovioKalmanFilter.o
	gcc ${CFLAGS} -c position.c

northstar.o: northstar.c matvec.o
	gcc ${CFLAGS} -c northstar.c

wheel_encoder.o: wheel_encoder.c matvec.o
	gcc ${CFLAGS} -c wheel_encoder.c

filter.o: filter.c
	gcc ${CFLAGS} -c filter.c

PID_Control.o: PID_Control.c
	gcc ${CFLAGS} -c PID_Control.c

matvec.o: matvec.c
	gcc ${CFLAGS} -c matvec.c

rovioKalmanFilter.o: rovioKalmanFilter.c
	gcc ${CFLAGS} -c rovioKalmanFilter.c

robot_vision.o: robot_vision.c
	gcc ${CFLAGS} -c robot_vision.c ${LIB_FLAGS}	

clean:
	rm -rf *.o *~ *.orig
	rm -rf ${PROGS}