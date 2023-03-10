CC = g++
CFLAGS = -g -Wall -std=c++11 -Iinclude

default: all

run : scripts/run.cpp
	#${CC} ${CFLAGS} -c scripts/run.cpp -o bin/example 
	${CC} ${CFLAGS} scripts/run.cpp src/* -o bin/run_script -pthread
	${CC} ${CFLAGS} scripts/test_run.cpp src/* -o bin/test -pthread

app: run.o
	mkdir -p bin

all : clean run

clean:
	rm -rf build/*
	rm -rf bin/*
