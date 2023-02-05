CC = g++
CFLAGS = -g -Wall -std=c++20 -Iinclude

default: all

run : scripts/run.cpp
	${CC} ${CFLAGS} -c scripts/run.cpp -o bin/example 
	${CC} ${CFLAGS} scripts/run.cpp src/* -o bin/run_script

app: run.o
	mkdir -p bin
	${CC} $()

all : clean run

clean:
	rm -rf build/*
	rm -rf bin/*