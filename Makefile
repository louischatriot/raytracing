.PHONY: main

all: main

main:
	gcc -o main main.c
	echo "Launching program"
	./main


