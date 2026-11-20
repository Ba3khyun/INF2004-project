# Host (PC) unit tests for subsystem 4.  Needs gcc and make (e.g. MSYS2/WSL on Windows).
SRC = ../../../app_program/subsystem4
INC = -I../../../app_program -I$(SRC)

test_imu_math: test_imu_math.c $(SRC)/imu_math.c $(SRC)/imu_math.h ../../../app_program/app_data.h
	gcc -std=c11 -Wall -Wextra -O2 $(INC) test_imu_math.c $(SRC)/imu_math.c -lm -o test_imu_math

run: test_imu_math
	./test_imu_math

clean:
	rm -f test_imu_math test_imu_math.exe

.PHONY: run clean
