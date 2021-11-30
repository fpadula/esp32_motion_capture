CC=g++
CFLAGS=-lstdc++ -lpthread -Wall -I../../firmware/include/
# CFLAGS=-Wall -lstdc++
# TESTFOLDER = tests/

all: IMUManager.o Imu.o ./src/main.cpp
	$(CC) $(CFLAGS) -o ./bin/main ./src/main.cpp ./bin/IMU.o ./bin/IMUManager.o

Imu.o: ./src/IMU.cpp ./src/IMU.h
	$(CC) $(CFLAGS) -c ./src/IMU.cpp -o ./bin/IMU.o

IMUManager.o: ./src/IMUManager.cpp ./src/IMUManager.h
	$(CC) $(CFLAGS) -c ./src/IMUManager.cpp -o ./bin/IMUManager.o

client:
	$(CC) $(CFLAGS) -o ./bin/client ./src/client.cpp

# Controller.o: ./src/Controller.cpp ./src/Controller.h
# 	$(CC) $(CFLAGS) -c ./src/Controller.cpp -o ./bin/Controller.o

# main.o: Controller.o ./src/main.cpp
# 	$(CC) $(CFLAGS) -c ./src/main.cpp -o ./bin/main.o

# python_api:	
# 	cd src; python setup.py build_ext --inplace

# ikfast.o: ./src/ikfast.cpp ./src/ikfast.h
# 	$(CC) $(CFLAGS) -c ./src/ikfast.cpp -o ./bin/ikfast.o 

# test: $(TESTFOLDER)/*
# 	bin/main < $^

clean:
	rm ./bin/*