
TARGET := learningKame

$(TARGET): $(TARGET).d
	dmd -c Oscillator.d -fPIC
	dmd -c params.d -fPIC
	dmd -c loadJson.d -fPIC -J.
	dmd -c japariSDK/japarilib.d -fPIC
	dmd -c $(TARGET).d -fPIC
	dmd -of$(TARGET).friends $(TARGET).o japarilib.o Oscillator.o params.o loadJson.o -shared -defaultlib=libphobos2.so -L-undefined -Ldynamic_lookup
