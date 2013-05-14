all:
	gcc -o main main.c `pkg-config --cflags --libs gtk+-2.0 gmodule-2.0 opencv`
	clang++ -o stereoalltune stereoalltune.cpp `pkg-config --cflags --libs opencv`

clean:
	rm main
