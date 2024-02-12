gcc -c -fPIC main.c -o tier2.o
gcc tier2.o -shared -o tier2.so
