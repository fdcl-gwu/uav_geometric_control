
CC=g++
RM=rm -f
CPPFLAGS=-g -Wall
LDFLAGS=
SOURCES=main.cpp controller.cpp
OBJS=$(SOURCES:.cpp=.o)
PROG=control

$(OBJS):$(OBJES)
	$(CC) $(CFLAGS) -o $(PROG) $(OBJS)

all:main.o controller.o

controller.o: controller.hpp math.hpp
	$(CC) $(CPPFLAGS) -c controller.cpp
main.o:
	$(CC) $(CPPFLAGS) -c main.cpp

# $(EXECUTABLE): $(OBJECTS)
#     $(CC) $(LDFLAGS) $(OBJECTS) -o $@

# .cpp.o:
#     $(CC) $(CFLAGS) $< -o $@
#
clean:
	${RM} core $(PROG) $(OBJS)
