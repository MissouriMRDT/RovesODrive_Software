#include "RovesODrive.h"
#include <Energia.h>
#include <Stream.h>


void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], int value)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], float value = 0.0)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %f\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

void writeODrive(Stream& mySerial, bool write_read, char id[MAX_STRING_CHARS], bool value = TRUE)
{
	char string[MAX_STRING_CHARS];
	sprintf(string, "%c %s %d\n", write_read? 'w':'r', id, value);
	mySerial.write(string);
	Serial.println(string);
}

