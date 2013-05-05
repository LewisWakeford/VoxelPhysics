#ifndef CONSOLE_H_INCLUDED
#define CONSOLE_H_INCLUDED

#include "VP.h"

#include <iostream>
#include <string>

void consolePrint(std::string outputString);
void consolePrint(std::string outputString, double var);
void errorCheck(int line, std::string filename);

std::string consoleGet();

void consoleClear();

#endif // CONSOLE_H_INCLUDED
