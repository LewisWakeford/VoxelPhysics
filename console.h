#ifndef CONSOLE_H_INCLUDED
#define CONSOLE_H_INCLUDED

#include "VP.h"

#include <iostream>
#include <string>

const bool DEBUG_INTERAL_SIMULATION = false;
const bool DEBUG_PHYSICS = false;
const bool DEBUG_MARCHING_CUBES = false;
const bool DEBUG_VOX_DECOMP = false;
const bool DEBUG_BREAKING = false;
const bool DEBUG_BRIDGES = false;

void debugPrint(bool notMuted, const std::string& message);
void debugPrint(bool notMuted, const std::string& message, int value);
void debugPrint(bool notMuted, const std::string& message, float value);
void debugPrint(bool notMuted, const std::string& message, double value);

void consolePrint(std::string outputString);
void consolePrint(std::string outputString, double var);
void errorCheck(int line, std::string filename);

std::string consoleGet();

void consoleClear();

#endif // CONSOLE_H_INCLUDED
