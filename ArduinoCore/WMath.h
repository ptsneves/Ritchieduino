/*
  Part of the Wiring project - http://wiring.org.co
  Copyright (c) 2004-06 Hernando Barragan
  Modified 04 March 2012, Paulo Neves for Ritchieduino - http://www.arduino.cc/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id$
*/

#ifndef WMATH_H_
#define WMATH_H_

void ArduinoRandomSeed(unsigned int seed);

long ArduinoRandom(long howbig);

long ArduinoMap(long x, long in_min, long in_max, long out_min, long out_max);

unsigned int ArduinoMakeWord(unsigned char h, unsigned char l);


#endif /* WMATH_H_ */
