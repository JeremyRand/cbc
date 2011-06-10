/**************************************************************************
 *  Copyright 2008,2009 KISS Institute for Practical Robotics             *
 *                                                                        *
 *  This file is part of CBC Firmware.                                    *
 *                                                                        *
 *  CBC Firmware is free software: you can redistribute it and/or modify  *
 *  it under the terms of the GNU General Public License as published by  *
 *  the Free Software Foundation, either version 2 of the License, or     *
 *  (at your option) any later version.                                   *
 *                                                                        *
 *  CBC Firmware is distributed in the hope that it will be useful,       *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *  GNU General Public License for more details.                          *
 *                                                                        *
 *  You should have received a copy of the GNU General Public License     *
 *  along with this copy of CBC Firmware.  Check the LICENSE file         *
 *  in the project root.  If not, see <http://www.gnu.org/licenses/>.     *
 **************************************************************************/

// System includes
#include <stdio.h>

// Local includes
#include "ctdebug.h"

// Self
#include "HSVRangeLUT.h"

void HSVRangeLUT::setModel(uint8 channel, const HSVRange &range)
{
  m_models[channel] = range;
  uint8 mask = 1<<channel;
  for (unsigned p = 0; p < Pixel565::MAXVAL+1; p++) {
    HSV hsv = Pixel565toHSV::convert(Pixel565(p));
    if (range.contains(Pixel565(p))) {
      m_lut[p] |= mask;
    } else {
      m_lut[p] &= ~mask;
    }
  }
}

// channel = channelIn
void HSVRangeLUT::applyCopy(uint8 channel, uint8 channelIn)
{
  //m_models[channel] = range;
  uint8 mask = 1<<channel;
  uint8 maskIn = 1<<channelIn;
  for (unsigned p = 0; p < Pixel565::MAXVAL+1; p++) {
    //HSV hsv = Pixel565toHSV::convert(Pixel565(p));
    //if (range.contains(Pixel565(p))) {
    if ( (m_lut[p] & maskIn) ) {
      m_lut[p] |= mask; // set the output
    } else {
      m_lut[p] &= ~mask; // clear the output
    }
  }
}

// channel = Not channelIn
void HSVRangeLUT::applyNot(uint8 channel, uint8 channelIn)
{
  //m_models[channel] = range;
  uint8 mask = 1<<channel;
  uint8 maskIn = 1<<channelIn;
  for (unsigned p = 0; p < Pixel565::MAXVAL+1; p++) {
    //HSV hsv = Pixel565toHSV::convert(Pixel565(p));
    //if (range.contains(Pixel565(p))) {
    if (! (m_lut[p] & maskIn) ) {
      m_lut[p] |= mask; // set the output
    } else {
      m_lut[p] &= ~mask; // clear the output
    }
  }
}

// channel = channelInA And channelInB
void HSVRangeLUT::applyAnd(uint8 channel, uint8 channelInA, uint8 channelInB)
{
  //m_models[channel] = range;
  uint8 mask = 1<<channel;
  uint8 maskInA = 1<<channelInA;
  uint8 maskInB = 1<<channelInB;
  for (unsigned p = 0; p < Pixel565::MAXVAL+1; p++) {
    //HSV hsv = Pixel565toHSV::convert(Pixel565(p));
    //if (range.contains(Pixel565(p))) {
    if ( (m_lut[p] & maskInA) && (m_lut[p] & maskInB) ) {
      m_lut[p] |= mask; // set the output
    } else {
      m_lut[p] &= ~mask; // clear the output
    }
  }
}

// channel = channelInA Or channelInB
void HSVRangeLUT::applyOr(uint8 channel, uint8 channelInA, uint8 channelInB)
{
  //m_models[channel] = range;
  uint8 mask = 1<<channel;
  uint8 maskInA = 1<<channelInA;
  uint8 maskInB = 1<<channelInB;
  for (unsigned p = 0; p < Pixel565::MAXVAL+1; p++) {
    //HSV hsv = Pixel565toHSV::convert(Pixel565(p));
    //if (range.contains(Pixel565(p))) {
    if ( (m_lut[p] & maskInA) || (m_lut[p] & maskInB) ) {
      m_lut[p] |= mask; // set the output
    } else {
      m_lut[p] &= ~mask; // clear the output
    }
  }
}

void HSVRangeLUT::test()
{
  HSVRangeLUT lut;

  lut.setModel(0, HSVRange(HSV(  0, 128, 128), HSV(120, 255, 255)));   // contains red, yellow, green
  lut.setModel(1, HSVRange(HSV(235, 128, 128), HSV(245, 255, 255))); // contains blue
  lut.setModel(2, HSVRange(HSV(300, 128, 128), HSV( 60, 255, 255))); // contains magenta, red, yellow
  lut.setModel(3, HSVRange(HSV(300, 255, 255), HSV( 60,   0,   0))); // contains no colors

  ctassert(!lut.contains(0, Pixel565::black()));
  ctassert(!lut.contains(0, Pixel565::white()));
  ctassert( lut.contains(0, Pixel565::red()));
  ctassert( lut.contains(0, Pixel565::yellow()));
  ctassert( lut.contains(0, Pixel565::green()));
  ctassert(!lut.contains(0, Pixel565::cyan()));
  ctassert(!lut.contains(0, Pixel565::blue()));
  ctassert(!lut.contains(0, Pixel565::magenta()));
  
  ctassert(!lut.contains(1, Pixel565::black()));
  ctassert(!lut.contains(1, Pixel565::white()));
  ctassert(!lut.contains(1, Pixel565::red()));
  ctassert(!lut.contains(1, Pixel565::yellow()));
  ctassert(!lut.contains(1, Pixel565::green()));
  ctassert(!lut.contains(1, Pixel565::cyan()));
  ctassert( lut.contains(1, Pixel565::blue()));
  ctassert(!lut.contains(1, Pixel565::magenta()));
  
  ctassert(!lut.contains(2, Pixel565::black()));
  ctassert(!lut.contains(2, Pixel565::white()));
  ctassert( lut.contains(2, Pixel565::red()));
  ctassert( lut.contains(2, Pixel565::yellow()));
  ctassert(!lut.contains(2, Pixel565::green()));
  ctassert(!lut.contains(2, Pixel565::cyan()));
  ctassert(!lut.contains(2, Pixel565::blue()));
  ctassert( lut.contains(2, Pixel565::magenta()));
  
  ctassert(!lut.contains(3, Pixel565::black()));
  ctassert(!lut.contains(3, Pixel565::white()));
  ctassert(!lut.contains(3, Pixel565::red()));
  ctassert(!lut.contains(3, Pixel565::yellow()));
  ctassert(!lut.contains(3, Pixel565::green()));
  ctassert(!lut.contains(3, Pixel565::cyan()));
  ctassert(!lut.contains(3, Pixel565::blue()));
  ctassert(!lut.contains(3, Pixel565::magenta()));
}

