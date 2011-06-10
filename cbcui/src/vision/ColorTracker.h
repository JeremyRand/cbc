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

#ifndef INCLUDE_ColorTracker_h
#define INCLUDE_ColorTracker_h

// Local includes
#include "ImageDisplay.h"
#include "FrameHandler.h"
#include "BlobAssembler.h"
#include "HSVRangeLUT.h"
#include <SharedMem.h>
#include "TrackingResults.h"

class ColorTracker : public FrameHandler {
public:
  enum DisplayMode {
    DisplayRaw=0,
    DisplayMatches,
    DisplayBlobs
  };
  ColorTracker(int nmodels);
  ~ColorTracker();
  virtual void processFrame(const Image &image);
  void processModelPipes();
  void setModel(uint8 channel, const HSVRange &range);
  HSVRange getModel(uint8 channel) const;
  bool loadModels();
  bool saveModels();
  void loadDefaultModels();
  
  void setDisplayModel(int model) { m_displayModel = model; }
  void setDisplayMode(DisplayMode mode) { m_displayMode = mode; }
  void setImageDisplay(ImageDisplay *image) { m_displayImage = image; }
  int getDisplayModel() const { return m_displayModel; }
  void shareResults(const char *filename);
  void stopSharingResults();
  static void test();

protected:
  void updateSharedResults(int frameTime);
  void testImage(const char *filename, int nblobs_expected);
  std::vector<BlobAssembler*> m_assemblers;

  std::string modelSaveFile() const;

  uint8 m_model_fifo, m_model_fifo_len, m_model_op, m_model_out, m_model_in_A, m_model_in_B, m_model_in_C;

  // For viewing images and tracking
  DisplayMode m_displayMode;
  unsigned int m_displayModel; // model # to display (0-based)
  ImageDisplay *m_displayImage; // image to display on
  
  bool m_recordSegments;
  SharedMem<TrackingResults> *m_sharedResults;
  int m_frameNumber;
  int m_lastFrameTime;
  void assembleBlobs(const Image &in, BlobAssembler &bass,
                     uint8 model, Image *out);
  HSVRangeLUT m_lut;
  char m_HSVmodelFile[];
};
  
#endif
