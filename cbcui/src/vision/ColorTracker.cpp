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
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>

// Local includes
#include "DrawBlobs.h"
#include "ctdebug.h"

// Self
#include "ColorTracker.h"

ColorTracker::ColorTracker(int nmodels)
  : m_displayMode(ColorTracker::DisplayRaw),
    m_displayModel(0),
    m_displayImage(NULL),
    m_recordSegments(false),
    m_sharedResults(NULL),
    m_frameNumber(0),
    m_lastFrameTime(0)
{
    this->loadModels();
    for (int i = 0; i < nmodels; i++) m_assemblers.push_back(new BlobAssembler());

    system("mkfifo /tmp/model_cmd");
    m_model_fifo = open("/tmp/model_cmd", O_RDONLY | O_NONBLOCK);
    m_model_fifo_len = 0;

    m_model_op = m_model_out = m_model_in_A = m_model_in_B = m_model_in_C = 0;

    m_min_x = 0;
    m_min_y = 0;
    m_max_x = 160;
    m_max_y = 120;
}

ColorTracker::~ColorTracker()
{
  for (unsigned i = 0; i < m_assemblers.size(); i++) delete m_assemblers[i];
  stopSharingResults();
}

std::string ColorTracker::modelSaveFile() const {
#ifdef QT_ARCH_ARM
  return "/mnt/user/vision/track_colors";
#else
  return QDir::homePath().toStdString() + "/track_colors";
#endif
}
  
void ColorTracker::setModel(uint8 channel, const HSVRange &range)
{
  m_lut.setModel(channel, range);
}

HSVRange ColorTracker::getModel(uint8 channel) const
{
  return m_lut.getModel(channel);
}

bool ColorTracker::loadModels()
{
  ifstream in(this->modelSaveFile().c_str());
  for (unsigned ch = 0; ch < m_assemblers.size(); ch++) {
    if (in.good()) {
      //printf("Loaded channel model %d from %s\n", ch, m_HSVmodelFile);
      HSVRange model(HSV(330, 127, 127), HSV(30, 255, 255));
      in >> model.h.min >> model.h.max >> model.s.min >> model.v.min;
      setModel(ch, model);
    } else {
      //qWarning("loading default models @ %s",this->modelSaveFile().c_str());
        this->loadDefaultModels();
    }
  }
  return true;
}

bool ColorTracker::saveModels()
{
  ofstream out(this->modelSaveFile().c_str());
  //printf("Saving models to %s\n", m_HSVmodelFile);
  for (unsigned ch = 0; ch < m_assemblers.size(); ch++) {
    HSVRange model = getModel(ch);
    out << model.h.min << " " << model.h.max << " "
        << model.s.min << " " << model.v.min << "\n";
  }
  out << "# File format\n";
  out << "# Each line is a channel (0-3)\n";
  out << "# Each channel has 4 numbers:\n";
  out << "# min hue (0-360)\n";
  out << "# max hue (0-360)\n";
  out << "# min saturation (0-255)\n";
  out << "# min brightness (or value) (0-255)\n";
  out.flush();
  system("sync");
  system("sync");
  return out.good();
}

void ColorTracker::loadDefaultModels()
{
    HSVRange redTraining(HSV(330, 127, 127), HSV(30, 255, 255));
    this->setModel(0,redTraining);

    HSVRange yellowTraining(HSV(30, 127, 127), HSV(90, 255, 255));
    this->setModel(1,yellowTraining);

    HSVRange greenTraining(HSV(90, 127, 127), HSV(150, 255, 255));
    this->setModel(2,greenTraining);

    HSVRange blueTraining(HSV(210, 127, 127), HSV(270, 255, 255));
    this->setModel(3,blueTraining);
    
    this->saveModels();
}

// time, in milliseconds
static int mtime()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_usec / 1000 + tv.tv_sec * 1000;
}

void ColorTracker::processFrame(const Image &image)
{
  processModelPipes();

  int thisFrameTime= mtime();
  m_frameNumber++;
  //printf("ColorTracker::processFrame begin\n");
  check_heap();

  Image *display = m_displayImage ? &m_displayImage->m_Image : NULL;

  if (display) display->copy_from(image);
    
  for (unsigned ch = 0; ch < m_assemblers.size(); ch++) {
    bool displayMatches = (ch == m_displayModel) && (m_displayMode == DisplayMatches || m_displayMode == DisplayBlobs);

    assembleBlobs(image,
                  *m_assemblers[ch],
                  ch,
                  displayMatches ? display : NULL);

    if (display &&
        (ch == m_displayModel) &&
        (m_displayMode == DisplayBlobs))
    {
      int minArea = 15;
      DrawBlobs::draw(*display,
                      *m_assemblers[ch],
                      minArea,
                      false,
                      false,
                      false,
                      false);
    }
  }
  if (display) m_displayImage->update();
  check_heap();
  //printf("ColorTracker::processFrame end\n");
  
  updateSharedResults(thisFrameTime);
  m_lastFrameTime = thisFrameTime;
}

void ColorTracker::processModelPipes()
{
  if(! m_model_op)
  {
    m_model_out = m_model_in_A = m_model_in_B = m_model_in_C = 0;
    m_model_fifo_len = read (m_model_fifo, &m_model_op, 1);
  }
  else if(! m_model_out)
  {
    m_model_in_A = m_model_in_B = m_model_in_C = 0;
    m_model_fifo_len = read (m_model_fifo, &m_model_out, 1);
  }
  else if(! m_model_in_A)
  {
    m_model_in_B = m_model_in_C = 0;
    m_model_fifo_len = read (m_model_fifo, &m_model_in_A, 1);
  }
  else if(! m_model_in_B)
  {
    m_model_in_C = 0;
    m_model_fifo_len = read (m_model_fifo, &m_model_in_B, 1);
  }
  else if(! m_model_in_C)
  {
    m_model_fifo_len = read (m_model_fifo, &m_model_in_C, 1);
  }
  else
  {
    if(m_model_op == 1) m_lut.applyCopy(m_model_out-1, m_model_in_A-1);
    if(m_model_op == 2) m_lut.applyNot(m_model_out-1, m_model_in_A-1);
    if(m_model_op == 3) m_lut.applyAnd(m_model_out-1, m_model_in_A-1, m_model_in_B-1);
    if(m_model_op == 4) m_lut.applyOr(m_model_out-1, m_model_in_A-1, m_model_in_B-1);
    if(m_model_op == 5) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.h.min = ((uint16) (m_model_in_B-1) ) << 8 | ((uint16) m_model_in_C);
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 6) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.h.max = ((uint16) (m_model_in_B-1) ) << 8 | ((uint16) m_model_in_C);
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 7) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.s.min = m_model_in_B;
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 8) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.s.max = m_model_in_B;
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 9) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.v.min = m_model_in_B;
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 10) 
    {
      HSVRange model = m_lut.getModel(m_model_in_A-1);
      model.v.max = m_model_in_B;
      m_lut.setModel(m_model_out, model);
    }
    if(m_model_op == 11) m_min_x = m_model_in_A-1;
    if(m_model_op == 12) m_max_x = m_model_in_A-1;
    if(m_model_op == 13) m_min_y = m_model_in_A-1;
    if(m_model_op == 14) m_max_y = m_model_in_A-1;

    m_model_op = m_model_out = m_model_in_A = m_model_in_B = m_model_in_C = 0;
  }
}

void ColorTracker::shareResults(const char *filename)
{
  stopSharingResults();
  m_sharedResults = new SharedMem<TrackingResults>(filename);
}

void ColorTracker::stopSharingResults()
{
  if (m_sharedResults)
  {
    delete m_sharedResults;
    m_sharedResults = NULL;
  }
}

void ColorTracker::updateSharedResults(int frameTime)
{
  if (!m_sharedResults) return;
  
  TrackingResults newResults;
  newResults.frame_number = m_frameNumber;
  newResults.frame_time = frameTime;
  newResults.previous_frame_time = m_lastFrameTime;

  for (newResults.n_channels = 0;
       newResults.n_channels < TRACKING_MAX_CHANNELS &&
       newResults.n_channels < (int)m_assemblers.size();
  newResults.n_channels++)
  {
      ChannelResults &cr = newResults.channels[newResults.n_channels];

      std::vector<Blob*> &sortedBlobs = m_assemblers[newResults.n_channels]->getSortedBlobs();
      
      for (cr.n_blobs=0;
           cr.n_blobs < CHANNEL_MAX_BLOBS &&
           cr.n_blobs < (int)sortedBlobs.size();
      cr.n_blobs++)
      {
          BlobResults &br = cr.blobs[cr.n_blobs];
          Blob *b=sortedBlobs[cr.n_blobs];
          MomentStats stats;
          b->moments.GetStats(stats);

          br.area = b->moments.area;
          // centroid location
          br.x = stats.centroidX;
          br.y = stats.centroidY;
          // TODO:
          // confidence, 0-100
          int bbox_area = (b->right - b->left + 1) * (b->bottom - b->top + 1);
          br.confidence = 100. * br.area / bbox_area;
          // TODO: this asserts sometimes, so there's a bug.
          //ctassert(0 <= br.confidence && br.confidence <= 100);
          br.bbox_left   = b->left;
          br.bbox_top    = b->top;
          br.bbox_right  = b->right;
          br.bbox_bottom = b->bottom;

          // angle in radians of the major axis of the blob.
          // Zero is horizontal and when the left end is higher than the right end the angle will be positive.
          // The range is -PI/2 to +PI/2.
          br.angle = stats.angle;
          // sizes, in pixels, of major and minor axes
          br.major_axis = stats.majorDiameter;
          br.minor_axis = stats.minorDiameter;
      }
  }
  m_sharedResults->write(newResults);
}

void ColorTracker::assembleBlobs(const Image &src, BlobAssembler &bass,
                                 uint8 channel, Image *dest)
{
  // Setup to record segments or not
  Moments::computeAxes= true;
  Pixel565 matchColor = Pixel565::white();

  // Clear any existing blobs
  bass.Reset();
  int dest_offset = dest ? (char*)dest->scanLine(0) - (char*)src.scanLine(0) : 0;

  // Process the image into segments and feed to the blob assembler
  for (int y= /*0*/ m_min_y; y< src.nrows && y<m_max_y; y++) {
    Segment seg;
    seg.row = y;
    const Pixel565 *in = src.scanLine(y);
    int x = /*0*/ m_min_x;
    while (1) {
      // Skip out-of-model pixels.
      while (1) {
        if (x >= src.ncols || x >= m_max_x) goto end_of_row;
        if (m_lut.contains(channel, *in)) break;
        in++; x++;
      }
      // Start segment of in-model pixels
      seg.left = x;
      if (dest) *(Pixel565*)((char*)in+dest_offset)= matchColor;
      in++; x++;
      // Scan through in-model pixels
      while (x < src.ncols && x < m_max_x && m_lut.contains(channel, *in)) {
        if (dest) *(Pixel565*)((char*)in+dest_offset)= matchColor;
        in++; x++;
      }
      // End segment
      seg.right = x-1;
      bass.Add(seg);
      in++; x++;
    }
    end_of_row:;
  }
  bass.EndFrame();
}


void ColorTracker::testImage(const char *filename, int nblobs_expected)
{
  printf("Testing image %s\n", filename);
  int npixels_expected=0;
  Image image(filename);
  for (int y = 0; y < image.nrows; y++) {
    for (int x = 0; x < image.ncols; x++) {
      if (image.pixel(x,y).r8() > 127) {
        image.pixel(x,y)= Pixel565::red();
        npixels_expected++;
      } else {
        image.pixel(x,y)= Pixel565::black();
      }
    }
  }
  ctassert(npixels_expected != image.nrows * image.ncols); // make sure image isn't solid
  processFrame(image);
  int nblobs= 0, npixels= 0;
  for (Blob *b = m_assemblers[0]->firstBlob(); b; b=m_assemblers[0]->nextBlob(b)) {
    nblobs++;
    npixels += b->moments.area;
  }
  ctassert(npixels == npixels_expected);
  ctassert(nblobs == nblobs_expected);
}

void ColorTracker::test()
{
  int num_color_channels=1;
  
  ColorTracker tracker(num_color_channels);
  HSVRange redTraining(HSV(330, 127, 127), HSV(30, 255, 255));
  tracker.setModel(0, redTraining);

  tracker.testImage("test/lumpyblob.png", 1);
  tracker.testImage("test/oneblob.png", 1);
  tracker.testImage("test/three_blobs.png", 3);
  tracker.testImage("test/blob_arch.png", 2);
  tracker.testImage("test/blob_arch_separated.png", 2);
  tracker.testImage("test/one_chevron.png", 1);
  tracker.testImage("test/one_vee.png", 1);
  tracker.testImage("test/three_chevrons.png", 3);
  tracker.testImage("test/three_vees.png", 3);
  tracker.testImage("test/nested_blob.png", 2);
  tracker.testImage("test/nested_vees.png", 3);
  tracker.testImage("test/nested_chevrons.png", 3);
}
