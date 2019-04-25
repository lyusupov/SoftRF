/*
MIT License

Copyright (c) 2018 Brian T. Park

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef ACE_BUTTON_TIMING_STATS_H
#define ACE_BUTTON_TIMING_STATS_H

#include <stdint.h>

namespace ace_button {

class TimingStats {
  public:
    /** Constructor. Default copy-constructor and assignment operator ok. */
    TimingStats(): mCounter(0) {
      reset();
    }

    void reset() {
      mExpDecayAvg = 0;
      mMin = UINT16_MAX;
      mMax = 0;
      mSum = 0;
      mCount = 0;
    }

    uint16_t getMax() const { return mMax; }

    uint16_t getMin() const { return mMin; }

    uint16_t getAvg() const { return (mCount > 0) ? mSum / mCount : 0; }

    /** An exponential decay average. */
    uint16_t getExpDecayAvg() const { return mExpDecayAvg; }

    /** Number of times update() was called since last reset. */
    uint16_t getCount() const { return mCount; }

    /**
     * Number of times update() was called from the beginning of time. Never
     * reset. This is useful to determining how many times update() was called
     * since it was last checked from the client code.
     */
    uint16_t getCounter() const { return mCounter; }

    void update(uint16_t duration) {
      mCount++;
      mCounter++;
      mSum += duration;
      if (duration < mMin) {
        mMin = duration;
      }
      if (duration > mMax) {
        mMax = duration;
      }
      mExpDecayAvg = (mExpDecayAvg + duration) / 2;
    }

  private:
    uint16_t mExpDecayAvg;
    uint16_t mMin;
    uint16_t mMax;
    uint32_t mSum;
    uint16_t mCount;
    uint16_t mCounter;
};

}

#endif
