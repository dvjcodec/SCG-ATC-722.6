
#ifndef nextion_h
#define nextion_h

#include "Stream.h"
#include <SoftTimer.h>

class Nextion
{
public:
  Nextion();
  void setupScreen(Stream &port);
  void setPage(const String page);
  void setText(const String name, const String value);
  void setNumber(const String name, const float value);
  void setVal(const String name, const String value);
  void setPic(const String name, const String value);
  void setVisibility(const String name, const String value);
  void setDim(const int dim);
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
  float mapInt(float x, float in_min, float in_max, int out_min, int out_max);

private:
  Stream *device;
  static Nextion *self;
  void flush();
};

#endif
