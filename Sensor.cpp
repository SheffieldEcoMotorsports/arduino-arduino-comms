
#include "Sensor.h"

bool Sensor::Connect(int ID)
{
  this-> ID = ID;
  bool Status = true;
  return (bool)Status;
}

float Sensor::Read()
{
  this->value = 0;
  return (float)this->value;
}

float Sensor::GetLatestVal()
{
  return (float)this->value;
}

float Sensor::GetID()
{
  return (int)this->ID;
}
