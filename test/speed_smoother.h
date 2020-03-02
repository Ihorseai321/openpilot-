#ifndef SPEED_SMOOTHER_H_
#define SPEED_SMOOTHER_H_
#include <cmath>
#include <algorithm>

float get_delta_out_limits(float aEgo, float aMax, float aMin, float jMax, float jMin)
{
  float tDelta = 0.0;
  if(aEgo > aMax){
    tDelta = (aMax - aEgo) / jMin;
  }
  else if(aEgo < aMin){
    tDelta = (aMin - aEgo) / jMax;
  }

  return tDelta;
}


void speed_smoother(float vEgo, float aEgo, float vT, float aMax, float aMin, float jMax, float jMin, float ts, float ret[])
{
  float dV = vT - vEgo;
  float t1, t2, t3, jMaxcopy, vChange;

  // recover quickly if dV is positive and aEgo is negative or viceversa
  if(dV > 0.0 && aEgo < 0.0){
    jMax *= 3.0;
  }
  else if(dV < 0.0 && aEgo > 0.0){
    jMin *= 3.0;
  }

  float tDelta = get_delta_out_limits(aEgo, aMax, aMin, jMax, jMin);

  if(ts <= tDelta){
    if(aEgo < aMin){
      vEgo += ts * aEgo + 0.5 * ts * ts * jMax;
      aEgo += ts * jMax;
      ret[0] = vEgo;
      ret[1] = aEgo;
    }
    else if(aEgo > aMax){
      vEgo += ts * aEgo + 0.5 * ts * ts * jMin;
      aEgo += ts * jMin;
      ret[0] = vEgo;
      ret[1] = aEgo;
    }
  }

  if(aEgo > aMax){
    dV -= 0.5 * (aMax * aMax - aEgo * aEgo) / jMin;
    vEgo += 0.5 * (aMax * aMax - aEgo * aEgo) / jMin;
    aEgo += tDelta * jMin;
  }
  else if(aEgo < aMin){
    dV -= 0.5 * (aMin * aMin - aEgo * aEgo) / jMax;
    vEgo += 0.5 * (aMin * aMin - aEgo * aEgo) / jMax;
    aEgo += tDelta * jMax;
  }

  ts -= tDelta;

  float jLim = aEgo >= 0 ? jMin : jMax;
  // if we reduce the accel to zero immediately, how much delta speed we generate?
  float dv_min_shift = -0.5 * aEgo * aEgo / jLim;

  // flip signs so we can consider only one case
  bool flipped = false;
  if(dV < dv_min_shift){
    flipped = true;
    dV *= -1;
    vEgo *= -1;
    aEgo *= -1;
    aMax = -aMin;
    jMaxcopy = -jMin;
    jMin = -jMax;
    jMax = jMaxcopy;
  }
  // small addition needed to avoid numerical issues with sqrt of ~zero
  float aPeak = sqrt((0.5 * aEgo * aEgo / jMax + dV + 1e-9) / (0.5 / jMax - 0.5 / jMin));

  if(aPeak > aMax){
    aPeak = aMax;
    t1 = (aPeak - aEgo) / jMax;
    if(aPeak <= 0){ // there is no solution, so stop after t1
      t2 = t1 + ts + 1e-9;
      t3 = t2;
    }
    else{
      vChange = dV - 0.5 * (aPeak * aPeak - aEgo *aEgo) / jMax + 0.5 * aPeak * aPeak / jMin;
      if(vChange < aPeak * ts){
        t2 = t1 + vChange / aPeak;
      }
      else{
        t2 = t1 + ts;
      }
      t3 = t2 - aPeak / jMin;
    }
  }
  else{
    t1 = (aPeak - aEgo) / jMax;
    t2 = t1;
    t3 = t2 - aPeak / jMin;
  }

  float dt1 = std::min(ts, t1);
  float dt2 = std::max(std::min(ts, t2) - t1, float(0.0));
  float dt3 = std::max(std::min(ts, t3) - t2, float(0.0));

  if(ts > t3){
    vEgo += dV;
    aEgo = 0.0;
  }
  else{
    vEgo += aEgo * dt1 + 0.5 * dt1 * dt1 * jMax + aPeak * dt2 + aPeak * dt3 + 0.5 * dt3 * dt3 * jMin;
    aEgo += jMax * dt1 + dt3 * jMin;
  }

  vEgo *= flipped ? -1 : 1;
  aEgo *= flipped ? -1 : 1;

  ret[0] = vEgo;
  ret[1] = aEgo;
}

#endif //SPEED_SMOOTHER_H_
