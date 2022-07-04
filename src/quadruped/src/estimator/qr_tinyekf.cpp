#include "estimator/qr_tinyekf.h"

qrTinyEKF::qrTinyEKF():TinyEKF(){

}

qrTinyEKF::~qrTinyEKF()
{

}

qrTinyEKF::qrTinyEKF(float x, float accelerometerVariance, float sensorVariance):TinyEKF(){
  for(int i = 0; i < Nsta; i++){
    setX(i, double(x));
  }

  for(int i=0; i< Nsta; ++i) {
    TinyEKF::setQ(i, i, double(accelerometerVariance));
    TinyEKF::setR(i, i, double(sensorVariance));
  }
}

void qrTinyEKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
{
  for(int i = 0; i < Nsta; i++){
    fx[i] = this->x[i];
  }

  for(int i = 0; i < Nsta; i++){
    F[i][i] = 1;
  }

  for(int i = 0; i < Mobs; i++){
    hx[i] = fx[i];
  }

  H[0][0] = 1;
  H[1][1] = 1;
  H[2][2] = 1;
}

void qrTinyEKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta], double deltaX[Nsta])
{
  for(int i = 0; i < Nsta; i++){
    fx[i] = this->x[i] + deltaX[i];
  }

  for(int i = 0; i < Nsta; i++){
    F[i][i] = 1;
  }

  for(int i = 0; i < Mobs; i++){
    hx[i] = fx[i];
  }

  H[0][0] = 1;
  H[1][1] = 1;
  H[2][2] = 1;
}

bool qrTinyEKF::step(double *deltaX, double *z)
{
  this->model(this->ekf.fx, this->ekf.F, this->ekf.hx, this->ekf.H, deltaX);
  return ekf_step(&this->ekf, z) ? false : true;
}

bool qrTinyEKF::step(Vec3<float> deltaX, Vec3<float> z)
{
  double deltaXArray[3] = {double(deltaX[0]), double(deltaX[1]), double(deltaX[2])};
  double zArray[3]      = {double(z[0]), double(z[1]), double(z[2])};
  return step(deltaXArray, zArray);
}
