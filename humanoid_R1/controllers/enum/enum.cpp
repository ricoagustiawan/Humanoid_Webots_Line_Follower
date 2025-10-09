#include "control.hpp"

//Manager Kontrol
#include <GaitManager.hpp>
#include <MotionPlayManager.hpp>
#include <VisionManager.hpp>

//Include Sensor
#include <webots/Accelerometer.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>

//Kontrol Gerakan
#include "Walking.h"
#include "Kinematics.h"
#include "Matrix.h"
#include "Point.h"

//Include Matematis
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace webots;
using namespace managers;
using namespace Robot;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"};

Walk::Walk() : Robot() {
  mTimeStep = getBasicTimeStep();
  mLineLostCounter = 0;
  mControlMode = LINE_FOLLOWER; // Mode awal adalah line follower
  
  //Sensor Yang Dibutuhkan
  mInertialUnit = getInertialUnit("InertialUnit");
  mInertialUnit->enable(mTimeStep);
  
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);

  mCamera = getCamera("Camera");
  mCamera->enable(mTimeStep);
  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  mGyro = getGyro("Gyro");
  mGyro->enable(mTimeStep);

  mKeyboard = getKeyboard();
  mKeyboard->enable(mTimeStep);

  for (int i = 0; i < NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }

  mVisionManager = new VisionManager(width, height, 0, 180, 0, 30, 0.1, 30.0);
  mMotionManager = new MotionPlayManager(this);
  mGaitManager = new GaitManager(this, "config.ini");
}

Walk::~Walk() {
  delete mVisionManager;
  delete mMotionManager;
  delete mGaitManager;
}

void Walk::myStep() {
  if (step(mTimeStep) == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double)ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

// Fungsi baru untuk menangani logika line follower
void Walk::handleLineFollowing() {
  const unsigned char *image = mCamera->getImage();
  if (!image) return;

  int width = mCamera->getWidth();
  int height = mCamera->getHeight();

  double line_position = mVisionManager->getLinePosition(image, width, height);
  bool line_detected = (line_position != -9999);

  if (!line_detected) {
    mLineLostCounter++;
    cout << "Status: Garis Hilang! Mencari... (" << mLineLostCounter << ")" << endl;
    mGaitManager->setXAmplitude(0.0);
    if (mLineLostCounter <= 100) mGaitManager->setAAmplitude(0.5);
    else if (mLineLostCounter <= 300) mGaitManager->setAAmplitude(-0.5);
    else mLineLostCounter = 0;
  } else {
    mLineLostCounter = 0;
    double error = line_position;
    double turn_threshold = width / 6.0;

    if (std::abs(error) < turn_threshold) {
      cout << "Status: Jalan Lurus (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(1.0);
      mGaitManager->setAAmplitude(-error * 0.005);
    } else if (error < 0) {
      cout << "Status: Belok Kiri (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(0.8);
      mGaitManager->setAAmplitude(0.4);
    } else {
      cout << "Status: Belok Kanan (Posisi: " << error << ")" << endl;
      mGaitManager->setXAmplitude(0.8);
      mGaitManager->setAAmplitude(-0.4);
    }
  }
}

void Walk::run() {
  myStep();
  mMotionManager->playPage(9);
  wait(200);

  bool loopplay = false;

  while (true) {
    checkIfFallen();
    myStep(); // myStep() di awal loop untuk update sensor

    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setYAmplitude(0.0);

    if (mControlMode == LINE_FOLLOWER) {
      cout << "Mode Kontrol: Line Follower" << endl;
    } else if (mControlMode == MANUAL_KEYBOARD) {
      cout << "Mode Kontrol: Manual Keyboard" << endl;
    }

    int key = 0;
    while ((key = mKeyboard->getKey()) >= 0) {
    switch (key) {
        case ' ':  // Space bar
          if (loopplay) {
            mGaitManager->stop();
            loopplay = false;
            wait(200);
          } else {
            mGaitManager->start();
            loopplay = true;
            wait(200);
          }
          break;
        case 'M':
          if (mControlMode == LINE_FOLLOWER) {
          mControlMode = MANUAL_KEYBOARD;
          cout << "Mode Kontrol: Manual Keyboard" << endl;
          wait(200);
        } else if (mControlMode == MANUAL_KEYBOARD) {
          mControlMode = LINE_FOLLOWER;
          cout << "Mode Kontrol: Line Follower" << endl;
          wait(200);
        }
        break;
        case Keyboard::UP:    
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setXAmplitude(1.0);} break;
        case Keyboard::DOWN:  
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setXAmplitude(-1.0);} break;
        case Keyboard::RIGHT: 
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setAAmplitude(-0.5);} break;
        case Keyboard::LEFT:  
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setAAmplitude(0.5);} break;
        case ',':             
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setYAmplitude(0.3);} break;
        case '.':             
        if(mControlMode == MANUAL_KEYBOARD){
          mGaitManager->setYAmplitude(-0.3);} break;
        }
      }

    // --- Jalankan Logika Sesuai Mode ---
    if (loopplay) {
      if (!mGaitManager->isWalking()) mGaitManager->start();

      switch (mControlMode) {
        case LINE_FOLLOWER:
          mMotors[19]->setPosition(-0.6); // Arahkan kepala ke bawah
          handleLineFollowing();
          break;
        case MANUAL_KEYBOARD:
          mMotors[19]->setPosition(0.0); // Arahkan kepala lurus
          break;
      }
      mGaitManager->step(mTimeStep);
    }
  }
}

//Kode untuk recover jatuh
void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static int fleft = 0;
  static int fright = 0;
  static const double acc_tolerance = 30.0;
  static const double acc_step = 100;

  const double *acc = mAccelerometer->getValues();

  // Cek jatuh ke depan (sumbu Y)
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;

  // Cek jatuh ke belakang (sumbu Y)
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;

  // Cek jatuh ke kiri (sumbu X)
  if (acc[0] < 400.0 + acc_tolerance)
    fleft++;
  else
    fleft = 0;

  // Cek jatuh ke kanan (sumbu X)
  if (acc[0] > 600.0 - acc_tolerance)
    fright++;
  else
    fright = 0;

  // Jatuh Kedepan
  if (fup > acc_step) {
    cout << "-------Jatuh Kedepan-------" << endl;
    mMotionManager->playPage(10);  // f_up
    mMotionManager->playPage(9);   // init position
    fup = 0; fdown = 0; fleft = 0; fright = 0; // Reset semua counter
  }
  // Jatuh Kebelakang
  else if (fdown > acc_step) {
    cout << "-------Jatuh Kebelakang-------" << endl;
    mMotionManager->playPage(11);  // b_up
    mMotionManager->playPage(9);   // init position
    fdown = 0; fup = 0; fleft = 0; fright = 0; // Reset semua counter
  }
  // Jatuh Kesamping Kiri
  else if (fleft > acc_step) {
    cout << "-------Jatuh Kesamping Kanan, mendorong dengan tangan kanan...-------" << endl;
    
    // Hentikan mode berjalan jika aktif
    mGaitManager->stop();
    
    // --- Urutan Gerakan Tangan Kanan ---
    // Tahap 1: Posisikan lengan untuk mendorong
    mMotors[2]->setPosition(-1.0); // ArmUpperR (bahu kanan ke samping)
    mMotors[0]->setPosition(-1.5); // ShoulderR (bahu kanan ke depan)
    mMotors[4]->setPosition(0.5);  // ArmLowerR (tekuk siku kanan)
    wait(500);

    // Tahap 2: Dorong!
    mMotors[0]->setPosition(1.5); // ArmUpperR (dorong dengan bahu kanan)
    wait(500);

    // Tahap 3: Kembali ke posisi siap
    mMotionManager->playPage(9); // Kembali ke posisi awal
    wait(200);

    // Reset semua counter agar loop berikutnya bisa mendeteksi kondisi baru (jatuh ke depan)
    fup = 0; fdown = 0; fleft = 0; fright = 0;
  }
  // Jatuh Kesamping Kanan
  else if (fright > acc_step) {
    cout << "-------Jatuh Kesamping Kiri, mendorong dengan tangan kiri...-------" << endl;
    
    // Hentikan mode berjalan jika aktif
    mGaitManager->stop();
    
    // --- Urutan Gerakan Tangan Kiri ---
    // Tahap 1: Posisikan lengan untuk mendorong
    mMotors[3]->setPosition(1.0); // ArmUpperL (bahu kiri ke samping)
    mMotors[1]->setPosition(-1.5); // ShoulderL (bahu kiri ke depan)
    mMotors[5]->setPosition(-0.5); // ArmLowerL (tekuk siku kiri)
    wait(500);

    // Tahap 2: Dorong!
    mMotors[3]->setPosition(-1.0); // ArmUpperL (dorong dengan bahu kiri)
    wait(500);

    // Tahap 3: Kembali ke posisi siap
    mMotionManager->playPage(9); // Kembali ke posisi awal
    wait(200);

    // Reset semua counter agar loop berikutnya bisa mendeteksi kondisi baru (jatuh ke depan)
    fup = 0; fdown = 0; fleft = 0; fright = 0;
  }
}
