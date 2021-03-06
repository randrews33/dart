/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dynamics/EllipsoidShape.h"

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

EllipsoidShape::EllipsoidShape(const Eigen::Vector3d& _size)
  : Shape(ELLIPSOID) {
  setSize(_size);
  initMeshes();
}

EllipsoidShape::~EllipsoidShape() {
}

void EllipsoidShape::setDim(const Eigen::Vector3d& _size) {
  setSize(_size);
}

void EllipsoidShape::setSize(const Eigen::Vector3d& _size) {
  assert(_size[0] > 0.0);
  assert(_size[1] > 0.0);
  assert(_size[2] > 0.0);
  mSize = _size;
  mBoundingBoxDim = _size;
  computeVolume();
}

const Eigen::Vector3d&EllipsoidShape::getSize() const {
  return mSize;
}

void EllipsoidShape::draw(renderer::RenderInterface* _ri,
                          const Eigen::Vector4d& _color,
                          bool _useDefaultColor) const {
  if (!_ri)
    return;
  if (!_useDefaultColor)
    _ri->setPenColor(_color);
  else
    _ri->setPenColor(mColor);
  _ri->pushMatrix();
  _ri->transform(mTransform);
  _ri->drawEllipsoid(mBoundingBoxDim);
  _ri->popMatrix();
}

Eigen::Matrix3d EllipsoidShape::computeInertia(double _mass) const {
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  inertia(0, 0) = _mass / 20.0 * (mSize(1) * mSize(1) + mSize(2) * mSize(2));
  inertia(1, 1) = _mass / 20.0 * (mSize(0) * mSize(0) + mSize(2) * mSize(2));
  inertia(2, 2) = _mass / 20.0 * (mSize(0) * mSize(0) + mSize(1) * mSize(1));

  return inertia;
}

bool EllipsoidShape::isSphere() const {
  if (mSize[0] == mSize[1] && mSize[1] == mSize[2])
    return true;
  else
    return false;
}

void EllipsoidShape::computeVolume() {
  // 4/3* Pi* a/2* b/2* c/2
  mVolume = DART_PI * mSize(0) * mSize(1) *mSize(2) / 6;
}

}  // namespace dynamics
}  // namespace dart
