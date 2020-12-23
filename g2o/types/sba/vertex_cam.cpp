// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "vertex_cam.h"

namespace g2o {

// constructor
VertexCam::VertexCam() {}

bool VertexCam::read(std::istream& is) {
  // first the position and orientation (vector3 and quaternion)
  Vector3 t;
  internal::readVector(is, t);
  Quaternion r;
  internal::readVector(is, r.coeffs());
  r.normalize();  // recover nummeric precision

  // form the camera object
  SBACam cam(r, t);

  // now fx, fy, cx, cy, baseline
  number_t fx, fy, cx, cy, tx;

  // try to read one value
  is >> fx;
  if (is.good()) {
    is >> fy >> cx >> cy >> tx;
    cam.setKcam(fx, fy, cx, cy, tx);
  } else {
    is.clear();
    std::cerr << "cam not defined, using defaults" << std::endl;
    cam.setKcam(300, 300, 320, 320, cst(0.1));
  }

  setEstimate(cam);
  return true;
}

bool VertexCam::write(std::ostream& os) const {
  const SBACam& cam = estimate();

  // first the position and orientation (vector3 and quaternion)
  internal::writeVector(os, cam.translation());
  internal::writeVector(os, cam.rotation().coeffs());

  // now fx, fy, cx, cy, baseline
  os << cam.Kcam(0, 0) << " ";
  os << cam.Kcam(1, 1) << " ";
  os << cam.Kcam(0, 2) << " ";
  os << cam.Kcam(1, 2) << " ";
  os << cam.baseline << " ";
  return os.good();
}

}  // namespace g2o
