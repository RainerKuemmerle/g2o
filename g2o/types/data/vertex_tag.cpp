// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "vertex_tag.h"

#include "g2o/stuff/macros.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/EXTERNAL/freeglut/freeglut_minimal.h"
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iomanip>

namespace g2o {

bool VertexTag::read(std::istream& is) {
  is >> name_;
  is >> position_.x() >> position_.y() >> position_.z();
  is >> odom2d_.x() >> odom2d_.y() >> odom2d_.z();
  is >> timestamp_;
  is >> hostname_;
  is >> loggerTimestamp_;
  return true;
}

bool VertexTag::write(std::ostream& os) const {
  os << name_ << " ";
  os << FIXED(position_.x()
              << " " << position_.y() << " " << position_.z() << " ");
  os << FIXED(odom2d_.x() << " " << odom2d_.y() << " " << odom2d_.z() << " ");
  os << FIXED(" " << timestamp() << " " << hostname() << " "
                  << loggerTimestamp());
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
VertexTagDrawAction::VertexTagDrawAction()
    : DrawAction(typeid(VertexTag).name()), textSize_(nullptr) {}

bool VertexTagDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    textSize_ = previousParams_->makeProperty<DoubleProperty>(
        typeName_ + "::TEXT_SIZE", 1);
  } else {
    textSize_ = nullptr;
  }
  return true;
}

bool VertexTagDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) {
    return true;
  }
  auto* that = static_cast<VertexTag*>(&element);

  glPushMatrix();
  glColor3f(1.F, 0.2F, 1.F);
  glTranslatef(that->position().x(), that->position().y(),
               that->position().z());
  float textSize = 1;
  if (textSize_) textSize = static_cast<float>(textSize_->value());
  opengl::drawBox(0.1F * textSize, 0.1F * textSize, 0.1F * textSize);
  glTranslatef(0.2F * textSize, 0.F, 0.F);
  glScalef(0.003F * textSize, 0.003F * textSize, 1.F);
  freeglut_minimal::glutStrokeString(freeglut_minimal::kGlutStrokeRoman,
                                     that->name().c_str());
  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
