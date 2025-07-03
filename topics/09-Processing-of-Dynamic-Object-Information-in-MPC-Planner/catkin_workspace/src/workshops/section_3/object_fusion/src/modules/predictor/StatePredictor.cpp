// Copyright (c) 2022 Institute for Automotive Engineering of RWTH Aachen University

// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "StatePredictor.h"
#include "definitions/utility/ika_utilities.h"

StatePredictor::StatePredictor(std::shared_ptr<Data> data, std::string name)
    : AbstractFusionModule(data, name) {}

void StatePredictor::runSingleSensor()
{
  /** START TASK 1 CODE HERE **/
  {
  Eigen::MatrixXf F = data_->F_const_ + data_->F_timevar_* data_->prediction_gap_in_seconds;
  Eigen::MatrixXf Q = data_->Q_timevar_ * data_->prediction_gap_in_seconds;

  for (auto &globalObject : data_->object_list_fused.objects) {
    auto x_hat_G = IkaUtilities::getEigenStateVec(&globalObject); // estimated global state
    x_hat_G = F * x_hat_G;
    globalObject.P() = F * globalObject.P() * F.transpose() + Q;

    // update global object time stamp
    globalObject.header.stamp = data_->object_list_measured.header.stamp;
  }

  // update global object list time stamp
  data_->object_list_fused.header.stamp = data_->object_list_measured.header.stamp;
}


  /** END TASK 1 CODE HERE **/
}
