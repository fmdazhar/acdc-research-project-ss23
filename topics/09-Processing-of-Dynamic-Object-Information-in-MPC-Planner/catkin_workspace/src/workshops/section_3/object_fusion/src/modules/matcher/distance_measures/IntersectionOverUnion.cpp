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

#include "modules/matcher/distance_measures/IntersectionOverUnion.h"
#include "data/Params.h"

IouCalculator::IouCalculator() {} // constructor 

float IouCalculator::extendedDistance(
    const definitions::IkaObject *measured_object,
    const GlobalObject *global_object) {

  // Get the values from objects and transform to a bbox format for both objects (top left and bottom right corners)
  float global_heading = IkaUtilities::getObjectHeading(*global_object);
  IouCalculator::Bbox measured_bbox = IouCalculator::getBbox(
    &IkaUtilities::getObjectPosition(*measured_object)[0], // X 
    &IkaUtilities::getObjectPosition(*measured_object)[1], // Y
    &IkaUtilities::getObjectSize(*measured_object)[0], // length
    &IkaUtilities::getObjectSize(*measured_object)[1], // width
    &global_heading); // rotate the coordinate system to global object heading
  IouCalculator::Bbox global_bbox = IouCalculator::getBbox(
    &IkaUtilities::getObjectPosition(*global_object)[0], // X 
    &IkaUtilities::getObjectPosition(*global_object)[1], // Y
    &IkaUtilities::getObjectSize(*global_object)[0], // length
    &IkaUtilities::getObjectSize(*global_object)[1], // width
    &global_heading); // rotate the coordinate system to global object heading
   
  float iou = IouCalculator::computeIoU(measured_bbox, global_bbox);

  // Check if within threshold and return inverted (iou: [1,0] with 1 best and 0 worst; for association the lower the value, the better)
  bool within_threshold = (iou > Params::get().cfg.iou_overlap_threshold);
  float association_value;
  if (within_threshold) {
    association_value = 1 - iou;
  } else {
    association_value = -1.f; // otherwise -1 to prevent association (convention)
  }
  return association_value;
}

float IouCalculator::computeIoU(IouCalculator::Bbox &a, IouCalculator::Bbox &b)
{
  /** START TASK 2.1 CODE HERE **/
  // The order of input bounding boxes a,b doesn't matter
  // Get the intersection bounding box 
  IouCalculator::Bbox inter; 
  inter.top_left_x = std::min(a.top_left_x, b.top_left_x);
  inter.top_left_y = std::min(a.top_left_y, b.top_left_y);
  inter.bottom_right_x = std::max(a.bottom_right_x, b.bottom_right_x);
  inter.bottom_right_y = std::max(a.bottom_right_y, b.bottom_right_y);
  inter.length = inter.top_left_x - inter.bottom_right_x;
  inter.width = inter.top_left_y - inter.bottom_right_y;

  // Edge cases with bbox being a line or no overlap between bboxes 
  bool is_no_overlap = inter.length <= 0 || inter.width <= 0;
  bool is_either_bbox_line = a.length == 0 || a.width == 0 || b.length == 0 || b.width == 0;
  if (is_no_overlap || is_either_bbox_line){
    return 0.f; // will be later on set to -1, as below threshold
  }
  // Compute bbox areas 
  float area_inter = inter.length * inter.width;
  float area_a = a.length * a.width;
  float area_b = b.length * b.width;
  float area_combined = area_a + area_b - area_inter;

  float iou = area_inter / area_combined;
  return iou;
}

IouCalculator::Bbox IouCalculator::getBbox(const float *pos_x, const float *pos_y, 
const float *length, const float *width, const float *rot_angle) {
  // Transform to a bbox format (top left and bottom right corners of the bbox)

  // rotate the coordinates to the angle of global object heading 
  float new_x = *pos_x * std::cos(*rot_angle) - *pos_y * std::sin(*rot_angle);
  float new_y = *pos_x * std::sin(*rot_angle) + *pos_y * std::cos(*rot_angle);
  
  IouCalculator::Bbox bbox;
  bbox.length = *length; // dereference the pointers to values 
  bbox.width = *width;
  bbox.top_left_x = new_x + 0.5 * (*length);
  bbox.top_left_y = new_y + 0.5 * (*width);
  bbox.bottom_right_x = new_x - 0.5 * (*length);
  bbox.bottom_right_y = new_y - 0.5 * (*width);

  return bbox; //return by value
}
