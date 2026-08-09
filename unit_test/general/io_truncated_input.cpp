// Tests that read() reports failure on incomplete input.

#include <sstream>
#include <string>
#include <vector>

#include "g2o/types/slam2d/edge_se2.h"
#include "gtest/gtest.h"

using namespace g2o;

namespace {

std::string truncatedRecord(const EdgeSE2& e, int dropTokens) {
  std::stringstream full;
  e.write(full);

  std::istringstream iss(full.str());
  std::vector<std::string> tokens;
  for (std::string t; iss >> t;) tokens.push_back(t);

  EXPECT_GT(static_cast<int>(tokens.size()), dropTokens)
      << "test needs a record longer than the number of dropped tokens";

  std::string out;
  for (int i = 0; i + dropTokens < static_cast<int>(tokens.size()); ++i)
    out += tokens[i] + " ";
  return out;
}

void fillEdge(EdgeSE2& e) {
  e.setMeasurement(SE2(1.0, 2.0, 0.3));
  e.setInformation(Matrix3::Identity() * 7.0);
}

}  // namespace

TEST(IoTruncatedInput, CompleteRecordReadsSuccessfully) {
  EdgeSE2 out;
  fillEdge(out);

  std::stringstream data;
  ASSERT_TRUE(out.write(data));

  EdgeSE2 in;
  ASSERT_TRUE(in.read(data));
  EXPECT_TRUE(out.measurement().toVector().isApprox(
      in.measurement().toVector(), 1e-9));
  EXPECT_TRUE(out.information().isApprox(in.information(), 1e-9));
}

TEST(IoTruncatedInput, TruncatedInformationMatrixIsRejected) {
  EdgeSE2 out;
  fillEdge(out);
  std::stringstream data(truncatedRecord(out, 3));

  EdgeSE2 in;
  in.setInformation(Matrix3::Zero());
  EXPECT_FALSE(in.read(data))
      << "read() accepted a record whose information matrix was cut short; "
         "the element would be kept with a singular information matrix";
}

TEST(IoTruncatedInput, TruncatedMeasurementIsRejected) {
  EdgeSE2 out;
  fillEdge(out);
  // An EdgeSE2 record is 3 measurement values + 6 upper-triangular
  // information values; dropping 7 leaves the measurement incomplete.
  std::stringstream data(truncatedRecord(out, 7));

  EdgeSE2 in;
  EXPECT_FALSE(in.read(data))
      << "read() accepted a record whose measurement was cut short";
}

TEST(IoTruncatedInput, EmptyStreamIsRejected) {
  std::stringstream data("");
  EdgeSE2 in;
  EXPECT_FALSE(in.read(data)) << "read() accepted an empty record";
}
