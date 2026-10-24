#include "atlas_sensor_kit/pointcloud_conversion.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
using Cloud = sensor_msgs::msg::PointCloud2;
using atlas_sensor_kit::ConversionError;
using atlas_sensor_kit::velodyne_relative_seconds_to_extended;
template<typename T> void put(Cloud & c, size_t offset, T value)
{
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (c.is_bigendian) std::reverse(bytes.begin(), bytes.end());
  std::copy(bytes.begin(), bytes.end(), c.data.begin() + offset);
}
template<typename T> T get(const Cloud & c, size_t offset)
{
  T v; std::memcpy(&v, c.data.data() + offset, sizeof(T)); return v;
}
Cloud input(bool big = false)
{
  Cloud c;
  c.header.frame_id = "recorded_layout_fixture";
  c.header.stamp.sec = 1752868633;
  c.header.stamp.nanosec = 17206000;
  c.width = 2; c.height = 2; c.point_step = 28; c.row_step = 65;
  c.is_bigendian = big; c.is_dense = false;
  // Deliberately shuffled fields, unaligned offsets, point and row padding.
  const std::array<const char *, 6> names{{"time", "ring", "z", "intensity", "x", "y"}};
  const std::array<uint32_t, 6> offsets{{1, 5, 7, 11, 15, 19}};
  for (size_t i = 0; i < 6; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i]; f.offset = offsets[i]; f.count = 1; f.datatype = i == 1 ? 4 : 7;
    c.fields.push_back(f);
  }
  c.data.resize(c.height * c.row_step, 0xab);
  const std::array<float, 4> times{{0.02F, 0.01F, 0.01F, 0.03F}};
  for (size_t i = 0; i < 4; ++i) {
    const size_t base = (i / 2) * c.row_step + (i % 2) * c.point_step;
    put<float>(c, base + 1, times[i]); put<uint16_t>(c, base + 5, uint16_t(500+i));
    put<float>(c, base + 7, float(i+3)); put<float>(c, base + 11, 12.5F);
    put<float>(c, base + 15, float(i+1)); put<float>(c, base + 19, -float(i+2));
  }
  return c;
}
TEST(Velodyne, OrganizedPaddedReorderedEndianAndCompact)
{
  for (bool big : {false, true}) {
    const auto c = input(big); const auto before = c;
    auto result = velodyne_relative_seconds_to_extended(c);
    const auto & out = result.cloud;
    EXPECT_EQ(c, before); EXPECT_TRUE(result.reordered);
    EXPECT_EQ(result.discarded_nonfinite_xyz, 0U);
    EXPECT_EQ(out.header, c.header); EXPECT_TRUE(out.is_dense); EXPECT_FALSE(out.is_bigendian);
    EXPECT_EQ(out.width, 4U); EXPECT_EQ(out.height, 1U); EXPECT_EQ(out.point_step, 32U);
    EXPECT_EQ(out.data.size(), 128U);
    const std::array<size_t, 4> order{{1,2,0,3}};
    for (size_t i = 0; i < order.size(); ++i) {
      EXPECT_FLOAT_EQ(get<float>(out, i*32), order[i]+1);
      EXPECT_FLOAT_EQ(get<float>(out, i*32+4), -float(order[i]+2));
      EXPECT_FLOAT_EQ(get<float>(out, i*32+8), order[i]+3);
      EXPECT_EQ(get<uint16_t>(out, i*32+14), 500+order[i]);
      EXPECT_EQ(out.data[i*32+12], 13); EXPECT_EQ(out.data[i*32+13], 0);
      EXPECT_TRUE(std::isfinite(get<float>(out, i*32+24)));
    }
    EXPECT_EQ(get<uint32_t>(out, 28), uint32_t(std::round(double(0.01F)*1e9)));
    auto compact = atlas_sensor_kit::extended_to_compact(out);
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_EQ(std::memcmp(out.data.data()+i*32, compact.data.data()+i*16, 16), 0);
    }
  }
}
TEST(Velodyne, SkipsOnlyNonfiniteXYZAndHandlesEmpty)
{
  auto c = input();
  put<float>(c, 15, std::numeric_limits<float>::quiet_NaN());
  put<float>(c, 65+19, std::numeric_limits<float>::infinity());
  auto r = velodyne_relative_seconds_to_extended(c);
  EXPECT_EQ(r.discarded_nonfinite_xyz, 2U); EXPECT_EQ(r.cloud.width, 2U);
  EXPECT_FLOAT_EQ(get<float>(r.cloud, 0), 2);
  put<float>(c, 28+15, std::numeric_limits<float>::quiet_NaN());
  put<float>(c, 65+28+15, std::numeric_limits<float>::quiet_NaN());
  r = velodyne_relative_seconds_to_extended(c);
  EXPECT_EQ(r.discarded_nonfinite_xyz, 4U); EXPECT_EQ(r.cloud.width, 0U);
  EXPECT_TRUE(r.cloud.data.empty());
  c.width = 0; c.height = 1; c.row_step = 0; c.data.clear();
  EXPECT_EQ(velodyne_relative_seconds_to_extended(c).cloud.width, 0U);
}
TEST(Velodyne, RejectsMalformedLayouts)
{
  const auto c = input(); auto bad = c;
  bad.row_step = 55; EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.data.pop_back(); EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.data.push_back(0); EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.point_step = 0; EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.height = 0; EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.width = UINT32_MAX; EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  for (size_t i = 0; i < 6; ++i) {
    bad = c; bad.fields.erase(bad.fields.begin()+i);
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
    bad = c; bad.fields.push_back(bad.fields[i]);
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].datatype = 8;
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].count = 2;
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
    bad = c; bad.fields[i].offset = UINT32_MAX;
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  }
  bad = c; bad.fields[0].offset = bad.fields[2].offset;
  EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
}
TEST(Velodyne, RejectsInvalidTimesAndPolarOverflowTransactionally)
{
  auto c = input();
  for (float t : {-0.1F, 5.0F, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()}) {
    auto bad = c; put<float>(bad, 1, t); const auto before = bad;
    EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
    EXPECT_EQ(bad, before);
  }
  auto bad = c; bad.header.stamp.sec = -1;
  EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.header.stamp.nanosec = 1000000000;
  EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  bad = c; bad.header.stamp.nanosec = 900000000; put<float>(bad, 1, 3.5F);
  EXPECT_THROW(velodyne_relative_seconds_to_extended(bad), ConversionError);
  for (size_t off : {7,15,19}) put<float>(c, off, std::numeric_limits<float>::max());
  EXPECT_THROW(velodyne_relative_seconds_to_extended(c), ConversionError);
}
TEST(Velodyne, IntensityPolicyAndAlreadyOrdered)
{
  auto c = input();
  const std::array<float,4> intensities{{-2,300,std::numeric_limits<float>::quiet_NaN(),12.49F}};
  for (size_t i=0;i<4;++i) {
    const size_t base=(i/2)*c.row_step+(i%2)*c.point_step;
    put<float>(c,base+1,float(i)*0.01F); put<float>(c,base+11,intensities[i]);
  }
  const auto r=velodyne_relative_seconds_to_extended(c);
  EXPECT_FALSE(r.reordered);
  EXPECT_EQ(r.cloud.data[12],0); EXPECT_EQ(r.cloud.data[44],255);
  EXPECT_EQ(r.cloud.data[76],0); EXPECT_EQ(r.cloud.data[108],12);
}
}  // namespace
