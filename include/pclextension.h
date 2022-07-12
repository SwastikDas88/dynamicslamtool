#include <vector>
#include <ostream>

#include <pcl/common/io.h>
#include <pcl/pcl_macros.h>
#include <pcl/exceptions.h>
#include <pcl/PCLPointCloud2.h>


#include <boost/predef/other/endian.h>

#include <pcl/PCLHeader.h>
#include <pcl/PCLPointField.h>
// #include <pcl/types.h>

bool
concatenate (pcl::PCLPointCloud2 &cloud1, const pcl::PCLPointCloud2 &cloud2)
{
  if (cloud1.is_bigendian != cloud2.is_bigendian)
  {
    // In future, it might be possible to convert based on pcl::getFieldSize(fields.datatype)
    PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Endianness of clouds does not match\n");
    return (false);
  }

  const auto size1 = cloud1.width * cloud1.height;
  const auto size2 = cloud2.width * cloud2.height;
  //if one input cloud has no points, but the other input does, just select the cloud with points
  switch ((bool (size1) << 1) + bool (size2))
  {
    case 1:
      cloud1 = cloud2;
      PCL_FALLTHROUGH
    case 0:
    case 2:
      cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);
      return (true);
    default:
      break;
  }

  // Ideally this should be in PCLPointField class since this is global behavior
  auto field_eq = [](const auto& field1, const auto& field2)
  {
    // We're fine with the special RGB vs RGBA use case
    return ((field1.name == field2.name) ||
            (field1.name == "rgb" && field2.name == "rgba") ||
            (field1.name == "rgba" && field2.name == "rgb"));
  };

  // A simple memcpy is possible if layout (name and order of fields) is same for both clouds
  bool simple_layout = std::equal(cloud1.fields.begin (),
                                    cloud1.fields.end (),
                                    cloud2.fields.begin (),
                                    cloud2.fields.end (),
                                    field_eq);

  struct FieldDetails
  {
    std::size_t idx1, idx2;
    std::uint16_t size;
    FieldDetails (std::size_t idx1_, std::size_t idx2_, std::uint16_t size_): idx1 (idx1_), idx2 (idx2_), size (size_)
    {}
  };
  std::vector<FieldDetails> valid_fields;
  const auto max_field_size = std::max (cloud1.fields.size (), cloud2.fields.size ());
  valid_fields.reserve (max_field_size);

  // @TODO: Refactor to return std::optional<std::vector<FieldDetails>>
  // Store the details of fields with common data in both cloud, exit early if any errors are found
  if (!simple_layout)
  {
    std::size_t i = 0, j = 0;
    while (i < cloud1.fields.size () && j < cloud2.fields.size ())
    {
      if (cloud1.fields[i].name == "_")
      {
        ++i;
        continue;
      }
      if (cloud2.fields[j].name == "_")
      {
        ++j;
        continue;
      }

      if (field_eq(cloud1.fields[i], cloud2.fields[j]))
      {
        // Assumption: cloud1.fields[i].datatype == cloud2.fields[j].datatype
        valid_fields.emplace_back(i, j, pcl::getFieldSize (cloud2.fields[j].datatype));
        ++i;
        ++j;
        continue;
      }
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Name of field %d in cloud1, %s, does not match name in cloud2, %s\n", i, cloud1.fields[i].name.c_str (), cloud2.fields[i].name.c_str ());
      return (false);
    }
    // Both i and j should have exhausted their respective cloud.fields
    if (i != cloud1.fields.size () || j != cloud2.fields.size ())
    {
      PCL_ERROR ("[pcl::PCLPointCloud2::concatenate] Number of fields to copy in cloud1 (%u) != Number of fields to copy in cloud2 (%u)\n", i, j);
      return (false);
    }
  }

  // Save the latest timestamp in the destination cloud
  cloud1.header.stamp = std::max (cloud1.header.stamp, cloud2.header.stamp);

  cloud1.is_dense = cloud1.is_dense && cloud2.is_dense;
  cloud1.height = 1;
  cloud1.width = size1 + size2;

  if (simple_layout)
  {
    cloud1.data.insert (cloud1.data.end (), cloud2.data.begin (), cloud2.data.end ());
    return (true);
  }
  const auto data1_size = cloud1.data.size ();
  cloud1.data.resize(data1_size + cloud2.data.size ());
  for (std::size_t cp = 0; cp < size2; ++cp)
  {
    for (const auto& field_data: valid_fields)
    {
      const auto& i = field_data.idx1;
      const auto& j = field_data.idx2;
      const auto& size = field_data.size;
      // Leave the data for the skip fields untouched in cloud1
      // Copy only the required data from cloud2 to the correct location for cloud1
      memcpy (reinterpret_cast<char*> (&cloud1.data[data1_size + cp * cloud1.point_step + cloud1.fields[i].offset]),
              reinterpret_cast<const char*> (&cloud2.data[cp * cloud2.point_step + cloud2.fields[j].offset]),
              cloud2.fields[j].count * size);
    }
  }
  return (true);
}