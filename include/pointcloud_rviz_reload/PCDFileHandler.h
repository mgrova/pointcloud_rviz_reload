
#ifndef PCD_FILE_PARSER_H_
#define PCD_FILE_PARSER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rviz/ogre_helpers/point_cloud.h>

#include <memory.h>

#include <QFile>

namespace pointcloud_rviz_reload
{

class PCDFileHandler
{
public:
   PCDFileHandler() {};
   ~PCDFileHandler() {};

   rviz::PointCloud* convertFileToPointCloud(const QString &path)
   {
      try
      {
         const auto croppedPath = path.split(".", QString::SkipEmptyParts).at(0);
         if(!QFile::exists(croppedPath + ".bin"))
            savePCDAsBINFile(croppedPath.toStdString());

         return convertBINToPointCloud(croppedPath.toStdString() + ".bin");
      }
      catch(const std::exception& e)
      {
         std::cerr << __PRETTY_FUNCTION__ << e.what() << '\n';
         return nullptr;
      }
   }

private:
   rviz::PointCloud* convertBINToPointCloud(const std::string& bin_path)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr readed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(bin_path, *readed_cloud) == -1)
         throw std::runtime_error("Couldn't read file: " + bin_path);

      return convertPCLCloudToRVizCloud(readed_cloud);
   }

   void savePCDAsBINFile(const std::string& cropped_path)
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr readed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(cropped_path + ".pcd", *readed_cloud) == -1)
         throw std::runtime_error("Couldn't read file: " + cropped_path);

      if(pcl::io::savePCDFileBinary<pcl::PointXYZRGB>(cropped_path + ".bin", *readed_cloud) == -1)
         throw std::runtime_error("Couldn't save file: " + cropped_path);
   }
   
   rviz::PointCloud* convertPCLCloudToRVizCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
   {
      rviz::PointCloud* target_cloud = new rviz::PointCloud();
      target_cloud->setRenderMode(rviz::PointCloud::RM_SPHERES);
      target_cloud->setDimensions(0.05f, 0.05f, 0.05f); // Render object dimension (i.e. spheres)
      target_cloud->setCommonDirection(Ogre::Vector3(0.0, 1.0, 0.0));
      target_cloud->setCommonUpVector(Ogre::Vector3(0.0, 0.0, -1.0));
      target_cloud->setAlpha(1.0);

      std::vector<rviz::PointCloud::Point> points;
      const float scale_factor = 1.0f;
      for (const auto& readed_point: *cloud)
      {
         rviz::PointCloud::Point point;
         point.position.x = readed_point.x * scale_factor;
         point.position.y = readed_point.y * scale_factor;
         point.position.z = readed_point.z * scale_factor;

         point.setColor(readed_point.r, readed_point.g, readed_point.b);
         points.push_back(point);
      }

      target_cloud->addPoints(&points.front(), points.size());
      return target_cloud;
   }
};
}

#endif